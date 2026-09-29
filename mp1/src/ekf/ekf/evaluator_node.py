#!/usr/bin/env python3
"""
Compare pose estimates against the ground truth and draw their paths.

For every ground-truth sample (/ground_truth/pose, ~116 Hz) the latest pose of
each estimator is compared to it (if that pose is not older than max_age):
    position error  = Euclidean distance in x, y          [m]
    heading error   = |wrapped yaw difference|            [rad]

Estimators (all nav_msgs/Odometry):
    ekf                 /ekf/odom                own EKF
    robot_localization  /rl/odometry/filtered    robot_localization ekf_node
    odometry            /odom                    raw wheel odometry (dead reckoning);
                        its odom frame coincides with map at the start of the bag

Paths are published on /eval/path/<name> for RViz. A summary (RMSE, mean,
median, 95th percentile, max) is logged every 10 s, and when the bag stops
(no ground truth for a few seconds) all samples are written to a CSV file for
the report.
"""

import csv
import math
import os
import time

from ekf.ekf_core import wrap_angle
from ekf.geometry import stamp_to_sec, yaw_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


ESTIMATORS = {
    'ekf': '/ekf/odom',
    'robot_localization': '/rl/odometry/filtered',
    'odometry': '/odom',
}


class EvaluatorNode(Node):

    def __init__(self):
        super().__init__('evaluator_node')
        self.world_frame = self.declare_parameter('world_frame', 'map').value
        self.max_age = self.declare_parameter('max_age', 0.1).value
        self.path_spacing = self.declare_parameter('path_spacing', 0.05).value
        self.output_dir = os.path.expanduser(
            self.declare_parameter('output_dir', '~/ros2_ws/results').value)
        self.run_name = self.declare_parameter('run_name', 'task1a').value

        self.latest = {}                          # name -> (t, x, y, yaw, sx, sy, syaw)
        self.rows = []                            # one row per (gt sample, estimator)
        self.paths = {name: self.new_path() for name in list(ESTIMATORS) + ['ground_truth']}
        self.path_pubs = {name: self.create_publisher(Path, f'/eval/path/{name}', 1)
                          for name in self.paths}

        for name, topic in ESTIMATORS.items():
            qos = qos_profile_sensor_data if name == 'odometry' else 10
            self.create_subscription(
                Odometry, topic, lambda msg, n=name: self.estimate_callback(n, msg), qos)
        self.create_subscription(PoseStamped, '/ground_truth/pose', self.gt_callback, 50)

        # Wall-clock timers: they keep running after the bag (and sim time) stops.
        wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.5, self.publish_paths, clock=wall)
        self.create_timer(10.0, self.log_summary, clock=wall)
        self.create_timer(1.0, self.check_finished, clock=wall)
        self.last_gt_wall = None
        self.saved_rows = 0

    def new_path(self):
        path = Path()
        path.header.frame_id = self.world_frame
        return path

    def append_to_path(self, name, stamp, pose):
        poses = self.paths[name].poses
        if poses:
            last = poses[-1]
            if stamp_to_sec(stamp) - stamp_to_sec(last.header.stamp) < self.path_spacing:
                return
            if stamp_to_sec(stamp) < stamp_to_sec(last.header.stamp):
                poses.clear()               # bag restarted
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.world_frame
        ps.pose = pose
        poses.append(ps)

    def estimate_callback(self, name, msg):
        pose = msg.pose.pose
        c = msg.pose.covariance
        self.latest[name] = (stamp_to_sec(msg.header.stamp), pose.position.x, pose.position.y,
                             yaw_from_quaternion(pose.orientation),
                             math.sqrt(max(c[0], 0.0)), math.sqrt(max(c[7], 0.0)),
                             math.sqrt(max(c[35], 0.0)))
        self.append_to_path(name, msg.header.stamp, pose)

    def gt_callback(self, msg):
        self.last_gt_wall = time.monotonic()
        t = stamp_to_sec(msg.header.stamp)
        gx, gy = msg.pose.position.x, msg.pose.position.y
        gyaw = yaw_from_quaternion(msg.pose.orientation)
        self.append_to_path('ground_truth', msg.header.stamp, msg.pose)

        for name, (te, x, y, yaw, sx, sy, syaw) in self.latest.items():
            if abs(t - te) > self.max_age:
                continue
            self.rows.append({
                't': t, 'estimator': name,
                'x': x, 'y': y, 'yaw': yaw,
                'gt_x': gx, 'gt_y': gy, 'gt_yaw': gyaw,
                'error_xy': math.hypot(x - gx, y - gy),
                'error_yaw': abs(wrap_angle(yaw - gyaw)),
                'sigma_x': sx, 'sigma_y': sy, 'sigma_yaw': syaw,
            })

    def publish_paths(self):
        now = self.get_clock().now().to_msg()
        for name, path in self.paths.items():
            if path.poses:
                path.header.stamp = now
                self.path_pubs[name].publish(path)

    def summary_lines(self):
        lines = []
        for name in ESTIMATORS:
            e = np.array([r['error_xy'] for r in self.rows if r['estimator'] == name])
            ey = np.array([r['error_yaw'] for r in self.rows if r['estimator'] == name])
            if len(e) == 0:
                continue
            lines.append(
                f'{name:>18}: n={len(e):5d}  pos RMSE {np.sqrt(np.mean(e ** 2)) * 100:5.2f} cm'
                f'  mean {e.mean() * 100:5.2f}  median {np.median(e) * 100:5.2f}'
                f'  p95 {np.percentile(e, 95) * 100:5.2f}  max {e.max() * 100:5.2f} cm'
                f' | yaw RMSE {math.degrees(np.sqrt(np.mean(ey ** 2))):5.2f} deg')
        return lines

    def log_summary(self):
        lines = self.summary_lines()
        if lines:
            self.get_logger().info('Error vs ground truth so far:\n' + '\n'.join(lines))

    def check_finished(self):
        # the bag has stopped if no ground truth arrived for 3 s
        if (self.last_gt_wall is not None and time.monotonic() - self.last_gt_wall > 3.0
                and len(self.rows) > self.saved_rows):
            self.save()

    def save(self):
        if not self.rows:
            return
        os.makedirs(self.output_dir, exist_ok=True)
        filename = os.path.join(
            self.output_dir, f'{self.run_name}_{time.strftime("%Y%m%d_%H%M%S")}.csv')
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)
        self.saved_rows = len(self.rows)
        self.get_logger().info(
            'Final error vs ground truth:\n' + '\n'.join(self.summary_lines()) +
            f'\nSaved {len(self.rows)} samples to {filename}')


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if len(node.rows) > node.saved_rows:
            node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

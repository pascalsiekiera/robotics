#!/usr/bin/env python3
"""
Compare pose estimates against the ground truth and draw their paths.

For every ground-truth sample (/ground_truth/pose, ~116 Hz) each estimator's
pose at that time is compared to it:
    position error  = Euclidean distance in x, y          [m]
    heading error   = |wrapped yaw difference|            [rad]

Two kinds of estimators:

  topic estimators  (parameter topic_estimators, entries "name:/topic")
      nav_msgs/Odometry topics, e.g. the own EKF (/ekf/odom) or the raw wheel
      odometry (/odom; its odom frame coincides with map at the start of the
      bag). The latest message is used if it is not older than max_age.

  TF estimator      (parameter tf_estimator, e.g. "amcl" or "slam_toolbox")
      AMCL and slam_toolbox do not publish a continuous pose topic: their
      estimate is the TF chain map -> odom -> base_footprint (they publish
      map -> odom, the bag publishes odom -> base_footprint). This chain is
      looked up at the ground-truth stamp. The lookup is delayed by tf_delay so
      that the TF data for that instant has arrived. Optionally the pose
      uncertainty is read from a PoseWithCovarianceStamped topic
      (tf_covariance_topic, e.g. /amcl_pose).

Paths are published on /eval/path/<name> for RViz. A summary (RMSE, mean,
median, 95th percentile, max) is logged every 10 s, and when the bag stops
(no ground truth for a few seconds) all samples are written to a CSV file for
the report.
"""

import collections
import csv
import math
import os
import time

from ekf.ekf_core import wrap_angle
from ekf.geometry import stamp_to_sec, yaw_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class EvaluatorNode(Node):

    def __init__(self):
        super().__init__('evaluator_node')
        p = self.declare_parameter
        self.world_frame = p('world_frame', 'map').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.max_age = p('max_age', 0.1).value
        self.path_spacing = p('path_spacing', 0.05).value
        self.output_dir = os.path.expanduser(p('output_dir', '~/ros2_ws/results').value)
        self.run_name = p('run_name', 'task1a').value
        topic_estimators = p('topic_estimators', ['ekf:/ekf/odom',
                                                  'robot_localization:/rl/odometry/filtered',
                                                  'odometry:/odom']).value
        self.tf_estimator = p('tf_estimator', '').value
        self.tf_delay = p('tf_delay', 0.3).value
        tf_covariance_topic = p('tf_covariance_topic', '').value

        self.estimators = [e.split(':', 1)[0] for e in topic_estimators if e]
        if self.tf_estimator:
            self.estimators.append(self.tf_estimator)

        self.latest = {}                  # topic estimator name -> (t, x, y, yaw, sx, sy, syaw)
        self.tf_sigma = (math.nan, math.nan, math.nan)
        self.pending_gt = collections.deque()   # gt samples waiting for the TF lookup
        self.newest_gt_t = None
        self.rows = []                    # one row per (gt sample, estimator)
        self.paths = {name: self.new_path() for name in self.estimators + ['ground_truth']}
        self.path_pubs = {name: self.create_publisher(Path, f'/eval/path/{name}', 1)
                          for name in self.paths}

        for entry in topic_estimators:
            if not entry:
                continue
            name, topic = entry.split(':', 1)
            self.create_subscription(
                Odometry, topic, lambda msg, n=name: self.estimate_callback(n, msg),
                qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/ground_truth/pose', self.gt_callback, 50)

        if self.tf_estimator:
            self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            if tf_covariance_topic:
                self.create_subscription(PoseWithCovarianceStamped, tf_covariance_topic,
                                         self.tf_covariance_callback, 10)

        # Wall-clock timers: they keep running after the bag (and sim time) stops.
        wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.5, self.publish_paths, clock=wall)
        self.create_timer(10.0, self.log_summary, clock=wall)
        self.create_timer(1.0, self.check_finished, clock=wall)
        if self.tf_estimator:
            self.create_timer(0.05, self.process_pending_gt, clock=wall)
        self.last_gt_wall = None
        self.saved_rows = 0
        self.get_logger().info(f'Evaluating: {", ".join(self.estimators)}')

    # ---------------------------------------------------------------- paths

    def new_path(self):
        path = Path()
        path.header.frame_id = self.world_frame
        return path

    def append_to_path(self, name, stamp, pose):
        poses = self.paths[name].poses
        if poses:
            dt = stamp_to_sec(stamp) - stamp_to_sec(poses[-1].header.stamp)
            if dt < 0:
                poses.clear()               # bag restarted
            elif dt < self.path_spacing:
                return
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.world_frame
        ps.pose = pose
        poses.append(ps)

    def publish_paths(self):
        now = self.get_clock().now().to_msg()
        for name, path in self.paths.items():
            if path.poses:
                path.header.stamp = now
                self.path_pubs[name].publish(path)

    # ------------------------------------------------------------ estimates

    def estimate_callback(self, name, msg):
        pose = msg.pose.pose
        c = msg.pose.covariance
        self.latest[name] = (stamp_to_sec(msg.header.stamp), pose.position.x, pose.position.y,
                             yaw_from_quaternion(pose.orientation),
                             math.sqrt(max(c[0], 0.0)), math.sqrt(max(c[7], 0.0)),
                             math.sqrt(max(c[35], 0.0)))
        self.append_to_path(name, msg.header.stamp, pose)

    def tf_covariance_callback(self, msg):
        c = msg.pose.covariance
        self.tf_sigma = (math.sqrt(max(c[0], 0.0)), math.sqrt(max(c[7], 0.0)),
                         math.sqrt(max(c[35], 0.0)))

    def add_row(self, name, t, estimate, gt):
        x, y, yaw, sx, sy, syaw = estimate
        gx, gy, gyaw = gt
        self.rows.append({
            't': t, 'estimator': name,
            'x': x, 'y': y, 'yaw': yaw,
            'gt_x': gx, 'gt_y': gy, 'gt_yaw': gyaw,
            'error_xy': math.hypot(x - gx, y - gy),
            'error_yaw': abs(wrap_angle(yaw - gyaw)),
            'sigma_x': sx, 'sigma_y': sy, 'sigma_yaw': syaw,
        })

    def gt_callback(self, msg):
        self.last_gt_wall = time.monotonic()
        t = stamp_to_sec(msg.header.stamp)
        gt = (msg.pose.position.x, msg.pose.position.y, yaw_from_quaternion(msg.pose.orientation))
        self.append_to_path('ground_truth', msg.header.stamp, msg.pose)

        for name, (te, *estimate) in self.latest.items():
            if abs(t - te) <= self.max_age:
                self.add_row(name, t, estimate, gt)

        if self.tf_estimator:
            self.pending_gt.append((msg.header.stamp, gt))
            self.newest_gt_t = t

    def process_pending_gt(self, flush=False):
        """Look up map -> base_footprint for ground-truth samples old enough."""
        while self.pending_gt:
            stamp, gt = self.pending_gt[0]
            t = stamp_to_sec(stamp)
            if not flush and t > self.newest_gt_t - self.tf_delay:
                return
            self.pending_gt.popleft()
            try:
                tf = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame,
                                                     Time.from_msg(stamp))
            except TransformException:
                continue       # e.g. before AMCL / SLAM published its first map -> odom
            tr = tf.transform
            yaw = yaw_from_quaternion(tr.rotation)
            self.add_row(self.tf_estimator, t,
                         (tr.translation.x, tr.translation.y, yaw, *self.tf_sigma), gt)
            pose = Pose()
            pose.position.x, pose.position.y = tr.translation.x, tr.translation.y
            pose.orientation = tr.rotation
            self.append_to_path(self.tf_estimator, stamp, pose)

    # -------------------------------------------------------------- results

    def summary_lines(self):
        lines = []
        for name in self.estimators:
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
        if self.last_gt_wall is None or time.monotonic() - self.last_gt_wall < 3.0:
            return
        if self.tf_estimator:
            self.process_pending_gt(flush=True)
        if len(self.rows) > self.saved_rows:
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

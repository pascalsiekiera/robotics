#!/usr/bin/env python3
"""Task 1a, part 2: drive the start/goal pairs in Gazebo and measure the path following.

For every pair: drive to the start (not evaluated), then send the goal and record
  - the planned path (/plan, published by planner_server; with the plan-once behavior
    tree this is the path the controller follows; replans are counted),
  - the estimated path: tf map -> base_footprint (AMCL + odometry) at 10 Hz,
  - the true path from the Gazebo ground truth (/ground_truth/pose).
Metrics: deviation from the planned path
  - of the estimated path: what the controller sees and corrects (controller error),
  - of the true path: controller error + localization error.

Requires: ros2 launch mp2/launch/sim.launch.py

Example:
    python3 drive_compare.py --planner GridBased --tag dijkstra_dwb
"""
import argparse
import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from metrics import deviation, path_length

HERE = os.path.dirname(os.path.abspath(__file__))
MP2 = os.path.dirname(HERE)


def make_pose(nav, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Recorder:
    """Collects planned paths, estimated and true poses with time stamp >= t0 (sim time).

    The time stamp filter matters: messages that queued up while nobody was spinning
    (e.g. from the drive to the start) are delivered later and must not be counted.
    """

    def __init__(self, nav):
        self.nav = nav
        self.t0 = math.inf
        self.reset()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, nav)
        nav.create_subscription(Path, 'plan', self.on_plan, 10)
        nav.create_subscription(PoseStamped, 'ground_truth/pose', self.on_gt, 50)
        nav.create_timer(0.1, self.on_timer)

    def start(self):
        self.reset()
        self.t0 = self.nav.get_clock().now().nanoseconds * 1e-9

    def stop(self):
        self.t0 = math.inf

    def reset(self):
        self.plans, self.gt, self.est = [], [], []

    def on_plan(self, msg):
        if stamp(msg) >= self.t0 and msg.poses:
            self.plans.append([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def on_gt(self, msg):
        if stamp(msg) >= self.t0:
            p = msg.pose.position
            self.gt.append([stamp(msg), p.x, p.y])

    def on_timer(self):
        if math.isinf(self.t0):
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return
        t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        if t >= self.t0 and (not self.est or t > self.est[-1][0]):
            self.est.append([t, tf.transform.translation.x, tf.transform.translation.y])


def navigate(nav, pose, timeout_s):
    """Send a NavigateToPose goal and wait. Returns (result, wall-clock duration)."""
    t0 = time.monotonic()
    if not nav.goToPose(pose):
        return TaskResult.FAILED, 0.0
    while not nav.isTaskComplete():
        if time.monotonic() - t0 > timeout_s:
            nav.cancelTask()
            nav.warn('timeout, goal cancelled')
            break
    return nav.getResult(), time.monotonic() - t0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--planner', default='GridBased',
                        help='planner id, must be the active one (see --tag)')
    parser.add_argument('--pairs', default=os.path.join(MP2, 'config', 'test_pairs.yaml'))
    parser.add_argument('--only', nargs='+', default=None, help='only these pairs, e.g. P1')
    parser.add_argument('--tag', default='dijkstra_dwb', help='name of this configuration')
    parser.add_argument('--timeout', type=float, default=240.0, help='per goal, wall clock [s]')
    parser.add_argument('--out', default=os.path.join(MP2, 'data', 'task1a'))
    args = parser.parse_args()

    with open(args.pairs) as f:
        pairs = yaml.safe_load(f)
    if args.only:
        pairs = {k: v for k, v in pairs.items() if k in args.only}
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    # not waitUntilNav2Active(): it would reset AMCL to (0, 0) (see README, pitfalls)
    nav._waitForNodeToActivate('bt_navigator')
    rec = Recorder(nav)

    runs = []
    for name, pair in pairs.items():
        (sx, sy), (gx, gy) = pair['start'], pair['goal']
        yaw = math.atan2(gy - sy, gx - sx)

        nav.info(f'{name}: driving to start ({sx}, {sy})')
        result, _ = navigate(nav, make_pose(nav, sx, sy, yaw), args.timeout)
        if result != TaskResult.SUCCEEDED:
            nav.error(f'{name}: could not reach the start ({result}), skipped')
            runs.append({'tag': args.tag, 'pair': name, 'success': False, 'note': 'start'})
            continue
        time.sleep(1.0)

        nav.info(f'{name}: measured run to ({gx}, {gy})')
        rec.start()
        result, wall_s = navigate(nav, make_pose(nav, gx, gy, yaw), args.timeout)
        rec.stop()

        run = {'tag': args.tag, 'pair': name, 'planner': args.planner,
               'success': result == TaskResult.SUCCEEDED, 'n_plans': len(rec.plans)}
        if not rec.plans or len(rec.gt) < 2 or len(rec.est) < 2:
            nav.error(f'{name}: no plan, ground truth or estimated pose recorded')
            runs.append(run)
            continue
        planned = np.array(rec.plans[0])
        gt = np.array(rec.gt)
        est = np.array(rec.est)
        dev_est = deviation(est[:, 1:], planned)
        dev_true = deviation(gt[:, 1:], planned)
        # localization error: estimated vs true pose at the time stamps of the estimate
        gt_at_est = np.column_stack([np.interp(est[:, 0], gt[:, 0], gt[:, i]) for i in (1, 2)])
        loc_err = np.linalg.norm(est[:, 1:] - gt_at_est, axis=1)
        run.update({
            'planned_length_m': path_length(planned),
            'driven_length_m': path_length(gt[:, 1:]),
            'duration_sim_s': float(gt[-1, 0] - gt[0, 0]),
            'duration_wall_s': wall_s,
            'deviation_est_mean_m': float(dev_est.mean()),
            'deviation_est_max_m': float(dev_est.max()),
            'deviation_true_mean_m': float(dev_true.mean()),
            'deviation_true_max_m': float(dev_true.max()),
            'localization_error_mean_m': float(loc_err.mean()),
            'localization_error_max_m': float(loc_err.max()),
            'final_error_m': float(np.hypot(gt[-1, 1] - gx, gt[-1, 2] - gy)),
            'planned': planned.round(4).tolist(),
            'ground_truth': gt.round(4).tolist(),
            'estimated': est.round(4).tolist(),
        })
        nav.info(f'{name}: {"ok" if run["success"] else "FAILED"}, planned {run["planned_length_m"]:.2f} m, '
                 f'driven {run["driven_length_m"]:.2f} m in {run["duration_sim_s"]:.1f} s, '
                 f'deviation est {run["deviation_est_mean_m"]:.3f} / true '
                 f'{run["deviation_true_mean_m"]:.3f} m, localization error '
                 f'{run["localization_error_mean_m"]:.3f} m, plans {run["n_plans"]}')
        runs.append(run)

    path = os.path.join(args.out, f'drives_{args.tag}.json')
    with open(path, 'w') as f:
        json.dump({'pairs': pairs, 'runs': runs}, f)
    nav.info(f'saved {path}')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

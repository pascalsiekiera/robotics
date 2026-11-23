#!/usr/bin/env python3
"""Task 1a, part 2: drive the start/goal pairs in Gazebo and measure the path following.

For every pair: drive to the start (not evaluated), then send the goal and record
  - the planned path (/plan, published by planner_server; with the plan-once behavior
    tree this is the path the controller follows; replans are counted),
  - the estimated path: tf map -> base_footprint (AMCL + odometry) at 10 Hz,
  - the true path from the Gazebo ground truth (/ground_truth/odom under Humble,
    /ground_truth/pose under Jazzy, see sim.launch.py).
Metrics: deviation from the planned path
  - of the estimated path: what the controller sees and corrects (controller error),
  - of the true path: controller error + localization error.

Localization check at the start: after the drive to the start, the AMCL pose is compared with
the ground truth. If they differ by more than --max-start-error (or the start was not reached),
the attempt is rejected: it is stored under "rejected", AMCL is set to the ground-truth pose and
the robot drives to the start again (at most --attempts times per pair). The measured run itself
is never corrected; it always starts after a normal drive to the start. Reason: in the Jazzy runs
the robot got stuck at a pillar on the way to the start of P2, AMCL ended up 1 m off and the
measured run started at the wrong place.

The active controller (plugin of controller_server/FollowPath, set by sim.launch.py
controller:=dwb|teb) is read from controller_server and stored with every run.

Requires: ros2 launch mp2/launch/sim.launch.py [controller:=teb]

Example:
    python3 drive_compare.py --planner GridBased --tag dijkstra_dwb
    python3 drive_compare.py --planner GridBased --tag dijkstra_teb   # with controller:=teb
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
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from metrics import deviation, path_length

PLAN_STAMP_MARGIN = 0.5  # [s], see Recorder.on_plan

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
        self.gt_now = None
        self.reset()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, nav)
        nav.create_subscription(Path, 'plan', self.on_plan, 10)
        # ground truth: Humble (Gazebo Classic, p3d plugin) Odometry, Jazzy (Gazebo Harmonic,
        # PosePublisher) PoseStamped
        if os.environ.get('ROS_DISTRO') == 'humble':
            nav.create_subscription(Odometry, 'ground_truth/odom', self.on_gt, 50)
        else:
            nav.create_subscription(PoseStamped, 'ground_truth/pose', self.on_gt, 50)
        nav.create_timer(0.1, self.on_timer)
        # driven paths of the current run for RViz (config/mp2.rviz); transient local, so the
        # last run stays visible
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_est = nav.create_publisher(Path, 'mp2/driven_estimated', latched)
        self.pub_true = nav.create_publisher(Path, 'mp2/driven_true', latched)

    def start(self):
        self.reset()
        self.t0 = self.nav.get_clock().now().nanoseconds * 1e-9

    def stop(self):
        self.t0 = math.inf

    def reset(self):
        self.plans, self.gt, self.est, self.plan_msgs = [], [], [], []

    def on_plan(self, msg):
        if not math.isinf(self.t0):  # diagnostics: every plan received during a run
            self.plan_msgs.append([round(stamp(msg) - self.t0, 3), len(msg.poses)])
        # PLAN_STAMP_MARGIN: Gazebo Classic publishes /clock at 10 Hz, so the planner's stamp can
        # be one clock step (0.1 s) older than t0 of this node; the plan of the drive to the
        # start is several seconds older
        if stamp(msg) >= self.t0 - PLAN_STAMP_MARGIN and msg.poses:
            self.plans.append([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def on_gt(self, msg):
        pose = msg.pose.pose if isinstance(msg, Odometry) else msg.pose
        self.gt_now = pose  # latest true pose, also outside of measured runs
        if stamp(msg) >= self.t0:
            self.gt.append([stamp(msg), pose.position.x, pose.position.y])

    def estimated_xy(self):
        """Latest AMCL + odometry pose (tf map -> base_footprint), or None."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

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
        self.publish(self.pub_est, self.est)
        self.publish(self.pub_true, self.gt)

    def publish(self, pub, samples):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.nav.get_clock().now().to_msg()
        for _, x, y in samples:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        pub.publish(msg)


def active_controller(nav):
    """Plugin of controller_server/FollowPath, e.g. dwb_core::DWBLocalPlanner."""
    client = nav.create_client(GetParameters, 'controller_server/get_parameters')
    if not client.wait_for_service(timeout_sec=10.0):
        return 'unknown'
    future = client.call_async(GetParameters.Request(names=['FollowPath.plugin']))
    rclpy.spin_until_future_complete(nav, future, timeout_sec=10.0)
    values = future.result().values if future.result() else []
    return values[0].string_value if values else 'unknown'


def spin_for(nav, seconds):
    """Wait while processing callbacks (time.sleep would not update tf and ground truth)."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(nav, timeout_sec=0.05)


def start_localization_error(rec):
    """Distance between AMCL pose and ground truth while standing at the start, or None."""
    est = rec.estimated_xy()
    if est is None or rec.gt_now is None:
        return None
    return float(math.hypot(est[0] - rec.gt_now.position.x, est[1] - rec.gt_now.position.y))


def reset_amcl_to_ground_truth(nav, rec):
    """Set the AMCL pose (topic initialpose) to the true pose, for the next attempt only."""
    q = rec.gt_now.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    nav.setInitialPose(make_pose(nav, rec.gt_now.position.x, rec.gt_now.position.y, yaw))
    spin_for(nav, 2.0)


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
    parser.add_argument('--max-start-error', type=float, default=0.10,
                        help='max. AMCL error at the start [m], else the attempt is rejected')
    parser.add_argument('--attempts', type=int, default=3, help='max. attempts per pair')
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
    controller = active_controller(nav)
    nav.info(f'controller: {controller}')
    rec = Recorder(nav)

    runs, rejected = [], []
    for name, pair in pairs.items():
        (sx, sy), (gx, gy) = pair['start'], pair['goal']
        yaw = math.atan2(gy - sy, gx - sx)

        # drive to the start until the robot is there and AMCL agrees with the ground truth
        for attempt in range(1, args.attempts + 1):
            nav.info(f'{name}: driving to start ({sx}, {sy}), attempt {attempt}')
            result, approach_s = navigate(nav, make_pose(nav, sx, sy, yaw), args.timeout)
            spin_for(nav, 1.0)
            start_err = start_localization_error(rec)
            if result == TaskResult.SUCCEEDED and start_err is not None \
                    and start_err <= args.max_start_error:
                break
            reason = ('start not reached' if result != TaskResult.SUCCEEDED
                      else 'no pose' if start_err is None else 'localization at start')
            nav.warn(f'{name}: attempt {attempt} rejected ({reason}, AMCL error '
                     f'{start_err if start_err is not None else float("nan"):.3f} m)')
            rejected.append({'tag': args.tag, 'pair': name, 'attempt': attempt,
                             'controller': controller, 'reason': reason,
                             'start_localization_error_m': start_err,
                             'approach_wall_s': approach_s})
            if rec.gt_now is not None:
                reset_amcl_to_ground_truth(nav, rec)
        else:
            nav.error(f'{name}: no valid start after {args.attempts} attempts, skipped')
            runs.append({'tag': args.tag, 'pair': name, 'controller': controller,
                         'success': False, 'note': 'no valid start'})
            continue

        nav.info(f'{name}: measured run to ({gx}, {gy}), AMCL error at start {start_err:.3f} m')
        rec.start()
        result, wall_s = navigate(nav, make_pose(nav, gx, gy, yaw), args.timeout)
        rec.stop()

        run = {'tag': args.tag, 'pair': name, 'planner': args.planner,
               'controller': controller,
               'success': result == TaskResult.SUCCEEDED, 'n_plans': len(rec.plans),
               'attempt': attempt, 'start_localization_error_m': start_err,
               # diagnostics: [stamp - t0 (s), poses] of every /plan message during the run
               'plan_msgs': rec.plan_msgs}
        if not rec.plans or len(rec.gt) < 2 or len(rec.est) < 2:
            nav.error(f'{name}: no plan, ground truth or estimated pose recorded (plans '
                      f'{len(rec.plans)}, /plan messages {rec.plan_msgs}, ground truth '
                      f'{len(rec.gt)}, estimated {len(rec.est)})')
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
        json.dump({'pairs': pairs, 'runs': runs, 'rejected': rejected,
                   'max_start_error_m': args.max_start_error}, f)
    nav.info(f'saved {path} ({len(runs)} runs, {len(rejected)} rejected attempts)')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

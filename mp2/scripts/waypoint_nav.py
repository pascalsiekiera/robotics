#!/usr/bin/env python3
"""Task 2a: visit the waypoints one after another with the Nav2 action NavigateToPose.

For every waypoint one NavigateToPose goal is sent; the program waits for the result before the
next one is sent. The yaw of a waypoint is the direction to the next waypoint (for the last one:
to the first), so the robot does not turn on the spot at the goal and starts the next leg forward.
A waypoint that fails (aborted, cancelled or timeout) is tried once more (--attempts), then
skipped.

Global planner: by default the one of the launch file (behavior tree, planner:=dijkstra|rrt).
With --planners one round is driven per entry, e.g. --planners dijkstra rrt: round 1 with
Dijkstra, round 2 with RRT, without restarting Nav2 (AMCL keeps its localization, so both rounds
start with the same localization). The planner is chosen per goal: the NavigateToPose goal
carries the behavior tree file (field behavior_tree), for RRT a copy of the default tree with
planner_id="RRT". Every RRT plan publishes rrt/stats; the number of RRT plans per attempt is
logged as a check that the RRT was really used. --tag only names the output file.

Log (JSON in --out): for every attempt waypoint, round, result, start and end time (ROS time, to
cut the rosbag into legs afterwards), duration, number of recoveries and the last pose from the
action feedback (AMCL estimate).

Start: place the robot, set its pose in RViz ("2D Pose Estimate", laser points on the map walls).
The program waits until Nav2 is active (velocity_smoother, the last node the lifecycle manager
activates; it is activated only after the first AMCL pose, before that goals are rejected) and
asks for Enter. --initial-pose x y yaw sets the AMCL pose instead (simulation). Not by default:
in the lab the AMCL pose was set to waypoint 4 while the robot stood 1 m away, facing another
direction -> laser 0.6 m off the map walls, nonsense plans and driving.

For RViz (config/mp2.rviz, green) the AMCL poses since the start are published as a path on
mp2/driven_estimated.

Example:
    python3 waypoint_nav.py --tag dijkstra_dwb --rounds 2
    python3 waypoint_nav.py --tag rrt_dwb --rounds 2 --no-prompt
    python3 waypoint_nav.py --tag dijkstra_rrt --planners dijkstra rrt
"""
import argparse
import json
import math
import os
import tempfile
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions

HERE = os.path.dirname(os.path.abspath(__file__))
MP2 = os.path.dirname(HERE)
NAV2_PARAMS = os.path.join(MP2, 'config', 'nav2_params.yaml')
STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                GoalStatus.STATUS_CANCELED: 'canceled'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def behavior_trees():
    """Behavior tree file per planner: the default plan-once tree of nav2_params.yaml (planner_id
    "GridBased" = NavFn/Dijkstra) and a copy with planner_id "RRT"."""
    with open(NAV2_PARAMS) as f:
        default = yaml.safe_load(f)['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']
    with open(default) as f:
        bt = f.read()
    assert 'planner_id="GridBased"' in bt, 'planner_id not found in ' + default
    tmp = tempfile.NamedTemporaryFile('w', suffix='_bt_rrt.xml', delete=False)
    tmp.write(bt.replace('planner_id="GridBased"', 'planner_id="RRT"'))
    tmp.close()
    return {'dijkstra': default, 'rrt': tmp.name}


def with_yaw(waypoints):
    """Yaw of every waypoint = direction to the next one (the last one looks at the first)."""
    out = []
    for i, wp in enumerate(waypoints):
        nxt = waypoints[(i + 1) % len(waypoints)]
        out.append(dict(wp, yaw=math.atan2(nxt['y'] - wp['y'], nxt['x'] - wp['x'])))
    return out


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_nav')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub_initial = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.feedback = None
        self.goal_handle = None  # goal in progress, cancelled on Ctrl+C
        # driven path (AMCL) for RViz; transient local, so it stays visible after the end
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_path = self.create_publisher(Path, 'mp2/driven_estimated', latched)
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, 10)
        self.rrt_plans = 0  # rrt/stats messages (one per RRT plan) during the current goal
        self.create_subscription(Float64MultiArray, 'rrt/stats', self.on_rrt_stats, 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose(self, x, y, yaw, frame_id='map'):
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        return msg

    def set_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        stamped = self.pose(x, y, yaw)
        msg.header, msg.pose.pose = stamped.header, stamped.pose
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25 ** 2   # x, y [m^2]
        msg.pose.covariance[35] = math.radians(15) ** 2               # yaw [rad^2]
        for _ in range(3):  # AMCL may not have subscribed yet
            self.pub_initial.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.5)

    def on_rrt_stats(self, msg):
        self.rrt_plans += 1

    def on_amcl(self, msg):
        pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
        self.path.poses.append(pose)
        self.path.header.stamp = msg.header.stamp
        self.pub_path.publish(self.path)

    def wait_until_active(self, node_name='velocity_smoother', timeout_s=120.0):
        """Wait until the Nav2 lifecycle node is active. Returns True if it is."""
        client = self.create_client(GetState, f'{node_name}/get_state')
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                    return True
            rclpy.spin_once(self, timeout_sec=1.0)
        return False

    def on_feedback(self, msg):
        self.feedback = msg.feedback

    def navigate(self, goal_pose, timeout_s, behavior_tree=''):
        """Send one NavigateToPose goal and wait for the result. Returns (result name, log dict).
        behavior_tree: tree file for this goal ('' = default tree of bt_navigator)."""
        self.feedback = None
        self.rrt_plans = 0
        t_start = self.now()
        goal = NavigateToPose.Goal(pose=goal_pose, behavior_tree=behavior_tree)
        send = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = self.goal_handle = send.result()
        if handle is None or not handle.accepted:
            return 'rejected', {'t_start': t_start, 't_end': self.now()}
        result = handle.get_result_async()
        t0 = time.monotonic()
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > timeout_s:
                self.get_logger().warn('timeout, goal cancelled')
                rclpy.spin_until_future_complete(self, handle.cancel_goal_async())
                rclpy.spin_until_future_complete(self, result, timeout_sec=10.0)
                name = 'timeout'
                break
        else:
            name = STATUS_NAMES.get(result.result().status, str(result.result().status))
        self.goal_handle = None
        log = {'t_start': t_start, 't_end': self.now(), 'duration_s': time.monotonic() - t0,
               'rrt_plans': self.rrt_plans}
        if self.feedback is not None:
            p = self.feedback.current_pose.pose
            log.update(recoveries=int(self.feedback.number_of_recoveries),
                       distance_remaining_m=float(self.feedback.distance_remaining),
                       final_pose=[p.position.x, p.position.y, yaw_of(p.orientation)])
        return name, log


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--waypoints', default=os.path.join(MP2, 'config', 'waypoints_lsdc3.yaml'))
    parser.add_argument('--tag', required=True, help='e.g. dijkstra_dwb or rrt_dwb')
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--planners', nargs='+', choices=['dijkstra', 'rrt'],
                        help='one round per entry with this planner (replaces --rounds)')
    parser.add_argument('--attempts', type=int, default=2, help='attempts per waypoint')
    parser.add_argument('--timeout', type=float, default=180.0, help='per attempt [s]')
    parser.add_argument('--initial-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW'),
                        help='set the AMCL pose (simulation); otherwise set it in RViz')
    parser.add_argument('--no-prompt', action='store_true', help='do not wait for Enter')
    parser.add_argument('--out', default=os.path.join(MP2, 'data', 'task2a'))
    args = parser.parse_args()

    with open(args.waypoints) as f:
        config = yaml.safe_load(f)
    waypoints = with_yaw(config['waypoints'])
    # planner per round (None = default tree of the launch file)
    planners = args.planners or [None] * args.rounds
    trees = behavior_trees() if args.planners else {}

    # without the rclpy signal handler: Ctrl+C raises KeyboardInterrupt and the context stays
    # usable, so the goal in progress can still be cancelled (otherwise the robot drives on)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WaypointNavigator()
    log = node.get_logger()
    log.info('waiting for the action server navigate_to_pose ...')
    node.client.wait_for_server()
    if args.initial_pose:
        node.set_initial_pose(*args.initial_pose)
        log.info(f'initial pose set to {args.initial_pose}')
    log.info('waiting until Nav2 is active (set the pose in RViz: "2D Pose Estimate") ...')
    if not node.wait_until_active(timeout_s=300.0):
        log.error('Nav2 not active after 300 s (no TF from the robot? pose set in RViz?) - stop')
        node.destroy_node()
        rclpy.try_shutdown()
        return
    log.info('Nav2 is active')
    if not args.no_prompt:
        input('Laser points on the map walls in RViz (otherwise "2D Pose Estimate")? '
              'Press Enter to start. ')
    node.path.poses.clear()  # driven path from the start of the first leg

    os.makedirs(args.out, exist_ok=True)
    out = os.path.join(args.out, f'waypoints_{args.tag}_{time.strftime("%Y%m%d_%H%M%S")}.json')
    data = {'tag': args.tag, 'waypoints_file': args.waypoints, 'map': config.get('map'),
            'waypoints': waypoints, 'planners': planners, 'behavior_trees': trees, 'attempts': []}
    try:
        for rnd, planner in enumerate(planners, start=1):
            for wp in waypoints:
                for attempt in range(1, args.attempts + 1):
                    log.info(f'round {rnd}' + (f' ({planner})' if planner else '')
                             + f', waypoint {wp["name"]}, attempt {attempt}')
                    goal = node.pose(wp['x'], wp['y'], wp['yaw'], config.get('frame_id', 'map'))
                    result, entry = node.navigate(goal, args.timeout, trees.get(planner, ''))
                    entry.update(round=rnd, planner=planner, waypoint=wp['name'], attempt=attempt,
                                 result=result)
                    data['attempts'].append(entry)
                    log.info(f'  -> {result} after {entry.get("duration_s", 0):.1f} s'
                             f', RRT plans {entry["rrt_plans"]}' if 'rrt_plans' in entry else
                             f'  -> {result}')
                    if planner == 'rrt' and result != 'rejected' and entry['rrt_plans'] == 0:
                        log.warn('RRT round, but no RRT plan seen (rrt/stats)')
                    with open(out, 'w') as f:  # after every attempt, nothing is lost on Ctrl+C
                        json.dump(data, f, indent=1)
                    if result == 'succeeded':
                        break
    except KeyboardInterrupt:
        log.warn('interrupted')
        if node.goal_handle is not None:
            rclpy.spin_until_future_complete(node, node.goal_handle.cancel_goal_async(), timeout_sec=5.0)
            log.warn('goal cancelled')
    finally:
        print(f'saved {out}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

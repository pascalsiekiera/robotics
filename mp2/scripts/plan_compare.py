#!/usr/bin/env python3
"""Task 1a, part 1: compare global planners without driving.

For every start/goal pair and every planner, request a path N times via the
ComputePathToPose action (use_start=True, so the robot does not have to be at the
start and does not move). Stores the paths and the metrics path length, planning
time and minimum clearance.

Requires the planning-only setup (no Gazebo, fixed costmap, wall-clock time):
    ros2 launch mp2/launch/planning_only.launch.py

Example:
    python3 plan_compare.py --planners GridBased AStar --reps 10 --tag baseline
"""
import argparse
import csv
import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

from metrics import StaticMap, min_clearance, path_length

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


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--planners', nargs='+', default=['GridBased', 'AStar'],
                        help='planner ids as in planner_server/planner_plugins')
    parser.add_argument('--reps', type=int, default=10, help='repetitions per pair and planner')
    parser.add_argument('--pairs', default=os.path.join(MP2, 'config', 'test_pairs.yaml'))
    parser.add_argument('--tag', default='baseline', help='name of this configuration')
    parser.add_argument('--out', default=os.path.join(MP2, 'data', 'task1a'))
    args = parser.parse_args()

    with open(args.pairs) as f:
        pairs = yaml.safe_load(f)
    static_map = StaticMap()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    nav = BasicNavigator()
    # Only the planner is needed. Not waitUntilNav2Active(): it publishes an initial pose
    # at (0, 0) if no amcl_pose arrives and would reset the localization.
    nav._waitForNodeToActivate('planner_server')

    runs = []
    for pair_name, pair in pairs.items():
        (sx, sy), (gx, gy) = pair['start'], pair['goal']
        yaw = math.atan2(gy - sy, gx - sx)
        start, goal = make_pose(nav, sx, sy, yaw), make_pose(nav, gx, gy, yaw)
        for planner in args.planners:
            for rep in range(args.reps):
                t0 = time.perf_counter()
                result = nav._getPathImpl(start, goal, planner_id=planner, use_start=True)
                wall_ms = (time.perf_counter() - t0) * 1e3
                ok = (nav.status == GoalStatus.STATUS_SUCCEEDED and result is not None
                      and len(result.path.poses) > 1)
                run = {'tag': args.tag, 'pair': pair_name, 'planner': planner, 'rep': rep,
                       'success': ok}
                if ok:
                    xy = np.array([[p.pose.position.x, p.pose.position.y]
                                   for p in result.path.poses])
                    t = result.planning_time
                    run.update({
                        # reported by planner_server (wall clock, use_sim_time: false)
                        'planning_time_ms': (t.sec + t.nanosec * 1e-9) * 1e3,
                        # whole action call incl. communication, for comparison only
                        'roundtrip_ms': wall_ms,
                        'length_m': path_length(xy),
                        'min_clearance_m': min_clearance(xy, static_map),
                        'n_poses': len(xy),
                        'path': xy.round(4).tolist(),
                    })
                    nav.info(f'{pair_name} {planner} #{rep}: '
                             f'{run["length_m"]:.2f} m, {run["planning_time_ms"]:.2f} ms, '
                             f'clearance {run["min_clearance_m"]:.2f} m')
                else:
                    nav.warn(f'{pair_name} {planner} #{rep}: no path '
                             f'(error {getattr(result, "error_code", "?")})')
                runs.append(run)

    json_path = os.path.join(args.out, f'plans_{args.tag}.json')
    with open(json_path, 'w') as f:
        json.dump({'pairs': pairs, 'runs': runs}, f)

    csv_path = os.path.join(args.out, f'plans_{args.tag}.csv')
    fields = ['tag', 'pair', 'planner', 'rep', 'success', 'planning_time_ms',
              'roundtrip_ms', 'length_m', 'min_clearance_m', 'n_poses']
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(runs)

    nav.info(f'saved {json_path} and {csv_path}')
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

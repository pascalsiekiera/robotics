"""Task 2a: Nav2 on the laptop for the real TurtleBot3 Waffle Pi in the LSDC3 lab (Humble).

The robot runs only its drivers (ssh, ros2 launch turtlebot3_bringup robot.launch.py): /scan,
/odom, /tf, /tf_static, /cmd_vel. This file starts on the laptop, with the real clock
(use_sim_time false):
    map_server   map from Mini-Project 1 (mp1/maps/map_2a_merged.yaml)
    amcl         parameters of nav2_params.yaml with the values tuned in Mini-Project 1
                 (LDS-01 range 0.12-3.5 m, update every 0.1 m / 0.1 rad); no initial pose:
                 AMCL waits for "2D Pose Estimate" in RViz
    Nav2         planner, controller (DWB), behavior tree etc. as in Task 1 (nav2_params.yaml),
                 except the goal checker: xy_goal_tolerance 0.10 m instead of 0.25 m (the robot
                 stopped 25 cm before the waypoint in the first lab test), and all
                 transform tolerances 1.0 s (AMCL, bt_navigator, DWB, both costmaps, behaviors):
                 /scan, /odom and /tf come over Wi-Fi; in two lab runs they arrived late and with
                 gaps (/odom at 5-9 Hz instead of 20 Hz), DWB aborted with "Transform data too
                 old when converting from odom to map" (map -> odom 0.5-0.85 s old with 0.5 s
                 tolerance) and Nav2 started its recovery behaviors
    rviz2        config/mp2.rviz
Global planner used by the behavior tree:
    planner:=dijkstra  "GridBased" (NavFn, default)
    planner:=rrt       "RRT" (mp2/rrt_planner)
The RRT always runs with rrt.allow_unknown true (the map has unknown speckles inside the room,
NavFn plans through them as well), also with planner:=dijkstra: waypoint_nav.py --planners can
switch to RRT per goal (behavior tree in the NavigateToPose goal) without restarting Nav2.

Run (laptop, ROS_DOMAIN_ID = robot id):
    ros2 launch mp2/launch/lab.launch.py
    ros2 launch mp2/launch/lab.launch.py planner:=rrt
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MP2 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP = os.path.join(os.path.dirname(MP2), 'mp1', 'maps', 'map_2a_merged.yaml')
NAV2_BRINGUP = get_package_share_directory('nav2_bringup')

# AMCL values tuned in Mini-Project 1 (mp1/src/ekf/config/amcl.yaml), rest as in nav2_params.yaml
AMCL_MP1 = {'laser_min_range': 0.12, 'laser_max_range': 3.5,
            'update_min_d': 0.1, 'update_min_a': 0.1,
            'set_initial_pose': False}


def nav2(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    planner = LaunchConfiguration('planner').perform(context)
    allow_unknown = LaunchConfiguration('rrt_allow_unknown').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    xy_tolerance = float(LaunchConfiguration('xy_goal_tolerance').perform(context))
    tf_tolerance = float(LaunchConfiguration('transform_tolerance').perform(context))
    if planner not in ('dijkstra', 'rrt'):
        raise RuntimeError(f'planner must be dijkstra or rrt, not {planner!r}')
    with open(params_file) as f:
        params = yaml.safe_load(f)
    params['amcl']['ros__parameters'].update(AMCL_MP1)
    params['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance'] = xy_tolerance
    for node in ('amcl', 'bt_navigator', 'behavior_server'):
        params[node]['ros__parameters']['transform_tolerance'] = tf_tolerance
    params['controller_server']['ros__parameters']['FollowPath']['transform_tolerance'] = tf_tolerance
    for costmap in ('local_costmap', 'global_costmap'):
        params[costmap][costmap]['ros__parameters']['transform_tolerance'] = tf_tolerance
    if planner == 'rrt':
        # the plan-once behavior tree has planner_id="GridBased" hard-coded -> copy with "RRT"
        bt_params = params['bt_navigator']['ros__parameters']
        with open(bt_params['default_nav_to_pose_bt_xml']) as f:
            bt = f.read()
        assert 'planner_id="GridBased"' in bt, 'planner_id not found in the behavior tree'
        bt_tmp = tempfile.NamedTemporaryFile('w', suffix='_bt_rrt.xml', delete=False)
        bt_tmp.write(bt.replace('planner_id="GridBased"', 'planner_id="RRT"'))
        bt_tmp.close()
        bt_params['default_nav_to_pose_bt_xml'] = bt_tmp.name
    params['planner_server']['ros__parameters']['RRT']['rrt.allow_unknown'] = allow_unknown == 'true'
    tmp = tempfile.NamedTemporaryFile('w', suffix=f'_nav2_lab_{planner}.yaml', delete=False)
    yaml.safe_dump(params, tmp)
    tmp.close()
    print(f'[lab] planner {planner}'
          + f' (RRT: allow unknown {allow_unknown})'
          + f', xy goal tolerance {xy_tolerance} m, transform tolerance {tf_tolerance} s'
          + f', map {map_file}, params {tmp.name}')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(NAV2_BRINGUP, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_file, 'use_sim_time': 'false',
                          'params_file': tmp.name}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='dijkstra', description='dijkstra or rrt'),
        DeclareLaunchArgument('rrt_allow_unknown', default_value='true',
                              description='true/false (RRT plugin)'),
        DeclareLaunchArgument('map', default_value=MAP),
        DeclareLaunchArgument('xy_goal_tolerance', default_value='0.10',
                              description='goal checker [m] (Task 1: 0.25)'),
        DeclareLaunchArgument('transform_tolerance', default_value='1.0',
                              description='[s] all Nav2 nodes (Task 1: 0.5 / costmap default 0.3)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(MP2, 'config', 'nav2_params.yaml')),
        OpaqueFunction(function=nav2),
        # software rendering: with the GPU (WSLg) the map display fails ("GLSL link result:
        # active samplers with a different type refer to the same texture image unit")
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(MP2, 'config', 'mp2.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')),
             additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}),
    ])

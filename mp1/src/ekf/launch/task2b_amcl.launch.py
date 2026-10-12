"""
Mini-Project 1, task 2b: Monte Carlo Localization (AMCL) on bags recorded with the real robot.

Starts
    map_server          serves the map from task 2a on /map
    amcl                particle filter, publishes map -> odom, /amcl_pose, /particle_cloud
    lifecycle manager   configures + activates map_server and amcl
    rviz2               map, laser, particles, AMCL pose with covariance, "2D Pose Estimate" tool
    ros2 bag play       the recorded run (play_bag:=false to start it yourself)
    ros2 bag record     /amcl_pose (pose + covariance), /odom, /tf, /scan, for the evaluation
                        -> ~/ros2_ws/results/bags/<run_name>_<time>

The bags contain everything the robot published (/scan, /odom, /tf, /tf_static), so no
robot_state_publisher is needed. All nodes use the bag's clock (use_sim_time).

Initial pose
    Known start pose (e.g. 2a_scan_1: the map was built from this run, so the robot
    started at the map origin): pass it, AMCL starts there.
        ros2 launch ekf task2b_amcl.launch.py bag:=.../2a_scan_1 x:=0.0 y:=0.0 yaw:=0.0

    Unknown start pose: let AMCL wait, start the bag paused in a second terminal, click
    "2D Pose Estimate" in RViz on the robot's start position + direction, then press SPACE
    in the bag terminal.
        ros2 launch ekf task2b_amcl.launch.py set_initial_pose:=false play_bag:=false
        ros2 bag play .../2a_raw_1 --clock --start-paused
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('ekf')
    mp1 = os.path.expanduser('~/ros2_ws/src/robotics/mp1')
    sim_time = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument('bag', default_value=os.path.join(mp1, 'data', '2a_scan_1'),
                              description='bag recorded with the real robot'),
        DeclareLaunchArgument('map', default_value=os.path.join(mp1, 'maps', 'map_2a_new.yaml'),
                              description='map from task 2a'),
        DeclareLaunchArgument('amcl_params',
                              default_value=os.path.join(pkg_share, 'config', 'amcl.yaml'),
                              description='AMCL parameter file (copy it to try other parameters)'),
        DeclareLaunchArgument('run_name', default_value='task2b',
                              description='name of the recorded result bag'),
        DeclareLaunchArgument('set_initial_pose', default_value='true',
                              description='false: wait for "2D Pose Estimate" in RViz'),
        DeclareLaunchArgument('x', default_value='0.0', description='initial pose x [m] in map'),
        DeclareLaunchArgument('y', default_value='0.0', description='initial pose y [m] in map'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='initial heading [rad]'),
        DeclareLaunchArgument('play_bag', default_value='true',
                              description='false: play the bag yourself (e.g. --start-paused)'),
        DeclareLaunchArgument('rate', default_value='1.0', description='bag playback speed'),
        DeclareLaunchArgument('record', default_value='true',
                              description='record the AMCL result for the evaluation'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': True}],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        parameters=[LaunchConfiguration('amcl_params'), sim_time, {
            'set_initial_pose': ParameterValue(LaunchConfiguration('set_initial_pose'),
                                               value_type=bool),
            'initial_pose.x': ParameterValue(LaunchConfiguration('x'), value_type=float),
            'initial_pose.y': ParameterValue(LaunchConfiguration('y'), value_type=float),
            'initial_pose.yaw': ParameterValue(LaunchConfiguration('yaw'), value_type=float),
        }],
        output='screen',
    )
    # map_server and amcl are lifecycle nodes: configure + activate them
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['map_server', 'amcl']}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'task1c.rviz')],
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # wait a few seconds so that map_server and amcl are active before the data arrives
    bag_play = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--clock',
             '--rate', LaunchConfiguration('rate'),
             '--qos-profile-overrides-path',
             os.path.join(pkg_share, 'config', 'bag_qos_overrides.yaml')],
        output='screen',
    )], condition=IfCondition(LaunchConfiguration('play_bag')))

    # setsid: own process group, so Ctrl+C reaches the recorder only once (via launch).
    # Getting SIGINT twice (terminal + launch) makes ros2 bag record hang on shutdown.
    record = ExecuteProcess(
        cmd=['setsid', 'ros2', 'bag', 'record',
             '-o', [os.path.expanduser('~/ros2_ws/results/bags/'),
                    LaunchConfiguration('run_name'), time.strftime('_%Y%m%d_%H%M%S')],
             '--topics', '/amcl_pose', '/odom', '/tf', '/tf_static', '/scan', '/map'],
        output='log',
        condition=IfCondition(LaunchConfiguration('record')),
    )

    return LaunchDescription(args + [
        map_server,
        amcl,
        lifecycle_manager,
        rviz,
        bag_play,
        record,
    ])

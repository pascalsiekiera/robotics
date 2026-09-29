"""
Mini-Project 1, task 1a: EKF localisation on the turtlebot3 dataset.

Starts
    robot_state_publisher   URDF + static TFs of the robot (RobotModel in RViz)
    static TF mocap -> map  initial ground-truth pose (same values as
                            turtlebot_datasets/publish_initial_tf)
    map_server              dataset map, for visualisation only
    ground_truth_node       mocap -> base_footprint pose in map, full rate + 1 Hz
    ekf_node                own EKF (odom + imu + 1 Hz ground truth), map -> odom TF
    rl_ekf                  robot_localization EKF with the same inputs (optional)
    evaluator_node          paths + error vs ground truth, CSV in ~/ros2_ws/results
    rviz2                   (optional)
    ros2 bag play --clock   started after a short delay so all nodes are ready

Usage
    ros2 launch ekf task1a.launch.py
    ros2 launch ekf task1a.launch.py rate:=2.0 robot_localization:=false
    ros2 launch ekf task1a.launch.py bag:=/path/to/other_bag rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ekf')
    datasets_share = get_package_share_directory('turtlebot_datasets')
    params = os.path.join(pkg_share, 'config', 'ekf_params.yaml')
    sim_time = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument(
            'bag', default_value=os.path.expanduser(
                '~/ros2_ws/src/turtlebot_datasets/data/fixed_slam_easy'),
            description='rosbag2 directory to play'),
        DeclareLaunchArgument('rate', default_value='1.0', description='bag playback speed'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(datasets_share, 'data', 'map.yaml'),
            description='map yaml (visualisation only)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('robot_localization', default_value='true',
                              description='also run robot_localization for comparison'),
    ]

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'launch', 'turtlebot3_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'namespace': ''}.items(),
    )

    # Initial ground-truth transform (see turtlebot_datasets/publish_initial_tf.py).
    # Attaching mocap to map makes map the frame in which the robot starts at (0, 0, 0).
    mocap_to_map = Node(
        package='tf2_ros', executable='static_transform_publisher', name='mocap_to_map',
        arguments=['--x', '0.935', '--y', '1.340', '--z', '-0.023',
                   '--qx', '0.001', '--qy', '-0.003', '--qz', '0.737', '--qw', '0.676',
                   '--frame-id', 'mocap', '--child-frame-id', 'map'],
        parameters=[sim_time],
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': True}],
    )
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}],
    )

    ground_truth = Node(
        package='ekf', executable='ground_truth_node', name='ground_truth_node',
        parameters=[params, sim_time], output='screen',
    )
    own_ekf = Node(
        package='ekf', executable='ekf_node', name='ekf_node',
        parameters=[params, sim_time], output='screen',
    )
    rl_ekf = Node(
        package='robot_localization', executable='ekf_node', name='rl_ekf',
        parameters=[os.path.join(pkg_share, 'config', 'robot_localization.yaml'), sim_time],
        remappings=[('odometry/filtered', '/rl/odometry/filtered')],
        condition=IfCondition(LaunchConfiguration('robot_localization')),
    )
    evaluator = Node(
        package='ekf', executable='evaluator_node', name='evaluator_node',
        parameters=[sim_time], output='screen',
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'task1a.rviz')],
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    bag_play = TimerAction(period=3.0, actions=[ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
             '--rate', LaunchConfiguration('rate'),
             '--qos-profile-overrides-path',
             os.path.join(pkg_share, 'config', 'bag_qos_overrides.yaml'),
             '--clock'],
        output='screen',
    )])

    return LaunchDescription(args + [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        robot_state_publisher,
        mocap_to_map,
        map_server,
        map_lifecycle,
        ground_truth,
        own_ekf,
        rl_ekf,
        evaluator,
        rviz,
        bag_play,
    ])

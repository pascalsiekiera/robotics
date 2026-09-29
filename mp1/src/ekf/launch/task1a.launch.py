"""
Mini-Project 1, task 1a: EKF localisation on the turtlebot3 dataset.

Includes dataset_common.launch.py (robot model, mocap -> map TF, ground truth,
RViz, bag playback) and adds
    map_server + lifecycle manager   dataset map, for visualisation only
    ekf_node                         own EKF (odom + imu + 1 Hz ground truth), map -> odom TF
    rl_ekf                           robot_localization EKF with the same inputs (optional)
    evaluator_node                   paths + error vs ground truth, CSV in ~/ros2_ws/results

Usage
    ros2 launch ekf task1a.launch.py
    ros2 launch ekf task1a.launch.py rate:=2.0 robot_localization:=false
    ros2 launch ekf task1a.launch.py bag:=/path/to/other_bag rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ekf')
    datasets_share = get_package_share_directory('turtlebot_datasets')
    sim_time = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument(
            'map', default_value=os.path.join(datasets_share, 'data', 'map.yaml'),
            description='map yaml (visualisation only)'),
        DeclareLaunchArgument('robot_localization', default_value='true',
                              description='also run robot_localization for comparison'),
    ]

    # bag, rate and rviz given on the command line are passed through automatically
    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch',
                                                   'dataset_common.launch.py')),
        launch_arguments={'rviz_config': os.path.join(pkg_share, 'config',
                                                      'task1a.rviz')}.items(),
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': True}],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}],
    )

    own_ekf = Node(
        package='ekf', executable='ekf_node', name='ekf_node',
        parameters=[os.path.join(pkg_share, 'config', 'ekf_params.yaml'), sim_time],
        output='screen',
    )
    rl_ekf = Node(
        package='robot_localization', executable='ekf_node', name='rl_ekf',
        parameters=[os.path.join(pkg_share, 'config', 'robot_localization.yaml'), sim_time],
        remappings=[('odometry/filtered', '/rl/odometry/filtered')],
        condition=IfCondition(LaunchConfiguration('robot_localization')),
    )
    evaluator = Node(
        package='ekf', executable='evaluator_node', name='evaluator_node',
        parameters=[sim_time, {'run_name': 'task1a'}], output='screen',
    )

    return LaunchDescription(args + [
        common,
        map_server,
        lifecycle_manager,
        own_ekf,
        rl_ekf,
        evaluator,
    ])

"""
Common part of every dataset task (1a, 1b, 1c).

Not meant to be launched alone: the task launch files include it and add
their own nodes.

Starts
    robot_state_publisher   URDF + static TFs of the robot (RobotModel in RViz)
    static TF mocap -> map  initial ground-truth pose (same values as
                            turtlebot_datasets/publish_initial_tf). This puts
                            the robot's start pose at the origin of map.
    ground_truth_node       mocap -> base_footprint pose in map
    rviz2                   with the config given by the including file
    ros2 bag play --clock   started after start_delay, so all nodes are listening

Arguments (set by the including launch file or on the command line)
    bag, rate, rviz, rviz_config, start_delay
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
    sim_time = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument(
            'bag', default_value=os.path.expanduser(
                '~/ros2_ws/src/turtlebot_datasets/data/fixed_slam_easy'),
            description='rosbag2 directory to play'),
        DeclareLaunchArgument('rate', default_value='1.0', description='bag playback speed'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(pkg_share, 'config', 'task1a.rviz')),
        DeclareLaunchArgument('start_delay', default_value='3.0',
                              description='seconds to wait before playing the bag'),
    ]

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'launch', 'turtlebot3_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'namespace': ''}.items(),
    )

    mocap_to_map = Node(
        package='tf2_ros', executable='static_transform_publisher', name='mocap_to_map',
        arguments=['--x', '0.935', '--y', '1.340', '--z', '-0.023',
                   '--qx', '0.001', '--qy', '-0.003', '--qz', '0.737', '--qw', '0.676',
                   '--frame-id', 'mocap', '--child-frame-id', 'map'],
        parameters=[sim_time],
    )

    ground_truth = Node(
        package='ekf', executable='ground_truth_node', name='ground_truth_node',
        parameters=[os.path.join(pkg_share, 'config', 'ground_truth.yaml'), sim_time],
        output='screen',
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    bag_play = TimerAction(period=LaunchConfiguration('start_delay'), actions=[ExecuteProcess(
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
        ground_truth,
        rviz,
        bag_play,
    ])

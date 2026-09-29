"""
Mini-Project 1, task 1b: build a map from the dataset with slam_toolbox.

(gmapping, named in the assignment, is not released for ROS 2 Jazzy; the course
recommends slam_toolbox instead.)

Includes dataset_common.launch.py (robot model, mocap -> map TF, ground truth,
RViz, bag playback) and adds
    slam_toolbox (sync)          builds /map from /scan + odometry, publishes map -> odom
    reference_map_server         the map provided with the dataset, on /reference_map,
                                 drawn underneath for comparison
    lifecycle manager            configures + activates the two lifecycle nodes above
    evaluator_node               pose error of SLAM and of raw odometry vs ground truth

Usage
    ros2 launch ekf task1b_mapping.launch.py
    # when the bag has finished (the map stays available), in a second terminal:
    ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/robotics/mp1/maps/map_1b \
        --ros-args -p use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ekf')
    datasets_share = get_package_share_directory('turtlebot_datasets')
    sim_time = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument(
            'reference_map', default_value=os.path.join(datasets_share, 'data', 'map.yaml'),
            description='map provided with the dataset, shown for comparison'),
        DeclareLaunchArgument(
            'slam_params', default_value=os.path.join(pkg_share, 'config', 'slam_toolbox.yaml'),
            description='slam_toolbox parameter file (copy it to try other parameters)'),
    ]

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch',
                                                   'dataset_common.launch.py')),
        launch_arguments={'rviz_config': os.path.join(pkg_share, 'config',
                                                      'task1b.rviz')}.items(),
    )

    # sync mode: every scan that passes the travel thresholds is processed, none are
    # dropped when the computer is busy (better for bags than async mode)
    slam = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
        parameters=[LaunchConfiguration('slam_params'), sim_time],
        output='screen',
    )

    reference_map_server = Node(
        package='nav2_map_server', executable='map_server', name='reference_map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('reference_map'),
                     'use_sim_time': True}],
        remappings=[('map', '/reference_map')],
    )

    # slam_toolbox and map_server are lifecycle nodes: they start "unconfigured" and
    # only work after configure + activate, which the lifecycle manager does for us
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['reference_map_server', 'slam_toolbox']}],
    )

    evaluator = Node(
        package='ekf', executable='evaluator_node', name='evaluator_node',
        parameters=[sim_time, {'run_name': 'task1b',
                               'topic_estimators': ['odometry:/odom'],
                               'tf_estimator': 'slam_toolbox'}],
        output='screen',
    )

    return LaunchDescription(args + [
        common,
        slam,
        reference_map_server,
        lifecycle_manager,
        evaluator,
    ])

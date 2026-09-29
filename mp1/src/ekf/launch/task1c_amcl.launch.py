"""
Mini-Project 1, task 1c: Monte Carlo Localization (AMCL) on the dataset.

Uses the map built in task 1b.

Includes dataset_common.launch.py (robot model, mocap -> map TF, ground truth,
RViz, bag playback) and adds
    map_server          serves the map from task 1b on /map
    amcl                particle filter, publishes map -> odom, /amcl_pose, /particle_cloud
    lifecycle manager   configures + activates map_server and amcl
    evaluator_node      pose error of AMCL and of raw odometry vs ground truth

Usage
    ros2 launch ekf task1c_amcl.launch.py
    ros2 launch ekf task1c_amcl.launch.py map:=/path/to/other_map.yaml
    ros2 launch ekf task1c_amcl.launch.py amcl_params:=/path/to/amcl_test.yaml
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
    sim_time = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument(
            'run_name', default_value='task1c',
            description='name of the result CSV in ~/ros2_ws/results, e.g. task1c_test1'),
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser(
                '~/ros2_ws/src/robotics/mp1/maps/map_1b.yaml'),
            description='map built in task 1b'),
        DeclareLaunchArgument(
            'amcl_params', default_value=os.path.join(pkg_share, 'config', 'amcl.yaml'),
            description='AMCL parameter file (copy it to try other parameters)'),
    ]

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch',
                                                   'dataset_common.launch.py')),
        launch_arguments={'rviz_config': os.path.join(pkg_share, 'config',
                                                      'task1c.rviz')}.items(),
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': True}],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        parameters=[LaunchConfiguration('amcl_params'), sim_time],
        output='screen',
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['map_server', 'amcl']}],
    )

    evaluator = Node(
        package='ekf', executable='evaluator_node', name='evaluator_node',
        parameters=[sim_time, {'run_name': LaunchConfiguration('run_name'),
                               'topic_estimators': ['odometry:/odom'],
                               'tf_estimator': 'amcl',
                               'tf_covariance_topic': '/amcl_pose'}],
        output='screen',
    )

    return LaunchDescription(args + [
        common,
        map_server,
        amcl,
        lifecycle_manager,
        evaluator,
    ])

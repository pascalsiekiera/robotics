"""Task 1a part 1 and Task 1b: global planner only, without Gazebo, AMCL and controller.

Starts map_server + planner_server (+ lifecycle manager) and a static transform
map -> base_link. The parameters are taken from mp2/config/nav2_params.yaml (same
planners and costmap settings as in the driving runs) with these changes:
  - global costmap: only static layer + inflation layer (no laser data), so the
    costmap is identical for every request
  - use_sim_time: false, so planner_server reports the planning time in wall-clock time

Run:
    ros2 launch mp2/launch/planning_only.launch.py
    ros2 launch mp2/launch/planning_only.launch.py inflation_radius:=0.3
    ros2 launch mp2/launch/planning_only.launch.py rrt_goal_bias:=0.1
Costmap parameters that can be overridden (empty = value from the params file):
    inflation_radius, cost_scaling_factor, robot_radius
RRT parameters (planner "RRT", Task 1b) that can be overridden:
    rrt_step, rrt_goal_bias, rrt_cost_threshold, rrt_smoothing (true/false),
    rrt_allow_unknown (true/false), rrt_seed
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MP2 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PARAMS = os.path.join(MP2, 'config', 'nav2_params.yaml')
DEFAULT_MAP = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')


def make_nodes(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    map_file = LaunchConfiguration('map').perform(context)

    with open(params_file) as f:
        params = yaml.safe_load(f)
    costmap = params['global_costmap']['global_costmap']['ros__parameters']
    costmap['plugins'] = ['static_layer', 'inflation_layer']
    for node in ('planner_server', 'map_server'):
        params[node]['ros__parameters']['use_sim_time'] = False
    costmap['use_sim_time'] = False
    params['map_server']['ros__parameters']['yaml_filename'] = map_file
    for name, target in (('inflation_radius', costmap['inflation_layer']),
                         ('cost_scaling_factor', costmap['inflation_layer']),
                         ('robot_radius', costmap)):
        value = LaunchConfiguration(name).perform(context)
        if value:
            target[name] = float(value)
    rrt = params['planner_server']['ros__parameters'].get('RRT')
    for name, convert in (('step', float), ('goal_bias', float), ('cost_threshold', int),
                          ('smoothing', lambda v: v.lower() == 'true'),
                          ('allow_unknown', lambda v: v.lower() == 'true'), ('seed', int)):
        value = LaunchConfiguration('rrt_' + name).perform(context)
        if value and rrt is not None:
            rrt['rrt.' + name] = convert(value)
    if rrt is not None:
        print('[planning_only] RRT: ' + ', '.join(f'{k} {v}' for k, v in rrt.items() if k != 'plugin'))
    print('[planning_only] global costmap: robot_radius %s, inflation_radius %s, '
          'cost_scaling_factor %s' % (costmap['robot_radius'],
                                      costmap['inflation_layer']['inflation_radius'],
                                      costmap['inflation_layer']['cost_scaling_factor']))

    tmp = tempfile.NamedTemporaryFile('w', suffix='_planning_only.yaml', delete=False)
    yaml.safe_dump(params, tmp)
    tmp.close()

    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[tmp.name]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[tmp.name]),
        # robot pose is irrelevant for planning with use_start=True, but the costmap needs it
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '-2.0', '--y', '-0.5', '--frame-id', 'map',
                        '--child-frame-id', 'base_link']),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_planning', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['map_server', 'planner_server']}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=DEFAULT_PARAMS),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP),
        DeclareLaunchArgument('inflation_radius', default_value=''),
        DeclareLaunchArgument('cost_scaling_factor', default_value=''),
        DeclareLaunchArgument('robot_radius', default_value=''),
        DeclareLaunchArgument('rrt_step', default_value=''),
        DeclareLaunchArgument('rrt_goal_bias', default_value=''),
        DeclareLaunchArgument('rrt_cost_threshold', default_value=''),
        DeclareLaunchArgument('rrt_smoothing', default_value=''),
        DeclareLaunchArgument('rrt_allow_unknown', default_value=''),
        DeclareLaunchArgument('rrt_seed', default_value=''),
        OpaqueFunction(function=make_nodes),
    ])

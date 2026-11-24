"""Task 1a, part 2: Gazebo Classic (turtlebot3_world) + Nav2 with mp2/config/nav2_params.yaml (Humble).

Same world, robot and Nav2 bringup as
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
but in one launch file, optionally without GUIs, with a selectable controller and with the
Gazebo ground truth published to ROS:
    /ground_truth/odom (nav_msgs/Odometry, 30 Hz): robot pose in the Gazebo world frame, from a
    gazebo_ros_p3d plugin added to the robot model.
The camera sensor is removed from the robot model (not used by Nav2): rendering it in software
overloads the CPU under WSL2 (seen with Gazebo Harmonic under Jazzy).
Controller (controller_server plugin "FollowPath"):
    controller:=dwb  DWB as in nav2_params.yaml (default)
    controller:=teb  TEB from config/teb_controller.yaml
Global planner used by the behavior tree (Task 1b):
    planner:=dijkstra  "GridBased" (NavFn, default)
    planner:=rrt       "RRT" (mp2/rrt_planner); rrt_smoothing:=true for the smoothed variant,
                       rrt_cost_threshold:=128 for the variant with more distance to obstacles,
                       rrt_allow_unknown:=true to plan through unknown cells (as NavFn, Task 2a)
The Jazzy version (Gazebo Harmonic) of this file is mp2/jazzy/sim.launch.py.

Run:
    ros2 launch mp2/launch/sim.launch.py                       # headless, DWB
    ros2 launch mp2/launch/sim.launch.py gui:=true rviz:=true  # with Gazebo window and RViz
    ros2 launch mp2/launch/sim.launch.py controller:=teb
    ros2 launch mp2/launch/sim.launch.py planner:=rrt rrt_smoothing:=true
"""
import os
import re
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# read by the included turtlebot3_gazebo launch files
os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle_pi')

MP2 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TB3_GAZEBO = get_package_share_directory('turtlebot3_gazebo')
TB3_NAV2 = get_package_share_directory('turtlebot3_navigation2')
GAZEBO_ROS = get_package_share_directory('gazebo_ros')
NAV2_BRINGUP = get_package_share_directory('nav2_bringup')

GROUND_TRUTH = """
    <plugin name="ground_truth" filename="libgazebo_ros_p3d.so">
      <ros>
        <remapping>odom:=ground_truth/odom</remapping>
      </ros>
      <body_name>base_footprint</body_name>
      <frame_name>world</frame_name>
      <update_rate>30</update_rate>
    </plugin>
  </model>"""


def robot_model():
    """Waffle Pi model.sdf without camera, with ground-truth plugin (temporary file)."""
    path = os.path.join(TB3_GAZEBO, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    with open(path) as f:
        sdf = f.read()
    sdf, n = re.subn(r'<sensor name="camera".*?</sensor>', '', sdf, flags=re.S)
    assert n == 1, 'camera sensor not found in ' + path
    sdf, n = re.subn(r'\s*</model>', GROUND_TRUTH, sdf, count=1)
    assert n == 1, '</model> not found in ' + path
    tmp = tempfile.NamedTemporaryFile('w', suffix='_waffle_pi_nocam.sdf', delete=False)
    tmp.write(sdf)
    tmp.close()
    return tmp.name


def nav2(context):
    """Nav2 bringup with the params file; FollowPath replaced by TEB if controller:=teb,
    behavior tree with planner_id "RRT" if planner:=rrt."""
    params_file = LaunchConfiguration('params_file').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    planner = LaunchConfiguration('planner').perform(context)
    smoothing = LaunchConfiguration('rrt_smoothing').perform(context)
    threshold = LaunchConfiguration('rrt_cost_threshold').perform(context)
    allow_unknown = LaunchConfiguration('rrt_allow_unknown').perform(context)
    if controller not in ('dwb', 'teb'):
        raise RuntimeError(f'controller must be dwb or teb, not {controller!r}')
    if planner not in ('dijkstra', 'rrt'):
        raise RuntimeError(f'planner must be dijkstra or rrt, not {planner!r}')
    with open(params_file) as f:
        params = yaml.safe_load(f)
    if controller == 'teb':
        with open(os.path.join(MP2, 'config', 'teb_controller.yaml')) as f:
            params['controller_server']['ros__parameters'].update(yaml.safe_load(f))
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
        rrt = params['planner_server']['ros__parameters']['RRT']
        if smoothing:
            rrt['rrt.smoothing'] = smoothing == 'true'
        if threshold:
            rrt['rrt.cost_threshold'] = int(threshold)
        if allow_unknown:
            rrt['rrt.allow_unknown'] = allow_unknown == 'true'
    if controller != 'dwb' or planner != 'dijkstra':
        tmp = tempfile.NamedTemporaryFile('w', suffix=f'_nav2_{controller}_{planner}.yaml', delete=False)
        yaml.safe_dump(params, tmp)
        tmp.close()
        params_file = tmp.name
    print(f'[sim] controller {controller}, planner {planner}'
          + (f' (smoothing {smoothing})' if planner == 'rrt' and smoothing else '')
          + (f' (cost threshold {threshold})' if planner == 'rrt' and threshold else '')
          + (f' (allow unknown {allow_unknown})' if planner == 'rrt' and allow_unknown else '')
          + f', params {params_file}')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(NAV2_BRINGUP, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': os.path.join(TB3_NAV2, 'map', 'map.yaml'),
                          'use_sim_time': 'true', 'params_file': params_file}.items())]


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = os.path.join(TB3_GAZEBO, 'worlds', 'turtlebot3_world.world')

    def include(path, args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                        launch_arguments=args.items())

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', description='Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('controller', default_value='dwb', description='dwb or teb'),
        DeclareLaunchArgument('planner', default_value='dijkstra', description='dijkstra or rrt'),
        DeclareLaunchArgument('rrt_smoothing', default_value='', description='true/false (planner:=rrt)'),
        DeclareLaunchArgument('rrt_cost_threshold', default_value='', description='e.g. 128 (planner:=rrt)'),
        DeclareLaunchArgument('rrt_allow_unknown', default_value='',
                              description='true/false (planner:=rrt)'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(MP2, 'config', 'nav2_params.yaml')),

        # Gazebo Classic, as in turtlebot3_world.launch.py
        include(os.path.join(GAZEBO_ROS, 'launch', 'gzserver.launch.py'), {'world': world}),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(GAZEBO_ROS, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui)),
        include(os.path.join(TB3_GAZEBO, 'launch', 'robot_state_publisher.launch.py'),
                {'use_sim_time': 'true'}),
        # spawn as in spawn_turtlebot3.launch.py, but without camera and with ground truth.
        # -timeout 120: the first gzserver start in a fresh environment took longer than the
        # default 30 s, the robot was never spawned and Nav2 waited for odom forever.
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'waffle_pi', '-file', robot_model(),
                        '-x', '-2.0', '-y', '-0.5', '-z', '0.01', '-timeout', '120']),

        # Nav2, as in navigation2.launch.py (without RViz)
        OpaqueFunction(function=nav2),
        # software rendering: with the GPU (WSLg, OpenGL 4.1) the 3D view stayed empty in
        # recordings (x11grab), only the background colour was captured
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(MP2, 'config', 'mp2.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(rviz),
             additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}),
    ])

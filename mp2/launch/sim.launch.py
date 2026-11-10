"""Task 1a, part 2: Gazebo (turtlebot3_world) + Nav2 with mp2/config/nav2_params.yaml.

Same world, robot and Nav2 bringup as
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
but in one launch file, optionally without GUIs, and with the Gazebo ground truth
bridged to ROS:
    /ground_truth/pose (geometry_msgs/PoseStamped, 30 Hz): robot pose in the Gazebo world
    frame, from a PosePublisher plugin added to the robot model.
The camera sensor is removed from the robot model (not used by Nav2): rendering it in
software and bridging the raw images overloaded the CPU under WSL2, so that laser
scans arrived only every few seconds and Nav2 did not start.

Run:
    ros2 launch mp2/launch/sim.launch.py                       # headless
    ros2 launch mp2/launch/sim.launch.py gui:=true rviz:=true  # with Gazebo window and RViz
"""
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MP2 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TB3_GAZEBO = get_package_share_directory('turtlebot3_gazebo')
TB3_NAV2 = get_package_share_directory('turtlebot3_navigation2')
ROS_GZ_SIM = get_package_share_directory('ros_gz_sim')
NAV2_BRINGUP = get_package_share_directory('nav2_bringup')


POSE_PUBLISHER = """
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_model_pose>true</publish_model_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <update_frequency>30</update_frequency>
    </plugin>
  </model>"""


def robot_model():
    """Waffle Pi model.sdf without camera, with ground-truth pose publisher (temporary file)."""
    path = os.path.join(TB3_GAZEBO, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    with open(path) as f:
        sdf = f.read()
    sdf, n = re.subn(r'<sensor name="camera".*?</sensor>', '', sdf, flags=re.S)
    assert n == 1, 'camera sensor not found in ' + path
    sdf, n = re.subn(r'\s*</model>', POSE_PUBLISHER, sdf, count=1)
    assert n == 1, '</model> not found in ' + path
    tmp = tempfile.NamedTemporaryFile('w', suffix='_waffle_pi_nocam.sdf', delete=False)
    tmp.write(sdf)
    tmp.close()
    return tmp.name


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    params_file = LaunchConfiguration('params_file')
    world = os.path.join(TB3_GAZEBO, 'worlds', 'turtlebot3_world.world')

    def include(path, args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                        launch_arguments=args.items())

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false', description='Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(MP2, 'config', 'nav2_params.yaml')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(TB3_GAZEBO, 'models')),

        # Gazebo, as in turtlebot3_world.launch.py
        include(os.path.join(ROS_GZ_SIM, 'launch', 'gz_sim.launch.py'),
                {'gz_args': f'-r -s -v2 {world}', 'on_exit_shutdown': 'true'}),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ROS_GZ_SIM, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(gui)),
        include(os.path.join(TB3_GAZEBO, 'launch', 'robot_state_publisher.launch.py'),
                {'use_sim_time': 'true'}),
        # spawn + bridge as in spawn_turtlebot3.launch.py, but without camera and image bridge
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'waffle_pi', '-file', robot_model(),
                        '-x', '-2.0', '-y', '-0.5', '-z', '0.01']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['--ros-args', '-p', 'config_file:=' + os.path.join(
                 TB3_GAZEBO, 'params', 'turtlebot3_waffle_pi_bridge.yaml')]),

        # ground truth: robot pose from the PosePublisher plugin -> /ground_truth/pose
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='ground_truth_bridge',
             arguments=['/model/waffle_pi/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'],
             remappings=[('/model/waffle_pi/pose', '/ground_truth/pose')],
             parameters=[{'use_sim_time': True}]),

        # Nav2, as in navigation2.launch.py (without RViz)
        include(os.path.join(NAV2_BRINGUP, 'launch', 'bringup_launch.py'),
                {'map': os.path.join(TB3_NAV2, 'map', 'map.yaml'),
                 'use_sim_time': 'true', 'params_file': params_file}),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(TB3_NAV2, 'rviz', 'tb3_navigation2.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(rviz)),
    ])

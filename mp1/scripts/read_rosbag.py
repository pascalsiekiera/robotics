#!/usr/bin/env python3

import sys
import os
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def read_rosbag(bag_path):
    """Read rosbag and extract odometry, IMU, and ground-truth data."""
    
    storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    odom_times = []
    odom_x = []
    odom_y = []
    odom_theta = []
    
    imu_times = []
    imu_ax = []
    imu_ay = []
    imu_az = []
    imu_wx = []
    imu_wy = []
    imu_wz = []
    
    topics_found = set()
    message_count = {}
    
    try:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            topics_found.add(topic)
            message_count[topic] = message_count.get(topic, 0) + 1
            
            if topic == "/odom":
                msg_type = get_message('nav_msgs/msg/Odometry')
                msg = deserialize_message(data, msg_type)
                
                t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                odom_times.append(t)
                odom_x.append(msg.pose.pose.position.x)
                odom_y.append(msg.pose.pose.position.y)
                
                # Extract yaw from quaternion
                q = msg.pose.pose.orientation
                yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                odom_theta.append(yaw)
            
            elif topic == "/imu":
                msg_type = get_message('sensor_msgs/msg/Imu')
                msg = deserialize_message(data, msg_type)
                
                t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                imu_times.append(t)
                imu_ax.append(msg.linear_acceleration.x)
                imu_ay.append(msg.linear_acceleration.y)
                imu_az.append(msg.linear_acceleration.z)
                imu_wx.append(msg.angular_velocity.x)
                imu_wy.append(msg.angular_velocity.y)
                imu_wz.append(msg.angular_velocity.z)
    
    except Exception as e:
        print(f"Error reading bag: {e}")
        import traceback
        traceback.print_exc()
    
    print(f"\nTopics found: {topics_found}")
    print(f"Message counts: {message_count}")
    
    return {
        'odom': {
            'times': np.array(odom_times),
            'x': np.array(odom_x),
            'y': np.array(odom_y),
            'theta': np.array(odom_theta)
        },
        'imu': {
            'times': np.array(imu_times),
            'accel': np.array([imu_ax, imu_ay, imu_az]).T,
            'angular_vel': np.array([imu_wx, imu_wy, imu_wz]).T
        }
    }


def plot_data(data):
    """Plot odometry and IMU data."""
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # Odometry path
    ax = axes[0, 0]
    ax.plot(data['odom']['x'], data['odom']['y'], 'b-', label='Odometry')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Robot Path (Odometry)')
    ax.grid(True)
    ax.axis('equal')
    ax.legend()
    
    # Odometry over time
    ax = axes[0, 1]
    t = data['odom']['times'] - data['odom']['times'][0]
    ax.plot(t, data['odom']['x'], label='X', alpha=0.7)
    ax.plot(t, data['odom']['y'], label='Y', alpha=0.7)
    ax.plot(t, data['odom']['theta'], label='Theta', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m) / Angle (rad)')
    ax.set_title('Odometry Over Time')
    ax.grid(True)
    ax.legend()
    
    # IMU acceleration
    ax = axes[1, 0]
    t_imu = data['imu']['times'] - data['imu']['times'][0]
    ax.plot(t_imu, data['imu']['accel'][:, 0], label='Ax', alpha=0.7)
    ax.plot(t_imu, data['imu']['accel'][:, 1], label='Ay', alpha=0.7)
    ax.plot(t_imu, data['imu']['accel'][:, 2], label='Az', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.set_title('IMU Acceleration')
    ax.grid(True)
    ax.legend()
    
    # IMU angular velocity
    ax = axes[1, 1]
    ax.plot(t_imu, data['imu']['angular_vel'][:, 0], label='Wx', alpha=0.7)
    ax.plot(t_imu, data['imu']['angular_vel'][:, 1], label='Wy', alpha=0.7)
    ax.plot(t_imu, data['imu']['angular_vel'][:, 2], label='Wz', alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angular Velocity (rad/s)')
    ax.set_title('IMU Angular Velocity')
    ax.grid(True)
    ax.legend()
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 read_rosbag.py <path_to_rosbag>")
        print("Example: python3 read_rosbag.py ~/turtlebot3_datasets/data/slam_easy")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    
    if not Path(bag_path).exists():
        print(f"Error: Bag path does not exist: {bag_path}")
        sys.exit(1)
    
    print(f"Reading rosbag from: {bag_path}")
    data = read_rosbag(bag_path)
    
    if len(data['odom']['times']) > 0:
        print("\nOdometry data summary:")
        print(f"  Messages: {len(data['odom']['times'])}")
        print(f"  Time range: {data['odom']['times'][0]:.2f} - {data['odom']['times'][-1]:.2f}")
        print(f"  X range: {data['odom']['x'].min():.2f} - {data['odom']['x'].max():.2f}")
        print(f"  Y range: {data['odom']['y'].min():.2f} - {data['odom']['y'].max():.2f}")
    
    if len(data['imu']['times']) > 0:
        print("\nIMU data summary:")
        print(f"  Messages: {len(data['imu']['times'])}")
        print(f"  Time range: {data['imu']['times'][0]:.2f} - {data['imu']['times'][-1]:.2f}")
        
        plot_data(data)
    else:
        print("No data extracted from bag.")

    # Save data for later use
    import pickle
    output_file = 'rosbag_data.pkl'
    with open(output_file, 'wb') as f:
        pickle.dump(data, f)
    print(f"\nData saved to {output_file}")
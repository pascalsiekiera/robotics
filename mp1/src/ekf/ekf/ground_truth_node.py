#!/usr/bin/env python3
"""
Turn the motion-capture ground truth of the dataset into robot poses.

The bag contains the mocap measurement as the TF  mocap -> mocap_laser_link
(~116 Hz). The marker centre sits on the laser scanner, not on the robot base,
and it is expressed in the mocap frame. This node converts every sample into
the pose of base_footprint in the world frame (map):

    T_map_base = T_map_mocap * T_mocap_laser(measured) * T_laser_base

T_map_mocap comes from the static TF published by publish_initial_tf /
static_transform_publisher, T_laser_base from the robot URDF.

Published topics
    /ground_truth/pose      PoseStamped                full rate, for evaluation
    /ground_truth/pose_1hz  PoseWithCovarianceStamped  down-sampled, fed to the EKFs

The mocap loses track for a few seconds in the middle of the dataset and
returns a few wrong samples around that (jumps of 10-20 cm, 180 deg flips).
Samples that jump further than the robot could have moved since the last good
sample are dropped.
"""

import math

from ekf.ekf_core import wrap_angle
from ekf.geometry import (matrix_to_planar_pose, stamp_to_sec, transform_to_matrix,
                          yaw_to_quaternion)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


class GroundTruthNode(Node):

    def __init__(self):
        super().__init__('ground_truth_node')

        self.world_frame = self.declare_parameter('world_frame', 'map').value
        self.mocap_frame = self.declare_parameter('mocap_frame', 'mocap').value
        self.marker_frame = self.declare_parameter('marker_frame', 'mocap_laser_link').value
        self.laser_frame = self.declare_parameter('laser_frame', 'base_scan').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.rate_hz = self.declare_parameter('downsampled_rate', 1.0).value
        sigma_xy = self.declare_parameter('sigma_xy', 0.02).value
        sigma_yaw = self.declare_parameter('sigma_yaw', 0.03).value
        # a sample is accepted if it moved less than  jump + speed * dt
        self.max_jump_xy = self.declare_parameter('max_jump_xy', 0.05).value
        self.max_speed = self.declare_parameter('max_speed', 0.5).value
        self.max_jump_yaw = self.declare_parameter('max_jump_yaw', 0.3).value
        self.max_yaw_rate = self.declare_parameter('max_yaw_rate', 2.0).value
        self.resync_after = self.declare_parameter('resync_after', 1.0).value

        self.covariance = [0.0] * 36
        self.covariance[0] = self.covariance[7] = sigma_xy ** 2
        self.covariance[35] = sigma_yaw ** 2
        self.covariance[14] = self.covariance[21] = self.covariance[28] = 1e6  # z, roll, pitch

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.T_world_mocap = None
        self.T_laser_base = None

        self.last_good = None       # (t, x, y, yaw) of the last accepted sample
        self.last_downsampled = None
        self.n_rejected = 0

        self.pub_full = self.create_publisher(PoseStamped, '/ground_truth/pose', 50)
        self.pub_1hz = self.create_publisher(
            PoseWithCovarianceStamped, '/ground_truth/pose_1hz', 10)
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 100)

        self.get_logger().info(
            f'Ground truth: {self.mocap_frame}->{self.marker_frame} converted to '
            f'{self.world_frame}->{self.base_frame}, down-sampled to {self.rate_hz} Hz')

    def static_transforms_ready(self):
        try:
            if self.T_world_mocap is None:
                self.T_world_mocap = transform_to_matrix(self.tf_buffer.lookup_transform(
                    self.world_frame, self.mocap_frame, Time()).transform)
            if self.T_laser_base is None:
                self.T_laser_base = transform_to_matrix(self.tf_buffer.lookup_transform(
                    self.laser_frame, self.base_frame, Time()).transform)
        except TransformException as e:
            self.get_logger().warn(
                f'Waiting for static transforms ({e}). Is the mocap->{self.world_frame} '
                'static TF and the robot_state_publisher running?', throttle_duration_sec=5.0)
            return False
        return True

    def is_plausible(self, t, x, y, yaw):
        if self.last_good is None:
            return True
        t0, x0, y0, yaw0 = self.last_good
        dt = t - t0
        if dt > self.resync_after:
            return True
        return (math.hypot(x - x0, y - y0) <= self.max_jump_xy + self.max_speed * dt and
                abs(wrap_angle(yaw - yaw0)) <= self.max_jump_yaw + self.max_yaw_rate * dt)

    def tf_callback(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == self.marker_frame and tf.header.frame_id == self.mocap_frame:
                self.process_sample(tf)

    def process_sample(self, tf):
        if not self.static_transforms_ready():
            return

        T = self.T_world_mocap @ transform_to_matrix(tf.transform) @ self.T_laser_base
        x, y, yaw = matrix_to_planar_pose(T)
        t = stamp_to_sec(tf.header.stamp)

        if not self.is_plausible(t, x, y, yaw):
            self.n_rejected += 1
            self.get_logger().warn(
                f'Dropped implausible ground-truth sample at t={t:.2f} '
                f'({self.n_rejected} so far)', throttle_duration_sec=1.0)
            return
        self.last_good = (t, x, y, yaw)

        pose = PoseStamped()
        pose.header.stamp = tf.header.stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        yaw_to_quaternion(yaw, pose.pose.orientation)
        self.pub_full.publish(pose)

        if self.last_downsampled is None or t - self.last_downsampled >= 1.0 / self.rate_hz:
            self.last_downsampled = t
            out = PoseWithCovarianceStamped()
            out.header = pose.header
            out.pose.pose = pose.pose
            out.pose.covariance = np.array(self.covariance, dtype=float)
            self.pub_1hz.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

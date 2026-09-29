#!/usr/bin/env python3
"""
Own EKF localisation node (Mini-Project 1, task 1a).

Fuses, each at the rate it arrives:
    /odom                  wheel odometry twist (v, w)          ~25 Hz
    /imu                   gyroscope yaw rate (w)               ~116 Hz
    /ground_truth/pose_1hz absolute pose (x, y, theta)          1 Hz
The filter itself lives in ekf_core.py; this node only moves data between ROS
and the filter.

Every message is processed at its header stamp: the filter is first predicted
from its current time to the stamp, then corrected with the measurement.

Outputs
    /ekf/odom   nav_msgs/Odometry in the world frame, with covariance
    TF map -> odom, so that map -> odom -> base_footprint equals the EKF
    estimate (same convention as robot_localization and AMCL). RViz then
    shows the robot model / laser scan at the EKF pose, and
    calculate_error.py (mocap_laser_link vs base_scan) measures the EKF error.
"""

from ekf.ekf_core import PlanarEKF
from ekf.geometry import (matrix_to_planar_pose, planar_pose_to_matrix, stamp_to_sec,
                          yaw_from_quaternion, yaw_to_quaternion)
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class EKFNode(Node):

    def __init__(self):
        super().__init__('ekf_node')

        p = self.declare_parameter
        self.world_frame = p('world_frame', 'map').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.publish_tf = p('publish_tf', True).value
        self.transform_tolerance = p('transform_tolerance', 0.1).value
        publish_rate = p('publish_rate', 30.0).value

        self.use_odometry = p('use_odometry', True).value
        self.use_imu = p('use_imu', True).value
        self.use_ground_truth = p('use_ground_truth', True).value

        # process noise spectral densities for [x, y, theta, v, w]
        self.q_diag = p('process_noise', [1e-4, 1e-4, 1e-4, 0.1, 2.0]).value
        self.P0_diag = p('initial_covariance', [0.01, 0.01, 0.01, 0.1, 0.1]).value
        self.initial_pose = p('initial_pose', [0.0, 0.0, 0.0]).value

        # The odometry in the dataset has all-zero covariances, so we set our own.
        self.R_odom = np.diag([p('odom_v_sigma', 0.01).value ** 2,
                               p('odom_w_sigma', 0.05).value ** 2])
        # <= 0 means: use the covariance written in the Imu message
        self.imu_w_sigma = p('imu_w_sigma', 0.0).value
        self.gate_sensors = p('gate_sensors', True).value
        self.gate_ground_truth = p('gate_ground_truth', True).value

        self.ekf = None
        self.t = None                   # filter time [s]
        self.last_odom_pose = None      # odom -> base_footprint, from the latest /odom
        self.rejected = {'odom': 0, 'imu': 0, 'ground_truth': 0}
        self.accepted = {'odom': 0, 'imu': 0, 'ground_truth': 0}

        self.create_subscription(Odometry, p('odom_topic', '/odom').value,
                                 self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, p('imu_topic', '/imu').value,
                                 self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,
                                 p('ground_truth_topic', '/ground_truth/pose_1hz').value,
                                 self.ground_truth_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/ekf/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / publish_rate, self.publish)
        self.create_timer(10.0, self.log_statistics)

        sensors = [n for n, on in (('odometry', self.use_odometry), ('imu', self.use_imu),
                                   ('ground truth', self.use_ground_truth)) if on]
        self.get_logger().info(f'EKF started, fusing: {", ".join(sensors)}')
        if not self.use_ground_truth:
            self.initialise(self.initial_pose, None)

    # ------------------------------------------------------------------ filter

    def initialise(self, pose, t):
        x0 = [pose[0], pose[1], pose[2], 0.0, 0.0]
        self.ekf = PlanarEKF(x0, np.diag(self.P0_diag), self.q_diag)
        self.t = t
        self.get_logger().info(
            f'EKF initialised at x={pose[0]:.3f} y={pose[1]:.3f} theta={pose[2]:.3f}')

    def predict_to(self, t):
        """Advance the filter to time t. Returns False if it is not initialised yet."""
        if self.ekf is None:
            return False
        if self.t is None:
            self.t = t
        elif t > self.t:
            self.ekf.predict(t - self.t)
            self.t = t
        # Messages that are slightly older than the filter time (the topics
        # interleave within a few ms) are applied without predicting backwards.
        return True

    def count(self, sensor, accepted):
        (self.accepted if accepted else self.rejected)[sensor] += 1

    # --------------------------------------------------------------- callbacks

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.last_odom_pose = (pose.position.x, pose.position.y,
                               yaw_from_quaternion(pose.orientation))
        if not self.use_odometry or not self.predict_to(stamp_to_sec(msg.header.stamp)):
            return
        ok, _ = self.ekf.update_odometry(msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                                         self.R_odom, gate=self.gate_sensors)
        self.count('odom', ok)

    def imu_callback(self, msg):
        if not self.use_imu or not self.predict_to(stamp_to_sec(msg.header.stamp)):
            return
        if self.imu_w_sigma > 0.0:
            var = self.imu_w_sigma ** 2
        else:
            var = msg.angular_velocity_covariance[8]
        ok, _ = self.ekf.update_gyro(msg.angular_velocity.z, var, gate=self.gate_sensors)
        self.count('imu', ok)

    def ground_truth_callback(self, msg):
        if not self.use_ground_truth:
            return
        t = stamp_to_sec(msg.header.stamp)
        pose = msg.pose.pose
        z = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        if self.ekf is None:
            self.initialise(z, t)
            return
        self.predict_to(t)
        c = np.array(msg.pose.covariance).reshape(6, 6)
        R = np.diag([c[0, 0], c[1, 1], c[5, 5]])
        ok, d2 = self.ekf.update_pose(*z, R=R, gate=self.gate_ground_truth)
        self.count('ground_truth', ok)
        if not ok:
            self.get_logger().warn(
                f'Rejected ground-truth update at t={t:.2f} (Mahalanobis^2 = {d2:.1f})')

    # ------------------------------------------------------------------ output

    def publish(self):
        if self.ekf is None or self.t is None:
            return
        x, y, th, v, w = self.ekf.x
        P = self.ekf.P
        stamp = Time(seconds=self.t).to_msg()

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        yaw_to_quaternion(th, msg.pose.pose.orientation)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = float(v), float(w)
        # 6x6 covariances are ordered (x, y, z, roll, pitch, yaw)
        pose_cov = np.zeros((6, 6))
        idx = [0, 1, 5]
        pose_cov[np.ix_(idx, idx)] = P[:3, :3]
        msg.pose.covariance = pose_cov.flatten()
        twist_cov = np.zeros((6, 6))
        twist_cov[np.ix_([0, 5], [0, 5])] = P[3:, 3:]
        msg.twist.covariance = twist_cov.flatten()
        self.odom_pub.publish(msg)

        if self.publish_tf and self.last_odom_pose is not None:
            # map->odom = (map->base, estimated) * (odom->base, from odometry)^-1
            T_map_odom = (planar_pose_to_matrix(x, y, th) @
                          np.linalg.inv(planar_pose_to_matrix(*self.last_odom_pose)))
            mx, my, myaw = matrix_to_planar_pose(T_map_odom)
            tf = TransformStamped()
            # post-date the transform a little, like AMCL does, so that RViz can
            # use it for sensor data that is slightly newer than the filter
            tf.header.stamp = (Time(seconds=self.t) +
                               Duration(seconds=self.transform_tolerance)).to_msg()
            tf.header.frame_id = self.world_frame
            tf.child_frame_id = self.odom_frame
            tf.transform.translation.x, tf.transform.translation.y = float(mx), float(my)
            yaw_to_quaternion(myaw, tf.transform.rotation)
            self.tf_broadcaster.sendTransform(tf)

    def log_statistics(self):
        if self.ekf is None:
            self.get_logger().info('Waiting for the first ground-truth pose to initialise...',
                                   throttle_duration_sec=10.0)
            return
        sx, sy, sth = np.sqrt(np.diag(self.ekf.P)[:3])
        stats = ', '.join(f'{k} {self.accepted[k]}/{self.rejected[k]}' for k in self.accepted)
        self.get_logger().info(
            f'sigma x={sx * 100:.1f}cm y={sy * 100:.1f}cm theta={np.degrees(sth):.1f}deg | '
            f'updates accepted/rejected: {stats}')


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

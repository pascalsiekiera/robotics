"""Small helpers to go between ROS transforms/quaternions and numpy."""

import math

import numpy as np


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_to_matrix(transform):
    """geometry_msgs/Transform -> 4x4 homogeneous matrix."""
    t, q = transform.translation, transform.rotation
    M = np.eye(4)
    M[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def planar_pose_to_matrix(x, y, yaw):
    M = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    M[:2, :2] = [[c, -s], [s, c]]
    M[0, 3], M[1, 3] = x, y
    return M


def matrix_to_planar_pose(M):
    """4x4 matrix -> (x, y, yaw), dropping z, roll and pitch."""
    return M[0, 3], M[1, 3], math.atan2(M[1, 0], M[0, 0])


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quaternion(yaw, q):
    """Fill geometry_msgs/Quaternion q with a pure rotation about z."""
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

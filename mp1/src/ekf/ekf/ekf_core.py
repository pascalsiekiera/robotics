"""
Extended Kalman Filter for a planar differential-drive robot (no ROS inside).

State vector  x = [x, y, theta, v, w]
    x, y   position of base_footprint in the world frame        [m]
    theta  heading (yaw) in the world frame                     [rad]
    v      forward (linear) velocity in the robot frame         [m/s]
    w      angular velocity (yaw rate)                          [rad/s]

Motion model (constant velocity, unicycle), for a time step dt:
    x'     = x + v*dt*cos(theta)
    y'     = y + v*dt*sin(theta)
    theta' = theta + w*dt
    v'     = v
    w'     = w
The velocities are modelled as random walks, i.e. the process noise Q is what
lets v and w change between measurements.

Measurements (all linear in the state, so H is constant):
    wheel odometry twist  z = [v, w]          H picks rows 3, 4
    IMU gyroscope         z = [w]             H picks row 4
    ground truth (1 Hz)   z = [x, y, theta]   H picks rows 0, 1, 2
"""

import math

import numpy as np

# Chi-square 99.9% quantiles, used for Mahalanobis outlier gating (index = dof).
CHI2_999 = {1: 10.828, 2: 13.816, 3: 16.266}

H_ODOM = np.array([[0, 0, 0, 1, 0],
                   [0, 0, 0, 0, 1]], dtype=float)
H_GYRO = np.array([[0, 0, 0, 0, 1]], dtype=float)
H_POSE = np.array([[1, 0, 0, 0, 0],
                   [0, 1, 0, 0, 0],
                   [0, 0, 1, 0, 0]], dtype=float)


def wrap_angle(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class PlanarEKF:

    def __init__(self, x0, P0, q_diag):
        """
        Create the filter.

        x0      initial state (5,)
        P0      initial covariance (5, 5)
        q_diag  process-noise spectral densities for [x, y, theta, v, w];
                Q = diag(q_diag) * dt, so the uncertainty added per step
                scales with the time elapsed.
        """
        self.x = np.asarray(x0, dtype=float).reshape(5)
        self.P = np.asarray(P0, dtype=float).reshape(5, 5)
        self.q_diag = np.asarray(q_diag, dtype=float).reshape(5)

    def predict(self, dt):
        if dt <= 0.0:
            return
        x, y, th, v, w = self.x
        c, s = math.cos(th), math.sin(th)

        self.x = np.array([x + v * dt * c,
                           y + v * dt * s,
                           wrap_angle(th + w * dt),
                           v,
                           w])

        # Jacobian of the motion model with respect to the state
        F = np.array([[1, 0, -v * dt * s, dt * c, 0],
                      [0, 1,  v * dt * c, dt * s, 0],
                      [0, 0,  1,          0,      dt],
                      [0, 0,  0,          1,      0],
                      [0, 0,  0,          0,      1]])
        Q = np.diag(self.q_diag * dt)
        self.P = F @ self.P @ F.T + Q

    def update(self, z, H, R, angle_rows=(), gate=True):
        """
        Run a linear measurement update z = H x + noise(R).

        angle_rows  rows of z that are angles (their innovation gets wrapped)
        gate        reject the measurement if its Mahalanobis distance is
                    beyond the 99.9% chi-square bound

        Returns (accepted, squared Mahalanobis distance).
        """
        z = np.asarray(z, dtype=float).reshape(-1)
        R = np.asarray(R, dtype=float)

        innovation = z - H @ self.x
        for i in angle_rows:
            innovation[i] = wrap_angle(innovation[i])

        S = H @ self.P @ H.T + R
        S_inv = np.linalg.inv(S)
        d2 = float(innovation @ S_inv @ innovation)
        if gate and d2 > CHI2_999[len(z)]:
            return False, d2

        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ innovation
        self.x[2] = wrap_angle(self.x[2])

        # Joseph form keeps P symmetric and positive definite
        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return True, d2

    def update_odometry(self, v, w, R, gate=True):
        return self.update([v, w], H_ODOM, R, gate=gate)

    def update_gyro(self, w, var, gate=True):
        return self.update([w], H_GYRO, [[var]], gate=gate)

    def update_pose(self, x, y, theta, R, gate=False):
        return self.update([x, y, theta], H_POSE, R, angle_rows=(2,), gate=gate)

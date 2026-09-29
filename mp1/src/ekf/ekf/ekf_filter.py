#!/usr/bin/env python3

import numpy as np
from scipy.linalg import block_diag


class ExtendedKalmanFilter:
    """Extended Kalman Filter for 2D robot localization."""
    
    def __init__(self, initial_state, process_noise, measurement_noise):
        """
        Initialize the EKF.
        
        Args:
            initial_state: [x, y, theta] initial pose
            process_noise: Q matrix (3x3) - uncertainty in motion model
            measurement_noise: R matrix (2x2) - uncertainty in measurements (x, y)
        """
        self.state = np.array(initial_state, dtype=float)  # [x, y, theta]
        self.Q = process_noise  # Process noise covariance (3x3)
        self.R = measurement_noise  # Measurement noise covariance (2x2)
        self.P = np.eye(3) * 0.1  # Estimation error covariance (3x3)
        
        self.state_history = [self.state.copy()]
        self.covariance_history = [self.P.copy()]
    
    def predict(self, dt, v, omega):
        """
        Prediction step using odometry (velocity and angular velocity).
        
        Args:
            dt: time step
            v: linear velocity (m/s)
            omega: angular velocity (rad/s)
        """
        x, y, theta = self.state
        
        # Motion model
        if abs(omega) < 1e-6:  # Straight line
            x_new = x + v * dt * np.cos(theta)
            y_new = y + v * dt * np.sin(theta)
            theta_new = theta
        else:  # Circular motion
            x_new = x + (v / omega) * (np.sin(theta + omega * dt) - np.sin(theta))
            y_new = y + (v / omega) * (-np.cos(theta + omega * dt) + np.cos(theta))
            theta_new = theta + omega * dt
        
        # Jacobian of motion model w.r.t state
        if abs(omega) < 1e-6:
            F = np.array([
                [1, 0, -v * dt * np.sin(theta)],
                [0, 1,  v * dt * np.cos(theta)],
                [0, 0, 1]
            ])
        else:
            F = np.array([
                [1, 0, (v / omega) * (np.cos(theta + omega * dt) - np.cos(theta))],
                [0, 1, (v / omega) * (np.sin(theta + omega * dt) - np.sin(theta))],
                [0, 0, 1]
            ])
        
        # Update state and covariance
        self.state = np.array([x_new, y_new, theta_new])
        self.P = F @ self.P @ F.T + self.Q
        
        self.state_history.append(self.state.copy())
        self.covariance_history.append(self.P.copy())
    
    def update(self, z):
        """
        Update step using position measurements (ground truth).
        
        Args:
            z: measurement [x, y]
        """
        # Measurement model: we measure x and y directly
        H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])
        
        # Innovation (measurement residual)
        z_pred = H @ self.state  # Predicted measurement
        y = z - z_pred  # Innovation
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.state = self.state + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P
        
        self.state_history[-1] = self.state.copy()
        self.covariance_history[-1] = self.P.copy()
    
    def get_state(self):
        """Return current state [x, y, theta]."""
        return self.state.copy()
    
    def get_covariance(self):
        """Return current estimation error covariance."""
        return self.P.copy()
    
    def get_uncertainty(self):
        """Return standard deviations of x, y, theta."""
        return np.sqrt(np.diag(self.P))
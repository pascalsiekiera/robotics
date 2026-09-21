#!/usr/bin/env python3

import sys
import pickle
import os
import numpy as np
import matplotlib.pyplot as plt
sys.path.insert(0, '/home/psi/code/IR/mp1/src/ekf')

from ekf.ekf_filter import ExtendedKalmanFilter


def load_data(pkl_file):
    """Load the rosbag data from pickle."""
    with open(pkl_file, 'rb') as f:
        data = pickle.load(f)
    return data


def run_ekf(data, Q, R, add_measurement_noise=False):
    """
    Run the EKF on the odometry and ground-truth data.
    
    Args:
        data: dict with 'odom' and 'imu' keys
        Q: process noise (3x3)
        R: measurement noise (2x2)
        add_measurement_noise: if True, add Gaussian noise to measurements
    """
    
    odom = data['odom']
    imu = data['imu']
    
    # Initialize filter
    initial_state = [odom['x'][0], odom['y'][0], odom['theta'][0]]
    ekf = ExtendedKalmanFilter(initial_state, Q, R)
    
    # Downsample ground truth to 1 Hz
    # Use odom times as reference
    t_odom = odom['times']
    
    # Simple approach: match timestamps
    # For each odom message, do prediction
    # For every ~1 second, do update with ground truth
    
    last_update_time = t_odom[0]
    
    for i in range(1, len(t_odom)):
        dt = t_odom[i] - t_odom[i-1]
        
        if dt <= 0 or dt > 1.0:  # Skip bad timestamps
            continue
        
        # Calculate velocities from odom
        dx = odom['x'][i] - odom['x'][i-1]
        dy = odom['y'][i] - odom['y'][i-1]
        d_theta = odom['theta'][i] - odom['theta'][i-1]
        
        # Normalize angle difference
        d_theta = np.arctan2(np.sin(d_theta), np.cos(d_theta))
        
        v = np.sqrt(dx**2 + dy**2) / dt
        omega = d_theta / dt
        
        # Prediction step
        ekf.predict(dt, v, omega)
        
        # Update step every ~1 second
        if t_odom[i] - last_update_time >= 1.0:
            z = np.array([odom['x'][i], odom['y'][i]])
            
            # Add measurement noise if requested
            if add_measurement_noise:
                noise = np.random.multivariate_normal([0, 0], R)
                z = z + noise
            
            ekf.update(z)
            last_update_time = t_odom[i]
    
    return ekf


def plot_results(data, ekf, save_path=None):
    """Plot the results."""
    
    odom = data['odom']
    history = np.array(ekf.state_history)
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Path comparison
    ax = axes[0]
    ax.plot(odom['x'], odom['y'], 'b-', label='Odometry (Ground Truth)', linewidth=2)
    ax.plot(history[:, 0], history[:, 1], 'r--', label='EKF Estimate', linewidth=2, alpha=0.7)
    ax.scatter(odom['x'][0], odom['y'][0], c='green', s=100, marker='o', label='Start', zorder=5)
    ax.scatter(odom['x'][-1], odom['y'][-1], c='red', s=100, marker='s', label='End', zorder=5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Robot Path: Odometry vs EKF')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend()
    
    # Error over time
    ax = axes[1]
    t = odom['times'] - odom['times'][0]
    
    # Calculate error
    error_x = odom['x'][:len(history)] - history[:, 0]
    error_y = odom['y'][:len(history)] - history[:, 1]
    error_norm = np.sqrt(error_x**2 + error_y**2)
    
    ax.plot(t[:len(error_norm)], error_norm, 'r-', linewidth=2)
    ax.fill_between(t[:len(error_norm)], 0, error_norm, alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (m)')
    ax.set_title('Localization Error Over Time')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  Plot saved to {save_path}")
    else:
        plt.show()
    
    plt.close()


if __name__ == "__main__":
    pkl_file = 'IR/mp1/data/rosbag_data.pkl'
    figures_dir = 'IR/mp1/figures'
    
    # Create figures directory if it doesn't exist
    os.makedirs(figures_dir, exist_ok=True)
    
    print(f"Loading data from {pkl_file}...")
    data = load_data(pkl_file)
    
    # Test different combinations
    configs = [
        {'Q': np.diag([0.001, 0.001, 0.001]), 'R': np.diag([0.001, 0.001]), 'name': 'Q small, R small (trust both)'},
        {'Q': np.diag([0.1, 0.1, 0.1]), 'R': np.diag([0.001, 0.001]), 'name': 'Q large, R small (trust measurements)'},
        {'Q': np.diag([0.001, 0.001, 0.001]), 'R': np.diag([0.1, 0.1]), 'name': 'Q small, R large (trust motion)'},
    ]
    
    for i, config in enumerate(configs):
        print(f"\n--- {config['name']} ---")
        Q = config['Q']
        R = config['R']
        
        print(f"Running EKF with Q={np.diag(Q)}, R={np.diag(R)}...")
        ekf = run_ekf(data, Q, R, add_measurement_noise=False) # NOISE
        
        print("Final state:")
        print(f"  Position: {ekf.get_state()[:2]}")
        print(f"  Uncertainty (std): {ekf.get_uncertainty()}")
        
        # Save plot
        filename = f"ekf_result_{i+1}_{config['name'].replace(', ', '_').replace(' ', '_')}.png"
        filepath = os.path.join(figures_dir, filename)
        plot_results(data, ekf, save_path=filepath)
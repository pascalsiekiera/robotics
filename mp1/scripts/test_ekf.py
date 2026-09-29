#!/usr/bin/env python3

import sys
import pickle
import os
import numpy as np
import matplotlib.pyplot as plt

# Add the ekf module path
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


def plot_results(data, ekf, Q, R, save_path=None):
    """Plot the results with Initial and Final covariance matrices displayed."""

    odom = data['odom']
    history = np.array(ekf.state_history)

    initial_P = ekf.covariance_history[0]
    final_P = ekf.covariance_history[-1]

    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(2, 2, hspace=0.35, wspace=0.3)

    # Top row: Configuration info with Initial and Final P
    ax_config = fig.add_subplot(gs[0, :])
    ax_config.axis('off')

    config_text = f"""Initial Covariance P                       Final Covariance P
{initial_P[0,0]:8.4f}  {initial_P[0,1]:8.4f}  {initial_P[0,2]:8.4f}              {final_P[0,0]:8.4f}  {final_P[0,1]:8.4f}  {final_P[0,2]:8.4f}
{initial_P[1,0]:8.4f}  {initial_P[1,1]:8.4f}  {initial_P[1,2]:8.4f}              {final_P[1,0]:8.4f}  {final_P[1,1]:8.4f}  {final_P[1,2]:8.4f}
{initial_P[2,0]:8.4f}  {initial_P[2,1]:8.4f}  {initial_P[2,2]:8.4f}              {final_P[2,0]:8.4f}  {final_P[2,1]:8.4f}  {final_P[2,2]:8.4f}"""

    ax_config.text(0.05, 0.5, config_text, fontsize=10, family='monospace',
                   verticalalignment='center', transform=ax_config.transAxes)

    # Bottom row: Path and Error
    ax1 = fig.add_subplot(gs[1, 0])
    ax2 = fig.add_subplot(gs[1, 1])

    # Path comparison
    ax1.plot(odom['x'], odom['y'], 'b-', label='Odometry (Ground Truth)', linewidth=2)
    ax1.plot(history[:, 0], history[:, 1], 'r--', label='EKF Estimate', linewidth=2, alpha=0.7)
    ax1.scatter(odom['x'][0], odom['y'][0], c='green', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(odom['x'][-1], odom['y'][-1], c='red', s=100, marker='s', label='End', zorder=5)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Robot Path: Odometry vs EKF')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.legend()

    # Error over time
    t = odom['times'] - odom['times'][0]
    error_x = odom['x'][:len(history)] - history[:, 0]
    error_y = odom['y'][:len(history)] - history[:, 1]
    error_norm = np.sqrt(error_x**2 + error_y**2)

    ax2.plot(t[:len(error_norm)], error_norm, 'r-', linewidth=2)
    ax2.fill_between(t[:len(error_norm)], 0, error_norm, alpha=0.3)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position Error (m)')
    ax2.set_title('Localization Error Over Time')
    ax2.grid(True, alpha=0.3)

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
        print(f"\n{'='*70}")
        print(f"Configuration {i+1}: {config['name']}")
        print(f"{'='*70}")
        Q = config['Q']
        R = config['R']

        print(f"\nProcess Noise (Q):")
        print(f"  Diagonal: {np.diag(Q)}")
        print(f"  Matrix:\n{Q}\n")

        print(f"Measurement Noise (R):")
        print(f"  Diagonal: {np.diag(R)}")
        print(f"  Matrix:\n{R}\n")

        print(f"Running EKF...")
        ekf = run_ekf(data, Q, R, add_measurement_noise=False)

        print("\nFinal State:")
        print(f"  Position (x, y): {ekf.get_state()[:2]}")
        print(f"  Orientation (theta): {ekf.get_state()[2]:.4f} rad")

        print("\nFinal Covariance Matrix P:")
        final_cov = ekf.get_covariance()
        print(final_cov)

        print("\nUncertainty (Standard Deviations):")
        uncertainty = ekf.get_uncertainty()
        print(f"  Std X:     {uncertainty[0]:.6f} m")
        print(f"  Std Y:     {uncertainty[1]:.6f} m")
        print(f"  Std Theta: {uncertainty[2]:.6f} rad")

        # Initial covariance for comparison
        print("\nInitial Covariance Matrix P:")
        initial_cov = ekf.covariance_history[0]
        print(initial_cov)

        print("\nInitial Uncertainty (Standard Deviations):")
        initial_uncertainty = np.sqrt(np.diag(initial_cov))
        print(f"  Std X:     {initial_uncertainty[0]:.6f} m")
        print(f"  Std Y:     {initial_uncertainty[1]:.6f} m")
        print(f"  Std Theta: {initial_uncertainty[2]:.6f} rad")

        # Save plot
        filename = f"ekf_result_{i+1}_{config['name'].replace(', ', '_').replace(' ', '_')}.png"
        filepath = os.path.join(figures_dir, filename)
        plot_results(data, ekf, Q, R, save_path=filepath)
        print(f"\n✓ Plot saved to {filepath}")
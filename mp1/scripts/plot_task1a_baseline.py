#!/usr/bin/env python3
"""
Presentation figure for Task 1a: own EKF with the baseline parameters vs ground truth,
drawn on the map provided with the dataset.

Usage:
    python3 plot_task1a_baseline.py [results.csv] [output.png]
"""

import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

CSV = sorted(glob.glob(os.path.expanduser('~/ros2_ws/results/task1a_baseline_*.csv')))[-1]
MAP = os.path.expanduser('~/ros2_ws/src/turtlebot_datasets/data/map.yaml')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'figures', 'task1a_tuning',
                   'task1a_baseline_path.png')


def main():
    csv = sys.argv[1] if len(sys.argv) > 1 else CSV
    out = sys.argv[2] if len(sys.argv) > 2 else OUT

    data = np.genfromtxt(csv, delimiter=',', names=True, dtype=None, encoding=None)
    ekf = data[data['estimator'] == 'ekf']

    m = yaml.safe_load(open(MAP))
    img = np.array(Image.open(os.path.join(os.path.dirname(MAP), m['image'])))
    H, W = img.shape
    res, ox, oy = m['resolution'], m['origin'][0], m['origin'][1]
    extent = (ox, ox + W * res, oy, oy + H * res)

    fig, ax = plt.subplots(figsize=(7, 5.2))
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper',
              interpolation='nearest')
    ax.plot(ekf['gt_x'], ekf['gt_y'], '-', color='tab:green', lw=3, label='ground truth (mocap)')
    ax.plot(ekf['x'], ekf['y'], '-', color='tab:red', lw=1.2, label='own EKF (baseline)')
    ax.plot(ekf['gt_x'][0], ekf['gt_y'][0], '^', color='black', ms=8, label='start')

    xs = np.r_[ekf['x'], ekf['gt_x']]
    ys = np.r_[ekf['y'], ekf['gt_y']]
    pad = 0.6
    ax.set_xlim(xs.min() - pad, xs.max() + pad)
    ax.set_ylim(ys.min() - pad, ys.max() + pad)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Task 1a: own EKF (baseline) vs ground truth\n'
                 'RMSE 0.84 cm, max 5.0 cm (mocap loss excluded)')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out, dpi=200, bbox_inches='tight')
    print(f'Saved {os.path.abspath(out)}')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Report figure for Task 2b: run L (recording 2b_run_2) localized with AMCL on a lab map
(default: the merged map, map_2a_merged = maps M2 + L).

Left:  AMCL estimate and pure wheel odometry in the map frame, with 2-sigma ellipses.
Right: AMCL position uncertainty over time.
The scan fit of AMCL and odometry (mean distance of the laser end points to the nearest
wall of the map) is printed for the evaluation table.

Needs ROS 2 sourced (rosbag2_py) and the loader in ~/ros2_ws/results/scripts/load2b.py.

Usage:
    python3 plot_task2b.py [bag_dir] [output.png] [map_name] [map_label]
    python3 plot_task2b.py ~/ros2_ws/results/bags/task2b_run_2_20261012_172640 out.png map_2a_new M2
"""

import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np

sys.path.insert(0, os.path.expanduser('~/ros2_ws/results/scripts'))
import load2b as L  # noqa: E402

BAG = os.path.expanduser('~/ros2_ws/results/bags/task2b_merged_run_2_20261012_224305')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'figures', 'task2b',
                   'task2b_L_on_merged.png')
MAP = 'map_2a_merged'
MAP_LABEL = 'the merged map'
ELLIPSE_EVERY = 8.0   # s


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else BAG
    out = sys.argv[2] if len(sys.argv) > 2 else OUT
    map_name = sys.argv[3] if len(sys.argv) > 3 else MAP
    map_label = sys.argv[4] if len(sys.argv) > 4 else MAP_LABEL

    mp = L.load_map(map_name)
    d = L.load(bag)
    amcl = d['amcl']   # t, x, y, yaw, cov_xx, cov_xy, cov_yy, cov_yawyaw
    path = L.amcl_continuous(d)
    odom = L.odometry_only(d, tuple(amcl[0, 1:4]))
    fit_amcl = L.scan_fit(mp, d, path)
    fit_odom = L.scan_fit(mp, d, odom)
    t0 = d['odom'][0, 0]

    plt.rcParams.update({'font.size': 10})
    fig = plt.figure(figsize=(11, 4.6))
    grid = fig.add_gridspec(1, 2, width_ratios=[1, 1.25], wspace=0.18)

    # ---------------------------------------------------------------- map + paths
    ax = fig.add_subplot(grid[0, 0])
    H, W = mp['img'].shape
    extent = (mp['ox'], mp['ox'] + W * mp['res'], mp['oy'], mp['oy'] + H * mp['res'])
    ax.imshow(mp['img'], cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper',
              interpolation='nearest')
    ax.plot(odom[:, 1], odom[:, 2], '--', color='orange', lw=1.4, label='wheel odometry only')
    ax.plot(path[:, 1], path[:, 2], '-', color='tab:red', lw=1.6, label='AMCL estimate')
    last = -np.inf
    for t, x, y, _, cxx, cxy, cyy, _ in amcl:
        if t - last < ELLIPSE_EVERY:
            continue
        last = t
        vals, vecs = np.linalg.eigh(np.array([[cxx, cxy], [cxy, cyy]]))
        vals = np.clip(vals, 0, None)
        angle = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
        ax.add_patch(Ellipse((x, y), 4 * math.sqrt(vals[1]), 4 * math.sqrt(vals[0]), angle=angle,
                             fill=False, color='tab:red', lw=0.7))
    ax.plot([], [], color='tab:red', lw=0.7, label=r'AMCL 2$\sigma$ ellipse, every 8 s')
    ax.plot(path[0, 1], path[0, 2], '^', color='green', ms=9, label='start')
    ax.plot(path[-1, 1], path[-1, 2], 's', color='black', ms=7, label='end')
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'Run L on {map_label}: paths in the map frame')
    ax.legend(loc='upper right', fontsize=7, framealpha=0.85)

    # ---------------------------------------------------------------- uncertainty
    ax = fig.add_subplot(grid[0, 1])
    ax.set_box_aspect(0.55)
    upd = amcl[1:]   # the first pose is the given start pose, not an AMCL update
    ta = upd[:, 0] - t0
    ax.plot(ta, 100 * np.sqrt(np.clip(upd[:, 4], 0, None)), label=r'$\sigma_x$')
    ax.plot(ta, 100 * np.sqrt(np.clip(upd[:, 6], 0, None)), label=r'$\sigma_y$')
    ax.set_ylim(bottom=0)
    ax.set_ylabel('[cm]')
    ax.set_title('AMCL position uncertainty (standard deviation)')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)

    ax.set_xlim(left=0)
    ax.set_xlabel('time since start of the recording [s]')

    fig.savefig(out, dpi=200, bbox_inches='tight')
    print(f'Saved {os.path.abspath(out)}')
    print(f'scan fit median: AMCL {100 * np.median(fit_amcl[:, 1]):.1f} cm, '
          f'odometry {100 * np.median(fit_odom[:, 1]):.1f} cm')


if __name__ == '__main__':
    main()

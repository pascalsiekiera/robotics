#!/usr/bin/env python3
"""
Report figure for Task 2a: pure wheel-odometry paths of runs M1, O1 and O2 drawn on the
merged map M2+L (map_2a_merged).

The merged map is in the frame of map M2, so the runs of 01.10. do not start at its origin.
The start pose of M1 is found by matching the laser scans recorded before the robot moves
to the map (grid search over x, y, theta minimising the mean distance of the scan points to
the nearest wall). O1 and O2 recorded no laser scans; like M1 they started from the same
start point, so they use the start pose of M1. From there on only the odometry is used,
so the paths drift over time.

Needs ROS 2 sourced (rosbag2_py) and the loader in ~/ros2_ws/results/scripts/load2b.py.

Usage:
    python3 plot_task2a_paths.py [output.png]
"""

import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.expanduser('~/ros2_ws/results/scripts'))
import load2b as L  # noqa: E402

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'figures', 'task2a',
                   'task2a_paths_on_merged.png')
RUNS = [('M1', '2a_raw_1'), ('O1', '2a_raw_11'), ('O2', '2a_raw_14')]
MAP = 'map_2a_merged'
UNKNOWN = 205   # pgm value of unknown cells (trinary map)


def scan_cost(mp, d, scan, poses):
    """Mean distance [m] of the scan end points to the nearest wall, for many poses at once."""
    H, W = mp['img'].shape
    _, sx, sy = scan
    lx, ly, lth = L.compose((poses[:, 0], poses[:, 1], poses[:, 2]), d['laser'])
    c, s = np.cos(lth)[:, None], np.sin(lth)[:, None]
    px = lx[:, None] + c * sx[None, :] - s * sy[None, :]
    py = ly[:, None] + s * sx[None, :] + c * sy[None, :]
    col = ((px - mp['ox']) / mp['res']).astype(int)
    row = (H - 1 - (py - mp['oy']) / mp['res']).astype(int)
    k = (col >= 0) & (col < W) & (row >= 0) & (row < H)
    dist = np.full(px.shape, 0.5)
    dist[k] = mp['dist'][row[k], col[k]]
    return np.minimum(dist, 0.5).mean(axis=1)


def match_start(mp, d):
    """Start pose (x, y, theta) of a run in the map frame, from the scans before it moves."""
    o = d['odom']
    moved = np.hypot(o[:, 1] - o[0, 1], o[:, 2] - o[0, 2]) > 0.01
    t_move = o[np.argmax(moved), 0]
    scans = [sc for sc in d['scans'] if sc[0] < t_move][:20] or d['scans'][:1]

    # coarse search with the first scan over the whole free area and all headings
    xs = np.arange(-1.5, 5.5, 0.1)
    ys = np.arange(-1.5, 5.5, 0.1)
    X, Y = np.meshgrid(xs, ys)
    candidates = []
    for th in np.radians(np.arange(-180, 180, 4)):
        poses = np.c_[X.ravel(), Y.ravel(), np.full(X.size, th)]
        cost = scan_cost(mp, d, scans[0], poses)
        for i in np.argsort(cost)[:3]:
            candidates.append((cost[i], poses[i]))
    candidates.sort(key=lambda c: c[0])

    # fine search around the 5 best candidates, averaged over the standing scans
    best = None
    for _, (x0, y0, t0) in candidates[:5]:
        X2, Y2, T2 = np.meshgrid(np.arange(x0 - 0.15, x0 + 0.15, 0.01),
                                 np.arange(y0 - 0.15, y0 + 0.15, 0.01),
                                 np.radians(np.arange(-6, 6.25, 0.5)) + t0)
        poses = np.c_[X2.ravel(), Y2.ravel(), T2.ravel()]
        cost = np.mean([scan_cost(mp, d, sc, poses) for sc in scans], axis=0)
        i = np.argmin(cost)
        if best is None or cost[i] < best[0]:
            best = (cost[i], tuple(poses[i]))
    return best[1], best[0]


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else OUT
    mp = L.load_map(MAP)
    H, W = mp['img'].shape
    extent = (mp['ox'], mp['ox'] + W * mp['res'], mp['oy'], mp['oy'] + H * mp['res'])

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.4))
    for ax, (name, rec) in zip(axes, RUNS):
        d = L.load(os.path.join(DATA, rec))
        if d['scans']:
            start, cost = match_start(mp, d)
            first_start = start
            label = 'start (scan matched)'
        else:
            # O1 and O2 recorded no laser scans; all runs of 01.10. started from the same point
            start, cost = first_start, float('nan')
            label = 'start (same as M1)'
        path = L.odometry_only(d, start)

        length = np.sum(np.hypot(np.diff(path[:, 1]), np.diff(path[:, 2])))
        col = ((path[:, 1] - mp['ox']) / mp['res']).astype(int)
        row = (H - 1 - (path[:, 2] - mp['oy']) / mp['res']).astype(int)
        k = (col >= 0) & (col < W) & (row >= 0) & (row < H)
        on_wall = 100 * np.mean(mp['dist'][row[k], col[k]] == 0)
        on_unknown = 100 * np.mean(mp['img'][row[k], col[k]] == UNKNOWN)
        print(f'{name} ({rec}): start x={start[0]:.2f} y={start[1]:.2f} '
              f'theta={math.degrees(start[2]):.1f} deg (scan match {100 * cost:.1f} cm), '
              f'{length:.1f} m, {on_wall:.1f} % on walls, {on_unknown:.1f} % unknown')

        ax.imshow(mp['img'], cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper',
                  interpolation='nearest')
        ax.plot(path[:, 1], path[:, 2], '-', color='tab:red', lw=1.2, label='odometry path')
        ax.plot(path[0, 1], path[0, 2], '^', color='green', ms=9, label=label)
        ax.plot(path[-1, 1], path[-1, 2], 's', color='blue', ms=7, label='end')
        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title(f'Run {name}: {length:.1f} m driven\n'
                     f'{on_wall:.1f} % of the path on walls, {on_unknown:.1f} % on unknown cells')
        ax.legend(loc='upper right', fontsize=8, framealpha=0.85)

    fig.tight_layout()
    fig.savefig(out, dpi=200, bbox_inches='tight')
    print(f'Saved {os.path.abspath(out)}')


if __name__ == '__main__':
    main()

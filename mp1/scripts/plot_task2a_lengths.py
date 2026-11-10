#!/usr/bin/env python3
"""
Presentation figure for Task 2a: odometry check of runs O1 (2a_raw_11) and O2 (2a_raw_14).

For each run:
  left   pure odometry path on the merged map MM (map_2a_merged); the segments that were
         measured with a tape measure are numbered and coloured, the rest of the drive is grey
  right  table: measured vs odometry length of each segment

O1 and O2 recorded no laser scans, so they start at the start pose of run M1 (2a_raw_1),
found by scan matching (see plot_task2a_paths.py); all runs of 01.10. started there.
The table values are the tape-measure results (figures/task2a/task2a_length_comparison.md).

Needs ROS 2 sourced (rosbag2_py) and the loader in ~/ros2_ws/results/scripts/load2b.py.

Usage:
    python3 plot_task2a_lengths.py [output.png]
"""

import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_task2a_paths as P  # noqa: E402
L = P.L

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'figures', 'task2a',
                   'task2a_O1_O2_lengths.png')

# tape-measure results: (turn before the segment, measured cm, odometry cm)
RUNS = {
    'O1': ('2a_raw_11', [('--', 192, 187.7), ('left 87°', 70, 66.1)]),
    'O2': ('2a_raw_14', [('left', 91, 91.6), ('right', 120, 112.9), ('left', 130, 125.1),
                         ('right ≈160°', 70, 56.7), ('right*', 192, 205.8)]),
}
COLORS = plt.cm.tab10.colors


def driving_segments(o, v_min=0.03, gap=0.8, min_len=0.5):
    """Index ranges (a, b) of the odometry where the robot translates."""
    t, x, y = o[:, 0], o[:, 1], o[:, 2]
    v = np.hypot(np.diff(x), np.diff(y)) / np.maximum(np.diff(t), 1e-3)
    moving = v > v_min
    phases, i = [], 0
    while i < len(moving):
        if moving[i]:
            j = i
            while j < len(moving) and moving[j]:
                j += 1
            if phases and t[i] - t[phases[-1][1]] < gap:
                phases[-1][1] = j
            else:
                phases.append([i, j])
            i = j
        else:
            i += 1
    return [(a, b) for a, b in phases if t[b] - t[a] > min_len]


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else OUT
    mp = L.load_map(P.MAP)
    H, W = mp['img'].shape
    extent = (mp['ox'], mp['ox'] + W * mp['res'], mp['oy'], mp['oy'] + H * mp['res'])
    start, _ = P.match_start(mp, L.load(os.path.join(P.DATA, '2a_raw_1')))

    fig, axes = plt.subplots(2, 2, figsize=(13, 10), gridspec_kw={'width_ratios': [1, 1.15]})
    for row, (name, (rec, segments)) in enumerate(RUNS.items()):
        d = L.load(os.path.join(P.DATA, rec))
        path = L.odometry_only(d, start)
        phases = driving_segments(d['odom'])[:len(segments)]

        # ------------------------------------------------------------ map
        ax = axes[row, 0]
        ax.imshow(mp['img'], cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper',
                  interpolation='nearest')
        ax.plot(path[:, 1], path[:, 2], '-', color='0.55', lw=1, label='rest of the drive')
        for k, (a, b) in enumerate(phases):
            c = COLORS[k % len(COLORS)]
            ax.plot(path[a:b + 1, 1], path[a:b + 1, 2], '-', color=c, lw=2.5)
            m = (a + b) // 2
            ax.annotate(str(k + 1), (path[m, 1], path[m, 2]), xytext=(6, 6),
                        textcoords='offset points', fontsize=11, fontweight='bold', color=c,
                        bbox=dict(boxstyle='round,pad=0.15', fc='white', ec=c, lw=0.8))
        ax.plot(path[0, 1], path[0, 2], '^', color='green', ms=9, label='start (same as M1)')
        ax.plot([], [], '-', color=COLORS[0], lw=2.5, label='measured segments 1, 2, ...')
        ax.set_xlim(extent[0], extent[1])
        ax.set_ylim(extent[2], extent[3])
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        length = np.sum(np.hypot(np.diff(path[:, 1]), np.diff(path[:, 2])))
        ax.set_title(f'Run {name}: odometry path on the merged map MM ({length:.1f} m driven)')
        ax.legend(loc='upper right', fontsize=8, framealpha=0.85)

        # ------------------------------------------------------------ table
        ax = axes[row, 1]
        ax.axis('off')
        rows, cell_colors = [], []
        for k, (turn, meas, odo) in enumerate(segments):
            err = odo - meas
            rows.append([str(k + 1), turn, f'{meas}', f'{odo:.1f}', f'{err:+.1f}',
                         f'{100 * err / meas:+.1f}'])
            cell_colors.append([(*COLORS[k % len(COLORS)], 0.25)] + ['white'] * 5)
        meas_sum = sum(s[1] for s in segments)
        odo_sum = sum(s[2] for s in segments)
        rows.append(['total', '', f'{meas_sum}', f'{odo_sum:.1f}', f'{odo_sum - meas_sum:+.1f}',
                     f'{100 * (odo_sum - meas_sum) / meas_sum:+.1f}'])
        cell_colors.append(['0.9'] * 6)
        table = ax.table(cellText=rows, cellColours=cell_colors,
                         colLabels=['segment', 'turn before', 'measured\n[cm]',
                                    'odometry\n[cm]', 'error\n[cm]', 'error\n[%]'],
                         loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 1.8)
        for (r, _), cell in table.get_celld().items():
            if r == 0 or r == len(rows):
                cell.set_text_props(fontweight='bold')
        ax.set_title(f'Run {name}: measured vs odometry segment lengths')

    fig.text(0.5, 0.01,
             'Measured: tape measure between the stop points. Odometry: straight-line distance '
             'between start and stop of each segment. Error = odometry - measured.\n'
             '* the robot turned about 130° while driving this segment, so its straight-line '
             'value is less reliable. O1 and O2 recorded no laser scans; they start at the start '
             'pose of run M1.', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(out, dpi=200, bbox_inches='tight')
    print(f'Saved {os.path.abspath(out)}')


if __name__ == '__main__':
    main()

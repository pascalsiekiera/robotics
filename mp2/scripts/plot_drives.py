#!/usr/bin/env python3
"""Task 1a, part 2: summary table and figure of the driven paths.

Reads one or more drives_<tag>.json written by drive_compare.py.

Example:
    python3 plot_drives.py ../data/task1a/drives_dijkstra_dwb.json
"""
import argparse
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from metrics import StaticMap  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FIG_DIR = os.path.join(os.path.dirname(HERE), 'figures')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('files', nargs='+')
    parser.add_argument('--name', default=None, help='output file name (default: from the tag)')
    args = parser.parse_args()

    runs, pairs = [], {}
    for path in args.files:
        with open(path) as f:
            data = json.load(f)
        runs += data['runs']
        pairs.update(data['pairs'])

    # deviation from the planned path: est = estimated pose (controller error),
    # true = ground truth (controller + localization error); mean / max in m
    print(f'{"tag":<16}{"pair":<6}{"ok":<4}{"plans":>6}{"planned":>9}{"driven":>8}{"time":>7}'
          f'{"dev est":>14}{"dev true":>14}{"loc err":>14}{"final":>7}')
    print(f'{"":<16}{"":<6}{"":<4}{"":>6}{"[m]":>9}{"[m]":>8}{"[s]":>7}'
          f'{"mean/max [m]":>14}{"mean/max [m]":>14}{"mean/max [m]":>14}{"[m]":>7}')
    for r in runs:
        if 'deviation_est_mean_m' not in r:
            print(f'{r["tag"]:<16}{r["pair"]:<6}{"no":<4}  ({r.get("note", "no data")})')
            continue
        print(f'{r["tag"]:<16}{r["pair"]:<6}{"yes" if r["success"] else "no":<4}{r["n_plans"]:>6}'
              f'{r["planned_length_m"]:>9.2f}{r["driven_length_m"]:>8.2f}{r["duration_sim_s"]:>7.1f}'
              f'{r["deviation_est_mean_m"]:>8.3f}/{r["deviation_est_max_m"]:.3f}'
              f'{r["deviation_true_mean_m"]:>8.3f}/{r["deviation_true_max_m"]:.3f}'
              f'{r["localization_error_mean_m"]:>8.3f}/{r["localization_error_max_m"]:.3f}'
              f'{r["final_error_m"]:>7.2f}')

    static_map = StaticMap()
    shown = np.where(static_map.occupied, 0.0, np.where(static_map.unknown, 0.8, 1.0))
    tags = list(dict.fromkeys(r['tag'] for r in runs))
    fig, axes = plt.subplots(len(tags), len(pairs), figsize=(4.2 * len(pairs), 4.4 * len(tags)),
                             squeeze=False)
    for row, tag in enumerate(tags):
        for ax, (pair_name, pair) in zip(axes[row], pairs.items()):
            ax.imshow(shown, cmap='gray', vmin=0, vmax=1, extent=static_map.extent())
            run = next((r for r in runs if r['tag'] == tag and r['pair'] == pair_name
                        and 'planned' in r), None)
            if run is not None:
                planned = np.array(run['planned'])
                gt = np.array(run['ground_truth'])
                ax.plot(planned[:, 0], planned[:, 1], '-', color='tab:blue', lw=2.2, label='planned')
                est = np.array(run['estimated'])
                ax.plot(est[:, 1], est[:, 2], ':', color='tab:green', lw=1.8, label='driven (estimated)')
                ax.plot(gt[:, 1], gt[:, 2], '--', color='tab:red', lw=1.6, label='driven (ground truth)')
                ax.set_title(f'{pair_name} ({tag}): deviation est {run["deviation_est_mean_m"] * 100:.1f} cm, '
                             f'true {run["deviation_true_mean_m"] * 100:.1f} cm (mean)', fontsize=8)
            else:
                ax.set_title(f'{pair_name} ({tag}): no data', fontsize=8)
            ax.plot(*pair['start'], 'ko', ms=6)
            ax.plot(*pair['goal'], 'k*', ms=11)
            ax.set_xlim(-3.1, 3.1)
            ax.set_ylim(-2.8, 2.8)
            ax.set_aspect('equal')
            ax.set_xlabel('x [m]')
            ax.legend(fontsize=7, loc='lower right')
        axes[row][0].set_ylabel('y [m]')
    fig.tight_layout()

    name = args.name or 'drives_' + '_'.join(tags)
    os.makedirs(FIG_DIR, exist_ok=True)
    out = os.path.join(FIG_DIR, f'{name}.png')
    fig.savefig(out, dpi=150)
    print(f'saved {out}')


if __name__ == '__main__':
    main()

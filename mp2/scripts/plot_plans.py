#!/usr/bin/env python3
"""Task 1a, part 1: summary table and figure of the planned paths.

Reads one or more plans_<tag>.json written by plan_compare.py.

Example:
    python3 plot_plans.py ../data/task1a/plans_baseline.json
"""
import argparse
import json
import os
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from metrics import StaticMap  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FIG_DIR = os.path.join(os.path.dirname(HERE), 'figures')
# planner ids in nav2_params.yaml -> names shown in tables and figures
PLANNER_NAMES = {'GridBased': 'Dijkstra', 'AStar': 'A*'}


def mean_std(values):
    return f'{np.mean(values):.3f} ± {np.std(values):.3f}' if values else '-'


def median_iqr(values):
    # planning times have single outliers (OS scheduling), so median and quartiles
    if not values:
        return '-'
    q1, med, q3 = np.percentile(values, [25, 50, 75])
    return f'{med:.2f} [{q1:.2f}-{q3:.2f}]'


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('files', nargs='+')
    parser.add_argument('--name', default=None, help='output file name (default: from the tag)')
    parser.add_argument('--planners', nargs='+', default=None,
                        help='only show these planner ids (default: all)')
    args = parser.parse_args()

    runs, pairs = [], {}
    for path in args.files:
        with open(path) as f:
            data = json.load(f)
        runs += data['runs']
        pairs.update(data['pairs'])
    if args.planners:
        runs = [r for r in runs if r['planner'] in args.planners]
    for r in runs:
        r['planner'] = PLANNER_NAMES.get(r['planner'], r['planner'])

    # summary: one row per (tag, pair, planner)
    groups = defaultdict(list)
    for r in runs:
        groups[(r['tag'], r['pair'], r['planner'])].append(r)
    print(f'{"tag":<12}{"pair":<6}{"planner":<12}{"ok":>6}  {"length [m]":<16}'
          f'{"time [ms] med [IQR]":<20}{"min clearance [m]":<16}')
    for (tag, pair, planner), rs in sorted(groups.items(), key=lambda g: (g[0][1], g[0][2], g[0][0])):
        ok = [r for r in rs if r['success']]
        print(f'{tag:<12}{pair:<6}{planner:<12}{len(ok):>3}/{len(rs):<2}  '
              f'{mean_std([r["length_m"] for r in ok]):<16}'
              f'{median_iqr([r["planning_time_ms"] for r in ok]):<20}'
              f'{mean_std([r["min_clearance_m"] for r in ok]):<16}')

    # figure: one panel per pair, first successful path of every (tag, planner)
    static_map = StaticMap()
    shown = np.where(static_map.occupied, 0.0, np.where(static_map.unknown, 0.8, 1.0))
    variants = list(dict.fromkeys((r['tag'], r['planner']) for r in runs))
    colors = plt.cm.tab10(np.arange(len(variants)))
    styles = ['-', '--', ':', '-.']
    fig, axes = plt.subplots(1, len(pairs), figsize=(4.2 * len(pairs), 4.4), squeeze=False)
    for ax, (pair_name, pair) in zip(axes[0], pairs.items()):
        ax.imshow(shown, cmap='gray', vmin=0, vmax=1, extent=static_map.extent())
        for k, (tag, planner) in enumerate(variants):
            ok = [r for r in runs if r['pair'] == pair_name and r['tag'] == tag
                  and r['planner'] == planner and r['success']]
            if not ok:
                continue
            xy = np.array(ok[0]['path'])
            label = planner if len({t for t, _ in variants}) == 1 else f'{tag} / {planner}'
            ax.plot(xy[:, 0], xy[:, 1], styles[k % len(styles)], color=colors[k], lw=1.8,
                    label=f'{label}: {ok[0]["length_m"]:.2f} m')
        ax.plot(*pair['start'], 'ko', ms=6)
        ax.plot(*pair['goal'], 'k*', ms=11)
        ax.set_title(pair_name)
        ax.set_xlim(-3.1, 3.1)
        ax.set_ylim(-2.8, 2.8)
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
        ax.legend(fontsize=7, loc='lower right')
    axes[0][0].set_ylabel('y [m]')
    fig.tight_layout()

    name = args.name or 'plans_' + '_'.join(sorted({r['tag'] for r in runs}))
    os.makedirs(FIG_DIR, exist_ok=True)
    out = os.path.join(FIG_DIR, f'{name}.png')
    fig.savefig(out, dpi=150)
    print(f'saved {out}')


if __name__ == '__main__':
    main()

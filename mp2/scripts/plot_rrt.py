#!/usr/bin/env python3
"""Task 1b: summary table and figures of the RRT plans (plans_<tag>.json from plan_compare.py).

Compares every RRT configuration with Dijkstra (Task 1a baseline, deterministic: one path per pair).
Figures:
  rrt_paths_<tag>.png  all successful paths of one configuration per pair, Dijkstra for reference
  rrt_summary.png      success rate, length, min. clearance, iterations per configuration and pair

Example:
    python3 plot_rrt.py mp2/data/task1b/plans_rrt_*.json
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
MP2 = os.path.dirname(HERE)
FIG_DIR = os.path.join(MP2, 'figures')
DIJKSTRA = os.path.join(MP2, 'data', 'task1a', 'plans_baseline.json')
# tag -> label in tables and figures (order = order in the summary figure)
CONFIG_NAMES = {'rrt_base': 'baseline', 'rrt_step0.5': 'step 0.5 m', 'rrt_bias0.1': 'goal bias 0.1',
                'rrt_thr128': 'threshold 128', 'rrt_smooth': 'smoothing'}
RRT_COLOR = '#1f77b4'
REF_COLOR = '#d62728'


def mean_std(values, fmt='.2f'):
    return f'{np.mean(values):{fmt}} ± {np.std(values):{fmt}}' if values else '-'


def median_iqr(values, fmt='.1f'):
    if not values:
        return '-'
    q1, med, q3 = np.percentile(values, [25, 50, 75])
    return f'{med:{fmt}} [{q1:{fmt}}-{q3:{fmt}}]'


def load_dijkstra():
    with open(DIJKSTRA) as f:
        runs = json.load(f)['runs']
    ref = {}
    for r in runs:
        if r['planner'] == 'GridBased' and r['success'] and r['pair'] not in ref:
            ref[r['pair']] = r
    return ref


def print_table(tags, pairs, runs, ref):
    print(f'{"config":<15}{"pair":<5}{"ok":>7}  {"length [m] mean±sd [min-max]":<30}'
          f'{"time [ms] med [IQR]":<22}{"clearance [m] mean (min)":<26}{"iterations med [IQR]":<22}'
          f'{"nodes med":<10}')
    for pair in pairs:
        d = ref.get(pair)
        if d:
            print(f'{"Dijkstra":<15}{pair:<5}{"":>7}  {d["length_m"]:<30.2f}{"":<22}'
                  f'{d["min_clearance_m"]:<26.2f}')
        for tag in tags:
            rs = [r for r in runs if r['tag'] == tag and r['pair'] == pair]
            ok = [r for r in rs if r['success']]
            lengths = [r['length_m'] for r in ok]
            clear = [r['min_clearance_m'] for r in ok]
            length = (f'{mean_std(lengths)} [{min(lengths):.2f}-{max(lengths):.2f}]'
                      if ok else '-')
            clearance = f'{np.mean(clear):.2f} ({min(clear):.2f})' if ok else '-'
            # iterations and nodes of all plans (failed ones ran up to max_num_nodes)
            its = [r['iterations'] for r in rs if 'iterations' in r]
            nodes = [r['tree_nodes'] for r in rs if 'tree_nodes' in r]
            print(f'{CONFIG_NAMES.get(tag, tag):<15}{pair:<5}{len(ok):>3}/{len(rs):<3}  {length:<30}'
                  f'{median_iqr([r["planning_time_ms"] for r in ok]):<22}{clearance:<26}'
                  f'{median_iqr(its, ".0f"):<22}{(f"{np.median(nodes):.0f}" if nodes else "-"):<10}')
        print()


def plot_paths(tag, pairs, runs, ref, static_map):
    shown = np.where(static_map.occupied, 0.0, np.where(static_map.unknown, 0.8, 1.0))
    fig, axes = plt.subplots(1, len(pairs), figsize=(4.2 * len(pairs), 4.4), squeeze=False)
    for ax, (pair_name, pair) in zip(axes[0], pairs.items()):
        ax.imshow(shown, cmap='gray', vmin=0, vmax=1, extent=static_map.extent())
        rs = [r for r in runs if r['tag'] == tag and r['pair'] == pair_name]
        ok = [r for r in rs if r['success']]
        for k, r in enumerate(ok):
            xy = np.array(r['path'])
            ax.plot(xy[:, 0], xy[:, 1], '-', color=RRT_COLOR, lw=0.8, alpha=0.35,
                    label=f'RRT ({len(ok)} paths)' if k == 0 else None)
        if pair_name in ref:
            xy = np.array(ref[pair_name]['path'])
            ax.plot(xy[:, 0], xy[:, 1], '--', color=REF_COLOR, lw=1.8,
                    label=f'Dijkstra: {ref[pair_name]["length_m"]:.2f} m')
        ax.plot(*pair['start'], 'ko', ms=6)
        ax.plot(*pair['goal'], 'k*', ms=11)
        mean = f', mean {np.mean([r["length_m"] for r in ok]):.2f} m' if ok else ''
        ax.set_title(f'{pair_name}: {len(ok)}/{len(rs)} found{mean}', fontsize=10)
        ax.set_xlim(-3.1, 3.1)
        ax.set_ylim(-2.8, 2.8)
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
        ax.legend(fontsize=7, loc='lower right')
    axes[0][0].set_ylabel('y [m]')
    fig.suptitle(f'RRT, {CONFIG_NAMES.get(tag, tag)}', fontsize=11)
    fig.tight_layout()
    out = os.path.join(FIG_DIR, f'rrt_paths_{tag.removeprefix("rrt_")}.png')
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f'saved {out}')


def plot_summary(tags, pairs, runs, ref):
    # rows: one metric each (one y-axis per panel), columns: pairs
    rows = [('success rate [%]', None), ('path length [m]', 'length_m'),
            ('min. clearance [m]', 'min_clearance_m'), ('iterations', 'iterations')]
    labels = [CONFIG_NAMES.get(t, t) for t in tags]
    x = np.arange(len(tags))
    fig, axes = plt.subplots(len(rows), len(pairs), figsize=(4.0 * len(pairs), 2.6 * len(rows)),
                             sharex=True, squeeze=False)
    for col, pair in enumerate(pairs):
        for row, (ylabel, key) in enumerate(rows):
            ax = axes[row][col]
            groups = [[r for r in runs if r['tag'] == t and r['pair'] == pair] for t in tags]
            if key is None:
                rate = [100 * np.mean([r['success'] for r in g]) if g else 0 for g in groups]
                ax.bar(x, rate, width=0.6, color=RRT_COLOR)
                for xi, v in zip(x, rate):
                    ax.text(xi, v + 2, f'{v:.0f}', ha='center', va='bottom', fontsize=7)
                ax.set_ylim(0, 115)
            else:
                # iterations of all plans (failed ones reach max_num_nodes), the rest of the successful
                values = [[r[key] for r in g if key in r and (key == 'iterations' or r['success'])]
                          for g in groups]
                ax.boxplot([v if v else [np.nan] for v in values], positions=x, widths=0.5,
                           showfliers=True, flierprops={'markersize': 2},
                           medianprops={'color': 'black'},
                           boxprops={'color': RRT_COLOR}, whiskerprops={'color': RRT_COLOR},
                           capprops={'color': RRT_COLOR})
                if key in ('length_m', 'min_clearance_m') and pair in ref:
                    ax.axhline(ref[pair][key], color=REF_COLOR, ls='--', lw=1.2,
                               label=f'Dijkstra {ref[pair][key]:.2f}')
                    ax.legend(fontsize=7, loc='upper right')
            ax.grid(axis='y', color='0.9')
            ax.set_axisbelow(True)
            if col == 0:
                ax.set_ylabel(ylabel)
            if row == 0:
                ax.set_title(pair)
        axes[-1][col].set_xticks(x)
        axes[-1][col].set_xticklabels(labels, rotation=30, ha='right', fontsize=8)
    fig.tight_layout()
    out = os.path.join(FIG_DIR, 'rrt_summary.png')
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f'saved {out}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('files', nargs='+')
    args = parser.parse_args()

    runs, pairs = [], {}
    for path in args.files:
        with open(path) as f:
            data = json.load(f)
        runs += data['runs']
        pairs.update(data['pairs'])
    present = list(dict.fromkeys(r['tag'] for r in runs))
    tags = [t for t in CONFIG_NAMES if t in present] + [t for t in present if t not in CONFIG_NAMES]
    ref = load_dijkstra()

    print_table(tags, pairs, runs, ref)
    os.makedirs(FIG_DIR, exist_ok=True)
    static_map = StaticMap()
    for tag in tags:
        plot_paths(tag, pairs, runs, ref, static_map)
    plot_summary(tags, list(pairs), runs, ref)


if __name__ == '__main__':
    main()

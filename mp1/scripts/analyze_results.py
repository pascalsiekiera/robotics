#!/usr/bin/env python3
"""
Evaluate the CSV files written by evaluator_node (tasks 1a, 1b, 1c).

Every CSV row compares ONE estimator with the ground truth at ONE instant:
    t, estimator, x, y, yaw, gt_x, gt_y, gt_yaw, error_xy, error_yaw,
    sigma_x, sigma_y, sigma_yaw

Usage (from ~/ros2_ws/src/robotics, after `source ~/ros2_ws/install/setup.bash`
if you want the map in the background):

    # one run
    python3 mp1/scripts/analyze_results.py ~/ros2_ws/results/task1c_XXX.csv:"AMCL default" \
            --map mp1/maps/map_1b.yaml --prefix task1c

    # compare runs (parameter study); odometry is the same in every run, so hide it
    python3 mp1/scripts/analyze_results.py a.csv:"2000 particles" b.csv:"100 particles" \
            --no-odometry --prefix task1c_particles

Each CSV can get a label after a colon (FILE.csv:LABEL); otherwise the file name is used.

Outputs (in --out-dir, default mp1/figures):
    <prefix>_summary.md       table with the error statistics (paste into the report)
    <prefix>_trajectory.png   estimated paths vs ground truth (over the map with --map)
    <prefix>_error_time.png   position and heading error over time
    <prefix>_error_cdf.png    error distribution: "x % of the time the error is below y cm"
"""

import argparse
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')                   # write PNG files, no window needed
import matplotlib.pyplot as plt         # noqa: E402
import numpy as np                      # noqa: E402

# The mocap lost the robot here (see ground_truth_node.py); errors in this
# window say more about the ground truth than about the estimator.
MOCAP_DROPOUT = (74.5, 77.5)

COLORS = {'ground truth': 'tab:green', 'odometry': 'goldenrod', 'ekf': 'tab:red',
          'robot_localization': 'tab:blue', 'amcl': 'tab:red', 'slam_toolbox': 'tab:red'}


# --------------------------------------------------------------------- loading

def parse_input(arg):
    """'file.csv:Label' -> (path, label). Without a label, the file name is used."""
    if os.path.exists(arg) or ':' not in arg:
        path, label = arg, None
    else:
        path, label = arg.rsplit(':', 1)
    path = os.path.expanduser(path)
    if not label:
        label = os.path.splitext(os.path.basename(path))[0]
    return path, label


def load_run(path):
    """Read one CSV into {estimator name: {column: numpy array}}."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f'{path} is empty')

    t0 = min(float(r['t']) for r in rows)          # bag time -> seconds since start
    columns = [c for c in rows[0] if c != 'estimator']
    estimators = {}
    for name in dict.fromkeys(r['estimator'] for r in rows):   # keeps the file order
        selected = [r for r in rows if r['estimator'] == name]
        data = {c: np.array([float(r[c]) for r in selected]) for c in columns}
        data['t'] = data['t'] - t0
        estimators[name] = data
    return estimators


def exclude_window(data, window):
    """Drop the samples with start <= t <= end."""
    if window is None:
        return data
    keep = (data['t'] < window[0]) | (data['t'] > window[1])
    return {c: v[keep] for c, v in data.items()}


def position_sigma(data):
    """sqrt(sigma_x^2 + sigma_y^2); NaN where no uncertainty was reported (0 or NaN)."""
    s = np.sqrt(data['sigma_x'] ** 2 + data['sigma_y'] ** 2)
    return np.where(np.isfinite(s) & (s > 0), s, np.nan)


def has_sigma(data):
    """True if the estimator reported a real uncertainty (odometry: 0, SLAM: NaN)."""
    return bool(np.any(np.isfinite(position_sigma(data))))


# ------------------------------------------------------------------ statistics

def statistics(data):
    e = data['error_xy'] * 100                       # m -> cm
    stats = {
        'n': len(e),
        'RMSE [cm]': math.sqrt(np.mean(e ** 2)),
        'mean [cm]': np.mean(e),
        'median [cm]': np.median(e),
        'p95 [cm]': np.percentile(e, 95),
        'max [cm]': np.max(e),
        'final [cm]': e[-1],
        'yaw RMSE [deg]': math.degrees(math.sqrt(np.mean(data['error_yaw'] ** 2))),
        '< 5 cm [%]': 100 * np.mean(e < 5),
    }
    if has_sigma(data):
        # consistency: a well-tuned filter has ~95 % of its errors inside 2 sigma.
        # Only samples where an uncertainty was reported (AMCL: after its first update).
        sigma = position_sigma(data)
        known = np.isfinite(sigma)
        stats['inside 2σ [%]'] = 100 * np.mean(data['error_xy'][known] <= 2 * sigma[known])
    return stats


def summary_table(runs, window):
    """Markdown table, one line per (run, estimator)."""
    keys = ['n', 'RMSE [cm]', 'mean [cm]', 'median [cm]', 'p95 [cm]', 'max [cm]',
            'final [cm]', 'yaw RMSE [deg]', '< 5 cm [%]', 'inside 2σ [%]']
    lines = []
    if window:
        lines.append(f'Samples between t = {window[0]} s and {window[1]} s excluded.\n')
    lines.append('| run | estimator | ' + ' | '.join(keys) + ' |')
    lines.append('|' + '---|' * (len(keys) + 2))
    for label, estimators in runs:
        for name, data in estimators.items():
            s = statistics(exclude_window(data, window))
            cells = [f'{s[k]:d}' if k == 'n' else (f'{s[k]:.2f}' if k in s else '–')
                     for k in keys]
            lines.append(f'| {label} | {name} | ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines)


# ----------------------------------------------------------------------- plots

def curve_label(runs, label, name):
    """Only mention the run label when several runs are plotted together."""
    return f'{label}: {name}' if len(runs) > 1 else name


def draw_map(ax, map_yaml):
    try:
        from ekf.compare_maps import GridMap, FREE, OCCUPIED
    except ImportError:
        print('Map background skipped: run  source ~/ros2_ws/install/setup.bash  first.')
        return
    grid = GridMap(map_yaml)
    image = np.full(grid.state.shape, 0.8)            # unknown: gray
    image[grid.state == FREE] = 1.0                    # free: white
    image[grid.state == OCCUPIED] = 0.2                # walls: dark
    h, w = grid.state.shape
    extent = [grid.origin[0], grid.origin[0] + w * grid.resolution,
              grid.origin[1], grid.origin[1] + h * grid.resolution]
    ax.imshow(image, cmap='gray', vmin=0, vmax=1, origin='lower', extent=extent)


def plot_trajectory(runs, map_yaml, filename):
    fig, ax = plt.subplots(figsize=(7, 7))
    if map_yaml:
        draw_map(ax, map_yaml)

    # the ground truth is the same in every run: draw it once
    first = next(iter(runs[0][1].values()))
    ax.plot(first['gt_x'], first['gt_y'], color=COLORS['ground truth'], lw=2.5,
            label='ground truth')
    for label, estimators in runs:
        for name, data in estimators.items():
            color = COLORS.get(name) if len(runs) == 1 else None
            ax.plot(data['x'], data['y'], lw=1.2, color=color,
                    label=curve_label(runs, label, name))
    ax.plot(first['gt_x'][0], first['gt_y'][0], 'k+', markersize=12, label='start')

    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Estimated paths vs ground truth (map frame)')
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(filename, dpi=150)
    plt.close(fig)


def plot_error_time(runs, filename):
    fig, (ax_pos, ax_yaw) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for ax in (ax_pos, ax_yaw):
        ax.axvspan(*MOCAP_DROPOUT, color='gray', alpha=0.25,
                   label='ground truth unreliable' if ax is ax_pos else None)

    for label, estimators in runs:
        for name, data in estimators.items():
            color = COLORS.get(name) if len(runs) == 1 else None
            text = curve_label(runs, label, name)
            line, = ax_pos.plot(data['t'], data['error_xy'] * 100, lw=0.8, color=color,
                                label=text)
            if has_sigma(data):
                # the estimator's own claim: "the error should be below this ~95 % of the time"
                ax_pos.plot(data['t'], 2 * position_sigma(data) * 100, '--', lw=0.8,
                            color=line.get_color(), label=f'{text}: 2σ bound')
            ax_yaw.plot(data['t'], np.degrees(data['error_yaw']), lw=0.8,
                        color=line.get_color())

    ax_pos.set_ylabel('position error [cm]')
    ax_yaw.set_ylabel('heading error [deg]')
    ax_yaw.set_xlabel('time since start of the bag [s]')
    ax_pos.set_title('Error vs ground truth over time')
    ax_pos.legend(loc='upper left', fontsize=8)
    for ax in (ax_pos, ax_yaw):
        ax.grid(alpha=0.3)
        ax.set_ylim(bottom=0)
    fig.tight_layout()
    fig.savefig(filename, dpi=150)
    plt.close(fig)


def plot_error_cdf(runs, window, filename):
    fig, ax = plt.subplots(figsize=(7, 5))
    for label, estimators in runs:
        for name, data in estimators.items():
            e = np.sort(exclude_window(data, window)['error_xy'] * 100)
            fraction = np.arange(1, len(e) + 1) / len(e) * 100
            color = COLORS.get(name) if len(runs) == 1 else None
            ax.plot(e, fraction, lw=1.5, color=color, label=curve_label(runs, label, name))

    ax.axhline(95, color='gray', ls=':', lw=1)
    ax.set_xlabel('position error [cm]')
    ax.set_ylabel('share of time with smaller error [%]')
    ax.set_title('Error distribution (curve further left = better)')
    ax.set_ylim(0, 101)
    ax.set_xlim(left=0)
    ax.grid(alpha=0.3)
    ax.legend(loc='lower right', fontsize=8)
    fig.tight_layout()
    fig.savefig(filename, dpi=150)
    plt.close(fig)


# ------------------------------------------------------------------------ main

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csv', nargs='+', help='result CSV(s), optionally FILE.csv:LABEL')
    parser.add_argument('--map', help='map yaml drawn under the trajectories')
    parser.add_argument('--out-dir', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'figures'))
    parser.add_argument('--prefix', default='results', help='start of the output file names')
    parser.add_argument('--exclude', metavar='START-END',
                        help='leave a time window out of the statistics, e.g. 74-84')
    parser.add_argument('--no-odometry', action='store_true',
                        help='hide the raw odometry baseline')
    args = parser.parse_args()

    window = tuple(float(v) for v in args.exclude.split('-')) if args.exclude else None

    runs = []                                    # [(label, {estimator: data})]
    for arg in args.csv:
        path, label = parse_input(arg)
        estimators = load_run(path)
        if args.no_odometry:
            estimators.pop('odometry', None)
        runs.append((label, estimators))

    os.makedirs(args.out_dir, exist_ok=True)
    out = os.path.join(os.path.abspath(args.out_dir), args.prefix)

    table = summary_table(runs, window)
    print(table)
    with open(out + '_summary.md', 'w') as f:
        f.write(table + '\n')

    plot_trajectory(runs, args.map, out + '_trajectory.png')
    plot_error_time(runs, out + '_error_time.png')
    plot_error_cdf(runs, window, out + '_error_cdf.png')
    print(f'\nWritten: {out}_summary.md, _trajectory.png, _error_time.png, _error_cdf.png')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Task 1a, part 1: control run - planning-only setup vs. full simulation.

Compares the paths of two plan_compare.py runs (default: plans_baseline = planning-only
setup, plans_fullsim = same requests in the full Gazebo simulation with the robot localized
at the spawn pose). For every pair and planner: length of both paths, distance between the
paths (each point of the full-simulation path to the nearest point of the planning-only path),
and the planning times of both setups.

Example:
    python3 compare_setups.py
"""
import argparse
import json
import os

import numpy as np

from metrics import deviation

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(os.path.dirname(HERE), 'data', 'task1a')
PLANNER_NAMES = {'GridBased': 'Dijkstra', 'AStar': 'A*'}


def first_paths(path):
    with open(path) as f:
        runs = json.load(f)['runs']
    out = {}
    for r in runs:
        if r['success']:
            key = (r['pair'], r['planner'])
            out.setdefault(key, {'path': np.array(r['path']), 'length': r['length_m'], 'times': []})
            out[key]['times'].append(r['planning_time_ms'])
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--reference', default=os.path.join(DATA, 'plans_baseline.json'))
    parser.add_argument('--other', default=os.path.join(DATA, 'plans_fullsim.json'))
    args = parser.parse_args()

    ref, other = first_paths(args.reference), first_paths(args.other)
    print(f'{"pair":<6}{"planner":<10}{"length ref/other [m]":>22}{"path distance mean/max [cm]":>30}'
          f'{"time ref [ms]":>15}{"time other [ms]":>17}')
    for key in sorted(ref):
        if key not in other:
            continue
        a, b = ref[key], other[key]
        d = deviation(b['path'], a['path']) * 100
        t_oth = sorted(set(np.round(b['times'], 2)))
        print(f'{key[0]:<6}{PLANNER_NAMES.get(key[1], key[1]):<10}'
              f'{a["length"]:>13.3f} / {b["length"]:.3f}{d.mean():>18.1f} / {d.max():.1f}'
              f'{np.median(a["times"]):>15.2f}{np.median(b["times"]):>17.2f}'
              f'   other: distinct values {t_oth}')


if __name__ == '__main__':
    main()

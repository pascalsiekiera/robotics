#!/usr/bin/env python3
"""
Compare two occupancy-grid maps (map_saver format: .yaml + .pgm).

Usage
    ros2 run ekf compare_maps  MAP_A.yaml  MAP_B.yaml  [--out overlay.png] [--align]
                               [--label-a 'A (own map)'] [--label-b 'B (provided map)']

Both maps are placed in world coordinates using their yaml origin and
resolution (for this dataset both have the robot's start pose at the origin).

Metrics, on the occupied cells (walls/obstacles):
    A -> B distance   for every occupied cell of A, distance to the nearest
                      occupied cell of B (mean, median, % within 10 cm)
    B -> A distance   the same the other way round
    The mean of both is the symmetric "chamfer" distance: 0 means the walls
    coincide, it grows with misalignment, distortion or missing/extra walls.
    Agreement         on cells that are known (free or occupied) in both maps,
                      fraction that have the same state

--align searches a small rigid offset (dx, dy, dyaw) of B that minimises the
chamfer distance, to tell a global offset apart from real shape differences.
"""

import argparse
import math
import os

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

FREE, UNKNOWN, OCCUPIED = 0, 1, 2


class GridMap:

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            info = yaml.safe_load(f)
        image_path = os.path.join(os.path.dirname(yaml_path), info['image'])
        pixels = np.asarray(Image.open(image_path).convert('L'), dtype=float)
        # probability of being occupied, as map_server interprets the image
        p = pixels / 255.0 if info.get('negate', 0) else (255.0 - pixels) / 255.0
        self.state = np.full(p.shape, UNKNOWN, dtype=np.uint8)
        self.state[p > info['occupied_thresh']] = OCCUPIED
        self.state[p < info['free_thresh']] = FREE
        self.state = np.flipud(self.state)   # row 0 = lowest y, like world coordinates
        self.resolution = float(info['resolution'])
        self.origin = np.array(info['origin'][:2], dtype=float)
        self.name = os.path.basename(yaml_path)

    def cells_to_world(self, rows, cols):
        return np.stack([self.origin[0] + (cols + 0.5) * self.resolution,
                         self.origin[1] + (rows + 0.5) * self.resolution], axis=1)

    def world_to_cells(self, xy):
        cols = np.floor((xy[:, 0] - self.origin[0]) / self.resolution).astype(int)
        rows = np.floor((xy[:, 1] - self.origin[1]) / self.resolution).astype(int)
        return rows, cols

    def occupied_points(self):
        rows, cols = np.nonzero(self.state == OCCUPIED)
        return self.cells_to_world(rows, cols)

    def distance_to_occupied(self, xy):
        """Distance [m] from each point to the nearest occupied cell (inf outside map)."""
        if not hasattr(self, '_dist'):
            self._dist = distance_transform_edt(self.state != OCCUPIED) * self.resolution
        rows, cols = self.world_to_cells(xy)
        inside = (rows >= 0) & (rows < self.state.shape[0]) & \
                 (cols >= 0) & (cols < self.state.shape[1])
        d = np.full(len(xy), np.inf)
        d[inside] = self._dist[rows[inside], cols[inside]]
        return d

    def state_at(self, xy):
        rows, cols = self.world_to_cells(xy)
        inside = (rows >= 0) & (rows < self.state.shape[0]) & \
                 (cols >= 0) & (cols < self.state.shape[1])
        s = np.full(len(xy), UNKNOWN, dtype=np.uint8)
        s[inside] = self.state[rows[inside], cols[inside]]
        return s


def transform(xy, dx, dy, dyaw):
    c, s = math.cos(dyaw), math.sin(dyaw)
    return xy @ np.array([[c, s], [-s, c]]) + [dx, dy]


def chamfer(a, b, pts_a, pts_b, offset):
    """Mean A->B and B->A wall distance, with map B moved by offset."""
    dx, dy, dyaw = offset
    d_ab = b.distance_to_occupied(transform(pts_a, *_inverse(dx, dy, dyaw)))
    d_ba = a.distance_to_occupied(transform(pts_b, dx, dy, dyaw))
    return d_ab, d_ba


def _inverse(dx, dy, dyaw):
    c, s = math.cos(dyaw), math.sin(dyaw)
    return (-(c * dx + s * dy), -(-s * dx + c * dy), -dyaw)


def describe(d, label):
    d = d[np.isfinite(d)]
    return (f'{label}: mean {d.mean() * 100:5.1f} cm, median {np.median(d) * 100:5.1f} cm, '
            f'{np.mean(d <= 0.10) * 100:5.1f} % within 10 cm  (n={len(d)})')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_a')
    parser.add_argument('map_b')
    parser.add_argument('--out', default='map_comparison.png', help='overlay image')
    parser.add_argument('--label-a', default='A (own map)', help='legend text for map A')
    parser.add_argument('--label-b', default='B (provided map)', help='legend text for map B')
    parser.add_argument('--align', action='store_true',
                        help='also search the best rigid offset of map B')
    args = parser.parse_args()

    a, b = GridMap(args.map_a), GridMap(args.map_b)
    pts_a, pts_b = a.occupied_points(), b.occupied_points()
    print(f'A = {a.name}: {a.state.shape[1]}x{a.state.shape[0]} cells @ {a.resolution} m, '
          f'{len(pts_a)} occupied')
    print(f'B = {b.name}: {b.state.shape[1]}x{b.state.shape[0]} cells @ {b.resolution} m, '
          f'{len(pts_b)} occupied')

    offset = (0.0, 0.0, 0.0)
    d_ab, d_ba = chamfer(a, b, pts_a, pts_b, offset)
    print('\nAs saved (same frame):')
    print('  ' + describe(d_ab, 'A -> B'))
    print('  ' + describe(d_ba, 'B -> A'))

    if args.align:
        best = None
        for dyaw in np.radians(np.arange(-10, 10.5, 1.0)):
            for dx in np.arange(-0.3, 0.31, 0.05):
                for dy in np.arange(-0.3, 0.31, 0.05):
                    d1, d2 = chamfer(a, b, pts_a, pts_b, (dx, dy, dyaw))
                    cost = np.mean(np.minimum(d1, 1.0)) + np.mean(np.minimum(d2, 1.0))
                    if best is None or cost < best[0]:
                        best = (cost, (dx, dy, dyaw))
        offset = best[1]
        d_ab, d_ba = chamfer(a, b, pts_a, pts_b, offset)
        print(f'\nBest rigid offset of B: dx={offset[0]:+.2f} m, dy={offset[1]:+.2f} m, '
              f'dyaw={math.degrees(offset[2]):+.1f} deg')
        print('  ' + describe(d_ab, 'A -> B'))
        print('  ' + describe(d_ba, 'B -> A'))

    # agreement on cells known in both maps, evaluated on A's grid
    rows, cols = np.nonzero(a.state != UNKNOWN)
    xy = a.cells_to_world(rows, cols)
    state_b = b.state_at(transform(xy, *_inverse(*offset)))
    known = state_b != UNKNOWN
    agree = np.mean(a.state[rows[known], cols[known]] == state_b[known])
    print(f'\nCells known in both maps: {known.sum()}, same state (free/occupied): '
          f'{agree * 100:.1f} %')

    save_overlay(a, pts_a, transform(pts_b, *offset), args.out, args.label_a, args.label_b)
    print(f'Overlay written to {os.path.abspath(args.out)}')


def save_overlay(a, pts_a, pts_b, filename, label_a, label_b):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 7))
    free = a.state == FREE
    extent = [a.origin[0], a.origin[0] + a.state.shape[1] * a.resolution,
              a.origin[1], a.origin[1] + a.state.shape[0] * a.resolution]
    ax.imshow(np.where(free, 0.93, 0.8), cmap='gray', vmin=0, vmax=1, origin='lower',
              extent=extent)
    ax.scatter(pts_b[:, 0], pts_b[:, 1], s=2, c='tab:blue', label=f'{label_b}: occupied')
    ax.scatter(pts_a[:, 0], pts_a[:, 1], s=2, c='tab:red', alpha=0.6, label=f'{label_a}: occupied')
    ax.plot(0, 0, 'k+', markersize=12, label='start pose (origin)')
    both = np.vstack([pts_a, pts_b])
    lo, hi = both.min(axis=0) - 0.3, both.max(axis=0) + 0.3
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    legend = ax.legend(loc='upper right', markerscale=4)
    handles = getattr(legend, 'legend_handles', None) or legend.legendHandles
    handles[-1].set_markersize(10)       # keep the start-pose cross normal size
    ax.set_title(f'Map comparison (light gray: free space of {label_a})')
    fig.tight_layout()
    fig.savefig(filename, dpi=150)


if __name__ == '__main__':
    main()

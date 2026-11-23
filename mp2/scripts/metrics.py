"""Path metrics for MP2 (Task 1a/1b): path length, minimum clearance, deviation.

Paths are (N, 2) arrays of x, y in the map frame [m].
"""
import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

DEFAULT_MAP = os.path.join('/opt/ros', os.environ.get('ROS_DISTRO', 'humble'),
                           'share/turtlebot3_navigation2/map/map.yaml')


class StaticMap:
    """Occupancy grid from a map_server yaml + distance to the nearest occupied cell."""

    def __init__(self, yaml_path=DEFAULT_MAP):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
        img = np.array(Image.open(img_path), dtype=float)
        if meta.get('negate', 0):
            img = 255 - img
        # map_server convention: occupancy probability p = (255 - value) / 255
        p = (255 - img) / 255
        self.occupied = p > meta['occupied_thresh']
        self.unknown = (p >= meta['free_thresh']) & ~self.occupied
        self.resolution = meta['resolution']
        self.origin = np.array(meta['origin'][:2])
        self.height, self.width = img.shape
        # distance [m] from each cell centre to the nearest occupied cell centre
        self.dist = ndimage.distance_transform_edt(~self.occupied) * self.resolution

    def world_to_cell(self, xy):
        """Row/col indices (row 0 = top of the image) for points in the map frame."""
        xy = np.atleast_2d(xy)
        col = np.floor((xy[:, 0] - self.origin[0]) / self.resolution).astype(int)
        row = self.height - 1 - np.floor((xy[:, 1] - self.origin[1]) / self.resolution).astype(int)
        return np.clip(row, 0, self.height - 1), np.clip(col, 0, self.width - 1)

    def clearance(self, xy):
        """Distance [m] from each point to the nearest occupied cell (resolution 5 cm)."""
        row, col = self.world_to_cell(xy)
        return self.dist[row, col]

    def extent(self):
        """Extent for matplotlib imshow (left, right, bottom, top)."""
        x0, y0 = self.origin
        return [x0, x0 + self.width * self.resolution, y0, y0 + self.height * self.resolution]


def path_length(xy):
    """Sum of the segment lengths [m]."""
    xy = np.asarray(xy)
    if len(xy) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def densify(xy, step=0.075):
    """Polyline with extra points so that consecutive points are at most step [m] apart.

    Default 0.075 m: just above a cell diagonal (0.0707 m), so NavFn paths (one pose per cell) stay
    unchanged; longer segments (RRT) are split. A finer step would move points of paths that run
    along a cell border into the neighbouring cell and change the 1a values by one cell (5 cm)."""
    xy = np.atleast_2d(np.asarray(xy, dtype=float))
    out = [xy[:1]]
    for a, b in zip(xy[:-1], xy[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out.append(a + (b - a) * (np.arange(1, n + 1) / n)[:, None])
    return np.vstack(out)


def min_clearance(xy, static_map):
    """Smallest distance [m] of the path to an obstacle of the static map (from the robot centre).

    Checked along the segments, not only at the poses: RRT paths have poses every 0.2 m, smoothed
    paths only a few poses (NavFn: one pose per cell, unchanged)."""
    return float(np.min(static_map.clearance(densify(xy))))


def deviation(driven_xy, planned_xy):
    """Distance [m] of each driven point to the nearest point on the planned polyline."""
    driven = np.atleast_2d(driven_xy)
    a = np.asarray(planned_xy)[:-1]
    b = np.asarray(planned_xy)[1:]
    ab = b - a
    ab_sq = np.maximum(np.sum(ab ** 2, axis=1), 1e-12)
    # projection of every driven point onto every segment, clipped to the segment
    ap = driven[:, None, :] - a[None, :, :]
    t = np.clip(np.sum(ap * ab[None], axis=2) / ab_sq[None], 0.0, 1.0)
    closest = a[None] + t[..., None] * ab[None]
    d = np.linalg.norm(driven[:, None, :] - closest, axis=2)
    return d.min(axis=1)


def deviation_active(samples, plans, plan_times):
    """Deviation [m] of each sample (t, x, y) from the plan that was active at time t.

    Active plan = the last plan with time stamp <= t (the first plan for earlier samples). With
    replanning (RRT: the controller got stuck, the new plan is a different random route) the
    deviation from the first plan would measure the change of the route, not the path following.
    With one plan this is the same as deviation()."""
    samples = np.atleast_2d(samples)
    idx = np.searchsorted(np.asarray(plan_times), samples[:, 0], side='right') - 1
    idx = np.clip(idx, 0, len(plans) - 1)
    d = np.empty(len(samples))
    for k, plan in enumerate(plans):
        sel = idx == k
        if sel.any():
            d[sel] = deviation(samples[sel, 1:], plan)
    return d


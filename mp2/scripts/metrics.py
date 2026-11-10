"""Path metrics for MP2 (Task 1a/1b): path length, minimum clearance, deviation.

Paths are (N, 2) arrays of x, y in the map frame [m].
"""
import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

DEFAULT_MAP = '/opt/ros/jazzy/share/turtlebot3_navigation2/map/map.yaml'


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


def min_clearance(xy, static_map):
    """Smallest distance [m] of the path to an obstacle of the static map (from the robot centre)."""
    return float(np.min(static_map.clearance(xy)))


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

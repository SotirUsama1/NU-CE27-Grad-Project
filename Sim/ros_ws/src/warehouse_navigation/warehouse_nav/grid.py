"""The warehouse map as a numpy grid, plus the two operations every script needs on it."""
import math
import re

import numpy as np
import yaml


class GridMap:
    """An occupancy grid. occupied[row, col] is True for obstacles; row 0 is the lowest y."""

    def __init__(self, occupied, resolution, origin_x, origin_y):
        self.occupied = occupied
        self.res = resolution
        self.ox = origin_x
        self.oy = origin_y

    @classmethod
    def from_yaml(cls, yaml_path):
        """Reads a map_server yaml and its .pgm (the files in warehouse_navigation/maps)."""
        import os
        meta = yaml.safe_load(open(yaml_path))
        data = open(os.path.join(os.path.dirname(yaml_path), meta['image']), 'rb').read()
        m = re.match(rb'P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+(\d+)\s', data)
        w, h = int(m.group(1)), int(m.group(2))
        img = np.frombuffer(data[m.end():], dtype=np.uint8)[:w * h].reshape(h, w)
        occupied = np.flipud(img) < 0.35 * 255    # pgm row 0 is the top (highest y); negate: 0
        return cls(occupied, meta['resolution'], meta['origin'][0], meta['origin'][1])

    @classmethod
    def from_msg(cls, msg):
        """From a nav_msgs/OccupancyGrid; unknown cells count as occupied."""
        info = msg.info
        grid = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
        return cls((grid >= 65) | (grid < 0), info.resolution, info.origin.position.x, info.origin.position.y)

    def cell(self, x, y):
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def xy(self, rows, cols):
        return self.ox + (cols + 0.5) * self.res, self.oy + (rows + 0.5) * self.res

    def clear_of_obstacles(self, radius_m):
        """True for the cells whose centre is at least radius_m from every obstacle."""
        return ~near_obstacle(self.occupied, radius_m / self.res)


def near_obstacle(occupied, radius_cells):
    """Cells with an occupied cell within radius_cells (a disk around every obstacle)."""
    out = occupied.copy()
    h, w = occupied.shape
    r = int(math.ceil(radius_cells))
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if (dx or dy) and dx * dx + dy * dy <= radius_cells * radius_cells:
                out[max(0, -dy):h - max(0, dy), max(0, -dx):w - max(0, dx)] |= \
                    occupied[max(0, dy):h - max(0, -dy), max(0, dx):w - max(0, -dx)]
    return out


def reachable_from(free, start):
    """Cells of `free` connected to `start` (row, col) through free cells, 4 neighbours."""
    reached = np.zeros_like(free)
    reached[start] = True
    frontier = reached.copy()
    while frontier.any():
        grown = frontier.copy()
        grown[1:, :] |= frontier[:-1, :]
        grown[:-1, :] |= frontier[1:, :]
        grown[:, 1:] |= frontier[:, :-1]
        grown[:, :-1] |= frontier[:, 1:]
        frontier = grown & free & ~reached
        reached |= frontier
    return reached

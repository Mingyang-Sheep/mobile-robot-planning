"""Shared math and grid utilities for Python planners."""


import math


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def euclidean_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def manhattan_distance(x1, y1, x2, y2):
    return abs(x2 - x1) + abs(y2 - y1)


def sign(value):
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


class GridMap:
    """Wraps an OccupancyGrid message and provides coordinate conversions."""

    def __init__(self, occupancy_grid):
        info = occupancy_grid.info
        self.width = info.width
        self.height = info.height
        self.resolution = info.resolution
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y
        self.data = occupancy_grid.data

    def world_to_grid(self, world_x, world_y):
        grid_x = int((world_x - self.origin_x) / self.resolution)
        grid_y = int((world_y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        world_x = self.origin_x + (grid_x + 0.5) * self.resolution
        world_y = self.origin_y + (grid_y + 0.5) * self.resolution
        return world_x, world_y

    def to_index(self, grid_x, grid_y):
        return grid_y * self.width + grid_x

    def index_to_grid(self, linear_index):
        return linear_index % self.width, linear_index // self.width

    def in_bounds(self, grid_x, grid_y):
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height

    def is_obstacle(self, grid_x, grid_y, obstacle_set):
        if not self.in_bounds(grid_x, grid_y):
            return True
        return self.to_index(grid_x, grid_y) in obstacle_set


def build_obstacle_set(grid_map, robot_radius):
    """Build inflated obstacle lookup from an OccupancyGrid.

    Cells with occupancy < 0 (unknown) or >= 50 (occupied) are inflated
    by robot_radius using circular Euclidean distance, consistent with
    the C++ planners.
    """
    obstacle_set = set()
    inflation_cells = int(math.ceil(robot_radius / grid_map.resolution))

    offsets = []
    for dy in range(-inflation_cells, inflation_cells + 1):
        for dx in range(-inflation_cells, inflation_cells + 1):
            if math.hypot(dx, dy) * grid_map.resolution <= robot_radius:
                offsets.append((dx, dy))

    for linear_index, occupancy in enumerate(grid_map.data):
        if occupancy < 0 or occupancy >= 50:
            ox = linear_index % grid_map.width
            oy = linear_index // grid_map.width
            for dx, dy in offsets:
                ix, iy = ox + dx, oy + dy
                if grid_map.in_bounds(ix, iy):
                    obstacle_set.add(grid_map.to_index(ix, iy))

    return obstacle_set

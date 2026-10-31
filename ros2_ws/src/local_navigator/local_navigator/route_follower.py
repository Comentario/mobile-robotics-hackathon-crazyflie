"""Ordered route tracking: no nearest-point jumps or direct-goal fallback."""
import math


class RouteFollower:
    def __init__(self, tolerance=.03):
        if not math.isfinite(tolerance) or tolerance <= 0:
            raise ValueError('Route point tolerance must be positive')
        self.tolerance = tolerance
        self.points = ()
        self.index = 0

    def clear(self):
        self.points, self.index = (), 0

    def set_path(self, points):
        points = tuple(tuple(p) for p in points)
        if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('Route must contain finite XYZ points')
        if points != self.points:
            self.points, self.index = points, 0

    def target(self, position):
        if not self.points:
            return None
        while self.index < len(self.points)-1:
            if math.dist(position[:2], self.points[self.index][:2]) > self.tolerance:
                break
            self.index += 1
        return self.points[self.index]

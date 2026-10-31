"""Ordered route tracking: no nearest-point jumps or direct-goal fallback."""
import math


class RouteFollower:
    def __init__(self, tolerance=.03, pass_radius=.15):
        if not math.isfinite(tolerance) or tolerance <= 0:
            raise ValueError('Route point tolerance must be positive')
        if not math.isfinite(pass_radius) or pass_radius < tolerance:
            raise ValueError('Route pass radius must be finite and >= tolerance')
        self.tolerance = tolerance
        self.pass_radius = pass_radius
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
        while self.index < len(self.points)-1 and self._reached(position):
            self.index += 1
        return self.points[self.index]

    def _reached(self, position):
        """Current point is within tolerance, or was overshot nearby along the route.

        A velocity-controlled vehicle can pass a point without entering the tight
        tolerance; it then counts as reached only within pass_radius, so the
        follower never jumps to a distant later point.
        """
        point, following = self.points[self.index][:2], self.points[self.index+1][:2]
        distance = math.dist(position[:2], point)
        if distance <= self.tolerance:
            return True
        if distance > self.pass_radius:
            return False
        if self.index == 0:
            # The start anchor is the previous waypoint, where the vehicle already is.
            return True
        along = ((position[0]-point[0])*(following[0]-point[0])
                 + (position[1]-point[1])*(following[1]-point[1]))
        return along > 0

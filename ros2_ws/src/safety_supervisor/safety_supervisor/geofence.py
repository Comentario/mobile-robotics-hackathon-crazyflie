"""Geofence and operating envelope (TP-39).

A geofence is an invisible "box" the drone must not leave.
  contains(p): is the point inside the box?
  clamp(p):    if the point is outside, move it to the nearest border of the box.
Pure logic: no rclpy. Points are (x, y, z) tuples in metres, map frame.
"""


class Geofence:
    def __init__(self, x_min, y_min, z_min, x_max, y_max, z_max):
        self.lower = (x_min, y_min, z_min)
        self.upper = (x_max, y_max, z_max)

    def contains(self, point, margin=0.0):
        """True if the point is inside the box enlarged by 'margin' m on every side.

        The margin avoids false alarms: when a setpoint is clamped right at the border,
        the drone stops at the border and pose noise "pushes it out" a few millimetres.
        """
        return all(lo - margin <= c <= hi + margin
                   for c, lo, hi in zip(point, self.lower, self.upper))

    def clamp(self, point):
        """Project a setpoint into the allowed space."""
        return tuple(min(max(c, lo), hi) for c, lo, hi in zip(point, self.lower, self.upper))

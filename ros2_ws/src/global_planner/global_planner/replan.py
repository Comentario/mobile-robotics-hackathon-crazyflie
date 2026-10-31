"""Pure replanning policy for TP-29."""

import math


def _point_segment_distance(point, start, end):
    """Return the 2-D distance from a point to a line segment."""
    px, py = float(point[0]), float(point[1])
    ax, ay = float(start[0]), float(start[1])
    bx, by = float(end[0]), float(end[1])

    dx = bx - ax
    dy = by - ay

    length_squared = dx * dx + dy * dy

    if length_squared == 0.0:
        return math.hypot(
            px - ax,
            py - ay,
        )

    projection = (
        (px - ax) * dx
        + (py - ay) * dy
    ) / length_squared

    projection = min(
        1.0,
        max(0.0, projection),
    )

    closest_x = ax + projection * dx
    closest_y = ay + projection * dy

    return math.hypot(
        px - closest_x,
        py - closest_y,
    )


def distance_to_path(position, path):
    """Return minimum 2-D distance from a position to a polyline."""
    if not path:
        return math.inf

    if len(path) == 1:
        return math.hypot(
            float(position[0]) - float(path[0][0]),
            float(position[1]) - float(path[0][1]),
        )

    return min(
        _point_segment_distance(
            position,
            start,
            end,
        )
        for start, end in zip(path, path[1:])
    )


class ReplanPolicy:
    """Evaluate the TP-29 replanning triggers."""

    def __init__(
        self,
        replan_period,
        max_path_deviation,
    ):
        self.replan_period = float(replan_period)
        self.max_path_deviation = float(
            max_path_deviation
        )

        if (
            not math.isfinite(self.replan_period)
            or self.replan_period <= 0.0
        ):
            raise ValueError(
                'replan_period must be positive and finite'
            )

        if (
            not math.isfinite(self.max_path_deviation)
            or self.max_path_deviation < 0.0
        ):
            raise ValueError(
                'max_path_deviation must be non-negative and finite'
            )

        self.last_plan_time = None

    def mark_planned(self, now):
        """Store the time of the latest planning attempt."""
        now = float(now)

        if not math.isfinite(now):
            raise ValueError(
                'planning time must be finite'
            )

        self.last_plan_time = now

    def reason(
        self,
        now,
        path,
        position,
        goal_changed,
        path_blocked,
        navigator_request,
    ):
        """Return a replanning reason or None.

        Priority:
          goal_changed
          blocked
          navigator
          deviation
          periodic
        """
        now = float(now)

        if not math.isfinite(now):
            raise ValueError('now must be finite')

        if goal_changed:
            return 'goal_changed'

        if path_blocked:
            return 'blocked'

        if navigator_request:
            return 'navigator'

        if path and position is not None:
            deviation = distance_to_path(
                position,
                path,
            )

            if deviation > self.max_path_deviation:
                return 'deviation'

        if self.last_plan_time is not None:
            elapsed = now - self.last_plan_time

            if elapsed >= self.replan_period:
                return 'periodic'

        return None

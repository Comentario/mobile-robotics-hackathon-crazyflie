"""Metrics of one run (TP-40, README "Metrics for the demo")."""
from dataclasses import dataclass, field
import math

DRONE_RADIUS = 0.06   # m, same approximation as simulation_adapter for the Crazyflie 2.1
INTERNAL_FIELDS = ('in_collision', 'state_stale')   # working flags, not part of the summary


def box_distance(point, center, size):
    """Euclidean distance from a point to an axis-aligned box (0 inside)."""
    return math.sqrt(sum(max(abs(p-c) - s/2, 0.0)**2 for p, c, s in zip(point, center, size)))


@dataclass
class RunMetrics:
    waypoints_reached: int = 0
    waypoints_total: int = 0
    mission_time: float = 0.0
    collisions: int = 0
    min_obstacle_distance: float = float('inf')
    replans: int = 0
    stale_state_events: int = 0
    safety_interventions: int = 0
    safety_states_seen: set = field(default_factory=set)
    health_states_seen: set = field(default_factory=set)
    planner_states_seen: set = field(default_factory=set)
    max_abs_x: float = 0.0
    max_abs_y: float = 0.0
    max_z: float = 0.0
    final_z: float = 0.0
    final_mission_state: int = 0
    in_collision: bool = field(default=False, repr=False)
    state_stale: bool = field(default=False, repr=False)

    def note_state_age(self, age, timeout):
        """Count one stale-state event per silence, however often it is checked.

        Counting only when a late message finally arrives misses a stream that
        never comes back, which is the localization-lost case this metric is for.
        """
        if age <= timeout:
            self.state_stale = False
        elif not self.state_stale:
            self.stale_state_events += 1
            self.state_stale = True

    def update_truth(self, position, obstacles, walls):
        """Fold one ground-truth position into the metrics.

        obstacles and walls are (center, size) boxes. Both count for collisions;
        min_obstacle_distance is the clearance from the drone surface to the
        obstacles only. The floor is not passed: landing is not a collision.
        """
        x, y, z = position
        self.max_abs_x = max(self.max_abs_x, abs(x))
        self.max_abs_y = max(self.max_abs_y, abs(y))
        self.max_z = max(self.max_z, z)
        self.final_z = z
        obstacle = min((box_distance(position, c, s) for c, s in obstacles), default=math.inf)
        wall = min((box_distance(position, c, s) for c, s in walls), default=math.inf)
        self.min_obstacle_distance = min(self.min_obstacle_distance,
                                         max(obstacle - DRONE_RADIUS, 0.0))
        touching = min(obstacle, wall) <= DRONE_RADIUS
        if touching and not self.in_collision:
            self.collisions += 1
        self.in_collision = touching

    def to_dict(self):
        """JSON-serializable dictionary (sets -> sorted lists, inf -> None)."""
        out = {}
        for name in self.__dataclass_fields__:
            if name in INTERNAL_FIELDS:
                continue
            value = getattr(self, name)
            if isinstance(value, set):
                value = sorted(value)
            elif isinstance(value, float) and not math.isfinite(value):
                value = None
            out[name] = value
        return out

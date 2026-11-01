"""2.5D simulator world: obstacles and ranger raycasts (TP-10).

Format of config/worlds/*.yaml:
  bounds: [x_min, y_min, x_max, y_max]   # room walls
  ceiling: 2.0                           # m
  obstacles:
    - {type: box, center: [x, y], size: [sx, sy]}
    - {type: box, center: [x, y], size: [sx, sy], height: 1.0,
       velocity: [vx, vy], t_start: 5.0, t_end: 20.0}   # moving and/or temporary

Pure logic (rule 6 of interfaces.md): no rclpy, tested with pytest.

"2.5D" means: the XY plane has walls and boxes; every box is a vertical prism
from the ground (z = 0) up to 'height' (infinite if not given).
There is also a floor (z = 0) and a ceiling (z = ceiling).

raycast: a ray is "cast" from the sensor and the distance to the first thing it
hits (wall, ceiling, floor or box) is computed. That is what a real laser sensor
like the Multi-ranger ones measures.
"""
from dataclasses import dataclass, field
import math


@dataclass
class Box:
    center: tuple
    size: tuple
    height: float = math.inf
    velocity: tuple = (0.0, 0.0)
    t_start: float = 0.0
    t_end: float = math.inf

    def active(self, t):
        """True if the obstacle exists at time t."""
        return self.t_start <= t <= self.t_end

    def center_at(self, t):
        """Obstacle centre at t (linear motion from t_start)."""
        elapsed = max(0.0, t - self.t_start)
        return (self.center[0] + self.velocity[0] * elapsed,
                self.center[1] + self.velocity[1] * elapsed)

    def bounds_at(self, t):
        """3D box at t: ((x_min, y_min, z_min), (x_max, y_max, z_max))."""
        cx, cy = self.center_at(t)
        hx, hy = self.size[0] / 2.0, self.size[1] / 2.0
        return (cx - hx, cy - hy, 0.0), (cx + hx, cy + hy, self.height)


@dataclass
class World:
    bounds: tuple
    ceiling: float
    obstacles: list = field(default_factory=list)
    collision_count: int = 0
    _colliding: bool = False

    @classmethod
    def from_dict(cls, data):
        """Build the world from the already loaded YAML."""
        data = data or {}
        obstacles = []
        for obstacle in data.get('obstacles', []) or []:
            if obstacle.get('type', 'box') != 'box':
                raise ValueError(f'Unsupported obstacle type: {obstacle.get("type")}')
            obstacles.append(Box(
                center=tuple(obstacle['center']),
                size=tuple(obstacle['size']),
                height=float(obstacle.get('height', math.inf)),
                velocity=tuple(obstacle.get('velocity', (0.0, 0.0))),
                t_start=float(obstacle.get('t_start', 0.0)),
                t_end=float(obstacle.get('t_end', math.inf)),
            ))
        bounds = tuple(data.get('bounds', (-math.inf, -math.inf, math.inf, math.inf)))
        return cls(bounds=bounds,
                   ceiling=float(data.get('ceiling', math.inf)),
                   obstacles=obstacles)

    @classmethod
    def empty(cls):
        """World without walls or ceiling (when no world_file is given)."""
        return cls(bounds=(-math.inf, -math.inf, math.inf, math.inf), ceiling=math.inf)

    def raycast(self, origin, direction, max_range, t):
        """Distance from origin (x, y, z) along direction (unit) to the first hit.

        Returns math.inf if there is no hit within max_range (REP-117 convention).
        Considers walls (bounds), ceiling, floor and obstacles active at t.
        """
        x, y, z = origin
        dx, dy, dz = direction
        x_min, y_min, x_max, y_max = self.bounds
        hits = []

        # Room walls (the drone is inside): plane the ray reaches ahead.
        if dx > 0:
            hits.append((x_max - x) / dx)
        elif dx < 0:
            hits.append((x_min - x) / dx)
        if dy > 0:
            hits.append((y_max - y) / dy)
        elif dy < 0:
            hits.append((y_min - y) / dy)

        # Ceiling and floor.
        if dz > 0:
            hits.append((self.ceiling - z) / dz)
        elif dz < 0:
            hits.append((0.0 - z) / dz)

        # Active boxes ("slabs" method: intersection with the 3 slabs of the prism).
        for box in self.obstacles:
            if box.active(t):
                d = _ray_box(origin, direction, *box.bounds_at(t))
                if d is not None:
                    hits.append(d)

        hits = [max(0.0, h) for h in hits if math.isfinite(h) and h >= -1e-9]
        distance = min(hits) if hits else math.inf
        return distance if distance <= max_range else math.inf

    def collides(self, position, radius, t):
        """True if a sphere of radius `radius` at `position` touches something. Counts collisions.

        A new collision is counted every time it goes from "not touching" to "touching".
        The floor does not count (the drone takes off and lands on it).
        """
        x, y, z = position
        x_min, y_min, x_max, y_max = self.bounds
        touching = (
            x - radius < x_min or x + radius > x_max
            or y - radius < y_min or y + radius > y_max
            or z + radius > self.ceiling
        )
        if not touching:
            for box in self.obstacles:
                if box.active(t) and _distance_to_box(position, *box.bounds_at(t)) < radius:
                    touching = True
                    break

        if touching and not self._colliding:
            self.collision_count += 1
        self._colliding = touching
        return touching


def _ray_box(origin, direction, lower, upper):
    """Distance at which the ray enters the box [lower, upper], or None if it misses it."""
    t_near, t_far = -math.inf, math.inf
    for o, d, lo, hi in zip(origin, direction, lower, upper):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None        # parallel and outside the slab
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        t_near, t_far = max(t_near, t1), min(t_far, t2)
        if t_near > t_far:
            return None
    if t_far < 0:
        return None                # the box is behind
    return max(t_near, 0.0)        # 0 if the origin is inside


def _distance_to_box(point, lower, upper):
    """Euclidean distance from a point to a box (0 if inside)."""
    return math.sqrt(sum(max(lo - p, 0.0, p - hi) ** 2
                         for p, lo, hi in zip(point, lower, upper)))

"""2.5D simulation world and range-sensor raycasting."""

from dataclasses import dataclass, field
import math


EPSILON = 1e-9


@dataclass
class Box:
    center: tuple
    size: tuple
    height: float = math.inf
    velocity: tuple = (0.0, 0.0)
    t_start: float = 0.0
    t_end: float = math.inf

    def active(self, t):
        """Return True while the obstacle exists."""
        return self.t_start <= t <= self.t_end

    def center_at(self, t):
        """Return the linearly translated center at time t."""
        elapsed = max(0.0, t - self.t_start)

        return (
            self.center[0] + self.velocity[0] * elapsed,
            self.center[1] + self.velocity[1] * elapsed,
        )


def _ray_aabb(origin, direction, minimum, maximum):
    """Return ray entry distance for an axis-aligned box."""
    t_min = -math.inf
    t_max = math.inf

    for index in range(3):
        coordinate = origin[index]
        component = direction[index]
        lower = minimum[index]
        upper = maximum[index]

        if abs(component) < EPSILON:
            if coordinate < lower or coordinate > upper:
                return math.inf

            continue

        first = (lower - coordinate) / component
        second = (upper - coordinate) / component

        if first > second:
            first, second = second, first

        t_min = max(t_min, first)
        t_max = min(t_max, second)

        if t_min > t_max:
            return math.inf

    if t_max < 0.0:
        return math.inf

    if t_min < 0.0:
        return 0.0

    return t_min


@dataclass
class World:
    bounds: tuple
    ceiling: float
    obstacles: list = field(default_factory=list)

    @classmethod
    def from_dict(cls, data):
        """Construct a world from a loaded YAML dictionary."""
        bounds = tuple(float(value) for value in data['bounds'])

        if len(bounds) != 4:
            raise ValueError(
                'World bounds must be [x_min, y_min, x_max, y_max]'
            )

        ceiling = float(data['ceiling'])

        obstacles = []

        for entry in data.get('obstacles', []) or []:
            obstacle_type = entry.get('type', 'box')

            if obstacle_type != 'box':
                raise ValueError(
                    f'Unsupported obstacle type: {obstacle_type}'
                )

            obstacles.append(
                Box(
                    center=tuple(
                        float(value)
                        for value in entry['center']
                    ),
                    size=tuple(
                        float(value)
                        for value in entry['size']
                    ),
                    height=float(
                        entry.get('height', math.inf)
                    ),
                    velocity=tuple(
                        float(value)
                        for value in entry.get(
                            'velocity',
                            [0.0, 0.0],
                        )
                    ),
                    t_start=float(
                        entry.get('t_start', 0.0)
                    ),
                    t_end=float(
                        entry.get('t_end', math.inf)
                    ),
                )
            )

        return cls(
            bounds=bounds,
            ceiling=ceiling,
            obstacles=obstacles,
        )

    def _inside_room(self, position):
        x_min, y_min, x_max, y_max = self.bounds
        x, y, z = position

        return (
            x_min <= x <= x_max
            and y_min <= y <= y_max
            and 0.0 <= z <= self.ceiling
        )

    def _room_exit_distance(self, origin, direction):
        """Return distance to the first room boundary."""
        if not self._inside_room(origin):
            return 0.0

        x_min, y_min, x_max, y_max = self.bounds
        x, y, z = origin
        dx, dy, dz = direction

        candidates = []

        if dx > EPSILON:
            candidates.append((x_max - x) / dx)
        elif dx < -EPSILON:
            candidates.append((x_min - x) / dx)

        if dy > EPSILON:
            candidates.append((y_max - y) / dy)
        elif dy < -EPSILON:
            candidates.append((y_min - y) / dy)

        if dz > EPSILON:
            candidates.append((self.ceiling - z) / dz)
        elif dz < -EPSILON:
            candidates.append((0.0 - z) / dz)

        positive = [
            distance
            for distance in candidates
            if distance >= 0.0
        ]

        if not positive:
            return math.inf

        return min(positive)

    def raycast(self, origin, direction, max_range, t):
        """Return distance to the first wall, floor, ceiling, or obstacle.

        math.inf is returned when no object is found within max_range.
        """
        origin = tuple(float(value) for value in origin)
        direction = tuple(float(value) for value in direction)

        norm = math.sqrt(
            sum(value * value for value in direction)
        )

        if norm < EPSILON:
            raise ValueError('Raycast direction must be non-zero')

        direction = tuple(
            value / norm
            for value in direction
        )

        nearest = self._room_exit_distance(
            origin,
            direction,
        )

        for obstacle in self.obstacles:
            if not obstacle.active(t):
                continue

            center_x, center_y = obstacle.center_at(t)

            half_x = obstacle.size[0] / 2.0
            half_y = obstacle.size[1] / 2.0

            obstacle_height = min(
                obstacle.height,
                self.ceiling,
            )

            minimum = (
                center_x - half_x,
                center_y - half_y,
                0.0,
            )

            maximum = (
                center_x + half_x,
                center_y + half_y,
                obstacle_height,
            )

            distance = _ray_aabb(
                origin,
                direction,
                minimum,
                maximum,
            )

            nearest = min(nearest, distance)

        if nearest <= float(max_range):
            return nearest

        return math.inf

    def collides(self, position, radius, t):
        """Return True when a spherical drone touches the world geometry."""
        x, y, z = position
        radius = float(radius)

        x_min, y_min, x_max, y_max = self.bounds

        if (
            x - radius <= x_min
            or x + radius >= x_max
            or y - radius <= y_min
            or y + radius >= y_max
            or z + radius >= self.ceiling
        ):
            return True

        for obstacle in self.obstacles:
            if not obstacle.active(t):
                continue

            center_x, center_y = obstacle.center_at(t)

            half_x = obstacle.size[0] / 2.0
            half_y = obstacle.size[1] / 2.0

            minimum = (
                center_x - half_x,
                center_y - half_y,
                0.0,
            )

            maximum = (
                center_x + half_x,
                center_y + half_y,
                min(obstacle.height, self.ceiling),
            )

            closest_x = min(
                max(x, minimum[0]),
                maximum[0],
            )

            closest_y = min(
                max(y, minimum[1]),
                maximum[1],
            )

            closest_z = min(
                max(z, minimum[2]),
                maximum[2],
            )

            distance_squared = (
                (x - closest_x) ** 2
                + (y - closest_y) ** 2
                + (z - closest_z) ** 2
            )

            if distance_squared <= radius * radius:
                return True

        return False
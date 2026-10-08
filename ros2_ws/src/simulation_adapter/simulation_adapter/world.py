"""Mundo 2.5D del simulador: obstáculos y raycast de los rangers (TP-10).

Formato de config/worlds/*.yaml:
  bounds: [x_min, y_min, x_max, y_max]   # paredes de la sala
  ceiling: 2.0                           # m
  obstacles:
    - {type: box, center: [x, y], size: [sx, sy]}
    - {type: box, center: [x, y], size: [sx, sy], height: 1.0,
       velocity: [vx, vy], t_start: 5.0, t_end: 20.0}   # dinámico y/o temporal

Lógica pura (regla 6 de interfaces.md): sin rclpy, se prueba con pytest.

"2.5D" significa: el plano XY tiene paredes y cajas; cada caja es un prisma
vertical que va del suelo (z = 0) hasta 'height' (infinito si no se indica).
Además hay suelo (z = 0) y techo (z = ceiling).

raycast: se "lanza" un rayo desde el sensor y se calcula a qué distancia choca con
lo primero que encuentra (pared, techo, suelo o caja). Es lo que mide un sensor
láser real como los del Multi-ranger.
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
        """True si el obstáculo existe en el instante t."""
        return self.t_start <= t <= self.t_end

    def center_at(self, t):
        """Centro del obstáculo en t (movimiento lineal desde t_start)."""
        elapsed = max(0.0, t - self.t_start)
        return (self.center[0] + self.velocity[0] * elapsed,
                self.center[1] + self.velocity[1] * elapsed)

    def bounds_at(self, t):
        """Caja 3D en t: ((x_min, y_min, z_min), (x_max, y_max, z_max))."""
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
        """Construye el mundo desde el YAML ya cargado."""
        data = data or {}
        obstacles = []
        for obstacle in data.get('obstacles', []) or []:
            if obstacle.get('type', 'box') != 'box':
                raise ValueError(f'Tipo de obstáculo no soportado: {obstacle.get("type")}')
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
        """Mundo sin paredes ni techo (si no se indica world_file)."""
        return cls(bounds=(-math.inf, -math.inf, math.inf, math.inf), ceiling=math.inf)

    def raycast(self, origin, direction, max_range, t):
        """Distancia desde origin (x, y, z) en direction (unitario) al primer impacto.

        Devuelve math.inf si no hay impacto dentro de max_range (convención REP-117).
        Considera paredes (bounds), techo, suelo y obstáculos activos en t.
        """
        x, y, z = origin
        dx, dy, dz = direction
        x_min, y_min, x_max, y_max = self.bounds
        hits = []

        # Paredes de la sala (el dron está dentro): plano que el rayo alcanza delante.
        if dx > 0:
            hits.append((x_max - x) / dx)
        elif dx < 0:
            hits.append((x_min - x) / dx)
        if dy > 0:
            hits.append((y_max - y) / dy)
        elif dy < 0:
            hits.append((y_min - y) / dy)

        # Techo y suelo.
        if dz > 0:
            hits.append((self.ceiling - z) / dz)
        elif dz < 0:
            hits.append((0.0 - z) / dz)

        # Cajas activas (método de "slabs": intersección con las 3 franjas del prisma).
        for box in self.obstacles:
            if box.active(t):
                d = _ray_box(origin, direction, *box.bounds_at(t))
                if d is not None:
                    hits.append(d)

        hits = [max(0.0, h) for h in hits if math.isfinite(h) and h >= -1e-9]
        distance = min(hits) if hits else math.inf
        return distance if distance <= max_range else math.inf

    def collides(self, position, radius, t):
        """True si una esfera de radio `radius` en `position` toca algo. Cuenta colisiones.

        Cuenta una colisión nueva cada vez que se pasa de "no toca" a "toca".
        El suelo no cuenta (el dron despega y aterriza en él).
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
    """Distancia a la que el rayo entra en la caja [lower, upper], o None si no la toca."""
    t_near, t_far = -math.inf, math.inf
    for o, d, lo, hi in zip(origin, direction, lower, upper):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None        # paralelo y fuera de la franja
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        t_near, t_far = max(t_near, t1), min(t_far, t2)
        if t_near > t_far:
            return None
    if t_far < 0:
        return None                # la caja está detrás
    return max(t_near, 0.0)        # 0 si el origen está dentro


def _distance_to_box(point, lower, upper):
    """Distancia euclídea de un punto a una caja (0 si está dentro)."""
    return math.sqrt(sum(max(lo - p, 0.0, p - hi) ** 2
                         for p, lo, hi in zip(point, lower, upper)))

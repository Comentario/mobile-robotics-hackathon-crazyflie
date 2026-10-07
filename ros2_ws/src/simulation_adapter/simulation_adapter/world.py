"""Geometría 2.5D: sala y cajas, distancias en metros, tiempo relativo en segundos."""
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

    def __post_init__(self):
        if any(len(v) != 2 for v in (self.center, self.size, self.velocity)):
            raise ValueError('center, size y velocity deben tener dos elementos')
        if not all(math.isfinite(v) for v in (*self.center, *self.size, *self.velocity, self.t_start)):
            raise ValueError('Geometría no finita')
        if min(self.size) <= 0 or self.height <= 0 or math.isnan(self.height):
            raise ValueError('Tamaño y altura deben ser positivos')
        if self.t_start < 0 or math.isnan(self.t_end) or self.t_end <= self.t_start:
            raise ValueError('Intervalo temporal inválido')

    def active(self, t):
        return self.t_start <= t < self.t_end

    def center_at(self, t):
        return tuple(c + v * max(0.0, t - self.t_start) for c, v in zip(self.center, self.velocity))

    def limits(self, t, ceiling):
        x, y = self.center_at(t)
        sx, sy = self.size
        return (x-sx/2, y-sy/2, 0.0), (x+sx/2, y+sy/2, min(self.height, ceiling))


def ray_box(origin, direction, lower, upper):
    """Método de slabs; un origen dentro de la caja devuelve distancia cero."""
    near, far = 0.0, math.inf
    for p, d, lo, hi in zip(origin, direction, lower, upper):
        if abs(d) < 1e-12:
            if p < lo or p > hi:
                return math.inf
        else:
            a, b = sorted(((lo-p)/d, (hi-p)/d))
            near, far = max(near, a), min(far, b)
            if near > far:
                return math.inf
    return near if far >= 0 else math.inf


@dataclass
class World:
    bounds: tuple
    ceiling: float
    obstacles: list = field(default_factory=list)

    def __post_init__(self):
        if len(self.bounds) != 4 or not all(math.isfinite(v) for v in (*self.bounds, self.ceiling)):
            raise ValueError('Bounds y techo deben ser finitos')
        x0, y0, x1, y1 = self.bounds
        if x0 >= x1 or y0 >= y1 or self.ceiling <= 0:
            raise ValueError('Sala sin volumen positivo')

    @classmethod
    def from_dict(cls, data):
        boxes = []
        for item in data.get('obstacles', []):
            if item.get('type', 'box') != 'box':
                raise ValueError('Solo se admiten obstáculos box')
            boxes.append(Box(tuple(item['center']), tuple(item['size']),
                             item.get('height', math.inf), tuple(item.get('velocity', (0, 0))),
                             item.get('t_start', 0.0), item.get('t_end', math.inf)))
        return cls(tuple(data['bounds']), data['ceiling'], boxes)

    def raycast(self, origin, direction, max_range, t):
        if len(origin) != 3 or len(direction) != 3:
            raise ValueError('Rayo XYZ requerido')
        if not all(math.isfinite(v) for v in (*origin, *direction, max_range, t)) or max_range <= 0:
            raise ValueError('Rayo no finito o alcance inválido')
        if not math.isclose(math.hypot(*direction), 1.0, abs_tol=1e-9):
            raise ValueError('La dirección debe ser unitaria')
        lower = (self.bounds[0], self.bounds[1], 0.0)
        upper = (self.bounds[2], self.bounds[3], self.ceiling)
        if any(p < lo or p > hi for p, lo, hi in zip(origin, lower, upper)):
            return 0.0  # Sensor fuera de la sala, dentro de material sólido.
        hit = math.inf
        for p, d, lo, hi in zip(origin, direction, lower, upper):
            if abs(d) > 1e-12:
                hit = min(hit, ((hi if d > 0 else lo)-p)/d)
        for box in self.obstacles:
            if box.active(t):
                hit = min(hit, ray_box(origin, direction, *box.limits(t, self.ceiling)))
        return hit if hit <= max_range else math.inf

    def collides(self, position, radius, t):
        if len(position) != 3 or not all(math.isfinite(v) for v in (*position, radius, t)) or radius < 0:
            raise ValueError('Esfera inválida')
        lower = (self.bounds[0], self.bounds[1], 0.0)
        upper = (self.bounds[2], self.bounds[3], self.ceiling)
        if any(p-radius <= lo or p+radius >= hi for p, lo, hi in zip(position, lower, upper)):
            return True
        for box in self.obstacles:
            if box.active(t):
                lo, hi = box.limits(t, self.ceiling)
                distance = math.hypot(*(p-min(max(p, a), b) for p, a, b in zip(position, lo, hi)))
                if distance <= radius:
                    return True
        return False

"""Mundo 2.5D del simulador: obstáculos y raycast de los rangers (TP-10).

Formato de config/worlds/*.yaml:
  bounds: [x_min, y_min, x_max, y_max]   # paredes de la sala
  ceiling: 2.0                           # m
  obstacles:
    - {type: box, center: [x, y], size: [sx, sy]}
    - {type: box, center: [x, y], size: [sx, sy], height: 1.0,
       velocity: [vx, vy], t_start: 5.0, t_end: 20.0}   # dinámico y/o temporal
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
        raise NotImplementedError

    def center_at(self, t):
        """Centro del obstáculo en t (movimiento lineal desde t_start)."""
        raise NotImplementedError


@dataclass
class World:
    bounds: tuple
    ceiling: float
    obstacles: list = field(default_factory=list)

    @classmethod
    def from_dict(cls, data):
        """Construye el mundo desde el YAML ya cargado."""
        raise NotImplementedError

    def raycast(self, origin, direction, max_range, t):
        """Distancia desde origin (x, y, z) en direction (unitario) al primer impacto.

        Devuelve math.inf si no hay impacto dentro de max_range (convención REP-117).
        Considera paredes (bounds), techo, suelo y obstáculos activos en t.
        """
        raise NotImplementedError

    def collides(self, position, radius, t):
        """True si una esfera de radio `radius` en `position` toca algo. Cuenta colisiones."""
        raise NotImplementedError

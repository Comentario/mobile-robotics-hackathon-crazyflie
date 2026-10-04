"""Seguimiento por lookahead (TP-32) y suavizado de comandos (TP-33)."""
from dataclasses import dataclass


@dataclass
class FollowerConfig:
    lookahead_distance: float
    max_speed: float
    max_accel: float
    goal_slowdown_radius: float
    z_nominal: float


def lookahead_point(path, position, lookahead_distance):
    """Punto de la ruta a lookahead_distance por delante de la proyección de position."""
    raise NotImplementedError


def desired_velocity(position, target, speed_limit, distance_to_goal, config):
    """Velocidad hacia target, limitada y reducida cerca del goal."""
    raise NotImplementedError


class RateLimiter:
    """Limita el cambio de comando entre ciclos (sin saltos grandes de referencia)."""

    def __init__(self, max_accel):
        self.max_accel = max_accel

    def apply(self, dt, velocity):
        raise NotImplementedError

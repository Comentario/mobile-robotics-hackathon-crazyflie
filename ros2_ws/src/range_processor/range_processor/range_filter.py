"""Validación y filtrado de rangos (TP-19, TP-20) y tasa de cierre d_dot (TP-36).

Convención REP-117 en toda la API: +inf sin retorno, -inf demasiado cerca, NaN inválido.
"""
from dataclasses import dataclass

SENSORS = ('front', 'back', 'left', 'right', 'up', 'down')


@dataclass
class RangeFilterConfig:
    range_min: float
    range_max: float
    filter_window: int      # muestras para la mediana; pequeño para no añadir retraso
    max_valid_jump: float   # m entre muestras; más grande se trata como spike


def classify(value, config):
    """Devuelve 'valid', 'no_return', 'too_close' o 'invalid'."""
    raise NotImplementedError


class RangeFilter:

    def __init__(self, config):
        self.config = config

    def update(self, t, raw):
        """raw: dict sensor -> m. Devuelve dict sensor -> m filtrado (mismo formato)."""
        raise NotImplementedError

    def closing_rate(self, sensor):
        """d_dot estimado del sensor en m/s, o None si no es estable."""
        raise NotImplementedError


def time_to_collision(distance, closing_rate):
    """TTC = distance / -d_dot si el obstáculo se acerca; math.inf en otro caso."""
    raise NotImplementedError

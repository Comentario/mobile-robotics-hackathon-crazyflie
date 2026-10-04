"""Evitación reactiva horizontal (TP-34), vertical (TP-35) y por TTC (TP-36)."""
from dataclasses import dataclass


@dataclass
class AvoidanceConfig:
    warning_distance: float
    critical_distance: float
    ttc_threshold: float


def reactive_correction(ranges, closing_rates, velocity_body, config):
    """Corrige la velocidad deseada (marco base_link) según los rangos laterales.

    - No avanzar hacia un sensor por debajo de critical_distance.
    - Entre warning y critical: escalar la componente hacia el obstáculo y empujar al lado contrario.
    - TTC < ttc_threshold: frenar / hover.
    Devuelve (velocity_body, blocked, reason). blocked=True -> pedir replan.
    """
    raise NotImplementedError


def vertical_limit(z_target, up_range, down_range, z_min, z_max, critical_distance):
    """Recorta z_target al envelope y a lo que permiten up_range (techo) y down_range (suelo)."""
    raise NotImplementedError

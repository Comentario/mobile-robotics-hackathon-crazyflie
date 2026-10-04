"""Monitor de salud de la localización (TP-15) y política de velocidad (TP-16)."""
from dataclasses import dataclass

GOOD, DEGRADED, LOST = 0, 1, 2   # igual que hackathon_interfaces/LocalizationHealth


@dataclass
class HealthConfig:
    state_timeout: float        # s sin pose -> LOST
    degraded_pose_age: float    # s sin pose -> DEGRADED
    max_position_jump: float    # m entre muestras consecutivas
    max_apparent_speed: float   # m/s


class LocalizationHealthMonitor:

    def __init__(self, config):
        self.config = config

    def update(self, t, position):
        """Registra una pose nueva (t en s, position = (x, y, z)). Detecta NaN/inf y saltos."""
        raise NotImplementedError

    def evaluate(self, now):
        """Devuelve (status, pose_age, reason). reason: '', 'stale', 'jump', 'nan', 'overspeed'."""
        raise NotImplementedError


def speed_limit_for(status, nominal_speed, degraded_speed):
    """GOOD -> nominal, DEGRADED -> reducida, LOST -> 0 (no navegar)."""
    raise NotImplementedError

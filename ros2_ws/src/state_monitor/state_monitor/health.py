"""Monitor de salud de la localización (TP-15) y política de velocidad (TP-16).

Lógica pura (regla 6 de interfaces.md): sin rclpy, se prueba con pytest.

Qué responde: "¿me puedo fiar de la posición que me llega?"

Cómo decide el estado (evaluate), en este orden de prioridad
-----------------------------------------------------------
  1. Nunca llegó una pose                       -> LOST      reason 'stale'
  2. La última pose tenía NaN o infinito          -> LOST      reason 'nan'
  3. pose_age > state_timeout                     -> LOST      reason 'stale'
  4. pose_age > degraded_pose_age                 -> DEGRADED  reason 'stale'
  5. Hubo un salto en el último JUMP_HOLD s       -> DEGRADED  reason 'jump'
  6. Hubo velocidad imposible en JUMP_HOLD s      -> DEGRADED  reason 'overspeed'
  7. Todo bien                                    -> GOOD      reason ''

pose_age (edad de la pose)
--------------------------
  Segundos desde la última pose NUEVA. Una pose cuenta como nueva si:
    - su hora (stamp) es reciente, y
    - su posición es distinta a la anterior.
  Así se detectan tres fallos distintos con la misma regla:
    - dropout: no llegan poses              -> la edad crece
    - delay:   llegan con hora vieja        -> la edad ya es grande al llegar
    - freeze:  llegan con valores idénticos -> no cuentan como nuevas, la edad crece
  Limitación conocida: un estimador real nunca repite exactamente los mismos
  decimales (siempre tiene ruido), por eso "valores idénticos" = estimador colgado.
  Si alguna vez se usa una fuente sin ruido, el freeze daría falsos positivos.
"""
import math
from dataclasses import dataclass

GOOD, DEGRADED, LOST = 0, 1, 2   # igual que hackathon_interfaces/LocalizationHealth

# Tiempo (s) que se mantiene DEGRADED después de un salto o una velocidad imposible,
# para que el resto del stack llegue a verlo y no sea un aviso de una sola muestra.
JUMP_HOLD = 1.0


@dataclass
class HealthConfig:
    state_timeout: float        # s sin pose -> LOST
    degraded_pose_age: float    # s sin pose -> DEGRADED
    max_position_jump: float    # m entre muestras consecutivas
    max_apparent_speed: float   # m/s


class LocalizationHealthMonitor:

    def __init__(self, config):
        self.config = config
        self._last_position = None     # última posición recibida
        self._last_stamp = None        # hora de esa posición
        self._last_fresh_stamp = None  # hora de la última pose NUEVA (ver docstring)
        self._nan = False              # la última pose era inválida
        self._jump_time = None         # hora del último salto detectado
        self._overspeed_time = None    # hora de la última velocidad imposible

    def update(self, t, position):
        """Registra una pose nueva (t en s, position = (x, y, z)). Detecta NaN/inf y saltos.

        t es la hora de la MEDIDA (header.stamp), no la hora de llegada.
        """
        # NaN / infinito: la pose no sirve. No se guarda como referencia.
        if not all(math.isfinite(c) for c in position):
            self._nan = True
            return
        self._nan = False

        if self._last_position is not None:
            distance = math.dist(position, self._last_position)
            dt = t - self._last_stamp

            if distance > self.config.max_position_jump:
                self._jump_time = t
            elif dt > 0.0 and distance / dt > self.config.max_apparent_speed:
                self._overspeed_time = t

            # Pose nueva = posición distinta a la anterior (ver "freeze").
            if distance > 0.0:
                self._last_fresh_stamp = t
        else:
            self._last_fresh_stamp = t

        self._last_position = tuple(position)
        self._last_stamp = t

    def evaluate(self, now):
        """Devuelve (status, pose_age, reason). reason: '', 'stale', 'jump', 'nan', 'overspeed'."""
        if self._last_fresh_stamp is None:
            return LOST, math.inf, 'stale'

        pose_age = max(0.0, now - self._last_fresh_stamp)

        if self._nan:
            return LOST, pose_age, 'nan'
        if pose_age > self.config.state_timeout:
            return LOST, pose_age, 'stale'
        if pose_age > self.config.degraded_pose_age:
            return DEGRADED, pose_age, 'stale'
        if self._jump_time is not None and now - self._jump_time < JUMP_HOLD:
            return DEGRADED, pose_age, 'jump'
        if self._overspeed_time is not None and now - self._overspeed_time < JUMP_HOLD:
            return DEGRADED, pose_age, 'overspeed'
        return GOOD, pose_age, ''


def speed_limit_for(status, nominal_speed, degraded_speed):
    """GOOD -> nominal, DEGRADED -> reducida, LOST -> 0 (no navegar)."""
    if status == GOOD:
        return nominal_speed
    if status == DEGRADED:
        return degraded_speed
    return 0.0

"""Máquina de estados de seguridad (TP-37).

Prioridad: ABORT > LAND > HOVER > SLOW > NORMAL. Se evalúa en cada ciclo; un estado
más grave solo se abandona cuando la causa desaparece (con histéresis).

Lógica pura (regla 6 de interfaces.md): sin rclpy, se prueba con pytest.

Qué decide evaluate() en cada ciclo (de más grave a menos grave)
---------------------------------------------------------------
  ABORT   Ninguna regla lo activa todavía (reservado). Si se activa, no se sale nunca.
  LAND    Una causa "de pérdida" (localización LOST, estado vencido o comandos
          vencidos) dura más de hover_timeout Y el dron está en el aire.
          Solo con lost_policy = 'hover_then_land'.
  HOVER   Localización LOST, estado vencido, comandos vencidos, rangos vencidos
          (si ranges_required), obstáculo a menos de critical_distance, dron fuera
          de la geofence, o el planner sin ruta.
  SLOW    Localización DEGRADED, o TTC menor que ttc_threshold.
  NORMAL  Nada de lo anterior.

Histéresis
----------
  Pasar a un estado MÁS grave es inmediato. Volver a uno MENOS grave exige que la
  causa haya desaparecido durante RECOVERY_TIME s seguidos. Así un fallo que va y
  viene no hace "parpadear" al dron entre HOVER y NORMAL.

Qué hace filter_command() con el comando del navegador
------------------------------------------------------
  NORMAL  Lo deja pasar (recortado a la geofence y a z_min/z_max).
  SLOW    Lo deja pasar, pero más despacio:
            VELOCITY: velocidad x speed_scale.
            POSITION: objetivo acercado al dron (actual + speed_scale x distancia);
                      como la velocidad del controlador es proporcional a la
                      distancia al objetivo, el dron va más despacio.
  HOVER   Lo sustituye por "quédate quieto aquí".
  LAND / ABORT  Lo descarta (None): el aterrizaje o la emergencia ya se pidieron
                por servicio.
"""
import math
from dataclasses import dataclass

NORMAL, SLOW, HOVER, LAND, ABORT = range(5)   # igual que hackathon_interfaces/SafetyState
STATE_NAMES = ('NORMAL', 'SLOW', 'HOVER', 'LAND', 'ABORT')

# Modos de Setpoint (igual que hackathon_interfaces/Setpoint).
MODE_POSITION, MODE_VELOCITY, MODE_HOVER = 0, 1, 2

# GOOD / DEGRADED / LOST (igual que hackathon_interfaces/LocalizationHealth).
GOOD, DEGRADED, LOST = 0, 1, 2

RECOVERY_TIME = 1.0   # s sin causa antes de volver a un estado menos grave
AIRBORNE_Z = 0.1      # m: por encima de esta altura se considera que el dron vuela


@dataclass
class SafetyInputs:
    now: float
    localization: int           # GOOD / DEGRADED / LOST
    state_stale: bool
    ranges_stale: bool
    command_stale: bool
    min_horizontal_range: float
    min_ttc: float
    z: float
    planner_no_path: bool
    outside_geofence: bool


@dataclass
class SupervisorConfig:
    hover_timeout: float = 3.0          # s en HOVER por pérdida antes de aterrizar
    lost_policy: str = 'hover_then_land'  # 'hover_then_land' | 'hover'
    critical_distance: float = 0.3      # m: obstáculo más cerca -> HOVER
    ttc_threshold: float = 1.5          # s: TTC menor -> SLOW
    ranges_required: bool = True        # sin rangos -> HOVER
    slow_speed_scale: float = 0.5       # factor de velocidad en SLOW


class SafetySupervisor:
    def __init__(self, config):
        self.config = config
        self.state = NORMAL
        self.reason = 'startup'
        self._loss_since = None      # desde cuándo dura una causa "de pérdida"
        self._calm_since = None      # desde cuándo la causa es menos grave que el estado

    # ------------------------------------------------------------------
    def evaluate(self, inputs):
        """Devuelve (state, speed_scale, reason)."""
        wanted, reason = self._wanted_state(inputs)

        if self.state == ABORT:
            pass                                    # ABORT no se abandona
        elif wanted >= self.state:
            # Igual o más grave: se aplica inmediatamente.
            self.state, self.reason = wanted, reason
            self._calm_since = None
        else:
            # Menos grave: solo tras RECOVERY_TIME s seguidos sin la causa.
            if self._calm_since is None:
                self._calm_since = inputs.now
            if inputs.now - self._calm_since >= RECOVERY_TIME:
                self.state, self.reason = wanted, reason
                self._calm_since = None

        return self.state, self._speed_scale(self.state), self.reason

    def _wanted_state(self, i):
        """Estado que pedirían las entradas ahora mismo, sin histéresis."""
        c = self.config

        # Causas "de pérdida": no sabemos dónde estamos o nadie nos manda.
        loss = []
        if i.localization == LOST:
            loss.append('localization_lost')
        if i.state_stale:
            loss.append('state_timeout')
        if i.command_stale:
            loss.append('command_timeout')

        if loss:
            if self._loss_since is None:
                self._loss_since = i.now
        else:
            self._loss_since = None

        airborne = i.z > AIRBORNE_Z
        if (loss and airborne and c.lost_policy == 'hover_then_land'
                and i.now - self._loss_since > c.hover_timeout):
            return LAND, loss[0]

        hover = list(loss)
        if c.ranges_required and i.ranges_stale:
            hover.append('ranges_timeout')
        if i.min_horizontal_range < c.critical_distance:
            hover.append('obstacle_close')
        if i.outside_geofence:
            hover.append('outside_geofence')
        if i.planner_no_path:
            hover.append('no_path')
        if hover:
            return HOVER, hover[0]

        if i.localization == DEGRADED:
            return SLOW, 'localization_degraded'
        if i.min_ttc < c.ttc_threshold:
            return SLOW, 'ttc'
        return NORMAL, ''

    def _speed_scale(self, state):
        if state == NORMAL:
            return 1.0
        if state == SLOW:
            return self.config.slow_speed_scale
        return 0.0

    # ------------------------------------------------------------------
    def filter_command(self, setpoint, state, speed_scale, current_position,
                       geofence=None, z_limits=None):
        """Aplica el estado al setpoint: escala velocidad, convierte a hover o lo descarta.

        setpoint: dict {'mode', 'position': (x,y,z), 'velocity': (vx,vy,vz), 'yaw'}.
        current_position: (x, y, z) actual del dron, o None si aún no se conoce.
        geofence: objeto Geofence (o None). z_limits: (z_min, z_max) (o None).
        Devuelve un dict con el mismo formato, o None si no hay que publicar nada.
        """
        if state in (LAND, ABORT):
            return None

        if state == HOVER or setpoint['mode'] == MODE_HOVER:
            if current_position is None:
                return None
            return {
                'mode': MODE_HOVER,
                'position': tuple(current_position),
                'velocity': (0.0, 0.0, 0.0),
                'yaw': setpoint['yaw'],
            }

        out = {
            'mode': setpoint['mode'],
            'position': tuple(setpoint['position']),
            'velocity': tuple(setpoint['velocity']),
            'yaw': setpoint['yaw'],
        }

        # SLOW: más despacio.
        if speed_scale < 1.0:
            if out['mode'] == MODE_VELOCITY:
                out['velocity'] = tuple(v * speed_scale for v in out['velocity'])
            elif current_position is not None:
                out['position'] = tuple(
                    c + speed_scale * (t - c)
                    for c, t in zip(current_position, out['position'])
                )

        # Geofence (TP-39) y banda de altura (TP-17): el objetivo nunca sale de la caja.
        if geofence is not None:
            out['position'] = geofence.clamp(out['position'])
        if z_limits is not None:
            x, y, z = out['position']
            out['position'] = (x, y, min(max(z, z_limits[0]), z_limits[1]))

        # En VELOCITY, no empujar hacia fuera si el dron ya está en el borde.
        if out['mode'] == MODE_VELOCITY and geofence is not None and current_position:
            v = list(out['velocity'])
            for k in range(3):
                if current_position[k] <= geofence.lower[k] and v[k] < 0:
                    v[k] = 0.0
                if current_position[k] >= geofence.upper[k] and v[k] > 0:
                    v[k] = 0.0
            out['velocity'] = tuple(v)

        return out


def min_finite(values):
    """Menor valor finito y positivo de una lista, o inf si no hay ninguno."""
    finite = [v for v in values if math.isfinite(v) and v > 0.0]
    return min(finite) if finite else math.inf

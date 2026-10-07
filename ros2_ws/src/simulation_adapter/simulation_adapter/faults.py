"""Inyección de fallos controlados para la campaña TP-41 y la test matrix T08–T16.

Formato en config/scenarios/*.yaml:
  faults:
    - {type: range_spike,    sensor: front, value: 0.1, t_start: 5.0, t_end: 5.2}
    - {type: range_dropout,  sensor: all,               t_start: 5.0, t_end: 8.0}
    - {type: pose_delay,     delay: 0.5,                t_start: 5.0, t_end: 10.0}
    - {type: pose_freeze,                               t_start: 5.0, t_end: 10.0}
    - {type: pose_jump,      offset: [0.5, 0.0, 0.0],   t_start: 5.0, t_end: 5.0}
    - {type: pose_dropout,                              t_start: 5.0}   # Lighthouse perdido
    - {type: command_pause,                             t_start: 5.0, t_end: 8.0}

Lógica pura (regla 6 de interfaces.md): sin rclpy. Los tiempos t son segundos de
simulación desde que arranca el simulador.

Cómo se interpreta cada fallo de pose (lo que vería el stack con un dron real)
----------------------------------------------------------------------------
  pose_dropout  No se publica pose. Imita perder la radio o el Lighthouse.
  pose_delay    Se publica la pose de hace 'delay' s, con su hora original (vieja).
                Imita latencia de la radio o un PC sobrecargado.
  pose_freeze   Se publica siempre la misma pose (valores idénticos) con hora nueva.
                Imita un estimador colgado: llegan datos, pero no cambian.
  pose_jump     Se suma 'offset' a la pose. Si t_end == t_start (o no hay t_end), el
                desplazamiento queda para siempre desde t_start: imita un reinicio
                del EKF o el Lighthouse "recolocando" el dron de golpe.

Los fallos se aplican SOLO a la pose publicada (/hk/adapter/odom), nunca a la pose
real (/hk/sim/ground_truth), como pide docs/simulations.md.

Formato de una muestra de pose (dict): {'t', 'stamp', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz'}
  t      tiempo de simulación (s)
  stamp  hora ROS de la medida (s), la que irá en header.stamp
"""
import math

FAULT_TYPES = (
    'range_spike', 'range_dropout', 'pose_delay', 'pose_freeze',
    'pose_jump', 'pose_dropout', 'command_pause',
)

RANGE_SENSORS = ('front', 'back', 'left', 'right', 'up', 'down')


class FaultInjector:

    def __init__(self, faults):
        """faults: lista de dicts como en el docstring del módulo."""
        self.faults = list(faults or [])
        for fault in self.faults:
            if fault.get('type') not in FAULT_TYPES:
                raise ValueError(f'Tipo de fallo desconocido: {fault.get("type")}')
        # Pose congelada mientras dura un pose_freeze (se guarda al empezar).
        self._frozen = None

    # ------------------------------------------------------------------
    def active(self, t, fault_type):
        """Fallos de ese tipo activos en t."""
        result = []
        for fault in self.faults:
            if fault['type'] != fault_type:
                continue
            t_start = fault.get('t_start', 0.0)
            t_end = fault.get('t_end', math.inf)
            if fault_type == 'pose_jump' and t_end <= t_start:
                t_end = math.inf      # salto permanente (ver docstring)
            if t_start <= t <= t_end:
                result.append(fault)
        return result

    def active_types(self, t):
        """Nombres de todos los tipos de fallo activos en t (para el log del nodo)."""
        return sorted({f['type'] for f in self.faults if self.active(t, f['type'])})

    # ------------------------------------------------------------------
    def apply_to_ranges(self, t, ranges):
        """Devuelve los rangos (dict sensor -> m) modificados por spikes/dropouts.

        Devuelve None si un range_dropout con sensor 'all' está activo (no se
        publica nada). Un dropout de un solo sensor lo marca como NaN (REP-117: inválido).
        """
        out = dict(ranges)
        for fault in self.active(t, 'range_dropout'):
            sensor = fault.get('sensor', 'all')
            if sensor == 'all':
                return None
            out[sensor] = math.nan
        for fault in self.active(t, 'range_spike'):
            sensors = RANGE_SENSORS if fault.get('sensor') == 'all' else (fault['sensor'],)
            for sensor in sensors:
                out[sensor] = float(fault['value'])
        return out

    def apply_to_pose(self, t, pose, pose_history):
        """Devuelve la pose a publicar (retrasada, congelada, con salto) o None si no se publica.

        pose: muestra actual (dict). pose_history: muestras anteriores, de la más
        antigua a la más nueva (debe incluir la actual).
        """
        # 1. Dropout: no se publica nada.
        if self.active(t, 'pose_dropout'):
            return None

        out = dict(pose)

        # 2. Retraso: se publica una muestra vieja, con su hora vieja.
        delays = self.active(t, 'pose_delay')
        if delays:
            delay = max(float(f.get('delay', 0.5)) for f in delays)
            old = self._sample_at(pose_history, t - delay)
            if old is None:
                return None          # aún no hay historia suficiente
            out = dict(old)

        # 3. Congelación: siempre los mismos valores, pero con hora actual.
        if self.active(t, 'pose_freeze'):
            if self._frozen is None:
                self._frozen = dict(out)
            frozen = dict(self._frozen)
            frozen['t'] = out['t']
            frozen['stamp'] = out['stamp']
            out = frozen
        else:
            self._frozen = None

        # 4. Salto: desplazamiento sumado a la posición.
        for fault in self.active(t, 'pose_jump'):
            dx, dy, dz = (list(fault.get('offset', [0.0, 0.0, 0.0])) + [0.0, 0.0, 0.0])[:3]
            out['x'] += dx
            out['y'] += dy
            out['z'] += dz

        return out

    def drop_command(self, t):
        """True si el comando recibido en t debe ignorarse (command_pause)."""
        return bool(self.active(t, 'command_pause'))

    # ------------------------------------------------------------------
    @staticmethod
    def _sample_at(history, t_target):
        """Última muestra con t <= t_target, o None si no existe."""
        candidate = None
        for sample in history:
            if sample['t'] <= t_target:
                candidate = sample
            else:
                break
        return candidate

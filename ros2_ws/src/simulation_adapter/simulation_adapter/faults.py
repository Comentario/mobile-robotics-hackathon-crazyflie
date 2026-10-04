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
"""

FAULT_TYPES = (
    'range_spike', 'range_dropout', 'pose_delay', 'pose_freeze',
    'pose_jump', 'pose_dropout', 'command_pause',
)


class FaultInjector:

    def __init__(self, faults):
        """faults: lista de dicts como en el docstring del módulo."""
        self.faults = faults

    def active(self, t, fault_type):
        """Fallos de ese tipo activos en t."""
        raise NotImplementedError

    def apply_to_ranges(self, t, ranges):
        """Devuelve los rangos (dict sensor -> m) modificados por spikes/dropouts."""
        raise NotImplementedError

    def apply_to_pose(self, t, pose, pose_history):
        """Devuelve la pose a publicar (retrasada, congelada, con salto) o None si no se publica."""
        raise NotImplementedError

    def drop_command(self, t):
        """True si el comando recibido en t debe ignorarse (command_pause)."""
        raise NotImplementedError

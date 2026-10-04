"""Máquina de estados de seguridad (TP-37).

Prioridad: ABORT > LAND > HOVER > SLOW > NORMAL. Se evalúa en cada ciclo; un estado
más grave solo se abandona cuando la causa desaparece (con histéresis).
"""
from dataclasses import dataclass

NORMAL, SLOW, HOVER, LAND, ABORT = range(5)   # igual que hackathon_interfaces/SafetyState


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


class SafetySupervisor:

    def __init__(self, config):
        self.config = config
        self.state = NORMAL

    def evaluate(self, inputs):
        """Devuelve (state, speed_scale, reason)."""
        raise NotImplementedError

    def filter_command(self, setpoint, state, speed_scale, current_position):
        """Aplica el estado al setpoint: escala velocidad, convierte a hover o lo descarta."""
        raise NotImplementedError

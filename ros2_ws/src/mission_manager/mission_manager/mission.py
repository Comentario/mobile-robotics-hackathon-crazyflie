"""Carga de waypoints y máquina de estados de la misión (TP-26).

Estados alineados con hackathon_interfaces/MissionStatus y con el diagrama del README:
IDLE -> PREFLIGHT -> TAKEOFF -> NAVIGATE -> LANDING -> COMPLETED
cualquier estado -> PAUSED / ABORTED (Seguro).
"""
from dataclasses import dataclass

IDLE, PREFLIGHT, TAKEOFF, NAVIGATE, LANDING, COMPLETED, PAUSED, ABORTED = range(8)


@dataclass
class Waypoint:
    x: float
    y: float
    z: float


def load_mission(path):
    """Lee config/missions/*.yaml y devuelve la lista de Waypoint."""
    raise NotImplementedError


class MissionStateMachine:

    def __init__(self, waypoints, tolerance, timeout):
        self.waypoints = waypoints
        self.tolerance = tolerance
        self.timeout = timeout
        self.state = IDLE
        self.index = 0

    def step(self, now, position, health, safety_state, planner_status):
        """Avanza la máquina y devuelve las acciones del ciclo.

        Acciones posibles: ('goal', Waypoint), ('takeoff',), ('land',), ('none',).
        PREFLIGHT exige health GOOD y safety NORMAL; safety LAND/ABORT lleva a ABORTED.
        """
        raise NotImplementedError

    def pause(self):
        raise NotImplementedError

    def resume(self):
        raise NotImplementedError

    def abort(self, reason):
        raise NotImplementedError

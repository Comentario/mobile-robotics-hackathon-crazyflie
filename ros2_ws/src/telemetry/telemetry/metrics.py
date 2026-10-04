"""Métricas de una ejecución (TP-40, README "Métricas para la demo")."""
from dataclasses import dataclass, field


@dataclass
class RunMetrics:
    waypoints_reached: int = 0
    waypoints_total: int = 0
    mission_time: float = 0.0
    collisions: int = 0
    min_obstacle_distance: float = float('inf')
    replans: int = 0
    stale_state_events: int = 0
    safety_interventions: int = 0
    safety_states_seen: set = field(default_factory=set)
    health_states_seen: set = field(default_factory=set)
    planner_states_seen: set = field(default_factory=set)
    max_abs_x: float = 0.0
    max_abs_y: float = 0.0
    max_z: float = 0.0
    final_z: float = 0.0
    final_mission_state: int = 0

    def to_dict(self):
        """Diccionario serializable a JSON (sets -> listas ordenadas)."""
        raise NotImplementedError

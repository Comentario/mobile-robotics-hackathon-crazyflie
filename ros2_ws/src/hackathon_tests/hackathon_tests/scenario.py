"""Carga de escenarios y evaluación pass/fail.

Formato de config/scenarios/*.yaml: ver docs/simulations.md.
Claves de `expected` soportadas:
  mission_state, max_collisions, min_obstacle_distance, must_see_safety, must_see_health,
  must_see_planner, max_abs_xy, max_z, final_z_max
"""


def load_scenario(path):
    raise NotImplementedError


def evaluate(metrics, expected):
    """Devuelve (passed, lista de fallos legibles)."""
    raise NotImplementedError

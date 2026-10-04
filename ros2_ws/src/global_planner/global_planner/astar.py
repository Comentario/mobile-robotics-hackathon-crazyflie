"""A* sobre rejilla de ocupación (TP-27) con costes y política para unknown (TP-28)."""


def build_cost_map(grid, unknown_policy, unknown_cost, proximity_weight):
    """Coste por celda: inf en ocupadas, extra cerca de obstáculos y en unknown.

    unknown_policy: 'block' | 'penalize' | 'free'.
    """
    raise NotImplementedError


def astar(cost_map, start, goal, connectivity=8, timeout=None):
    """Ruta de celdas start -> goal, o None si no existe o se supera timeout (s)."""
    raise NotImplementedError


def direct_path(start, goal, spacing):
    """Ruta recta muestreada cada `spacing` m. Modo direct (P0), sin mapa."""
    raise NotImplementedError

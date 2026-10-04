"""Cuándo replanificar (TP-29)."""


class ReplanPolicy:

    def __init__(self, replan_period, max_path_deviation):
        self.replan_period = replan_period
        self.max_path_deviation = max_path_deviation

    def reason(self, now, path, position, goal_changed, path_blocked, navigator_request):
        """Motivo de replan ('goal_changed', 'blocked', 'deviation', 'periodic',
        'navigator') o None si la ruta actual sigue valiendo."""
        raise NotImplementedError

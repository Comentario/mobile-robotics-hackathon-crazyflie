"""Geofence y envelope operativo (TP-39)."""


class Geofence:

    def __init__(self, x_min, y_min, z_min, x_max, y_max, z_max):
        self.lower = (x_min, y_min, z_min)
        self.upper = (x_max, y_max, z_max)

    def contains(self, point):
        raise NotImplementedError

    def clamp(self, point):
        """Proyecta un setpoint al interior del espacio permitido."""
        raise NotImplementedError

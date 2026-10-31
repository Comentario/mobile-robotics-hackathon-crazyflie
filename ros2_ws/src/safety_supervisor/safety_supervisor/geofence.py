"""Geofence y envelope operativo (TP-39).

Una geofence es una "caja" invisible: el dron no debe salir de ella.
  contains(p): ¿está el punto dentro de la caja?
  clamp(p):    si el punto está fuera, lo mueve al borde más cercano de la caja.
Lógica pura: sin rclpy. Los puntos son tuplas (x, y, z) en metros, frame map.
"""


class Geofence:
    def __init__(self, x_min, y_min, z_min, x_max, y_max, z_max):
        self.lower = (x_min, y_min, z_min)
        self.upper = (x_max, y_max, z_max)

    def contains(self, point, margin=0.0):
        """True si el punto está dentro de la caja ampliada 'margin' m por cada lado.

        El margen evita falsas alarmas: si un setpoint se recorta justo al borde,
        el dron se para en el borde y el ruido de la pose lo "saca" unos milímetros.
        """
        return all(lo - margin <= c <= hi + margin
                   for c, lo, hi in zip(point, self.lower, self.upper))

    def clamp(self, point):
        """Proyecta un setpoint al interior del espacio permitido."""
        return tuple(min(max(c, lo), hi) for c, lo, hi in zip(point, self.lower, self.upper))

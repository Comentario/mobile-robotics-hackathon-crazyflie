"""Rejilla de ocupación 2D móvil (TP-22), inflado (TP-23) y caducidad (TP-24)."""
UNKNOWN, FREE, OCCUPIED = -1, 0, 100   # valores de nav_msgs/OccupancyGrid


class RollingGrid:

    def __init__(self, resolution, width, height):
        """resolution en m/celda; width y height en m. Centrada en el dron."""
        self.resolution = resolution
        self.width = width
        self.height = height

    def recenter(self, x, y):
        """Desplaza la ventana para que el dron quede en el centro, conservando lo observado."""
        raise NotImplementedError

    def world_to_cell(self, x, y):
        raise NotImplementedError

    def cell_to_world(self, i, j):
        raise NotImplementedError

    def integrate_ray(self, t, origin, end, hit):
        """Marca libres las celdas del rayo (Bresenham) y ocupada la final si hit."""
        raise NotImplementedError

    def decay(self, now, decay_time):
        """Vuelve a UNKNOWN las celdas ocupadas no reobservadas en decay_time s."""
        raise NotImplementedError

    def inflated(self, radius):
        """Copia de la rejilla con obstáculos inflados `radius` m (radio del dron + margen)."""
        raise NotImplementedError

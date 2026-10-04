"""Simplificación de rutas (TP-30)."""


def remove_collinear(points, tolerance=1e-3):
    raise NotImplementedError


def shortcut(points, segment_is_free):
    """Une puntos no consecutivos si segment_is_free(a, b) (line of sight segura)."""
    raise NotImplementedError


def densify(points, max_spacing):
    """Inserta puntos para que ningún tramo supere max_spacing m."""
    raise NotImplementedError

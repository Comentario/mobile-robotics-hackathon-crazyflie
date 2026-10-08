"""Validación y filtrado de rangos (TP-19, TP-20) y tasa de cierre d_dot (TP-36).

Convención REP-117 en toda la API: +inf sin retorno, -inf demasiado cerca, NaN inválido.

Lógica pura (regla 6 de interfaces.md): sin rclpy, se prueba con pytest.

Qué hace update() con cada sensor, en cada mensaje
-------------------------------------------------
  1. Clasificar la lectura (classify):
       NaN                    -> 'invalid'    (no se usa)
       +inf o > range_max     -> 'no_return'  (nada delante, se trata como +inf)
       -inf o < range_min     -> 'too_close'  (algo pegado, se trata como 0 m)
       resto                  -> 'valid'
  2. Rechazo de spikes: si la lectura salta más de max_valid_jump respecto al valor
     aceptado, NO se cree enseguida. Se guarda como "candidata" y solo se acepta si
     las lecturas siguientes la confirman durante spike_confirm_time segundos.
     Un spike de 1-4 muestras (T08) se descarta; un obstáculo real se acepta
     con un retraso de spike_confirm_time.
  3. Mediana de las últimas filter_window lecturas aceptadas (quita ruido sin
     añadir mucho retraso: con 3 muestras a 20 Hz son ~0,05 s).
  4. Si el sensor da NaN más de invalid_hold segundos seguidos, la salida es NaN;
     antes se mantiene el último valor filtrado.

closing_rate (d_dot): pendiente (regresión lineal) de la distancia filtrada en los
últimos rate_window s. Negativo = el obstáculo se acerca.
"""
from collections import deque
from dataclasses import dataclass
import math

SENSORS = ('front', 'back', 'left', 'right', 'up', 'down')


@dataclass
class RangeFilterConfig:
    range_min: float
    range_max: float
    filter_window: int      # muestras para la mediana; pequeño para no añadir retraso
    max_valid_jump: float   # m entre muestras; más grande se trata como spike
    spike_confirm_time: float = 0.3   # s que un salto debe mantenerse para aceptarlo
    invalid_hold: float = 0.3         # s que se mantiene el último valor si llega NaN
    rate_window: float = 0.3          # s de historia para estimar d_dot
    rate_min_samples: int = 3         # muestras mínimas para dar un d_dot


def classify(value, config):
    """Devuelve 'valid', 'no_return', 'too_close' o 'invalid'."""
    if math.isnan(value):
        return 'invalid'
    if value == math.inf or value > config.range_max:
        return 'no_return'
    if value == -math.inf or value < config.range_min:
        return 'too_close'
    return 'valid'


def _to_distance(value, kind):
    """Lectura -> distancia interna: too_close = 0 m, no_return = +inf."""
    if kind == 'too_close':
        return 0.0
    if kind == 'no_return':
        return math.inf
    return float(value)


class _SensorState:
    def __init__(self, window):
        self.accepted = deque(maxlen=window)   # distancias aceptadas (para la mediana)
        self.reference = None                  # último valor aceptado (para detectar saltos)
        self.candidate = None                  # (t_inicio, [valores]) de un salto sin confirmar
        self.output = math.nan                 # último valor filtrado publicado
        self.last_ok_t = None                  # hora de la última lectura no-NaN
        self.history = deque()                 # (t, distancia filtrada) para d_dot


class RangeFilter:

    def __init__(self, config):
        self.config = config
        self._sensors = {name: _SensorState(max(1, config.filter_window)) for name in SENSORS}
        self.rejected_spikes = 0               # contador, útil para logs y tests

    def update(self, t, raw):
        """raw: dict sensor -> m. Devuelve dict sensor -> m filtrado (mismo formato)."""
        out = {}
        for name in SENSORS:
            s = self._sensors[name]
            value = raw.get(name, math.nan)
            kind = classify(value, self.config)

            if kind == 'invalid':
                # Mantener el último valor un rato; después, NaN.
                if s.last_ok_t is None or t - s.last_ok_t > self.config.invalid_hold:
                    s.output = math.nan
                    s.history.clear()
                out[name] = s.output
                continue

            s.last_ok_t = t
            distance = _to_distance(value, kind)
            self._accept_or_hold(t, s, distance)

            median = _median(s.accepted)
            s.output = -math.inf if median < self.config.range_min else median
            self._record_history(t, s, median)
            out[name] = s.output
        return out

    def closing_rate(self, sensor):
        """d_dot estimado del sensor en m/s, o None si no es estable."""
        history = self._sensors[sensor].history
        if len(history) < self.config.rate_min_samples:
            return None
        ts = [h[0] for h in history]
        ds = [h[1] for h in history]
        t_mean = sum(ts) / len(ts)
        d_mean = sum(ds) / len(ds)
        den = sum((ti - t_mean) ** 2 for ti in ts)
        if den <= 0.0:
            return None
        return sum((ti - t_mean) * (di - d_mean) for ti, di in zip(ts, ds)) / den

    # ------------------------------------------------------------------
    def _accept_or_hold(self, t, s, distance):
        """Paso 2 del docstring: aceptar la lectura o tratarla como posible spike."""
        jump = self.config.max_valid_jump
        # Para comparar, "sin retorno" (+inf) cuenta como range_max: pasar de 3.48 m
        # a +inf es el obstáculo saliendo del alcance, no un spike (el LaserScan de
        # Crazyswarm2 corta en 3.49 m y con ruido parpadea entre 3.48 y +inf).
        cmp_now = min(distance, self.config.range_max)
        cmp_ref = None if s.reference is None else min(s.reference, self.config.range_max)
        if cmp_ref is None or abs(cmp_now - cmp_ref) <= jump:
            # Lectura coherente con lo aceptado: se acepta y se olvida cualquier candidata.
            if s.candidate is not None:
                self.rejected_spikes += 1
                s.candidate = None
            s.accepted.append(distance)
            s.reference = distance
            return

        # Salto grande. ¿Continúa una candidata coherente?
        last = min(s.candidate[1][-1], self.config.range_max) if s.candidate else None
        if s.candidate is not None and abs(cmp_now - last) <= jump:
            s.candidate[1].append(distance)
        else:
            if s.candidate is not None:
                self.rejected_spikes += 1
            s.candidate = (t, [distance])

        if t - s.candidate[0] >= self.config.spike_confirm_time:
            # Confirmado: es un cambio real (obstáculo nuevo o que desaparece).
            s.accepted.clear()
            s.accepted.extend(s.candidate[1][-s.accepted.maxlen:])
            s.reference = distance
            s.candidate = None
            s.history.clear()      # el d_dot anterior ya no vale

    def _record_history(self, t, s, distance):
        """Guarda la distancia filtrada para d_dot (solo valores finitos)."""
        if not math.isfinite(distance):
            s.history.clear()
            return
        s.history.append((t, distance))
        while s.history and t - s.history[0][0] > self.config.rate_window:
            s.history.popleft()


def _median(values):
    ordered = sorted(values)
    n = len(ordered)
    if n == 0:
        return math.nan
    mid = n // 2
    if n % 2:
        return ordered[mid]
    low, high = ordered[mid - 1], ordered[mid]
    if math.isinf(low) or math.isinf(high):
        return high if math.isinf(high) else low
    return (low + high) / 2.0


def time_to_collision(distance, closing_rate):
    """TTC = distance / -d_dot si el obstáculo se acerca; math.inf en otro caso."""
    if closing_rate is None or not math.isfinite(closing_rate) or closing_rate >= 0.0:
        return math.inf
    if math.isnan(distance) or distance == math.inf:
        return math.inf
    if distance <= 0.0:          # -inf (demasiado cerca) o 0
        return 0.0
    return distance / -closing_rate

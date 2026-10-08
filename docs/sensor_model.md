# Modelo de sensores de distancia (TP-19)

Qué miden los rangers del Crazyflie, por qué camino llegan al stack, cómo se
simulan y cómo se filtran. Complementa `docs/frames.md` (TP-13) y `docs/interfaces.md`.

Convención en todo el documento: "verificado" = comprobado en la fuente citada;
*nota mía* = supuesto propio, pendiente de confirmar.

## 1. Hardware

| Campo de `MultiRange` | Deck | Chip | Dirección (base_link) | Alcance |
|---|---|---|---|---|
| front, back, left, right, up | Multi-ranger deck | VL53L1x ×5 | +x, −x, +y, −y, +z | hasta 4 m |
| down | Flow deck v2 | VL53L1x | −z | hasta 4 m |

Verificado en las páginas de producto de Bitcraze: Multi-ranger *"5 x VL53L1x ToF
sensor to measure distance up to 4 meters"*; Flow deck v2 *"Measures absolute range
up to 4m"* (además lleva el sensor de flujo óptico PMW3901).
*Nota mía:* el alcance real baja con luz fuerte y con superficies oscuras o
brillantes; se mide en Dresden (sección 8).

## 2. Camino de los datos: igual en simulación y en Dresden

```
Dresden:     sensores -> firmware -> Crazyradio -> crazyflie_server -> /cf231/* -> cf_adapter -> /hk/adapter/ranges
Simulación:  world.py (raycast)     -> sim_crazyflie (misma API)    -> /cf231/* -> cf_adapter -> /hk/adapter/ranges
```

`cf_adapter`, `range_processor` y `safety_supervisor` son los mismos en los dos casos.
En Dresden solo se cambia `sim_crazyflie` por `crazyflie_server` + Crazyradio + dron.
Lanzar la simulación: `ros2 launch hackathon_bringup simulation_cf.launch.py scenario:=<nombre>`.

## 3. Formato en Crazyswarm2 (lado `/cf231/*`)

Verificado en el código fuente de Crazyswarm2 1.0.7
(`crazyflie_server_cpp/src/crazyflie_server.cpp`, `on_logging_scan`; igual en
`crazyflie_server_py/crazyflie_server.py`):

| Tópico | Tipo | Contenido |
|---|---|---|
| `/cf231/scan` | `sensor_msgs/LaserScan` | `ranges = [back, right, front, left]` en m; `angle_min = −π`, `angle_increment = π/2`; `range_min = 0.01`, `range_max = 3.49`; un valor > 3.49 m se publica como `+inf`. Variables del firmware: `range.front/left/back/right` (mm) |
| `/cf231/ranges_extra` | `crazyflie_interfaces/LogDataGeneric` | `values = [range.up, range.zrange]` en **mm**, en el orden de `vars` |

`up` y `down` **no** van en el scan: hay que pedirlos como *custom topic*. Configuración
para Dresden en `crazyflies.yaml` (formato verificado en el `crazyflies.yaml` de
ejemplo de Crazyswarm2 1.0.7):

```yaml
all:
  firmware_logging:
    enabled: true
    default_topics:
      pose:
        frequency: 50   # Hz  (el ejemplo trae 10; nota mía: 50 para state_timeout=0.3 s)
      scan:
        frequency: 20   # Hz
    custom_topics:
      ranges_extra:
        frequency: 20   # Hz
        vars: ["range.up", "range.zrange"]
```

*Nota mía:* qué valor da el firmware en `range.up`/`range.zrange` cuando no hay
retorno está por verificar en Dresden. `cf_adapter` trata cualquier valor mayor
que `range_max` como `+inf`, así que funciona con cualquier valor grande.

## 4. Conversión en `cf_adapter` (`cf_adapter/ranges.py`)

- Asigna cada rayo del scan a un sensor **por su ángulo** (`angle_min + i·angle_increment`),
  no por su posición en la lista: sigue funcionando si Crazyswarm2 cambia el orden.
- `ranges_extra`: mm → m.
- Convención REP-117 en `/hk/adapter/ranges` (`hackathon_interfaces/MultiRange`,
  `frame_id = base_link`):

| Valor | Significado | Cuándo |
|---|---|---|
| número en [range_min, range_max] | distancia medida | normal |
| `+inf` | sin retorno | > `range_max` (3.49 m) |
| `-inf` | demasiado cerca | < `range_min` (0.01 m); p. ej. `down` en el suelo |
| `NaN` | inválido / no disponible | sensor ausente o `ranges_extra` sin llegar en 0.5 s |

- Publica un `MultiRange` por cada `/cf231/scan`, con el último `up`/`down`.

## 5. Modelo en el simulador (`sim_crazyflie`, TP-10)

| Bloque real | Cómo se simula |
|---|---|
| Multi-ranger, ToF del Flow deck | `world.py`: raycast desde el centro del dron contra paredes, techo, suelo y cajas; los rayos horizontales giran con el yaw |
| Ruido | gaussiano, `range_noise_std` = 0.01 m (*nota mía:* supuesto, medir en Dresden) |
| Recorte | igual que Crazyswarm2: > 3.49 m → `+inf` en el scan |
| Fallos (`faults.py`) | `range_spike` (valor fijo un intervalo), `range_dropout` (`all` → no se publica nada; un sensor → `NaN`) |
| No modelado | roll/pitch, posición exacta de cada sensor (~2 cm del centro, ver `config/frames.yaml`), cono del sensor (*nota mía:* ~27° de campo de visión; el rayo único es optimista con obstáculos finos) |

## 6. Filtrado (`range_processor/range_filter.py`, TP-20)

Por sensor y por mensaje:

1. **Clasificar**: valid / no_return / too_close / invalid.
2. **Rechazar spikes**: un salto > `max_valid_jump` (1.0 m) respecto al último valor
   aceptado solo se acepta si se mantiene `spike_confirm_time` (0.3 s).
   "Sin retorno" se compara como `range_max`: pasar de 3.48 m a `+inf` es el
   obstáculo saliendo del alcance, no un spike. Encontrado en la prueba T08: sin
   esta regla, el recorte de 3.49 m de Crazyswarm2 generaba 74 falsos spikes.
3. **Mediana** de `filter_window` (3) lecturas: ~0.05 s de retraso a 20 Hz.
4. **NaN** más de `invalid_hold` (0.3 s) → salida NaN.

Compromiso: un obstáculo que aparece de golpe (salto > 1 m) se acepta 0.3 s tarde.

## 7. Uso en seguridad (`safety_supervisor`, TP-36/37)

- `d_dot` (`/hk/ranges/closing_rate`): pendiente de la distancia filtrada en 0.3 s.
  **Negativo = se acerca.** `TTC = distancia / −d_dot`; `TTC < 1.5 s` → SLOW.
- **Distancia de frenado**: HOVER si `rango − (v·reaction_time + v²/(2·max_decel)) < critical_distance`,
  con `v` = velocidad de acercamiento vista por ese sensor (`reaction_time` 0.3 s,
  `max_decel` 2 m/s², `critical_distance` 0.3 m). Moverse en paralelo a una pared no cuenta.
  Encontrado en la prueba T03: comparando solo el rango con 0.3 m, el dron a 1 m/s tocaba la pared.
- `-inf` cuenta como 0 m (lo más peligroso); `+inf` y `NaN` se ignoran.

## 8. Verificación

### 8.1 En simulación (hecho el 2026-10-08)

| Prueba | Resultado |
|---|---|
| `python3 -m pytest test/test_range_filter.py` (range_processor) | 7 passed |
| `python3 -m pytest test/test_ranges.py` (cf_adapter) | 3 passed |
| rqt_graph / `ros2 node info` contra el diagrama de la sección 2 | coincide |
| T03_static_wall, dron hacia la pared a 1 m/s | SLOW → HOVER, sin colisión |
| T08_range_spike (0.1 m en `front`, 0.2 s) | 1 spike descartado, supervisor en NORMAL |

Valores esperados con T03_static_wall (dron en (−1.5, 0), pared en x = −0.05, sala ±2 m, techo 2 m):

| Sensor | En el suelo (z = 0) | En vuelo (z = 0.5) |
|---|---|---|
| front | 1.45 | 1.45 |
| back | 0.50 | 0.50 |
| left / right | 2.00 | 2.00 |
| up | 2.00 | 1.50 |
| down | `-inf` (a veces ~0.01, por el ruido) | 0.50 |

### 8.2 En Dresden (pendiente)

1. `ros2 topic echo /cf231/scan --once`: confirmar orden `[back, right, front, left]`
   tapando cada sensor con la mano, uno a uno.
2. `ros2 topic echo /cf231/ranges_extra --once`: confirmar `[up, zrange]` en mm y el
   valor sin retorno (apuntar `up` al techo lejano).
3. Dron quieto a 0.5 m y a 2 m de una pared, 30 s: desviación típica → `range_noise_std`.
4. Alcance real en la sala → `range_max`.
5. Frecuencia real: `ros2 topic hz /hk/adapter/ranges` → `range_timeout` del supervisor.

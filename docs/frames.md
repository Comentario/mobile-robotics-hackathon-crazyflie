# Frames de coordenadas (TP-13)

Qué sistemas de coordenadas usa el stack, cómo se relacionan y con qué convenciones. Complementa la sección "Frames (TF)" de [interfaces.md](interfaces.md).

## Convenciones

| Tema | Convención | Referencia |
|---|---|---|
| Unidades | Metros, segundos y radianes (SI). | Regla 5 de interfaces.md |
| Ejes de `map` y `base_link` | **x adelante, y izquierda, z arriba** (mano derecha). | REP-103 |
| Ángulos | `yaw` positivo = giro **antihorario** visto desde arriba (de x hacia y). `yaw = 0` mira hacia +x de `map`. | REP-103 |
| Ejes de un sensor de rango | El **eje x del frame del sensor apunta en la dirección del rayo**. | `sensor_msgs/Range` |
| Valores de rango | `+inf` = sin retorno, `-inf` = demasiado cerca, `NaN` = inválido. | REP-117 |

## Árbol de frames

```text
map ──(state_monitor, dinámico)──> base_link ──(estáticos, frames.yaml)──> front_range
                                                                        ├─> back_range
                                                                        ├─> left_range
                                                                        ├─> right_range
                                                                        ├─> up_range
                                                                        └─> down_tof
```

| Frame | Padre | Qué representa | Quién lo publica | Tipo |
|---|---|---|---|---|
| `map` | — | La sala. Origen fijo en el suelo. Todo el stack trabaja aquí (waypoints, mapa, ruta, setpoints). | — (raíz) | — |
| `base_link` | `map` | El centro del dron. Se mueve con él. | `state_monitor`, a partir de `/hk/state` | Dinámico |
| `front_range` … `up_range` | `base_link` | Cada sensor del Multi-ranger. | `static_transform_publisher` (launch, desde `frames.yaml`) | Estático |
| `down_tof` | `base_link` | ToF inferior del Flow Deck (altura sobre el suelo). | Igual que los anteriores | Estático |

**`odom` no se usa por ahora:** la pose del EKF onboard ya llega fusionada en un único frame global (ver interfaces.md).

## Posición de los sensores en el dron

Valores de `hackathon_bringup/config/frames.yaml` (`xyz` en m, `rpy` en rad):

| Frame | xyz | rpy | Hacia dónde apunta el rayo |
|---|---|---|---|
| `front_range` | (0.02, 0, 0.03) | (0, 0, 0) | +x de `base_link` (delante) |
| `back_range` | (-0.02, 0, 0.03) | (0, 0, π) | -x (detrás) |
| `left_range` | (0, 0.02, 0.03) | (0, 0, π/2) | +y (izquierda) |
| `right_range` | (0, -0.02, 0.03) | (0, 0, -π/2) | -y (derecha) |
| `up_range` | (0, 0, 0.04) | (0, -π/2, 0) | +z (arriba) |
| `down_tof` | (0, 0, -0.01) | (0, π/2, 0) | -z (abajo) |

Los offsets son **aproximados**: hay que medirlos sobre el Crazyflie real con el Multi-ranger montado (TODO en `frames.yaml`). Un error de 1–2 cm es despreciable frente al inflado de obstáculos (`obstacle_inflation_radius: 0.25`); el **sentido** de cada rayo sí es crítico.

## Relación con Crazyswarm2 y el hardware

- Crazyswarm2 publica su propio TF `world -> cfXXX`. **El stack no lo usa**: `cf_adapter` convierte la pose a `/hk/adapter/odom` en `map`, igual que el simulador.
- Con **Lighthouse**, el origen de `map` es el origen del espacio calibrado por las estaciones base.
- Con solo **Flow Deck**, la pose es relativa al punto de despegue: el origen de `map` es donde despega el dron.

## Cómo verificarlo

Con `simulation.launch.py` en marcha:

```bash
ros2 run tf2_ros tf2_echo map base_link        # dónde está el dron en la sala
ros2 run tf2_ros tf2_echo base_link left_range # dónde está un sensor en el dron
ros2 run tf2_tools view_frames                 # genera frames.pdf con el árbol completo
```

## Estado y pendientes

- [x] Convenciones, árbol y offsets documentados.
- [x] TF estáticos de los 6 sensores publicados por el launch.
- [ ] `map -> base_link` publicado por `state_monitor`.
- [ ] Medir los offsets reales en el dron del evento.
- [ ] Verificar signos con el dron real: mover el dron hacia delante debe aumentar x, y un obstáculo a la izquierda debe aparecer en `left_range` (runbook del evento, paso 6).

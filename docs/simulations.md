# Plan de simulación y pruebas

Qué simulamos, con qué herramienta y cómo se decide si un escenario pasa. Cubre la test matrix T01–T18 de [roadmap.md](roadmap.md#test-matrix-mínima).

## Cuatro niveles

| Nivel | Qué es | Qué valida | Qué **no** valida | Prioridad |
|---|---|---|---|---|
| **0 — Tests unitarios** | pytest sobre los módulos de lógica pura (`ros2_ws/src/*/test/`). | Algoritmos aislados: filtro de rangos, salud, A*, evitación, safety, geofence. | Integración, tiempos, ROS. | P0 |
| **1 — Simulador propio** | `simulation_adapter`: modelo cinemático + mundo 2D con raycast de los 5 rangers + inyección de fallos. | Stack completo de punta a punta, escenarios T01–T16. | Dinámica real del Crazyflie y API de Crazyswarm2. | **P0, el principal** |
| **2 — Crazyswarm2 `backend:=sim`** | Firmware real en software-in-the-loop, con `cf_adapter`. | Mapeo de comandos (takeoff, setpoints, land), nombres de topics, watchdog del firmware. | Rangos: no hay Multi-ranger simulado. | P0 (corto) |
| **3 — Replay de rosbag** | `ros2 bag play` de ejecuciones grabadas. | Diagnóstico offline, regresiones (T18). | Nada nuevo; reproduce lo grabado. | P1 |

**¿Por qué no Gazebo o Webots?** Montar el modelo con Multi-ranger cuesta días y añade dependencias. Para navegar en 2D con cinco rayos basta el nivel 1, rápido, determinista y con fallos inyectables. Queda como stretch goal si el nivel 1 se queda corto.

## Cómo ejecutar

```bash
# Compilar (desde la raíz del repo, en WSL)
./scripts/build.sh
source ros2_ws/install/setup.bash

# Nivel 0
cd ros2_ws && colcon test && colcon test-result --verbose

# Nivel 1: un escenario con RViz
ros2 launch hackathon_bringup simulation.launch.py scenario:=T05_popup_obstacle rviz:=true

# Nivel 1: un escenario con evaluación pass/fail
ros2 launch hackathon_bringup simulation.launch.py scenario:=T09_range_dropout evaluate:=true

# Nivel 1: toda la matriz
./scripts/run_all_scenarios.sh

# Nivel 2: firmware SIL de Crazyswarm2 (dos terminales)
ros2 launch crazyflie launch.py backend:=sim
ros2 launch hackathon_bringup real.launch.py params:=simulation.yaml overlay:=cs2_sim.yaml mission:=missions/single.yaml

# Nivel 3: grabar y reproducir
./scripts/record_bag.sh T05
ros2 bag play data/bags/<carpeta>
```

## Estructura de un escenario

Un escenario es mundo + misión + fallos + resultado esperado. Ficheros en [`ros2_ws/src/hackathon_bringup/config/`](../ros2_ws/src/hackathon_bringup/config/):

```yaml
# config/scenarios/T09_range_dropout.yaml
world: worlds/wall.yaml          # obstáculos (simulation_adapter/world.py)
mission: missions/line.yaml      # waypoints
start: [-1.5, 0.0, 0.0, 0.0]     # x, y, z, yaw iniciales
timeout: 60.0
faults:                          # simulation_adapter/faults.py
  - {type: range_dropout, sensor: all, t_start: 5.0, t_end: 8.0}
expected:                        # hackathon_tests/scenario.py
  max_collisions: 0
  must_see_safety: [SLOW, HOVER]
```

Fallos disponibles: `range_spike`, `range_dropout`, `pose_delay`, `pose_freeze`, `pose_jump`, `pose_dropout`, `command_pause`.

Claves de `expected`: `mission_state`, `max_collisions`, `min_obstacle_distance`, `must_see_safety`, `must_see_health`, `must_see_planner`, `max_abs_xy`, `max_z`, `final_z_max`. Las colisiones y la distancia mínima se calculan contra la pose real del simulador (`/hk/sim/ground_truth`), no contra la estimada.

Para añadir un escenario, copien uno existente, cambien mundo, fallos y `expected`, y añadan una fila a la tabla siguiente.

## Catálogo de escenarios

| Escenario | Test | Mundo | Fallo | Comprueba | Necesita | Prioridad |
|---|---|---|---|---|---|---|
| `T01_single_waypoint` | T01 | empty | — | Llega al waypoint | VS-1 | P0 |
| `T02_multi_waypoint` | T02 | empty | — | Recorre los waypoints en orden | VS-1 | P0 |
| `T03_static_wall` | T03 | wall | — | Rodea la pared | A* (`planner_mode: astar`) | P1 |
| `T04_corridor` | T04 | corridor | — | Margen en el pasillo | Mapa + A* | P1 |
| `T05_popup_obstacle` | T05 | popup | — | Slow/hover sin colisión | Evitación reactiva | P0 |
| `T06_vanishing_obstacle` | T06 | vanishing | — | El decay libera la ruta | Mapa con decay | P1 |
| `T07_approaching_obstacle` | T07 | approaching | — | El TTC frena | d_dot / TTC | P1 |
| `T08_range_spike` | T08 | empty | `range_spike` | El spike no detiene la misión | Filtro de rangos | P0 |
| `T09_range_dropout` | T09 | wall | `range_dropout` | Degrada sin chocar | Watchdog de rangos | P0 |
| `T10_pose_delay` | T10 | empty | `pose_delay` | Health DEGRADED/LOST | Health monitor | P0 |
| `T11_pose_freeze` | T11 | empty | `pose_freeze` | LOST → hover | Health + safety | P0 |
| `T12_localization_lost` | T12 | empty | `pose_dropout` | Hover → land | Safety (`lost_policy`) | P0 |
| `T13_no_path` | T13 | blocked_goal | — | NO_PATH, no avanza a ciegas | A* | P1 |
| `T14_geofence` | T14 | empty | — | Setpoint recortado | Geofence | P0 |
| `T15_altitude_envelope` | T15 | low_ceiling | — | No supera z_max ni el techo | Envelope + `up_range` | P0 |
| `T16_command_stop` | T16 | empty | `command_pause` | Watchdog / hover | Watchdog del sim | P0 |
| — | T17 | — | Matar un nodo | Ver procedimiento manual | — | P1 |
| — | T18 | — | — | Replay de rosbag | — | P1 |

Los escenarios P1 fallarán con `planner_mode: direct`: es lo esperado mientras A* no exista.

## Procedimientos manuales

**T17 — reinicio de un nodo.** Con `T02_multi_waypoint` en marcha, ejecutar `pkill -f local_navigator_node`. Esperado: sin `/hk/cmd/nav`, safety_supervisor detecta `command_timeout` → HOVER. Relanzar el nodo con `ros2 run local_navigator local_navigator_node --ros-args --params-file ...` y comprobar que la misión continúa o aborta de forma controlada. Repetir con `state_monitor` (→ LOST) y con `safety_supervisor` (→ watchdog del adapter: hover).

**T18 — replay.** Grabar `T05_popup_obstacle` con `scripts/record_bag.sh`, reproducir el bag sin el simulador y comprobar en RViz/PlotJuggler que se ve el momento del obstáculo, el cambio de `safety_state` y el replan.

## Fidelidad mínima del simulador propio

Para que los resultados del nivel 1 sirvan de algo:

- **Rangos:** raycast contra paredes, techo, suelo y obstáculos; ruido gaussiano; `+inf` más allá de `range_max`; tasa ~20 Hz.
- **Pose:** ruido pequeño, tasa ~50 Hz; los fallos se aplican **solo a la pose publicada**, nunca a la pose real.
- **Dinámica:** límites de velocidad y aceleración; respuesta de primer orden al setpoint.
- **Watchdog:** si no llegan setpoints durante `command_watchdog`, hover. Ajustar al comportamiento real del firmware en cuanto se confirme (TP-08).
- **Colisión:** si la pose real toca un obstáculo, se cuenta y se publica; el escenario falla con `max_collisions: 0`.

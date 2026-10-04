# Interfaces del stack (TP-06, TP-09)

Contrato entre paquetes. Si cambian un topic, un mensaje o un servicio, actualicen este documento **en el mismo PR**.

## Reglas

1. **Solo `cf_adapter` conoce Crazyswarm2.** Ningún otro paquete usa topics o servicios `/cfXXX/...` (TP-09).
2. **`simulation_adapter` y `cf_adapter` exponen exactamente la misma API** (`/hk/adapter/*`, `/hk/takeoff`, `/hk/land`, `/hk/emergency`). Pasar de simulación a hardware consiste en cambiar el adapter (VS-5).
3. **Solo `safety_supervisor` publica `/hk/cmd/safe`**, que es lo único que escuchan los adapters. `local_navigator` solo *propone* comandos en `/hk/cmd/nav`.
4. Todo el stack consume **un único estado**: `/hk/state` (TP-14).
5. Unidades SI (m, s, rad), ejes REP-103 (x adelante, y izquierda, z arriba) y rangos según REP-117 (`+inf` sin retorno, `-inf` demasiado cerca, `NaN` inválido).
6. La lógica va en módulos de **Python puro** (sin `rclpy`) dentro de cada paquete, y el `*_node.py` solo hace de envoltorio ROS. Así la lógica se prueba con pytest sin lanzar ROS.

## Grafo de nodos

```mermaid
flowchart LR
  AD["simulation_adapter<br/>o cf_adapter"]
  SM[state_monitor]
  RP[range_processor]
  WM[local_world_model]
  MM[mission_manager]
  GP[global_planner]
  PP[path_processor]
  LN[local_navigator]
  SS[safety_supervisor]
  TL[telemetry]
  RS[hackathon_tests<br/>run_scenario]

  AD -- /hk/adapter/odom --> SM
  AD -- /hk/adapter/ranges --> RP
  SM -- /hk/state --> RP & WM & MM & GP & LN & SS
  SM -- /hk/localization_health --> MM & LN & SS
  RP -- /hk/ranges --> WM & LN & SS
  WM -- /hk/map --> GP & PP
  MM -- /hk/goal --> GP
  GP -- /hk/path --> PP
  GP -- /hk/planner_status --> MM & SS
  PP -- /hk/path/processed --> LN
  LN -- /hk/replan_request --> GP
  LN -- /hk/cmd/nav --> SS
  SS -- /hk/cmd/safe --> AD
  SS -- /hk/safety_state --> MM
  TL -- /hk/metrics --> RS
```

## Topics

| Topic | Tipo | Publica | Suscriben |
|---|---|---|---|
| `/hk/adapter/odom` | `nav_msgs/Odometry` | adapter | state_monitor |
| `/hk/adapter/ranges` | `hackathon_interfaces/MultiRange` | adapter | range_processor |
| `/hk/state` | `nav_msgs/Odometry` | state_monitor | casi todos |
| `/hk/localization_health` | `hackathon_interfaces/LocalizationHealth` | state_monitor | mission_manager, local_navigator, safety_supervisor, telemetry |
| `/hk/ranges` | `hackathon_interfaces/MultiRange` | range_processor | local_world_model, local_navigator, safety_supervisor, telemetry |
| `/hk/ranges/closing_rate` | `hackathon_interfaces/MultiRange` | range_processor | local_navigator, safety_supervisor |
| `/hk/ranges/markers` | `visualization_msgs/MarkerArray` | range_processor | RViz |
| `/hk/map` | `nav_msgs/OccupancyGrid` | local_world_model | global_planner, path_processor, RViz |
| `/hk/goal` | `geometry_msgs/PoseStamped` | mission_manager | global_planner, RViz |
| `/hk/mission_status` | `hackathon_interfaces/MissionStatus` | mission_manager | telemetry, run_scenario |
| `/hk/path` | `nav_msgs/Path` | global_planner | path_processor, RViz |
| `/hk/planner_status` | `hackathon_interfaces/PlannerStatus` | global_planner | mission_manager, safety_supervisor, telemetry |
| `/hk/path/processed` | `nav_msgs/Path` | path_processor | local_navigator, RViz |
| `/hk/replan_request` | `std_msgs/String` (motivo) | local_navigator | global_planner |
| `/hk/cmd/nav` | `hackathon_interfaces/Setpoint` | local_navigator | safety_supervisor |
| `/hk/cmd/safe` | `hackathon_interfaces/Setpoint` | safety_supervisor | adapter |
| `/hk/safety_state` | `hackathon_interfaces/SafetyState` | safety_supervisor | mission_manager, telemetry |
| `/hk/metrics` | `std_msgs/String` (JSON) | telemetry | run_scenario |
| `/hk/sim/ground_truth` | `nav_msgs/Odometry` | simulation_adapter | telemetry (solo en simulación) |
| `/hk/sim/obstacles` | `visualization_msgs/MarkerArray` | simulation_adapter | RViz (solo en simulación) |

## Servicios

| Servicio | Tipo | Servidor | Clientes |
|---|---|---|---|
| `/hk/takeoff` | `hackathon_interfaces/Takeoff` | adapter | mission_manager |
| `/hk/land` | `hackathon_interfaces/Land` | adapter | mission_manager, safety_supervisor |
| `/hk/emergency` | `std_srvs/Trigger` | adapter | safety_supervisor, operador |
| `/hk/mission/start`, `pause`, `resume`, `abort` | `std_srvs/Trigger` | mission_manager | operador |

## Mensajes propios (`hackathon_interfaces`)

| Mensaje | Contenido |
|---|---|
| `MultiRange` | `front`, `back`, `left`, `right`, `up`, `down` (ToF del Flow Deck), `range_min`, `range_max`; REP-117. |
| `Setpoint` | `mode` (POSITION / VELOCITY / HOVER), `position`, `velocity`, `yaw`; frame `map`. |
| `LocalizationHealth` | `status` (GOOD / DEGRADED / LOST), `pose_age`, `reason`. |
| `SafetyState` | `state` (NORMAL / SLOW / HOVER / LAND / ABORT), `speed_scale`, `reason`. |
| `MissionStatus` | `state` (IDLE … ABORTED), `waypoint_index`, `waypoint_count`, `reason`. |
| `PlannerStatus` | `status` (IDLE / OK / NO_PATH / TIMEOUT / NO_MAP), `replan_count`, `plan_time`, `reason`. |
| `Takeoff.srv`, `Land.srv` | Altura y duración → `success`, `message`. |

## Frames (TF)

```text
map ──(state_monitor)──> base_link ──(static, config/frames.yaml)──> front_range
                                                                  ├─> back_range
                                                                  ├─> left_range
                                                                  ├─> right_range
                                                                  ├─> up_range
                                                                  └─> down_tof
```

Crazyswarm2 publica su propio TF (`world -> cfXXX`). No lo usamos directamente: `cf_adapter` convierte la pose a `/hk/adapter/odom` y `state_monitor` publica `map -> base_link`, igual que en simulación. Por eso `odom` no se usa por ahora (TP-13).

## Lado Crazyswarm2 (pendiente de TP-08)

Por confirmar con `ros2 topic list` / `ros2 service list` durante un vuelo, ya sea con `backend:=sim` o con hardware:

- La pose, la odometría y el `scan` del Multi-ranger solo se publican si se activan en `firmware_logging` de `crazyflies.yaml`. Eso explicaría por qué no aparecieron en la prueba de `hello_world`.
- Setpoints de streaming (`cmd_position`, `cmd_hover`, `cmd_full_state`…): frecuencia mínima y watchdog del firmware.
- El orden de los rangos dentro del `scan` y cómo se representa "sin retorno".

Todo eso se resuelve dentro de [`cf_adapter/mapping.py`](../ros2_ws/src/cf_adapter/cf_adapter/mapping.py).

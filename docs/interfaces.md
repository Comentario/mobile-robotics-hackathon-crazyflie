# Interfaces del stack (TP-06, TP-09)

Contrato entre paquetes. Si cambian un topic, un mensaje o un servicio, actualicen este documento **en el mismo PR**.

Las secciones [Conceptos de ROS 2](#conceptos-de-ros-2) a [Sensores: qué llega al stack](#sensores-qué-llega-al-stack) son una guía para entender cómo está organizado el stack. A partir de [Reglas](#reglas) empieza el contrato en sí.

## Conceptos de ROS 2

| Concepto | Qué es | Analogía |
|---|---|---|
| **Paquete** | Carpeta dentro de `ros2_ws/src/` con su código, `package.xml` y `setup.py`. Es la unidad que compila `colcon`. | Un módulo o librería |
| **Nodo** | Programa que se ejecuta. Cada paquete de este stack tiene uno (`*_node.py`). | Un proceso o microservicio |
| **Topic** | Canal con nombre (`/hk/state`) en el que un nodo **publica** datos de forma continua y otros se **suscriben**. | Una emisora de radio |
| **Mensaje** | Formato de los datos de un topic: estándar de ROS (`nav_msgs/Odometry`) o propio (`hackathon_interfaces/MultiRange`). | El esquema de un JSON |
| **Servicio** | Petición puntual con respuesta, por ejemplo "despega a 0.5 m" → "ok". | Una llamada a una API |
| **Frame (TF)** | Sistema de coordenadas con nombre (`map`, `base_link`…). ROS convierte posiciones de uno a otro. | "Delante del dron" frente a "norte del mapa" |
| **Launch file** | Script que arranca varios nodos a la vez con su configuración. | Un `docker-compose` |
| **Workspace** | `ros2_ws/`: agrupa todos los paquetes; `colcon build` los compila juntos. | Un monorepo |

## Recorrido de los datos

Un ciclo completo va del dron (o del simulador) al comando que vuelve al dron:

1. **Sensar:** el adapter publica la pose estimada (`/hk/adapter/odom`) y los rangos (`/hk/adapter/ranges`).
2. **Normalizar:** `state_monitor` publica el estado único (`/hk/state`) y la salud de la localización. `range_processor` publica los rangos filtrados (`/hk/ranges`).
3. **Modelar el entorno:** `local_world_model` convierte los rangos en un mapa local (`/hk/map`).
4. **Decidir el objetivo:** `mission_manager` publica el waypoint actual (`/hk/goal`).
5. **Planificar:** `global_planner` calcula la ruta (`/hk/path`) y `path_processor` la simplifica (`/hk/path/processed`).
6. **Proponer un comando:** `local_navigator` sigue la ruta, esquiva lo que aparezca y propone un setpoint (`/hk/cmd/nav`).
7. **Validar:** `safety_supervisor` acepta, reduce o anula el comando y publica el definitivo (`/hk/cmd/safe`).
8. **Actuar:** el adapter envía el setpoint al Crazyflie (o lo integra en el simulador). El control de actitud y motores sigue siendo onboard.

El diagrama completo está en [Grafo de nodos](#grafo-de-nodos).

Dos ideas de diseño:

- **Solo el adapter cambia entre simulación y hardware.** El resto del stack no sabe si vuela un dron real o el simulador, así que se puede desarrollar todo sin hardware.
- **El navigator propone y el supervisor decide.** El dron solo escucha `/hk/cmd/safe`, que únicamente publica `safety_supervisor`. Un fallo en la navegación no llega al dron sin pasar por el filtro de seguridad.

## Qué hace cada paquete

| Módulo | Paquete | Qué hace | Pregunta que responde | TPs |
|---|---|---|---|---|
| 1 | `simulation_adapter` | **Dron simulado**: publica pose y rangos sintéticos (raycast contra obstáculos en YAML) e integra los setpoints recibidos. Inyección de fallos de escenarios disponible; campaña TP-41 pendiente. | ¿Dónde estoy y qué veo? (simulación) | TP-10 |
| 1 | `cf_adapter` | **Dron real**: traduce entre Crazyswarm2 y la API `/hk/*`. Es el único paquete que conoce Crazyswarm2. | ¿Dónde estoy y qué veo? (hardware) | TP-08, TP-09 |
| 2 | `state_monitor` | Publica el estado único (`map → base_link`) y la salud de la localización: `GOOD` / `DEGRADED` / `LOST`. No es un EKF nuevo. | ¿Me puedo fiar de mi posición? | TP-13 a TP-17 |
| 3 | `range_processor` | Valida y filtra los rangos (NaN, fuera de rango, picos) y los convierte en rayos con dirección. | ¿Qué obstáculos detecto ahora? | TP-19 a TP-22 |
| 4 | `local_world_model` | Rejilla 2D alrededor del dron (libre/ocupado/desconocido) con inflado y expiración de obstáculos viejos. | ¿Cómo es el entorno cercano? | TP-23 a TP-25 |
| 0 | `mission_manager` | Lee los waypoints del YAML y lleva la máquina de estados de la misión (despegar, navegar, aterrizar, abortar). | ¿Adónde tengo que ir ahora? | TP-26 a TP-28 |
| 5 | `global_planner` | A* sobre el mapa local, con replan, timeout y estado de fallo. | ¿Por dónde voy? | TP-29 a TP-31 |
| 6 | `path_processor` | Simplifica la ruta (shortcut por línea de visión), genera el lookahead y limita la velocidad. | ¿Cómo suavizo la ruta? | TP-32, TP-33 |
| 7 | `local_navigator` | Sigue la ruta, aplica evitación reactiva y pide replan si hace falta. | ¿Qué hago en este instante? | TP-34 a TP-36 |
| 8 | `safety_supervisor` | Anula la navegación normal (`SLOW`, `HOVER`, `LAND`, `ABORT`) por obstáculo cercano, pose vieja, pérdida de sensores, geofence, altura o planner sin ruta. | ¿Es seguro hacer esto? | TP-37 a TP-40 |
| 9 | `telemetry` | Métricas de frecuencia, latencia y distancia mínima. | ¿Qué tal ha ido? | TP-41 |
| 9 | `hackathon_tests` | Ejecuta los escenarios T01–T16 (pared, pasillo, obstáculo que aparece, pérdida de localización…) y evalúa el resultado. | ¿El stack aguanta? | TP-41 |
| – | `hackathon_bringup` | Sin nodo propio: launch files y configuración YAML (mundos, misiones, escenarios, frames). | ¿Cómo arranco todo? | TP-12 |
| – | `hackathon_interfaces` | Sin nodo: define los mensajes y servicios propios. | ¿Qué formato tienen los datos? | TP-06 |

La columna "Módulo" es la numeración de la arquitectura del [roadmap](roadmap.md#módulos-y-paquetes).

## Estructura de un paquete

Todos los paquetes Python siguen el mismo patrón (ejemplo: `state_monitor`):

```text
state_monitor/
├── package.xml                 # nombre y dependencias (lo lee colcon)
├── setup.py / setup.cfg        # instalación del paquete y ejecutables
├── resource/state_monitor      # marcador vacío que ROS usa para encontrar el paquete
├── state_monitor/
│   ├── state_monitor_node.py   # envoltorio ROS: parámetros, publishers, subscribers, timers
│   └── health.py               # lógica pura, sin rclpy
└── test/
    └── test_health.py          # tests pytest de la lógica pura
```

La lógica se separa del nodo (regla 6) para poder probarla con `pytest` en milisegundos, sin lanzar ROS. El nodo solo conecta esa lógica con los topics.

**Estado actual:** `cf_adapter` implementa TP-09 y tiene pruebas de conversiones, comunicación ROS y conexión al SIL. TP-10 implementa `simulation_adapter`: POSITION/VELOCITY XY/HOVER, yaw, velocidad/aceleración limitadas, servicios de vuelo y seis rangos sintéticos contra un mundo YAML. Publica odometría, ground truth, TF y obstáculos; 79 pruebas pasan. Se conserva e integra la inyección de fallos aportada en main; la campaña completa sigue pendiente en TP-41. `mission_manager` implementa TP-26 (aporte de main, 8 pruebas de lógica); `state_monitor` reenvía la odometría a `/hk/state`; navegación y seguridad siguen mayormente como esqueletos. La numeración de referencia está en el roadmap.

## Sensores: qué llega al stack

| Sensor | Dónde aparece | Notas |
|---|---|---|
| Lighthouse | Indirectamente en `/hk/adapter/odom` | El EKF onboard lo fusiona en la pose. |
| Flow Deck: sensor de flujo óptico (PMW3901) | Indirectamente en `/hk/adapter/odom` (sobre todo en la velocidad) | Es una cámara de muy baja resolución, pero el firmware solo expone el desplazamiento medido (`motion.deltaX`, `motion.deltaY`), no la imagen. No hay topic de imagen. |
| Flow Deck: ToF inferior (VL53L1x) | `MultiRange.down` | Altura sobre el suelo. También lo usa el EKF. |
| Multi-ranger | `MultiRange.front/back/left/right/up` | Cinco rayos; ver REP-117 en el mensaje. |
| IMU | Indirectamente en `/hk/adapter/odom` | Solo la usan el EKF y el controlador onboard. |

Si hiciera falta el flujo óptico crudo (por ejemplo, para diagnosticar superficies sin textura), `cf_adapter` puede pedirlo a Crazyswarm2 como variables de log (`motion.deltaX`, `motion.deltaY`, `motion.squal`). Hoy no forma parte de la API.

## Reglas

1. **Solo `cf_adapter` conoce Crazyswarm2.** Los nodos de misión, navegación y seguridad no usan topics o servicios `/cfXXX/...` (TP-09). Los simuladores y `sim_sensors` producen/emulan la API externa.
2. **`simulation_adapter` y `cf_adapter` deben exponer la misma API** (`/hk/adapter/*`, `/hk/takeoff`, `/hk/land`, `/hk/emergency`). Ambos adapters implementan esos endpoints; las limitaciones físicas y del backend se documentan abajo. Pasar de simulación a hardware consiste en cambiar el adapter (VS-5).
3. **Solo `safety_supervisor` publica `/hk/cmd/safe`**, que es lo único que escuchan los adapters. `local_navigator` solo *propone* comandos en `/hk/cmd/nav`.
4. Todo el stack consume **un único estado**: `/hk/state` (TP-14).
5. Unidades SI (m, s, rad), ejes REP-103 (x adelante, y izquierda, z arriba) y rangos según REP-117 (`+inf` sin retorno, `-inf` demasiado cerca, `NaN` inválido).
6. La lógica va en módulos de **Python puro** (sin `rclpy`) dentro de cada paquete, y el `*_node.py` solo hace de envoltorio ROS. Así la lógica se prueba con pytest sin lanzar ROS.
7. **Parámetros:** nombres en `snake_case`, en unidades SI (`state_timeout` en s, `z_max` en m). Los valores viven en `hackathon_bringup/config/*.yaml`, no en el código: la sección `/**` tiene los compartidos (`map_frame`, `base_frame`, `z_min`…) y cada nodo tiene su sección con su nombre. El nodo declara cada parámetro con un valor por defecto y lo lista en la cabecera de su `*_node.py`.

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

En el simulador propio, `success=true` confirma aceptación, no llegada. El modelo
`KinematicDrone` separa GROUND, TAKING_OFF, FLYING, LANDING y EMERGENCY. Solo FLYING
acepta referencias de navegación; para salir de GROUND debe solicitarse takeoff.
Despegue/aterrizaje generan una referencia vertical suave durante la duración
solicitada y esperan posición/velocidad dentro de `position_tolerance` y
`settle_speed`; los límites del modelo pueden prolongar la llegada. Landing tiene
prioridad sobre navegación y puede interrumpir un despegue. Al contactar el suelo
se fija Z=0 y velocidad cero. Emergency interrumpe cualquier maniobra y simula
caída por `gravity`, sin control de motores ni drag, hasta contactar el suelo;
queda enclavada hasta reiniciar el nodo. No reproduce el firmware de emergencia
ni una dinámica 6DOF. En vuelo libre, perder comandos captura HOVER; ese watchdog
no cancela las maniobras de servicio, que no requieren un stream de setpoints.

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

Crazyswarm2 publica su propio TF (`world -> cfXXX`). En SIL, `cf_adapter` puede leerlo para publicar `/hk/adapter/odom`; en hardware puede consumir odometría o pose. El resto del stack consume `/hk/state`, y `state_monitor` publica `map -> base_link`. El frame llamado `odom` no se usa por ahora; el mensaje `nav_msgs/Odometry` sí.

## Adapter Crazyswarm2 — TP-09

TP-08 y las limitaciones del SIL se documentan en [crazyflie_command_interface.md](crazyflie_command_interface.md). Solo `cf_adapter` conoce los endpoints del robot; sus nombres pueden cambiar mediante parámetros sin modificar los paquetes de los colegas.

| Parámetro | Uso / valor por defecto |
|---|---|
| `robot_name`, `group_mask` | Robot `cf231`, grupo `0` |
| `map_frame`, `base_frame`, `source_frame` | `map`, `base_link`, `world` |
| `state_source` | `odom` (por defecto), `pose` o `tf` |
| `odom_topic`, `pose_topic`, `scan_topic` | Vacío usa `/<robot>/odom`, `/<robot>/pose`, `/<robot>/scan` |
| `odom_linear_frame`, `odom_angular_units` | Convención de entrada: `body`/`world`, `radians`/`degrees`. Por defecto, ROS estándar: `body`, `radians` |
| `vertical_ranges_topic` | Vacío desactiva datos verticales. Si se configura, recibe `LogDataGeneric` con **exactamente** `[range.up, range.zrange]` en mm |
| `state_timeout`, `range_timeout`, `command_timeout` | Edad máxima en segundos de estado, rangos y comandos; `0.3`, `0.3`, `0.5` |
| `service_timeout` | `2.0` s de reloj monotónico; funciona aunque se pause `/clock` |
| `reference_initial_dt`, `reference_max_dt` | Integración de velocidad: primer paso `0.05` s; máximo intervalo `0.1` s |
| `land_height`, `streaming_valid_ms` | Altura final `0.0` m; argumento de notify `100` ms, cuyo efecto depende del firmware (ver TP-08) |

**Frames y estado:** `source_frame` debe coincidir en origen y orientación con `map_frame`; el adapter no realiza una transformación de calibración entre ambos. Esta correspondencia se debe confirmar en TP-13 y en hardware. Preserva timestamps y rechaza estado vencido, de otro robot/frame o con pose inválida. Odometría estándar conserva twist en el frame del cuerpo; el servidor `cflib` instalado entrega velocidad lineal global y gyro en grados/s, por lo que `event.yaml` configura `world`/`degrees` y se transforman también las covarianzas. Confirmar esas convenciones al elegir otro backend. Fuentes de pose/TF no miden velocidad: se publica twist cero con covarianza alta, no una medición fiable de velocidad cero.

**Rangos:** el servidor `cflib` instalado publica cuatro rayos `[back, right, front, left]` con ángulo inicial `-pi` y paso `pi/2`. El adapter identifica direcciones por los ángulos y conserva valores REP-117. Los rangos `up` y `down` son `NaN` si no hay logging vertical válido; no se inventan medidas. El logging vertical opcional se convierte de mm a m y se combina al recibir un scan horizontal. Si contribuye información vertical, el mensaje usa el timestamp más antiguo de ambos streams; los datos verticales vencidos se descartan. La frecuencia de salida sigue la del scan horizontal.

**Comandos:** solo se acepta `/hk/cmd/safe` en `map_frame`, con timestamp reciente y estado válido. POSITION se convierte a FullState. HOVER captura posición/yaw medidos al entrar y mantiene esa referencia. VELOCITY integra XY hacia referencias absolutas y añade velocidad como feedforward; mantiene `position.z` y exige `velocity.z=0`. No es control nativo exclusivamente por velocidad. Un intervalo excesivo o retroceso del reloj reinicia/rechaza la integración; no se regeneran comandos cuando el supervisor deja de publicar. El filtrado de obstáculos, geofence y watchdog de misión corresponden a `safety_supervisor`.

**Servicios:** `success=true` significa que Crazyswarm2 respondió, no que acabó la maniobra. Se espera su respuesta con timeout; fallos y servicios ausentes devuelven `success=false`. Durante el cambio a órdenes de alto nivel se bloquean setpoints. Si había streaming, notify debe responder antes de enviar la siguiente orden; su efecto físico sigue dependiendo del backend. Después de aterrizaje/emergencia o un fallo, el streaming queda bloqueado. Un nuevo despegue confirmado lo habilita cuando transcurre su duración solicitada. La emergencia invalida transiciones pendientes, pero no puede cancelar una orden ya enviada al servidor ni garantizar un efecto que el backend no implemente.

**Configuración:** activar `odom` (o `pose`) y `scan` en `firmware_logging` del servidor real, y el bloque vertical opcional con el orden indicado. Sin ellos el adapter no publica datos ficticios. En SIL usar el overlay `cs2_sim.yaml`: `state_source=tf`, `use_sim_time=true`; no hay rangos reales. Estos requisitos de hardware se confirman con los mentores, no mediante pruebas sintéticas.

Pruebas desde la raíz del repo, después de compilar los paquetes:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/ros2_ws/src/cf_adapter:/home/asus/crazyflie-firmware/build${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/tp09-ros-logs
/usr/bin/python3 -m pytest ros2_ws/src/cf_adapter/test -q
```

Usar un dominio ROS de pruebas sin otros nodos y adaptar la ruta de bindings a cada equipo. Las pruebas ROS usan telemetría y servicios sintéticos; la prueba SIL usa callbacks del servidor **a través del transporte ROS y del adapter**, con física `np`. Sin bindings instalados, la prueba SIL se omite explícitamente. Ninguna prueba conecta Crazyradio ni valida vuelo real.


## Simulador propio (TP-10)

`simulation_adapter` sustituye a `cf_adapter`; arrancar solo uno por dominio ROS.
POSITION sigue XYZ y yaw; VELOCITY sigue velocidad XY en `map`, mantiene
`position.z` y rechaza `velocity.z != 0`; HOVER captura posición y yaw al entrar.
La velocidad vectorial y su cambio están limitados por `max_speed` y `max_accel`;
el yaw recorre el camino angular más corto con límite `max_yaw_rate`. Puede haber
sobrepaso por inercia. Se rechazan comandos de otro frame, vencidos, futuros,
fuera de orden, no finitos o de un modo desconocido.

`world_file` carga bounds, ceiling y cajas desde YAML; sin archivo usa una sala
vacía de 4×4×2 m. `scenario_file` aporta start=[x,y,z,yaw]. Cajas con altura,
velocidad y ventana [t_start,t_end) usan segundos relativos al arranque. El
modelo avanza 1/sim_rate por tick; bajo carga puede avanzar más lento que el
reloj ROS, usado para timestamps y watchdog. No publica `/clock`.

Los seis rayos consideran paredes, cajas, techo y suelo. Los offsets nominales
son los de `config/frames.yaml`, todavía sujetos a medición en TP-13; se rotan
con yaw. Rangos en metros, frame `base_link`, a `range_rate`: +inf fuera de
`range_max`, -inf por debajo de `range_min`. Un sensor fuera de la sala devuelve
-inf. Los obstáculos son geometría de percepción; no bloquean físicamente el
movimiento ni sustituyen la seguridad del supervisor. `World.collides` permite
consultar contacto geométrico; no se implementa dinámica de impactos.

`/hk/sim/ground_truth` publica el estado sin ruido; `/hk/adapter/odom` y su TF
`map → base_link` incluyen ruido XYZ opcional y covarianza correspondiente.
Twist se expresa en `base_link`, con velocidad angular Z en rad/s.
`range_noise_std` y `pose_noise_std` son desviaciones estándar; `random_seed`
permite reproducir las muestras. Por defecto el nodo usa ruido cero;
`simulation.yaml` configura ruido de rangos 0.01 m y pose 0.005 m.
`/hk/sim/obstacles` publica la sala y las cajas activas como MarkerArray.
Se conservan los eventos `faults` de main: range_spike/dropout, pose_delay/
freeze/jump/dropout y command_pause. Se aplican con el tiempo relativo del modelo.
Ground truth continúa durante pose_dropout; delay/freeze mantienen el timestamp
de publicación del aporte original. Hay regresiones ROS de conexión de fallos,
pero la campaña completa de escenarios y métricas de TP-41 sigue pendiente.

Verificación desde la raíz, con un dominio sin otros nodos:

```bash
bash scripts/build.sh --packages-select simulation_adapter hackathon_bringup
bash scripts/verify_tp10.sh
```

Resultado tras integrar main: 79 pruebas correctas, sin omisiones. Se comprueban geometría, sensores, límites, watchdog, modos, yaw,
servicios ROS, YAML, ruido y el ciclo despegue → navegación → rangos → aterrizaje.
Esto valida el adapter; la misión autónoma del stack se integra en TP-12 y los
TP de navegación/seguridad.


## Bringup (TP-12)

`simulation.launch.py` arranca con un solo comando el adapter, los nueve nodos
comunes y seis TF estáticos de sensores. Los argumentos `scenario`, `params`
y `overlay` eligen archivos instalados; params/overlay también aceptan rutas
absolutas. El overlay prevalece sobre la base y su `base_frame` se aplica a los
TF estáticos. Se comprueba que existan parámetros, escenario, mundo y misión
antes de iniciar procesos. Si termina un nodo, se cierra el bringup completo;
Ctrl+C cierra los procesos. `rviz:=true` añade RViz sin una escena guardada
(TP-11 pendiente). `evaluate:=true` requiere el evaluador de escenarios, todavía
pendiente junto con la misión autónoma y los TP de navegación/seguridad.

`real.launch.py` conserva la estructura común y sustituye solo el adapter.
Requiere un servidor de Crazyswarm2 externo; no arranca la radio ni se ha probado
con hardware. Parámetros base `event.yaml`, misión `missions/event.yaml`.

Uso desde la raíz, después de compilar el workspace:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch hackathon_bringup simulation.launch.py
```

Para verificar automáticamente, cerrar antes otros nodos del dominio elegido:

```bash
ROS_DOMAIN_ID=92 /usr/bin/python3 scripts/verify_tp12.py
ROS_DOMAIN_ID=92 /usr/bin/python3 scripts/verify_tp12.py --scenario T03_static_wall --stop-node
/usr/bin/python3 -m pytest ros2_ws/src/hackathon_bringup/test -q
```

Las dos verificaciones de launch pasaron, con overlays y cierre sin nodos
residuales; las dos pruebas de configuración también. La prueba no envía órdenes
de vuelo y comprueba arranque y conexiones disponibles. No confirma aún una
misión autónoma completa: TP-26 está implementado; planificación y seguridad siguen en desarrollo.


## Configuración del evento (TP-42 C/E)

`config/event.yaml` contiene parámetros del stack con `autostart=false` y
`robot_name=ROBOT_ID_TODO`. `config/event_hardware.yaml` usa el formato de
Crazyswarm2 y contiene el robot deshabilitado y URI/tipo pendientes. La URI no
es un parámetro de `cf_adapter`. Mantener robot_name y clave robots iguales,
frames alineados y waypoints dentro de los límites. Las tasas de logging,
variables verticales y tipo se confirman con el equipo entregado.

```bash
/usr/bin/python3 scripts/verify_event_config.py         # plantilla coherente + pendientes
/usr/bin/python3 scripts/verify_event_config.py --ready # código 1 mientras haya placeholders
```

El verificador comprueba archivos y consistencia (alturas, geofence, mapa,
velocidades, tiempos, logging y misión); no mide sensores ni valida que los
nodos todavía pendientes apliquen sus parámetros. Acepta `--params`, `--hardware`
y `--mission` para validar copias ajustadas. Procedimiento y responsabilidades:
[hackathon_day_runbook.md](hackathon_day_runbook.md).

Para comprobar el bringup instalado con el perfil de evento, sin radio:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ROS_DOMAIN_ID=92 /usr/bin/python3 scripts/verify_tp12.py --params event.yaml
```

Verificado el 8 de octubre: 15 pruebas de configuración, compilación y launch
con 16 nodos, API/TF y cierre sin nodos residuales. Usa el simulador y un overlay
de prueba, sin órdenes de vuelo. TP-42 D queda
pendiente; el runbook usa el servidor de Crazyswarm2 y `real.launch.py` en
terminales separadas. Los parámetros actuales son de desarrollo, no del evento.


### Integración MVP 2026-10-08

`simulation_cf.launch.py` utiliza el cf_adapter robusto de main con fuente `pose`.
El backend emulator conserva la API de persona 2; SIL usa el servidor de Crazyswarm2
y sim_sensors para publicar pose/scan/ranges_extra desde TF y raycast.
Los tópicos de la tabla anterior se mantienen. El navegador publica VELOCITY XY
con la altura del waypoint y yaw constante; cf_adapter integra referencias FullState.
MissionStatus habilita el navegador únicamente en NAVIGATE. La misión deja de
avanzar waypoints durante HOVER y descuenta ese tiempo del timeout; seguridad
sigue teniendo prioridad para aterrizar. Todos los temporizadores de control
usan reloj ROS; en SIL se configura use_sim_time=true por nodo.

Ver instrucciones reproducibles en [simulations.md](simulations.md) y límites
de sensores en [sensor_model.md](sensor_model.md).

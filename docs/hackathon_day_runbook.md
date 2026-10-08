# Runbook del evento — TP-42 E

Plantilla preparada el 8 de octubre de 2026. No certifica vuelo real. Usar una
revisión común del repositorio y completar los datos del equipo entregado.

## Responsables y archivos

- Persona 1: configuración, launch, grabación y registro de cada ensayo.
- Persona 2: frames, salud de localización, rangos y supervisor de seguridad.
- Persona 3: misión, ruta directa, seguimiento y waypoints.
- Operador de la prueba: una persona designada que conoce la parada disponible;
  acordar con los mentores espacio de ensayo y procedimiento de emergencia.

| Archivo | Uso | Confirmar antes de hardware |
|---|---|---|
| `config/event.yaml` | Parámetros del stack | Robot, frames, límites, alturas, umbrales, velocidades, mapa y tolerancias |
| `config/event_hardware.yaml` | Plantilla Crazyswarm2 | URI, robot/type, firmware, estimador, decks, logging y `enabled` |
| `config/missions/event.yaml` | Waypoints en map | Origen/ejes, orden, alturas y recorrido dentro de geofence |
| `config/frames.yaml` | TF de sensores | Offsets físicos; hoy son nominales |

Las rutas config son relativas a `ros2_ws/src/hackathon_bringup`. Los valores
numéricos actuales son de desarrollo. La URI pertenece a Crazyswarm2;
`cf_adapter.robot_name` debe coincidir exactamente con la clave `robots`.
El bloque `EVENT_TYPE_TODO` es una plantilla: sustituirlo por la configuración
del equipo recibido. No configura Lighthouse/Flow ni reemplaza la calibración.

## 1. Preparación y comprobación sin hardware

Desde la raíz, en cada terminal ROS:

```bash
source /opt/ros/humble/setup.bash
bash scripts/build.sh
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1
```

El dominio 92 es un ejemplo para pruebas en un solo equipo. Para varios equipos,
acordar el dominio y la configuración de red; `ROS_LOCALHOST_ONLY=1` impide
comunicarse con otros equipos. No compartir un dominio con pruebas ajenas.

```bash
/usr/bin/python3 scripts/verify_event_config.py
bash scripts/verify_tp10.sh
/usr/bin/python3 scripts/verify_tp12.py
```

La primera orden debe dar coherencia estática y listar los placeholders. Las
pruebas automáticas no requieren radio. TP-12 prueba arranque/conexiones; T01/T02
no cuentan como misión aprobada hasta que se integren navegación y safety.

## 2. Al recibir el dron

1. Confirmar URI, id/type del robot y revisión de firmware con los mentores.
2. Verificar Crazyradio, permisos USB, batería y los decks realmente montados.
   Crazyradio/Ubuntu nativo corresponden a TP-42 A/B; registrar pendientes.
3. Confirmar cómo se obtiene XYZ/yaw: Lighthouse, Flow u otra fuente. Verificar
   pose moviendo el dron manualmente, signos y unidades, sin ordenar despegue.
4. Confirmar frame global, origen, orientación y límites del espacio real.
   `source_frame` y `map_frame` deben compartir origen/ejes: el adapter no hace
   una calibración entre ellos. Los límites incluyen suelo Z=0 para aterrizar.
5. Verificar cinco rayos del Multi-ranger y ToF inferior si están disponibles.
   Para logging vertical, comprobar `range.up`/`range.zrange` en el firmware,
   activar el bloque comentado y configurar el topic real en `event.yaml`.
   Sin logging vertical el adapter publica NaN; no interpretar NaN como espacio libre.
6. Ejecutar primero el procedimiento/ejemplo oficial que indiquen los mentores.
   La presencia de un endpoint no prueba su efecto (ver TP-08).

Actualizar los tres YAML, offsets de sensores y configuración del estimador
entregada. Mantener `mission_manager.autostart=false`. Ajustar el envelope y
velocidades conjuntamente con personas 2/3; no desactivar rangos requeridos para
ocultar una integración incompleta.

```bash
/usr/bin/python3 scripts/verify_event_config.py --ready
```

Debe terminar con código 0 y sin placeholders. Esto verifica archivos, no la
calibración ni el comportamiento de firmware. Registrar la revisión de Git y
los valores confirmados; los waypoints de ejemplo aún necesitan aprobación.

## 3. Arranque del equipo y del stack

En dos terminales, con ROS cargado y el mismo dominio:

```bash
# Terminal A: servidor externo. Ejecutar solo tras completar event_hardware.yaml.
EVENT_CONFIG="$PWD/ros2_ws/src/hackathon_bringup/config"
ros2 launch crazyflie launch.py backend:=cflib \
  crazyflies_yaml_file:="$EVENT_CONFIG/event_hardware.yaml" \
  teleop:=False mocap:=False rviz:=False
```

Este ejemplo usa el backend cflib instalado y no mocap externo. Si los mentores
requieren mocap, otro backend o configuración de servidor/estimador, usar su
procedimiento. No modificar parámetros de firmware a ciegas.

```bash
# Terminal B: stack propio, sin iniciar misión automáticamente.
EVENT_CONFIG="$PWD/ros2_ws/src/hackathon_bringup/config"
ros2 launch hackathon_bringup real.launch.py \
  params:="$EVENT_CONFIG/event.yaml" \
  mission:="$EVENT_CONFIG/missions/event.yaml"
```

`event.launch.py` de un solo comando corresponde a TP-42 D y está pendiente.
No arrancar `simulation_adapter` y `cf_adapter` juntos en el mismo dominio.

## 4. Comprobaciones antes de iniciar misión

```bash
ros2 node list
ros2 topic echo --once /hk/state
ros2 topic echo --once /hk/adapter/ranges
ros2 topic echo --once /hk/localization_health
ros2 topic echo --once /hk/safety_state
ros2 topic info /hk/cmd/safe --verbose
ros2 service list
```

Confirmar timestamps recientes, XYZ/yaw coherentes y tasas suficientes para los
timeouts configurados; si un echo espera indefinidamente, cancelar y resolver
el origen. Salud GOOD, supervisor sin fallo, rangos válidos requeridos y un
único publisher de `/hk/cmd/safe`: `safety_supervisor`. Confirmar las conexiones
`/hk/goal → /hk/path → /hk/path/processed → /hk/cmd/nav → /hk/cmd/safe`.
Servicios takeoff/land/emergency y mission/start/abort disponibles. No publicar
comandos sintéticos directamente a `/hk/cmd/safe` en hardware.

Si faltan nodos de los colegas o entradas, no iniciar misión. La simulación no
verifica el watchdog de firmware ni el funcionamiento real de emergency.

## 5. Ensayos y registro

Comprobar en simulación primero: un waypoint (T01), varios (T02), y después
obstáculos/fallos cuando estén implementados sus TP. Un launch sin errores no
es evidencia de llegada a waypoints ni de evitación.

En hardware, avanzar por etapas según el procedimiento de mentores: despegue y
hover, T01, T02, obstáculo estático, replanning y dinámicos solo si esas capacidades
están implementadas y verificadas. Repetir una etapa fallida antes de ampliar.

```bash
# Terminal C: iniciar antes del ensayo; finalizar con Ctrl+C después de aterrizar.
bash scripts/record_bag.sh T01_evento
# Terminal de operador: iniciar misión solo después de las comprobaciones anteriores.
ros2 service call /hk/mission/start std_srvs/srv/Trigger '{}'
ros2 topic echo /hk/mission_status
```

`success=true` en takeoff/land confirma aceptación, no fin de maniobra; comprobar
altura y contacto con suelo. Registrar ensayo, commit, robot/firmware, YAML usados,
resultado, tiempo, intervención y ruta del bag. Copiar bags y configuración a
almacenamiento persistente; `/tmp/hk_runs` no es archivo definitivo.

## 6. Parada y recuperación

- Abort de misión: `/hk/mission/abort` pide terminar/aterrizar según su lógica.
  Si no responde, usar el procedimiento de parada acordado con los mentores.
- Aterrizaje directo disponible: `/hk/land`; confirmar descenso y suelo.
- `/hk/emergency` es una última medida de corte, puede causar caída; su efecto
  depende del backend/firmware. En nuestro simulador queda enclavada hasta reiniciar.
- Ctrl+C del launch cierra todos los nodos; no es una orden de aterrizaje.
  Aterrizar y verificar primero, cerrar bag, stack y servidor después.

```bash
ros2 service call /hk/mission/abort std_srvs/srv/Trigger '{}'
# Si corresponde solicitar aterrizaje al adapter:
ros2 service call /hk/land hackathon_interfaces/srv/Land '{duration: 2.0}'
```

Ante pose perdida, rangos inválidos, comandos vencidos o errores de radio:
detener el ensayo con el procedimiento probado, guardar logs, corregir la causa
y volver a las comprobaciones previas. No asumir hover por silencio del stream.
Antes del siguiente ensayo, verificar batería, estado del estimador y que no
queden otros publishers/adapters. Congelar los YAML de la configuración que
pasó las pruebas; registrar cualquier ajuste posterior.

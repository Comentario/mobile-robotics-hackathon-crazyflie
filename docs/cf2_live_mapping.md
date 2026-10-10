# Mapeado 2D durante el vuelo del dron 2

**Para navegación con evitación y retorno, usar la nueva
[guía autónoma CF2](cf2_autonomous_demo.md).** Los comandos de recorrido
de este documento sin `--navigate` son pruebas abiertas de mapeado.

Un único vuelo produce telemetría, mapa incremental y visualización. No hace
falta aterrizar para calcular el mapa. No utiliza Lighthouse ni conecta al dron 1.

## Terminal 1: mapa y publicación ROS

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ..
.venv-cf2/bin/python scripts/cf2_live_map.py --ros
```

Abrir en el navegador de Windows o Ubuntu **http://localhost:8765**. Mantener
esta terminal abierta. Sin `--ros`, el navegador funciona sin cargar ROS.
El proceso no tiene conexión radio ni capacidad para mandar órdenes al dron.

## Terminal 2: vuelo y envío en vivo

Cerrar cfclient y cualquier servidor que use la misma radio. Preparar zona
despejada, obstáculos fuera de la trayectoria y una superficie texturada.
El recorrido es abierto: no hay evitación de obstáculos ni cortes automáticos
en el script, por solicitud del operador. El aterrizaje anterior tuvo incidentes
cuya causa no se determinó; contar con el mentor durante la ejecución.

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie
.venv-cf2/bin/python scripts/cf2_first_hover.py \
  --uri 'radio://0/80/2M/E7E7E7E743' \
  --fly --height 0.3 --hover-seconds 3 \
  --square-side 0.2 --live-map
```

En un vuelo: ascenso, hover, cuatro tramos de 20 cm (frente → izquierda → atrás
→ derecha), pausa de 1 s tras cada tramo y aterrizaje. Velocidad 0.1 m/s.
Direcciones relativas al dron, sin giros de yaw programados. El cierre físico
del cuadrado no está garantizado por la integración de velocidad.
Como alternativa, `--forward 0.2 --live-map` conserva el recorrido frontal ya
ensayado. No combinar `--forward` y `--square-side`.

Para mayor cobertura, `--square-side 0.5` solicita lados de 50 cm.
`--rectangle 0.5 0.3` solicita frente 50 cm → izquierda 30 cm → atrás 50 cm
→ derecha 30 cm. No combinar rutas. Mantener toda la huella del recorrido
despejada: sillas fuera del perímetro y margen para dimensiones del dron/deriva.
Con retorno frontal de unos 60 cm en ensayos anteriores, no ejecutar avance
de 50 cm desde la misma posición hacia esa silla; redistribuir antes el espacio.

## Datos y visualización

- Telemetría a 10 Hz por UDP local (puerto 8766); navegador y ROS actualizan a 5 Hz.
- Mapa de 8 × 8 m, celdas de 5 cm. Pose XY en metros, yaw en grados,
  rangos originales en milímetros. Asociación de timestamps hasta 50 ms.
- Solo integrar ventana aérea `hovering=true` y altura estimada de 0.2–0.4 m.
  No usar muestras de suelo, reset inicial del EKF o aterrizaje para mapear.
- Cada retorno lateral de 50–4000 mm marca libre hasta el obstáculo y ocupado
  en el extremo. Rangos inválidos/32766 dejan espacio desconocido.
- Observaciones pueden corregirse con nuevos rayos y expiran después de 120 s
  por defecto (`--decay-seconds` permite cambiarlo):
  es un mapa local temporal, no un mapa permanente ni SLAM.
- Caja sostenida por una persona: representa un objeto móvil. Puede verse en
  vivo, pero no valida posición de una pared fija. Usar paredes o cajas apoyadas
  si se necesita validar el mapa estático. No exigir sostener un objeto inmóvil.
- Nuevo vuelo tiene sesión y frame propios; no fusionar orígenes distintos.
- Si la telemetría se detiene, la página lo indica. El dron no recibe órdenes
  desde el mapa ni desde el navegador. Enter no es un paro de emergencia.
- Cada vuelo conserva un JSONL en `data/runs/cf2_flight_<sesión>.jsonl`.

## Interfaz para Persona 1 y 2

Con `--ros`, publicar:

| Topic | Tipo | Contenido |
|---|---|---|
| `/cf2/map` | `nav_msgs/msg/OccupancyGrid` | -1 desconocido, 0 libre, 100 ocupado |
| `/cf2/odom` | `nav_msgs/msg/Odometry` | Pose local con yaw; covarianza provisional, no la del EKF |
| `/cf2/ranges` | `hackathon_interfaces/msg/MultiRange` | Seis rangos en metros; retorno inválido como NaN |
| `/cf2/range/front`, `/left`, `/back`, `/right` | `sensor_msgs/msg/Range` | Solo retornos utilizables, en metros |

Frame del mapa: `cf2_local_<sesión>`. Los frames de sensores son propios del
dron; todavía no se publican sus transformaciones TF ni una transformación
al frame Lighthouse de dron 1. Antes de usarlo para navegación compartida,
alinear ambos frames y medir incertidumbre. Los rangos no válidos no publican
una nueva muestra: los consumidores deben comprobar antigüedad.
Topics son accesibles a otros nodos ROS del mismo entorno; comunicación entre
laptops WSL/Ubuntu requiere comprobar descubrimiento DDS, red y ROS_DOMAIN_ID.

Esta es la ruta de hardware de dron 2. `real.launch.py` pertenece al stack de
Crazyswarm2 y no sustituye estos dos procesos. No arrancarlo simultáneamente
contra la misma radio. Los TPs de planificación están integrados en la rama,
pero todavía no mandan órdenes al script de recorrido. El mapa local no se
debe renombrar a `map` de Lighthouse sin una transformación medida.

La interfaz HTTP `GET http://localhost:8765/state` entrega el mismo mapa, pose,
rangos, trayectoria, sesión, edad de datos y estadísticas en JSON. Se sirve
solo en loopback; no está expuesta a otros equipos.

## Verificación sin vuelo

```bash
.venv-cf2/bin/python -m unittest discover -s scripts -p 'test_cf2_*.py' -v
.venv-cf2/bin/python scripts/cf2_live_map.py \
  --replay data/runs/cf2_flight_9de3a2898a4a.jsonl
```

Replay reproduce datos grabados con su ritmo original, sin conexión radio.
El indicador de antigüedad permite distinguir el fin de la reproducción.

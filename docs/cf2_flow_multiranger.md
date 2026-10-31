# Dron 2: Flow Deck + Multi-ranger

**Actualización vigente:** por petición explícita del operador se retiraron
el bloqueo de `--fly`, la supervisión automática de altura/telemetría,
los límites de recorrido y el corte manual de Enter/Ctrl+C añadido por el script.
La grabación automática se conserva, con prefijo `cf2_flight_`.
No se modificaron las protecciones del firmware. Enter ya no corta motores;
Ctrl+C tiene el comportamiento normal de Python y la salida del contexto
MotionCommander intenta aterrizar. Los controles y bloqueos mencionados abajo
son históricos. La causa de los ascensos inesperados sigue pendiente.

**Estado actual: vuelo nuevamente bloqueado.** En
`cf2_guarded_hover_a56d393b7832.jsonl`, solicitado hover a 0.3 m con avance
posterior de 0.2 m, se produjo ascenso durante la ventana de hover:
z estimada 0.206 → 0.513 m entre t=4.3 y 4.7 s desde primer log;
rango inferior 168 → 498 mm. Se activó paro por altura, antes del avance.
La altura alcanzó después aproximadamente 0.595 m (rango inferior 598 mm).
El operador reporta vuelco por impacto. No fue un aterrizaje controlado.
Este registro no valida el avance ni se debe usar como mapa del recorrido.
La causa del ascenso sigue sin determinar. Revisar con mentor el control
vertical y la configuración real del firmware antes de reactivar vuelos.
Las instrucciones de vuelo siguientes describen ensayos históricos.

**Incidente (10 de octubre de 2026):** el operador reporta ascenso
al techo al comenzar el aterrizaje de la prueba izquierda a 1 m. El registro
de consola contiene solo rangos laterales; no permite determinar la causa.
Tras autorización del mentor, `--fly` permite únicamente diagnóstico a 0.3 m,
durante hasta 3 segundos, con grabación automática y terminal interactiva.
Los comandos históricos a 1 m siguen bloqueados.
Revisar hardware tras el contacto y diagnosticar pose, rango inferior y parámetros
del estimador/controlador con motores desarmados. La causa sigue pendiente;
el siguiente vuelo requiere la supervisión del mentor y la prueba previa del paro.

### Prueba diagnóstica autorizada

Cerrar cfclient y Crazyswarm2 antes de usar el script directo. El botón del
cliente y `/hk/emergency` del CF Adapter requieren sus respectivas conexiones;
no usar conexiones simultáneas como mecanismo de emergencia del script.
Primero probar `--test-emergency` con el dron apoyado: solicita paro sin armar.
Comprobar el estado bloqueado en el cliente y reiniciar después.

El vuelo corto se ejecuta con `--fly --height 0.3 --hover-seconds 3`.
Enter o Ctrl+C en la terminal del script solicita corte de motores, no aterrizaje.
La supervisión usa tiempo monotónico y corta por telemetría de pose/rangos ausente
durante más de 1 s, altura estimada o rango inferior superior a 0.5 m, rango
inferior inválido o retorno superior a menos de 0.3 m. Es supervisión desde la
laptop: el paro depende del enlace, no tiene confirmación física y puede causar
caída. No es una garantía ni resuelve la causa del incidente.
Se conserva telemetría durante aterrizaje y un segundo posterior.

Hover `cf2_guarded_hover_bff901ea20d3.jsonl`: altura máxima 0.327 m;
descenso final hasta rango inferior 9–27 mm, sin paro de emergencia registrado.
Esto aporta evidencia del descenso; la causa del incidente a 1 m sigue pendiente.
El mapa de esa ventana se generó con `--hover-only --min-height 0.2 --max-height 0.4`.

Siguiente recorrido mínimo: añadir `--forward 0.2` al vuelo corto. Avanza a
0.1 m/s, espera 1 s y aterriza en el nuevo punto, sin volver al origen.
La zona frontal debe estar despejada por el operador: un rango sin retorno
no demuestra que lo esté. Se omite el avance si el retorno frontal inicial
es menor o igual a distancia solicitada + 0.35 m; durante el avance, retorno
frontal menor de 0.35 m solicita paro. Desde el inicio del avance se supervisa
desplazamiento XY máximo 0.6 m según el estimador. Son límites de diagnóstico,
no navegación autónoma ni garantía de frenado. Mantener obstáculos inmóviles.

Prioridad: telemetría → hover breve → recorrido controlado → mapa 2D local.
Comunicación con el dron 1 queda para después. No se requiere Lighthouse para
la prueba de hover. El Multi-ranger mide obstáculos; Flow/estimador proporciona
movimiento local. Esta combinación no garantiza una posición global sin deriva.

## Base oficial

- https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/
- https://www.bitcraze.io/documentation/tutorials/getting-started-with-stem-drone-bundle/
- https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/step-by-step/sbs_motion_commander.py

cflib preparado desde commit `3c1cb0d024fef6f45d08f0ad236a22ac395dd924`.
El script propio adapta las APIs oficiales y exige URI explícita. Sin `--fly`
solo conecta y lee sensores. La detección de decks y logs se valida en hardware.

## Ejecución

Confirmar primero URI del dron 2, acceso USB/Crazyradio, batería, montaje y
firmware compatible con el ejemplo oficial. Cerrar cualquier cliente o servidor
que controle ese mismo dron. Preparar una zona despejada con suelo texturado.

Terminal 1, telemetría (sustituir la URI):

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ..
.venv-cf2/bin/python scripts/cf2_first_hover.py --uri 'radio://RADIO/CHANNEL/RATE/ADDRESS'
```

Tras validar pose y los seis rangos, el mismo comando con `--fly` solicita
despegue a 0.3 m, hover durante 3 segundos y aterrizaje. Validado por el operador.
Es una prueba básica, sin evitación automática ni garantía de aterrizaje si
se pierde el enlace. No arrancar la misión completa del repositorio para ella.

## Mapa posterior

### Hover con lectura de un sensor

`--fly --height 1.0 --hover-seconds 10 --sensor front` solicita ascenso a
1 m, lectura frontal destacada durante 10 segundos de hover y aterrizaje.
`--sensor` también acepta `left`, `back` y `right`, respecto al cuerpo del dron.
Todos los canales siguen registrados si se añade `--record`; cada fila incluye
`hovering` para identificar la ventana de medición. Sin estas opciones se
conserva el hover original a 0.3 m durante 3 segundos.
El vuelo a 1 m aún requiere validación física. Mantener personas y objetos
fuera del alcance de las hélices; usar un soporte para medir obstáculos cercanos.
El script no evita obstáculos automáticamente.

La siguiente prueba es **sin vuelo**. Cerrar cfclient y grabar durante 30 segundos:

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ..
.venv-cf2/bin/python scripts/cf2_first_hover.py --uri 'radio://0/80/2M/E7E7E7E743' --seconds 30 --record data/runs/cf2_sensor_check_01.jsonl
```

Con el dron apoyado, presentar una tarjeta plana delante de cada sensor lateral,
uno a la vez, a 20 cm y después 50 cm. Mantener cada posición unos segundos.
Esperamos aproximadamente 200 y 500 mm en el rayo correspondiente. Las lecturas
sin retorno no prueban por sí solas que el espacio esté libre. Esta prueba valida
sensores; las tarjetas aparecerán como obstáculos acumulados en el mapa de prueba.

```bash
.venv-cf2/bin/python scripts/cf2_build_map.py data/runs/cf2_sensor_check_01.jsonl --output data/runs/cf2_sensor_map_01
```

Produce `map.pgm`, `map.yaml` y `map.json`: resolución 5 cm, área local 8 × 8 m,
gris desconocido, blanco libre, negro ocupado. Archivo de grabación y carpeta de
salida deben ser nuevos para evitar sobrescribir resultados anteriores.
Para grabaciones posteriores de hover, usar `--min-height 0.2 --max-height 0.4`
en el generador para excluir muestras de suelo, despegue y aterrizaje.

El generador usa el yaw en grados y convierte milímetros a metros. Empareja
rangos con la pose más cercana a menos de 50 ms; descarta retornos fuera de
50–4000 mm, incluyendo 32766. Usa el centro del dron como origen aproximado de
cada rayo. Acumula obstáculos sin borrarlos: objetos movidos dejan marcas.
No realiza SLAM ni corrige deriva; el mapa todavía no habilita navegación autónoma.

Guardar pose XY/yaw y rayos front/back/left/right con timestamps. Transformar
los rayos al frame local, marcar espacio libre hasta el retorno y ocupación en
el extremo; tratar rangos inválidos y sin retorno explícitamente. Mantener una
rejilla 2D a altura aproximadamente constante. Ese mapa representa un corte
horizontal, no el volumen completo. Antes de navegación autónoma, validar
deriva, coherencia temporal, frenado, obstáculos y límites de vuelo.

Compartir datos con dron 1 exige alinear frames y medir la incertidumbre; enviar
topics entre drones no hace que sus coordenadas coincidan.

## Estado

Rama `feat/cf2-flow-multiranger`. Trabajo rover retirado. Primer vuelo básico
validado el 10 de octubre de 2026 desde WSL de Persona 3: el operador confirma
ascenso, hover y aterrizaje satisfactorios. Telemetría muestra altura cercana
a 0.3 m y descenso. El registro acaba antes del contacto final; la confirmación
de aterrizaje procede del operador. Esto no valida navegación ni mapa 2D.

URI funcional: `radio://0/80/2M/E7E7E7E743`. Canal 43 no validado: falló con
`Too many packets lost`. Crazyradio 2.0 adjunto con usbipd; permisos plugdev
configurados. `bcFlow2=1`, `bcZRanger2=1`, `bcMultiranger=1`. El Flow Deck
no se descubría cuando estaba invertido; corregir montaje resolvió la detección.
El sensor trasero dejó de reportar milímetros persistentemente; falta validar
cada rayo con objetos a distancias conocidas antes de mapear.

La rama de integración solicitada no está en las referencias locales;
fetch requiere autenticación.

## Cliente y configuración de radio en WSL

Instalar `libxcb-cursor0` y `usbutils` en Ubuntu. En Windows, usar usbipd-win
para compartir y adjuntar a WSL el Crazyflie conectado por USB de datos y luego
el Crazyradio. `usbipd list` identifica el BUSID; `usbipd bind --busid BUSID`
requiere PowerShell administrador; `usbipd attach --wsl --busid BUSID` adjunta.
Comprobar `lsusb` y permisos antes de conectar con el cliente.

Abrir el cliente instalado en el entorno aislado:

```bash
cd ~/hackathon/mobile-robotics-hackathon-crazyflie
.venv-cf2/bin/cfclient
```

Conectar por `usb://0`, abrir Connect → Configure 2.x y registrar los valores
actuales. Acordar canal con el equipo y usar dirección exclusiva. Write guarda
la configuración; reiniciar y escanear con la nueva dirección. El índice de
Crazyradio, el canal RF y la dirección del dron son conceptos diferentes.
La dirección se configuró como `E7E7E7E743`; el canal funcional observado es 80.

Fuentes: https://learn.microsoft.com/en-us/windows/wsl/connect-usb y
https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/

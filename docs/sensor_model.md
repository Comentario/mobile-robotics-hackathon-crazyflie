# Sensores e integración del MVP

Se conserva la API de las ramas de persona 2:

| Entrada de cf_adapter | Tipo | Convención |
|---|---|---|
| `/cf231/pose` | PoseStamped | pose en `world`, sello de la medida |
| `/cf231/scan` | LaserScan | `[back, right, front, left]`, metros, ángulos −π + i·π/2 |
| `/cf231/ranges_extra` | LogDataGeneric | `[up, down]`, milímetros |

`cf_adapter` publica `/hk/adapter/odom` y `/hk/adapter/ranges`.
`state_monitor` publica `/hk/state`, `/hk/localization_health` y TF `map → base_link`.
`range_processor` publica `/hk/ranges` y `/hk/ranges/closing_rate`.

## Dos formas de simular

- `backend:=emulator`: `sim_crazyflie` de persona 2 genera dinámica cinemática,
  pose, raycast y fallos. Sus modelos se guardan en `simulation_adapter/emulator`
  para conservar el TP-10 original. No ejecuta firmware real; su watchdog es una aproximación.
- `backend:=sil`: el servidor externo de Crazyswarm2 genera movimiento y `/clock`.
  `sim_sensors` lee TF `world → cf231`, añade ruido de pose y calcula rayos contra
  el mundo del escenario. Considera orientación completa y offsets de sensores.
  No genera movimiento, EKF ni flujo óptico. No implementa los fallos del emulador.

El scan recorta a 3.49 m. `+inf` significa sin retorno, `-inf` demasiado cerca,
`NaN` inválido. `-inf` atraviesa el filtro inmediatamente; no se espera confirmación
ni mediana. NaN puede conservar el último valor durante `invalid_hold` (0.3 s).
Después, un horizontal inválido causa HOVER si `ranges_required` está activo.
Los rayos individuales no modelan el cono óptico ni materiales.

Los rates usan la variación de distancia filtrada: negativos indican acercamiento.
El navegador mantiene el yaw para evitar que un giro aparente una velocidad de cierre.
Seguridad considera TTC y frenado (`max_decel` por defecto 0.5 m/s²,
`reaction_time` 0.3 s); estos valores necesitan ajuste y prueba en hardware.

## Paso a hardware

Sustituir el simulador y `sim_sensors` por el servidor de hardware, configurar logging
`pose`, `scan` y el custom topic `ranges_extra` con variables `[range.up, range.zrange]`.
Verificar orden, unidades, timestamps, frames, retorno inválido y frecuencia reales.
No cambiar los tópicos del equipo. Consultar `command_interface.md`: los servicios
emergency y notify del SIL instalado no demuestran soporte funcional en hardware.

La validación de integración y comandos para Ubuntu/ROS nativos se encuentran
al final de `simulations.md`.

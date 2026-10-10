# Integración de TPs y dron 2

Rama de trabajo: `integration/cf2-navigation-mapping`. No modifica `main`.

## Auditoría de referencias locales

`main` local apunta a `a101bb9` (merge PR #12, TP-32). TP-26 ya estaba
integrado por PR #8 (`dc3a4f6`). Después de autenticar GitHub se descargó `origin/integration-mvp-crazyswarm`
(`61e8316`). Su contenido se integra exclusivamente en nuestra rama,
conservando las referencias de `main` y de Persona 1/2.

Se guardó el bringup y mapa en vivo en `f45cc57`, y se fusionó
`tp31-planner-test-matrix` mediante `d9e7b66`. Esa rama incluye por ascendencia
todos los siguientes commits, sin necesidad de cherry-picks repetidos:

| TP | Commit | Componente |
|---|---|---|
| 34 | ffe029c | Evitación horizontal |
| 16 | f5444bb | Política de velocidad por localización |
| 27 | 27548ba | A* |
| 28 | 56b06c8 | Costes y política de desconocidos |
| 29 | ecf7b9b | Replanning |
| 30 | 0489953 | Simplificación y lookahead |
| 33 | d33367a | Suavizado de velocidad |
| 35 | ca6a1aa | Evitación vertical |
| 36 | 49d6894 | Obstáculos dinámicos |
| 31 | 711a434 | Matriz de pruebas del planner |

## Resultado y alcance

La integración de Persona 1/2 aporta estado, salud, rangos, supervisor,
misión y emulación. Se conserva el A*, procesamiento de ruta y evitación
completa de Persona 3. Navegación usa comandos de velocidad compatibles con
el adapter y solo emite durante NAVIGATE, con estado y rangos recientes.

- Compilación completa: 14 paquetes, sin errores.
- Pruebas de componentes: 95 pasan, 16 omitidas por los esqueletos existentes.
- Pruebas del mapa: 9 pasan.
- Verificación completa T01 emulada: **pendiente**. Las ejecuciones detectaron
  huecos de aproximadamente 55–56 s en las publicaciones y terminaron por
  timeout. Una ejecución solapada además detectó publishers duplicados;
  se repitió en un dominio aislado y persistió el problema temporal.
  No se considera validada la navegación de extremo a extremo.

## Ruta seleccionada para dron 2: mapa primero

Usar `cf2_first_hover.py --live-map` y `cf2_live_map.py --ros`, según
[la guía de mapeado](cf2_live_mapping.md). Es una única conexión radio,
Flow deck para estimación local y Multiranger para rayos laterales.
No requiere Lighthouse, Crazyswarm2 ni ejecutar toda la misión integrada.
El navegador muestra el mapa incremental durante el recorrido.

El mapper publica `/cf2/map`, `/cf2/odom`, `/cf2/ranges` (MultiRange) y
rangos individuales. Conserva un frame propio por sesión; no representa
posición absoluta ni SLAM. Rangos inválidos permanecen desconocidos.

El recorrido actual es abierto y no consume `/hk/cmd/safe`. A* y evitación
están disponibles en la rama, pero no controlan el dron físico todavía.
`local_world_model` del stack general continúa siendo un esqueleto: para
nuestro primer objetivo el mapa lo produce el mapper CF2 implementado.
No activar `real.launch.py` junto al script de radio.

## Próximas etapas, después del mapa

1. Validar mapa continuo con obstáculos fijos y recorrido despejado a 30 cm.
2. Resolver la prueba emulada y conectar mapa/estado/rangos al pipeline de
   navegación con parámetros de frame explícitos y un único adapter real.
3. Validar comandos, altura y aterrizaje antes del control cerrado.
4. Solo después, alinear el frame local con Lighthouse del dron 1 para
   compartir información. Compartir topics no alinea coordenadas.

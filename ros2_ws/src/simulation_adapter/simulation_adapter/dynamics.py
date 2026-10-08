"""Modelo cinemático simple del dron (TP-10).

No modela la física del quadrotor: el controlador onboard real se aproxima con una
respuesta de primer orden hacia el setpoint, limitada en velocidad y aceleración.

Este módulo es lógica pura (regla 6 de docs/interfaces.md): no importa rclpy ni
mensajes ROS, así que se puede probar con pytest sin lanzar ROS. El nodo
(sim_node.py) es quien traduce los mensajes ROS a la clase Command de aquí.

Modos internos del dron (atributo ``KinematicDrone.mode``)
---------------------------------------------------------
  'idle'       En el suelo, motores parados. Ignora los setpoints.
  'takeoff'    Subiendo hasta la altura pedida (rampa lineal durante ``duration``).
  'land'       Bajando hasta el suelo (rampa lineal durante ``duration``).
  'command'    Siguiendo el último setpoint recibido (POSITION / VELOCITY / HOVER).
  'hover'      Quieto en un punto fijo. Se entra al terminar el despegue (como el
               high-level commander del firmware, que sigue generando setpoints).
               Sin firmware_watchdog: también cuando salta el watchdog.
  'level_out'  Solo con firmware_watchdog: sin setpoints durante command_watchdog
               (0.5 s). Roll/pitch a 0: mantiene la altura pero sigue deslizando
               con la velocidad horizontal que llevaba.
  'emergency'  Motores fuera: cae hasta z = 0 y ya no acepta ningún comando.

Watchdog del firmware real (firmware_watchdog=True)
---------------------------------------------------
  Copiado de crazyflie-firmware 2025.02, src/modules/src/supervisor.c:
    COMMANDER_WDT_TIMEOUT_STABILIZE = 500 ms  -> estado WarningLevelOut
    COMMANDER_WDT_TIMEOUT_SHUTDOWN  = 2000 ms -> estado ExceptFreeFall (motores fuera)
  Solo afecta al streaming de setpoints de bajo nivel (modo 'command'). El
  high-level commander (takeoff/land/hover tras despegar) genera sus propios
  setpoints y no dispara el watchdog.

Qué hace cada paso de simulación (step)
---------------------------------------
  1. Si está en 'command' y el último setpoint es demasiado viejo -> 'hover'.
  2. Calcula una velocidad deseada según el modo.
  3. La limita a max_speed (norma 3D).
  4. Limita el cambio de velocidad a max_accel * dt (aceleración máxima).
  5. Integra la posición: p = p + v * dt, sin bajar del suelo (z >= 0).
  6. Gira el yaw hacia el objetivo a velocidad angular limitada.
"""
import math
from dataclasses import dataclass

# Constante de tiempo (s) de la respuesta de primer orden: con un error de
# posición e, la velocidad deseada es e / TIME_CONSTANT. Cuanto menor, más
# "agresivo" es el seguimiento. 0.3 s da un movimiento suave y creíble.
TIME_CONSTANT = 0.3

# Velocidad angular máxima del yaw (rad/s).
MAX_YAW_RATE = 1.5

# Aceleración de caída en emergencia (m/s^2).
GRAVITY = 9.81

# Distancia (m) a partir de la cual se considera alcanzada la altura de despegue
# o el suelo al aterrizar.
ARRIVAL_TOLERANCE = 0.02


@dataclass
class DroneState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    flying: bool = False


@dataclass
class Command:
    """Setpoint ya traducido desde el mensaje ROS (lo crea sim_node.py).

    mode:
      'position'  ir a (x, y, z) con orientación yaw.
      'velocity'  moverse en el plano a velocidad (vx, vy) manteniendo la
                  altura z (semántica de Setpoint.msg: "seguir velocity
                  manteniendo position.z"). vz se ignora.
      'hover'     quedarse quieto donde esté al recibir el primer HOVER.
    """
    mode: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0


def _clamp_norm(vector, max_norm):
    """Escala un vector (tupla) para que su norma no supere max_norm."""
    norm = math.sqrt(sum(c * c for c in vector))
    if norm <= max_norm or norm == 0.0:
        return vector
    scale = max_norm / norm
    return tuple(c * scale for c in vector)


def _wrap_angle(angle):
    """Normaliza un ángulo a [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class KinematicDrone:

    def __init__(self, max_speed, max_accel, command_watchdog,
                 firmware_watchdog=False, shutdown_timeout=2.0):
        """command_watchdog: s sin setpoints de bajo nivel antes de reaccionar.

        firmware_watchdog=False: reacción "amable" (hover), la del simulador antiguo.
        firmware_watchdog=True:  como el firmware real: level_out tras
        command_watchdog y caída (emergency) tras shutdown_timeout.
        """
        self.state = DroneState()
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.command_watchdog = command_watchdog
        self.firmware_watchdog = firmware_watchdog
        self.shutdown_timeout = shutdown_timeout
        self.emergency_reason = ''

        self.mode = 'idle'
        self._t = 0.0                       # último instante visto en step()
        self._command = None                # último Command aceptado
        self._command_time = None           # instante en que llegó
        self._hover_point = (0.0, 0.0, 0.0)
        self._yaw_target = 0.0
        # Rampa vertical de takeoff/land: (t0, x0, y0, z0, z1, duration)
        self._ramp = None

    # ------------------------------------------------------------------
    # Configuración inicial
    # ------------------------------------------------------------------
    def reset_pose(self, x, y, z, yaw):
        """Coloca el dron en una pose inicial (la 'start' del escenario)."""
        self.state = DroneState(x=x, y=y, z=z, yaw=yaw)
        self._yaw_target = yaw

    # ------------------------------------------------------------------
    # Entradas (las llama el nodo ROS)
    # ------------------------------------------------------------------
    def set_command(self, t, setpoint):
        """Guarda el último Setpoint (modo POSITION, VELOCITY o HOVER) y su timestamp.

        Devuelve True si se acepta y False si se ignora (dron en el suelo,
        en emergencia, o modo desconocido).
        """
        if self.mode in ('idle', 'emergency'):
            return False
        if setpoint.mode not in ('position', 'velocity', 'hover'):
            return False

        # Si ya estábamos recibiendo HOVER, conservamos el punto de hover:
        # así un flujo continuo de HOVER no hace "derivar" el dron.
        already_hovering = (
            self.mode == 'command'
            and self._command is not None
            and self._command.mode == 'hover'
        )

        if self.mode in ('takeoff', 'land'):
            # Un setpoint de bajo nivel corta la maniobra de alto nivel (el
            # firmware da prioridad al streaming de bajo nivel).
            self._ramp = None
        self._command = setpoint
        self._command_time = t
        self.mode = 'command'

        if setpoint.mode == 'hover':
            if not already_hovering:
                s = self.state
                self._hover_point = (s.x, s.y, s.z)
        else:
            self._yaw_target = setpoint.yaw
        return True

    def takeoff(self, height, duration):
        """Sube desde el suelo hasta z = height en 'duration' segundos.

        Devuelve (success, message), igual que el servicio /hk/takeoff.
        """
        if self.mode == 'emergency':
            return False, 'Emergencia activa: el dron no acepta comandos'
        if self.mode != 'idle':
            return False, f'El dron ya está en vuelo (modo {self.mode})'
        if height <= 0.0 or duration <= 0.0:
            return False, 'height y duration deben ser positivos'

        self.state.flying = True
        self._start_ramp(height, duration)
        self.mode = 'takeoff'
        return True, f'Despegando a {height:.2f} m en {duration:.1f} s'

    def land(self, duration):
        """Baja hasta el suelo en 'duration' segundos y apaga motores al llegar."""
        if self.mode in ('idle', 'emergency'):
            return False, f'No se puede aterrizar en modo {self.mode}'
        if duration <= 0.0:
            return False, 'duration debe ser positiva'

        self._start_ramp(0.0, duration)
        self._command = None
        self.mode = 'land'
        return True, f'Aterrizando en {duration:.1f} s'

    def high_level_hover(self):
        """El high-level commander toma el control: hover donde está.

        Lo usa el nodo al recibir notify_setpoints_stop (el firmware deja de dar
        prioridad al streaming de bajo nivel).
        """
        if self.mode in ('command', 'level_out'):
            s = self.state
            self._enter_hover((s.x, s.y, s.z))
            self._command = None
            return True
        return False

    def emergency_stop(self, reason='emergency'):
        """Motores fuera: el dron cae (z -> 0) y deja de aceptar comandos."""
        self.emergency_reason = reason
        self.mode = 'emergency'
        self.state.flying = False
        self._command = None
        self._ramp = None
        self.state.vx = 0.0
        self.state.vy = 0.0

    # ------------------------------------------------------------------
    # Simulación
    # ------------------------------------------------------------------
    def step(self, t, dt):
        """Avanza la simulación dt segundos y devuelve el DroneState."""
        self._t = t
        s = self.state

        if self.mode == 'emergency':
            self._fall(dt)
            return s

        if self.mode == 'idle':
            s.vx = s.vy = s.vz = 0.0
            return s

        # 1. Watchdog: sin setpoints de bajo nivel recientes.
        if self.mode in ('command', 'level_out'):
            age = t - self._command_time
            if not self.firmware_watchdog:
                if age > self.command_watchdog:
                    self._enter_hover((s.x, s.y, s.z))
            elif age > self.shutdown_timeout:
                self.emergency_stop('watchdog')        # ExceptFreeFall
                self._fall(dt)
                return s
            elif age > self.command_watchdog and self.mode == 'command':
                self.mode = 'level_out'                # WarningLevelOut
                self._hover_point = (s.x, s.y, s.z)

        # 2-4. Velocidad deseada, limitada en velocidad y aceleración.
        desired = _clamp_norm(self._desired_velocity(t), self.max_speed)
        delta = (desired[0] - s.vx, desired[1] - s.vy, desired[2] - s.vz)
        delta = _clamp_norm(delta, self.max_accel * dt)
        s.vx += delta[0]
        s.vy += delta[1]
        s.vz += delta[2]

        # 5. Integrar posición, sin atravesar el suelo.
        s.x += s.vx * dt
        s.y += s.vy * dt
        s.z += s.vz * dt
        if s.z < 0.0:
            s.z = 0.0
            s.vz = max(s.vz, 0.0)

        # 6. Yaw con velocidad angular limitada.
        yaw_error = _wrap_angle(self._yaw_target - s.yaw)
        max_step = MAX_YAW_RATE * dt
        s.yaw = _wrap_angle(s.yaw + max(-max_step, min(max_step, yaw_error)))

        self._check_maneuver_end(t)
        return s

    # ------------------------------------------------------------------
    # Funciones internas
    # ------------------------------------------------------------------
    def _desired_velocity(self, t):
        """Velocidad que el 'controlador' querría ahora, según el modo."""
        s = self.state

        if self.mode in ('takeoff', 'land'):
            _, x0, y0, _, _, _ = self._ramp
            z_ref, vz_feedforward = self._ramp_reference(t)
            return self._towards((x0, y0, z_ref), vz_feedforward)

        if self.mode == 'hover':
            return self._towards(self._hover_point)

        if self.mode == 'level_out':
            # Roll/pitch = 0: sin aceleración horizontal (sigue deslizando).
            return (s.vx, s.vy, (self._hover_point[2] - s.z) / TIME_CONSTANT)

        # mode == 'command'
        cmd = self._command
        if cmd.mode == 'velocity':
            # XY: la velocidad pedida. Z: mantener la altura del setpoint.
            return (cmd.vx, cmd.vy, (cmd.z - s.z) / TIME_CONSTANT)
        if cmd.mode == 'hover':
            return self._towards(self._hover_point)
        return self._towards((cmd.x, cmd.y, cmd.z))

    def _towards(self, target, vz_feedforward=0.0):
        """Respuesta de primer orden: velocidad proporcional al error."""
        s = self.state
        return (
            (target[0] - s.x) / TIME_CONSTANT,
            (target[1] - s.y) / TIME_CONSTANT,
            (target[2] - s.z) / TIME_CONSTANT + vz_feedforward,
        )

    def _start_ramp(self, z_goal, duration):
        """Prepara una rampa vertical desde la altura actual hasta z_goal."""
        s = self.state
        self._ramp = (self._t, s.x, s.y, s.z, z_goal, duration)

    def _ramp_reference(self, t):
        """Altura de referencia de la rampa en el instante t y su velocidad."""
        t0, _, _, z0, z1, duration = self._ramp
        progress = min(max((t - t0) / duration, 0.0), 1.0)
        z_ref = z0 + progress * (z1 - z0)
        vz_feedforward = (z1 - z0) / duration if progress < 1.0 else 0.0
        return z_ref, vz_feedforward

    def _check_maneuver_end(self, t):
        """Termina el despegue (-> hover) o el aterrizaje (-> idle)."""
        if self._ramp is None or self.mode not in ('takeoff', 'land'):
            return
        t0, x0, y0, _, z1, duration = self._ramp
        ramp_done = t - t0 >= duration
        arrived = abs(self.state.z - z1) < ARRIVAL_TOLERANCE
        if not (ramp_done and arrived):
            return

        if self.mode == 'takeoff':
            self._enter_hover((x0, y0, z1))
        else:
            s = self.state
            s.z = 0.0
            s.vx = s.vy = s.vz = 0.0
            s.flying = False
            self._ramp = None
            self.mode = 'idle'

    def _enter_hover(self, point):
        self._hover_point = point
        self._ramp = None
        self.mode = 'hover'

    def _fall(self, dt):
        """Caída libre vertical hasta el suelo (modo emergencia)."""
        s = self.state
        if s.z > 0.0:
            s.vz -= GRAVITY * dt
            s.z = max(0.0, s.z + s.vz * dt)
        if s.z == 0.0:
            s.vz = 0.0

"""Modelo cinemático simple del dron (TP-10).

No modela la física del quadrotor: el controlador onboard real se aproxima con una
respuesta de primer orden hacia el setpoint, limitada en velocidad y aceleración.
"""
from dataclasses import dataclass


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


class KinematicDrone:

    def __init__(self, max_speed, max_accel, command_watchdog):
        """command_watchdog: s sin setpoints tras los que el dron hace hover.

        Imita el watchdog del firmware; verificar el comportamiento real en TP-08.
        """
        self.state = DroneState()
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.command_watchdog = command_watchdog

    def set_command(self, t, setpoint):
        """Guarda el último Setpoint (modo POSITION, VELOCITY o HOVER) y su timestamp."""
        raise NotImplementedError

    def takeoff(self, height, duration):
        raise NotImplementedError

    def land(self, duration):
        raise NotImplementedError

    def emergency_stop(self):
        """Motores fuera: el dron cae (z -> 0) y deja de aceptar comandos."""
        raise NotImplementedError

    def step(self, t, dt):
        """Avanza la simulación dt segundos y devuelve el DroneState."""
        raise NotImplementedError

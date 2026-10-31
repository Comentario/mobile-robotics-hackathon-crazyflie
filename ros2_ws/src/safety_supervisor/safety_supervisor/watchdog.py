"""Watchdogs de streams vencidos (TP-38): pose, rangos y bucle de comandos.

Un watchdog es un "perro guardián": se le avisa cada vez que llega un mensaje
(stamp) y responde si hace demasiado que no llega ninguno (stale).
Lógica pura: sin rclpy. Los tiempos son segundos (float).
"""


class StreamWatchdog:
    def __init__(self, timeout):
        self.timeout = timeout
        self.last_stamp = None

    def stamp(self, t):
        """Registra que ha llegado un mensaje en el instante t."""
        self.last_stamp = t

    def received_any(self):
        """True si alguna vez llegó un mensaje."""
        return self.last_stamp is not None

    def stale(self, now):
        """True si nunca llegó un mensaje o el último tiene más de timeout s."""
        if self.last_stamp is None:
            return True
        return now - self.last_stamp > self.timeout

"""Watchdogs de streams vencidos (TP-38): pose, rangos y bucle de comandos."""


class StreamWatchdog:

    def __init__(self, timeout):
        self.timeout = timeout
        self.last_stamp = None

    def stamp(self, t):
        raise NotImplementedError

    def stale(self, now):
        """True si nunca llegó un mensaje o el último tiene más de timeout s."""
        raise NotImplementedError

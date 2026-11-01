"""Stale-stream watchdogs (TP-38): pose, ranges and command loop.

A watchdog is told every time a message arrives (stamp) and answers whether too
much time has passed since the last one (stale).
Pure logic: no rclpy. Times are seconds (float).
"""


class StreamWatchdog:
    def __init__(self, timeout):
        self.timeout = timeout
        self.last_stamp = None

    def stamp(self, t):
        """Record that a message arrived at time t."""
        self.last_stamp = t

    def received_any(self):
        """True if a message ever arrived."""
        return self.last_stamp is not None

    def stale(self, now):
        """True if no message ever arrived or the last one is older than timeout s."""
        if self.last_stamp is None:
            return True
        return now - self.last_stamp > self.timeout

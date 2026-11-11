"""cyberdog2：小米 CyberDog 2 的 Python 开发库。

    from cyberdog2 import CyberDog
    with CyberDog.connect("192.168.31.50", password="123") as dog:
        dog.motion.stand()
        dog.motion.walk(vx=0.3, duration=2)
"""
from .constants import CmdSource, MotionId, NavType, StopTask, SwitchStatus
from .dog import CyberDog
from .errors import (AgentStartError, CallbackError, CommandFailed, ConnectionLost, CyberDogError, GoalRejected,
                     MotionRejected, NamespaceNotFound, SensorTimeout, ServiceTimeout, ServiceUnavailable)
from .frame import Frame

__version__ = "0.1.0"

__all__ = [
    "CyberDog", "MotionId", "CmdSource", "NavType", "StopTask", "SwitchStatus", "Frame",
    "CyberDogError", "AgentStartError", "CallbackError", "CommandFailed", "ConnectionLost", "GoalRejected",
    "MotionRejected", "NamespaceNotFound", "SensorTimeout", "ServiceTimeout", "ServiceUnavailable",
]

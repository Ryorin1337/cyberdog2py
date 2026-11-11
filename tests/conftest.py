import os
import socket
import sys
import threading

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from cyberdog2 import CyberDog  # noqa: E402
from cyberdog2.agent.server import AgentServer  # noqa: E402
from cyberdog2.backends.mock import MockBackend  # noqa: E402
from cyberdog2.backends.remote import RemoteBackend  # noqa: E402


@pytest.fixture
def dog():
    d = CyberDog.mock()
    yield d
    d.close()


class RemotePair(object):
    """本机回环上的“代理 + 远程客户端”，代理内部用 MockBackend。"""

    def __init__(self, watchdog=1.0, ping_interval=0.1):
        self.mock = MockBackend()
        self.server = AgentServer(self.mock, watchdog=watchdog)
        self.thread = threading.Thread(target=self.server.serve_one, daemon=True)
        self.thread.start()
        self.sock = socket.create_connection(("127.0.0.1", self.server.port))
        self.backend = RemoteBackend(self.sock, ping_interval=ping_interval)
        self.dog = CyberDog(self.backend, remote=True)


@pytest.fixture
def remote():
    pair = RemotePair()
    yield pair
    pair.dog.close()
    pair.thread.join(3)

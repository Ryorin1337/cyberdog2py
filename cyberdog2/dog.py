"""CyberDog 门面类。"""

import atexit
import sys
import threading
import traceback
import weakref
from typing import Callable, Dict, Optional

from .audio import Audio
from .backends.base import Backend
from .constants import CmdSource
from .errors import CallbackError, CyberDogError
from .led import Led
from .motion import Motion
from .navigation import Navigation
from .sensors import Sensors
from .vision import Vision


def _close_at_exit(ref):
    dog = ref()
    if dog is not None:
        dog.close()


class CyberDog(object):
    """一只 CyberDog 2。

    通常用下面三个工厂方法之一创建实例：
        CyberDog.connect("192.168.31.50", password="123")   # 电脑远程控制（SSH + 狗端代理）
        CyberDog.local()                                     # 程序直接运行在狗上
        CyberDog.mock()                                      # 没有真机时用模拟狗

    错误处理遵循 let it crash：
    - 命令失败直接抛异常；
    - 用户回调里抛出的异常会被记录，同时停止运动，下一次调用任何 API 时抛 CallbackError；
    - 后台连接断开时，下一次调用抛 ConnectionLost。
    """

    def __init__(self, backend: Backend, cmd_source: CmdSource = CmdSource.APP, remote: bool = False):
        self._backend = backend
        self.cmd_source = cmd_source
        self._remote = remote
        self._fatal = None  # type: Optional[BaseException]
        self._closed = False
        self._lock = threading.Lock()
        self.motion = Motion(self)
        self.sensors = Sensors(self)
        self.led = Led(self)
        self.audio = Audio(self)
        self.vision = Vision(self)
        self.nav = Navigation(self)
        atexit.register(_close_at_exit, weakref.ref(self))

    # ------------------------------------------------------------ 工厂方法
    @classmethod
    def connect(cls, host: str, user: str = "mi", password: Optional[str] = None, *,
                key_file: Optional[str] = None, port: int = 22, namespace: Optional[str] = None,
                **agent_options) -> "CyberDog":
        """通过 SSH 连接狗：自动上传并启动狗端代理，之后所有指令经 SSH 隧道转发。

        agent_options 会原样传给 RemoteBackend.connect_ssh，可用参数见该函数的说明。
        """
        from .backends.remote import RemoteBackend

        backend = RemoteBackend.connect_ssh(host, user=user, password=password, key_file=key_file, port=port,
                                            namespace=namespace, **agent_options)
        return cls(backend, remote=True)

    @classmethod
    def local(cls, namespace: Optional[str] = None, discovery_timeout: float = 5.0) -> "CyberDog":
        """程序运行在狗本机上时使用，直接通过 rclpy 访问 ROS。"""
        from .backends.ros import RosBackend

        return cls(RosBackend(namespace=namespace, discovery_timeout=discovery_timeout))

    @classmethod
    def mock(cls, **kw) -> "CyberDog":
        """模拟狗。用 dog.backend 拿到 MockBackend 对象，可以注入事件、检查发出的指令。"""
        from .backends.mock import MockBackend

        return cls(MockBackend(**kw))

    # ------------------------------------------------------------ 公共
    @property
    def backend(self) -> Backend:
        return self._backend

    @property
    def namespace(self) -> str:
        return self._backend.namespace

    def unsubscribe(self, sub_id: int) -> None:
        self._check()
        self._backend.unsubscribe(sub_id)

    def list_names(self) -> Dict[str, list]:
        """列出狗上所有的话题、服务和动作。"""
        self._check()
        return self._backend.list_names()

    def close(self) -> None:
        """停止运动并断开连接。可以重复调用。"""
        with self._lock:
            if self._closed:
                return
            self._closed = True
        try:
            if self._backend.fatal_error is None:
                try:
                    self._backend.servo_stop()
                finally:
                    try:
                        if self.nav.started:
                            self.nav._release()
                    finally:
                        self.sensors._release()
        finally:
            self._backend.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.close()
        except Exception:
            if exc_type is None:
                raise
            # 正在处理别的异常时，close 的失败不能把原始异常盖掉
            traceback.print_exc()

    # ------------------------------------------------------------ 内部
    def _ensure_open(self) -> None:
        if self._closed:
            raise CyberDogError("CyberDog 已关闭")

    def _check(self) -> None:
        self._ensure_open()
        if self._fatal is not None:
            raise CallbackError("回调函数抛出了异常，程序已停止运动") from self._fatal
        err = self._backend.fatal_error
        if err is not None:
            raise err

    def _call(self, name: str, srv_type: str, request: dict, timeout: float = 10.0) -> dict:
        self._check()
        return self._backend.call_service(name, srv_type, request, timeout=timeout)

    def _wrap(self, callback: Callable) -> Callable:
        def wrapped(data):
            if self._fatal is not None:
                return
            try:
                callback(data)
            except BaseException as exc:
                self._fatal = exc
                sys.stderr.write("[cyberdog2] 回调函数抛出异常，正在停止运动:\n")
                traceback.print_exc()
                try:
                    self._backend.servo_stop()
                except BaseException:
                    traceback.print_exc()
        return wrapped

    def _subscribe(self, topic: str, msg_type: str, callback: Callable, wrap: bool = True) -> int:
        self._check()
        return self._backend.subscribe(topic, msg_type, self._wrap(callback) if wrap else callback)

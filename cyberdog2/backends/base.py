"""后端抽象接口。

MockBackend、RosBackend、RemoteBackend 都实现这一组方法，消息一律用 dict 表示。
接口名不以 "/" 开头时视为相对名，后端会自动拼上狗的命名空间（例如 /mi_desktop_xx/）。
"""

import threading
from typing import Callable, Dict, List, Optional

from ..constants import Iface, SensorOperation
from ..errors import CommandFailed

Callback = Callable[[dict], None]


class Backend(object):
    namespace = ""

    def __init__(self):
        self._sensor_lock = threading.Lock()
        self._started_sensors = set()  # 本次会话由库打开的传感器，退出时只关这些

    # ---- 传感器开关
    def _sensor_operation(self, sensor_id: int, operation: int, timeout: float) -> None:
        name, srv_type = Iface.SENSOR_OPERATION
        resp = self.call_service(name, srv_type, {"sensor_id": int(sensor_id), "operation": int(operation)},
                                 timeout=timeout)
        if not resp["success"]:
            raise CommandFailed("传感器 %d 执行操作 %d 失败, code=%s" % (sensor_id, operation, resp.get("code")),
                                resp.get("code", 0), resp)

    def sensor_start(self, sensor_id: int, timeout: float = 15.0) -> None:
        """打开传感器并记下来。同一个传感器只打开一次，失败抛 CommandFailed。"""
        with self._sensor_lock:
            if sensor_id in self._started_sensors:
                return
            self._sensor_operation(sensor_id, SensorOperation.START, timeout)
            self._started_sensors.add(sensor_id)

    def sensors_stop_started(self, timeout: float = 15.0) -> None:
        """关闭本次会话打开过的传感器。每个都会尝试，最后抛出第一个错误。别的程序打开的不受影响。"""
        with self._sensor_lock:
            started = list(self._started_sensors)
            self._started_sensors.clear()
        first_error = None
        for sensor_id in started:
            try:
                self._sensor_operation(sensor_id, SensorOperation.STOP, timeout)
            except BaseException as exc:
                first_error = first_error or exc
        if first_error is not None:
            raise first_error

    # ---- 服务 / 话题
    def call_service(self, name: str, srv_type: str, request: dict, timeout: float = 5.0) -> dict:
        raise NotImplementedError

    def publish(self, topic: str, msg_type: str, data: dict) -> None:
        raise NotImplementedError

    def subscribe(self, topic: str, msg_type: str, callback: Callback) -> int:
        raise NotImplementedError

    def unsubscribe(self, sub_id: int) -> None:
        raise NotImplementedError

    # ---- 动作
    def send_goal(self, name: str, action_type: str, goal: dict,
                  feedback_cb: Optional[Callback] = None, timeout: float = 10.0) -> str:
        """发送目标，等到服务端接受后返回 goal_id；被拒绝则抛 GoalRejected。"""
        raise NotImplementedError

    def wait_goal(self, goal_id: str, timeout: Optional[float] = None) -> dict:
        raise NotImplementedError

    def cancel_goal(self, goal_id: str, timeout: float = 5.0) -> None:
        raise NotImplementedError

    # ---- 伺服（心跳在后端一侧执行）
    def servo_set(self, cmd: dict) -> None:
        """设置伺服目标。cmd 是 MotionServoCmd 的字段（不含 cmd_type），后端以 20Hz 持续下发。"""
        raise NotImplementedError

    def servo_stop(self) -> None:
        raise NotImplementedError

    # ---- 其他
    def latest_image(self, topic: str, timeout: float = 3.0, jpeg: bool = False):
        """返回 Frame，见 cyberdog2.frame。"""
        raise NotImplementedError

    def image_stream_start(self, topic: str, fps: float, jpeg: bool, quality: int,
                           callback: Callable[["object"], None]) -> int:
        """开始推流：每帧（限速到 fps）以 Frame 对象调用 callback。返回流 id。

        callback 必须很快返回（只存一下最新帧），否则会拖慢后端线程。
        """
        raise NotImplementedError

    def image_stream_stop(self, stream_id: int) -> None:
        raise NotImplementedError

    def list_names(self) -> Dict[str, List[str]]:
        """返回 {"topics": [...], "services": [...], "actions": [...]}，均为完整名称。"""
        raise NotImplementedError

    @property
    def fatal_error(self) -> Optional[BaseException]:
        """后台线程中发生的致命错误（例如连接断开）。为 None 表示正常。"""
        return None

    def close(self) -> None:
        raise NotImplementedError


def resolve_name(namespace: str, name: str) -> str:
    """把相对接口名拼成完整名。以 "/" 开头的名字原样返回。"""
    if name.startswith("/"):
        return name
    ns = namespace.rstrip("/")
    return "%s/%s" % (ns, name) if ns else "/" + name

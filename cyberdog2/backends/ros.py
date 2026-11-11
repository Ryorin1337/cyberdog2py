"""rclpy 后端：在狗本机直接访问 ROS 2。

由狗端代理（cyberdog2.agent）或 CyberDog.local() 使用。rclpy 和 protocol 包都在用到时才 import，
所以电脑上即使没有安装 ROS，也可以正常 import 整个库。
"""

import array
import importlib
import itertools
import threading
import time
import uuid
from typing import Dict, Iterable, Optional, Tuple

from ..errors import GoalRejected, NamespaceNotFound, SensorTimeout, ServiceTimeout, ServiceUnavailable
from ..frame import Frame, encode_jpeg
from .base import Backend, resolve_name
from .servo import ServoLoop
from .streaming import RateLimiter

NAMESPACE_PREFIX = "mi_"


def pick_namespace(nodes: Iterable[Tuple[str, str]]) -> str:
    """从 (节点名, 命名空间) 列表中找出唯一的 /mi_xxx 命名空间。找不到或找到多个都直接抛异常。"""
    found = set()
    for _, ns in nodes:
        parts = [p for p in ns.split("/") if p]
        if parts and parts[0].startswith(NAMESPACE_PREFIX):
            found.add("/" + parts[0])
    if not found:
        raise NamespaceNotFound("ROS 图中没有以 /%s 开头的命名空间，狗的 ROS 服务是否在运行？" % NAMESPACE_PREFIX)
    if len(found) > 1:
        raise NamespaceNotFound("发现多个候选命名空间 %s，请用 namespace= 参数手动指定" % sorted(found))
    return found.pop()


def load_type(type_str: str):
    """把 "protocol/srv/MotionResultCmd" 这样的类型字符串导入成对应的 Python 类。"""
    parts = type_str.split("/")
    if len(parts) != 3:
        raise ValueError("类型格式应为 包名/msg|srv|action/类名，收到 %r" % type_str)
    pkg, kind, name = parts
    return getattr(importlib.import_module("%s.%s" % (pkg, kind)), name)


def to_plain(value):
    """把 ROS 消息（或 OrderedDict、array、numpy 类型）递归转换成 dict、list 和 Python 基本类型。"""
    if hasattr(value, "get_fields_and_field_types"):
        return {k: to_plain(getattr(value, k)) for k in value.get_fields_and_field_types()}
    if isinstance(value, dict):
        return {k: to_plain(v) for k, v in value.items()}
    if isinstance(value, (bytes, bytearray)):
        return bytes(value)
    if isinstance(value, (list, tuple, array.array)):
        return [to_plain(v) for v in value]
    if hasattr(value, "tolist"):  # numpy 数组或标量
        return to_plain(value.tolist())
    return value


def fill_message(msg, data: dict):
    """用 dict 填充 ROS 消息，支持嵌套消息和消息数组。字段名写错时直接抛 AttributeError。"""
    from rosidl_runtime_py.set_message import set_message_fields

    set_message_fields(msg, data)
    return msg


class RosBackend(Backend):
    def __init__(self, namespace: Optional[str] = None, discovery_timeout: float = 5.0,
                 node_name: str = "cyberdog2py", servo_period: float = 0.05):
        super().__init__()
        import rclpy
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor

        self._rclpy = rclpy
        self._ctx = rclpy.Context()
        rclpy.init(context=self._ctx)
        self._node = rclpy.create_node(node_name + "_" + uuid.uuid4().hex[:6], context=self._ctx)
        self._group = ReentrantCallbackGroup()
        self._executor = MultiThreadedExecutor(num_threads=4, context=self._ctx)
        self._executor.add_node(self._node)
        self._fatal = None  # type: Optional[BaseException]
        self._spin_thread = threading.Thread(target=self._spin, name="cyberdog2-ros", daemon=True)
        self._spin_thread.start()
        self._lock = threading.Lock()
        self._ids = itertools.count(1)
        self._clients = {}  # type: Dict[tuple, object]
        self._publishers = {}  # type: Dict[tuple, object]
        self._subs = {}  # type: Dict[int, object]
        self._action_clients = {}  # type: Dict[tuple, object]
        self._goals = {}  # type: Dict[str, dict]
        self._images = {}  # type: Dict[str, dict]
        self._streams = {}  # type: Dict[int, object]
        self._closed = False
        self.namespace = namespace if namespace is not None else self._discover(discovery_timeout)
        self._servo = ServoLoop(lambda m: self.publish("motion_servo_cmd", "protocol/msg/MotionServoCmd", m),
                                period=servo_period)

    # ---------------------------------------------------------- 基础
    def _spin(self):
        try:
            self._executor.spin()
        except BaseException as exc:
            if not self._closed:
                self._fatal = exc

    def _discover(self, timeout: float) -> str:
        deadline = time.monotonic() + timeout
        while True:
            try:
                return pick_namespace(self._node.get_node_names_and_namespaces())
            except NamespaceNotFound:
                if time.monotonic() >= deadline:
                    raise
            time.sleep(0.2)

    def _guard(self, fn):
        """包装回调：执行器线程里的异常记录为 fatal，下一次 API 调用时抛出。"""
        def run(*args):
            if self._fatal is not None:
                return
            try:
                fn(*args)
            except BaseException as exc:
                self._fatal = exc
        return run

    @staticmethod
    def _wait(future, timeout: Optional[float], what: str):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            raise ServiceTimeout("%s 在 %.1fs 内没有返回" % (what, timeout))
        return future.result()

    # ---------------------------------------------------------- 服务 / 话题
    def call_service(self, name, srv_type, request, timeout=5.0):
        full = resolve_name(self.namespace, name)
        key = (full, srv_type)
        with self._lock:
            client = self._clients.get(key)
            if client is None:
                client = self._node.create_client(load_type(srv_type), full, callback_group=self._group)
                self._clients[key] = client
        # timeout 是整个调用（等服务出现 + 等结果）的总时长，客户端据此设置自己的等待上限
        deadline = time.monotonic() + timeout
        if not client.wait_for_service(timeout_sec=timeout):
            raise ServiceUnavailable("服务 %s 在 %.1fs 内不可用" % (full, timeout))
        req = fill_message(load_type(srv_type).Request(), request)
        return to_plain(self._wait(client.call_async(req), _left(deadline), "服务 " + full))

    def publish(self, topic, msg_type, data):
        full = resolve_name(self.namespace, topic)
        key = (full, msg_type)
        with self._lock:
            pub = self._publishers.get(key)
            if pub is None:
                pub = self._node.create_publisher(load_type(msg_type), full, 10)
                self._publishers[key] = pub
        pub.publish(fill_message(load_type(msg_type)(), data))

    def subscribe(self, topic, msg_type, callback):
        from rclpy.qos import qos_profile_sensor_data

        full = resolve_name(self.namespace, topic)
        sub = self._node.create_subscription(load_type(msg_type), full,
                                             self._guard(lambda m: callback(to_plain(m))),
                                             qos_profile_sensor_data, callback_group=self._group)
        sub_id = next(self._ids)
        with self._lock:
            self._subs[sub_id] = sub
        return sub_id

    def unsubscribe(self, sub_id):
        with self._lock:
            sub = self._subs.pop(sub_id)
        self._node.destroy_subscription(sub)

    # ---------------------------------------------------------- 动作
    def send_goal(self, name, action_type, goal, feedback_cb=None, timeout=10.0):
        from rclpy.action import ActionClient

        full = resolve_name(self.namespace, name)
        cls = load_type(action_type)
        key = (full, action_type)
        with self._lock:
            client = self._action_clients.get(key)
            if client is None:
                client = ActionClient(self._node, cls, full, callback_group=self._group)
                self._action_clients[key] = client
        deadline = time.monotonic() + timeout
        if not client.wait_for_server(timeout_sec=timeout):
            raise ServiceUnavailable("动作服务器 %s 在 %.1fs 内不可用" % (full, timeout))
        fb = None
        if feedback_cb is not None:
            fb = self._guard(lambda m: feedback_cb(to_plain(m.feedback)))
        handle = self._wait(client.send_goal_async(fill_message(cls.Goal(), goal), feedback_callback=fb),
                            _left(deadline), "发送目标到 " + full)
        if not handle.accepted:
            raise GoalRejected("目标被 %s 拒绝" % full)
        goal_id = bytes(handle.goal_id.uuid).hex()
        with self._lock:
            self._goals[goal_id] = {"handle": handle, "result": handle.get_result_async()}
        return goal_id

    def wait_goal(self, goal_id, timeout=None):
        entry = self._goals[goal_id]
        res = self._wait(entry["result"], timeout, "等待任务 " + goal_id)
        out = to_plain(res.result)
        out["_status"] = int(res.status)
        return out

    def cancel_goal(self, goal_id, timeout=5.0):
        self._wait(self._goals[goal_id]["handle"].cancel_goal_async(), timeout, "取消任务 " + goal_id)

    # ---------------------------------------------------------- 伺服
    def servo_set(self, cmd):
        self._servo.set(cmd)

    def servo_stop(self):
        self._servo.stop()

    # ---------------------------------------------------------- 图像
    def latest_image(self, topic, timeout=3.0, jpeg=False):
        full = resolve_name(self.namespace, topic)
        with self._lock:
            slot = self._images.get(full)
            if slot is None:
                slot = {"cond": threading.Condition(), "msg": None, "t": 0.0}
                self._images[full] = slot
                self._node.create_subscription(load_type("sensor_msgs/msg/Image"), full,
                                               self._guard(lambda m, s=slot: self._on_image(s, m)),
                                               _image_qos(), callback_group=self._group)
        requested = time.monotonic()
        with slot["cond"]:
            # 接受"请求发出前 1 秒之内及之后"收到的帧；更早的帧视为过期，继续等新帧。
            # 图像订阅创建后一直保留，作为缓存供后续调用复用（相机关闭时它只是收不到消息）
            while slot["msg"] is None or slot["t"] < requested - 1.0:
                left = requested + timeout - time.monotonic()
                if left <= 0:
                    raise SensorTimeout("%.1fs 内没有收到图像 %s（相机是否已用 camera.start() 开启？）"
                                        % (timeout, full))
                slot["cond"].wait(left)
            m = slot["msg"]
        frame = Frame(m.width, m.height, m.encoding, bytes(m.data), "raw",
                      m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        return encode_jpeg(frame) if jpeg else frame

    def image_stream_start(self, topic, fps, jpeg, quality, callback):
        full = resolve_name(self.namespace, topic)
        limiter = RateLimiter(fps)

        def on_image(m):
            if not limiter.allow():
                return  # 超过限速的帧直接丢弃，不做转换和压缩，节省狗的 CPU
            frame = Frame(m.width, m.height, m.encoding, bytes(m.data), "raw",
                          m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            callback(encode_jpeg(frame, quality) if jpeg else frame)

        sub = self._node.create_subscription(load_type("sensor_msgs/msg/Image"), full, self._guard(on_image),
                                             _image_qos(), callback_group=self._group)
        stream_id = next(self._ids)
        with self._lock:
            self._streams[stream_id] = sub
        return stream_id

    def image_stream_stop(self, stream_id):
        with self._lock:
            sub = self._streams.pop(stream_id)
        self._node.destroy_subscription(sub)

    @staticmethod
    def _on_image(slot, msg):
        with slot["cond"]:
            slot["msg"] = msg
            slot["t"] = time.monotonic()
            slot["cond"].notify_all()

    # ---------------------------------------------------------- 其他
    def list_names(self):
        from rclpy.action import get_action_names_and_types

        return {
            "topics": sorted(n for n, _ in self._node.get_topic_names_and_types()),
            "services": sorted(n for n, _ in self._node.get_service_names_and_types()),
            "actions": sorted(n for n, _ in get_action_names_and_types(self._node)),
        }

    @property
    def fatal_error(self):
        return self._fatal or self._servo.error

    def close(self):
        if self._closed:
            return
        try:
            self._servo.stop()
        finally:
            self._closed = True
            self._executor.shutdown()
            self._node.destroy_node()
            self._rclpy.shutdown(context=self._ctx)


def _left(deadline: float) -> float:
    """距离截止时间还剩多少秒；已经过了就返回 0（随后的等待会立即超时并抛 ServiceTimeout）。"""
    return max(0.0, deadline - time.monotonic())


def _image_qos():
    from rclpy.qos import qos_profile_sensor_data

    return qos_profile_sensor_data

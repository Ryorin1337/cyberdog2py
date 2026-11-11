"""模拟后端：没有真机时用于开发和测试。

- 发出去的指令记录在 `published`（话题）、`calls`（服务）、`goals`（动作）里。
- 服务默认返回成功；用 `set_response()` 或 `set_handler()` 可以模拟失败。
- 用 `emit(topic, data)` 注入传感器消息，例如模拟一次触摸。
- 调用未登记的服务会直接抛 ServiceUnavailable，和真机上找不到服务时的行为一致。
"""

import itertools
import threading
import time
from typing import Callable, Dict, List, Optional

from ..constants import StopTask
from ..errors import GoalRejected, SensorTimeout, ServiceUnavailable
from ..frame import Frame
from .base import Backend, resolve_name
from .servo import ServoLoop
from .streaming import RateLimiter

MOCK_NAMESPACE = "/mi_mock"


def _default_services() -> Dict[str, Callable[[dict], dict]]:
    return {
        "motion_result_cmd": lambda r: {"motion_id": r.get("motion_id", 0), "result": True, "code": 0},
        "led_execute": lambda r: {"code": 0},
        "speech_text_play": lambda r: {"status": 0, "code": 0},
        "audio_volume_get": lambda r: {"volume": 50, "code": 0},
        "audio_volume_set": lambda r: {"success": True, "code": 0},
        "camera_service": lambda r: {"result": 0, "msg": "", "code": 0},
        "sensor_operation": lambda r: {"success": True, "code": 0},
        "algo_manager": lambda r: {"result_enable": 0, "result_disable": 0, "code": 0},
        "cyberdog_face_recognition_srv": lambda r: {"result": 0, "code": 0},
        "gesture_action_control": lambda r: {"code": 0},
        "stop_algo_task": lambda r: {"result": 0, "code": 0},
    }


def _default_latched() -> Dict[str, dict]:
    return {
        "bms_status": {"batt_volt": 25000, "batt_curr": -1500, "batt_soc": 80, "batt_temp": 30,
                       "power_normal": True, "power_wired_charging": False,
                       "power_finished_charging": False},
        "motion_status": {"motion_id": 111, "switch_status": 0, "order_process_bar": 100},
        "ultrasonic_payload": {"range": 1.2, "min_range": 0.03, "max_range": 4.0},
        "head_tof_payload": {
            "left_head": {"data_available": True, "tof_position": 0, "data": [0.5] * 64},
            "right_head": {"data_available": True, "tof_position": 1, "data": [0.5] * 64},
        },
        "rear_tof_payload": {
            "left_rear": {"data_available": True, "tof_position": 2, "data": [0.66] * 64},
            "right_rear": {"data_available": True, "tof_position": 3, "data": [0.66] * 64},
        },
        "gps_payload": {"fix_type": 3, "num_sv": 8, "lon": 116.3, "lat": 40.05},
        "uwb_raw": {"dist": 1.5, "angle": 0.1, "n_los": 0.0, "rssi_1": -60.0, "rssi_2": -61.0},
        "odom_out": {"pose": {"pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0},
                                       "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
                     "twist": {"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                         "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}},
        "algo_task_status": {"task_status": 101, "task_sub_status": 0},
    }


class MockBackend(Backend):
    def __init__(self, namespace: str = MOCK_NAMESPACE, servo_period: float = 0.05):
        super().__init__()
        self.namespace = namespace
        self._lock = threading.Lock()
        self._ids = itertools.count(1)
        self.services = _default_services()
        self.services["stop_algo_task"] = self._on_stop_task
        self.latched = _default_latched()
        self.published = []  # type: List[tuple]   # (时间, 话题短名, dict)
        self.calls = []  # type: List[tuple]       # (服务短名, 请求 dict)
        self.goals = {}  # type: Dict[str, dict]
        self.goal_results = {}  # type: Dict[str, dict]
        self.reject_goals = False
        self.image = Frame(4, 3, "rgb8", bytes(4 * 3 * 3), "raw")
        self._subs = {}  # type: Dict[int, tuple]
        self._streams = {}  # type: Dict[int, threading.Event]
        self._servo = ServoLoop(lambda m: self.publish("motion_servo_cmd", "protocol/msg/MotionServoCmd", m),
                                period=servo_period)
        self.closed = False

    # ---------------------------------------------------------- 测试辅助
    def short(self, name: str) -> str:
        full = resolve_name(self.namespace, name)
        prefix = self.namespace.rstrip("/") + "/"
        return full[len(prefix):] if full.startswith(prefix) else full

    def set_response(self, service: str, response: dict) -> None:
        """之后对该服务的调用都返回固定的 response。"""
        self.services[service] = lambda r: dict(response)

    def set_handler(self, service: str, handler: Callable[[dict], dict]) -> None:
        self.services[service] = handler

    def emit(self, topic: str, data: dict) -> None:
        """模拟狗发布一条消息（在调用线程里同步分发给订阅者）。"""
        short = self.short(topic)
        with self._lock:
            targets = [cb for (t, cb) in self._subs.values() if t == short]
        for cb in targets:
            cb(dict(data))

    def published_on(self, topic: str) -> List[dict]:
        short = self.short(topic)
        with self._lock:
            return [d for (_, t, d) in self.published if t == short]

    def calls_to(self, service: str) -> List[dict]:
        short = self.short(service)
        with self._lock:
            return [r for (s, r) in self.calls if s == short]

    def _set_task_status(self, status: int) -> None:
        self.latched["algo_task_status"] = {"task_status": status, "task_sub_status": 0}

    def _on_stop_task(self, req: dict) -> dict:
        """规则与 cyberdog_nav2 的 HandleStopTaskCallback 一致。"""
        status = self.latched["algo_task_status"]["task_status"]
        task = req["task_id"]
        if task == StopTask.ALL:
            if status not in (1, 21, 7, 17, 8, 18):
                return {"result": 1, "code": 0}
            self._set_task_status(101)
        elif {15: 5, 17: 7, 3: 13, 21: 1}.get(status, status) == task:
            self._set_task_status(101)
        return {"result": 0, "code": 0}

    # ---------------------------------------------------------- Backend
    def call_service(self, name, srv_type, request, timeout=5.0):
        short = self.short(name)
        if short not in self.services:
            raise ServiceUnavailable("服务 %s 不可用（mock 未登记）" % resolve_name(self.namespace, name))
        with self._lock:
            self.calls.append((short, dict(request)))
        return self.services[short](dict(request))

    def publish(self, topic, msg_type, data):
        with self._lock:
            self.published.append((time.time(), self.short(topic), dict(data)))

    def subscribe(self, topic, msg_type, callback):
        short = self.short(topic)
        sub_id = next(self._ids)
        with self._lock:
            self._subs[sub_id] = (short, callback)
        if short in self.latched:
            callback(dict(self.latched[short]))
        return sub_id

    def unsubscribe(self, sub_id):
        with self._lock:
            self._subs.pop(sub_id)

    def send_goal(self, name, action_type, goal, feedback_cb=None, timeout=10.0):
        if self.reject_goals:
            raise GoalRejected("目标被拒绝（mock）")
        goal_id = "goal-%d" % next(self._ids)
        self._set_task_status(int(goal["nav_type"]))
        with self._lock:
            self.goals[goal_id] = {"name": self.short(name), "goal": dict(goal), "feedback_cb": feedback_cb,
                                   "cancelled": False}
        return goal_id

    def wait_goal(self, goal_id, timeout=None):
        if goal_id not in self.goals:
            raise KeyError("未知 goal_id: %s" % goal_id)
        return dict(self.goal_results.get(goal_id, {"result": 0}))

    def cancel_goal(self, goal_id, timeout=5.0):
        self.goals[goal_id]["cancelled"] = True

    def servo_set(self, cmd):
        self._servo.set(cmd)

    def servo_stop(self):
        self._servo.stop()

    @property
    def servo_active(self) -> bool:
        return self._servo.active

    def latest_image(self, topic, timeout=3.0, jpeg=False):
        if self.image is None:
            raise SensorTimeout("%.1fs 内没有收到图像 %s" % (timeout, topic))
        return self.image

    def image_stream_start(self, topic, fps, jpeg, quality, callback):
        """模拟推流：按 fps 不断发送 self.image（stamp 为当前时间）。

        mock 不做 JPEG 压缩，jpeg 参数会被忽略，始终返回 raw 帧。
        self.image 为 None 时不发送任何帧，可以用来测试超时。
        """
        RateLimiter(fps)  # 只做参数校验
        stream_id = next(self._ids)
        stop = threading.Event()

        def run():
            seq = 0
            while not stop.wait(1.0 / fps):
                img = self.image
                if img is None:
                    continue
                seq += 1
                callback(Frame(img.width, img.height, img.encoding, img.data, img.format, time.time()))

        with self._lock:
            self._streams[stream_id] = stop
        threading.Thread(target=run, name="mock-stream", daemon=True).start()
        return stream_id

    def image_stream_stop(self, stream_id):
        with self._lock:
            stop = self._streams.pop(stream_id)
        stop.set()

    @property
    def active_streams(self) -> int:
        with self._lock:
            return len(self._streams)

    def list_names(self):
        ns = self.namespace.rstrip("/")
        services = sorted(ns + "/" + s for s in self.services)
        topics = sorted({ns + "/" + t for t in self.latched} | {ns + "/motion_servo_cmd", ns + "/touch_status"})
        return {"topics": topics, "services": services, "actions": [ns + "/start_algo_task"]}

    @property
    def fatal_error(self) -> Optional[BaseException]:
        return self._servo.error

    def close(self):
        if self.closed:
            return
        self.closed = True
        for sid in list(self._streams):
            self.image_stream_stop(sid)
        self._servo.stop()

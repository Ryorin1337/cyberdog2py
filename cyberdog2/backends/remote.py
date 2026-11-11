"""远程后端：电脑端，通过 SSH 隧道和狗端代理交换 JSON 消息。"""

import itertools
import sys
import threading
import traceback
from typing import Callable, Dict, Optional

from ..errors import ConnectionLost, CyberDogError, ServiceTimeout, rebuild
from ..frame import Frame
from ..protocol import PROTOCOL_VERSION, LineStream
from .base import Backend

# 客户端额外等待的余量：服务端的超时由狗端自己判断，客户端只兜底网络卡死的情况
_NET_MARGIN = 5.0


class RemoteBackend(Backend):
    def __init__(self, sock, ping_interval: float = 0.2, on_close: Optional[Callable[[], None]] = None,
                 log_tail: Optional[Callable[[], str]] = None):
        self._stream = LineStream(sock)
        self._ping_interval = ping_interval
        self._on_close = on_close
        self._log_tail = log_tail
        self._ids = itertools.count(1)
        self._pending = {}  # type: Dict[int, dict]
        self._subs = {}  # type: Dict[int, Callable]
        self._early = {}  # type: Dict[int, list]
        self._feedback = {}  # type: Dict[str, Callable]
        self._early_feedback = {}  # type: Dict[str, list]
        self._frame_handlers = {}  # type: Dict[int, Callable]
        self._early_frames = {}  # type: Dict[int, dict]
        self._lock = threading.Lock()
        self._fatal = None  # type: Optional[BaseException]
        self._closed = False
        self._closed_evt = threading.Event()
        # 用户回调放在单独的线程里执行，避免阻塞网络读取
        self._cb_queue = []
        self._cb_cond = threading.Condition()
        threading.Thread(target=self._reader, name="cyberdog2-reader", daemon=True).start()
        threading.Thread(target=self._pinger, name="cyberdog2-ping", daemon=True).start()
        threading.Thread(target=self._dispatcher, name="cyberdog2-callbacks", daemon=True).start()
        info = self._request("info", {}, 10.0)
        if info.get("protocol") != PROTOCOL_VERSION:
            self.close()
            raise CyberDogError("代理协议版本 %r 与本地 %r 不一致：狗端运行的不是同一版本的 cyberdog2"
                                % (info.get("protocol"), PROTOCOL_VERSION))
        self.namespace = info["namespace"]

    # ------------------------------------------------------------ SSH 工厂
    @classmethod
    def connect_ssh(cls, host: str, user: str = "mi", password: Optional[str] = None, key_file: Optional[str] = None,
                    port: int = 22, namespace: Optional[str] = None, **agent_options) -> "RemoteBackend":
        """登录 SSH，部署并启动代理，打开隧道，返回已连接的后端。

        agent_options 可用参数：
            setup_scripts      在狗上依次 source 的 ROS 环境脚本列表
            inherit_env_from   从哪个进程继承 ROS 环境变量（默认 "motion_manager"，传 None 关闭）
            env                额外导出的环境变量 dict，例如 {"ROS_DOMAIN_ID": "42"}
            start_timeout      等待代理启动的秒数（默认 30）
            watchdog           代理看门狗超时秒数（默认 1）
            discovery_timeout  代理等待发现 /mi_xxx 命名空间的秒数（默认 8）
            accept_timeout     代理启动后等待电脑连入的秒数（默认 30）
            ssh_timeout        SSH 建立连接的超时秒数（默认 10）
            python             狗上运行代理的解释器（默认 "python3"），必须与 rclpy 的编译版本一致
        """
        from ..deploy import SshSession

        ssh_timeout = agent_options.pop("ssh_timeout", 10.0)
        session = SshSession(host, user=user, password=password, key_file=key_file, port=port, timeout=ssh_timeout)
        try:
            agent = session.start_agent(namespace=namespace, **agent_options)
            sock = session.open_tunnel(agent.port)
        except BaseException:
            session.close()
            raise

        def on_close():
            agent.close()
            session.close()

        return cls(sock, on_close=on_close, log_tail=agent.log_tail)

    # ------------------------------------------------------------ 通信
    def _fail(self, exc: BaseException) -> None:
        if self._fatal is None and not self._closed:
            self._fatal = exc
        with self._lock:
            pending = list(self._pending.values())
        for p in pending:
            p["error"] = self._fatal or exc
            p["event"].set()

    def _reader(self) -> None:
        try:
            while True:
                msg = self._stream.recv()
                if msg is None:
                    break
                self._on_message(msg)
        except BaseException as exc:
            if not self._closed:
                self._fail(ConnectionLost("读取代理消息失败: %r" % exc))
                return
        if not self._closed:
            tail = self._log_tail() if self._log_tail else ""
            self._fail(ConnectionLost("与狗端代理的连接已断开" + ("\n--- 代理输出 ---\n" + tail if tail else "")))
        self._closed_evt.set()

    def _on_message(self, msg: dict) -> None:
        if "id" in msg:
            with self._lock:
                p = self._pending.pop(msg["id"], None)
            if p is None:
                return  # 请求已经在本地超时，丢弃迟到的响应
            if msg["ok"]:
                p["result"] = msg.get("result")
            else:
                e = msg["error"]
                p["error"] = rebuild(e["type"], e["message"], e.get("traceback", ""), e.get("extra"))
            p["event"].set()
            return
        ev = msg.get("ev")
        if ev == "sub":
            with self._lock:
                cb = self._subs.get(msg["sub"])
                if cb is None:
                    self._early.setdefault(msg["sub"], []).append(msg["data"])
                    return
            self._enqueue(cb, msg["data"])
        elif ev == "frame":
            # 帧直接在读取线程里交给处理函数（只是存入最新帧槽，很快），不经过回调队列
            with self._lock:
                handler = self._frame_handlers.get(msg["stream"])
                if handler is None:
                    self._early_frames[msg["stream"]] = msg["data"]
                    return
            handler(Frame.from_dict(msg["data"]))
        elif ev == "feedback":
            with self._lock:
                cb = self._feedback.get(msg["goal"])
                if cb is None:
                    # send_goal 的响应还没到，先缓存，注册回调时再补发
                    self._early_feedback.setdefault(msg["goal"], []).append(msg["data"])
                    return
            self._enqueue(cb, msg["data"])
        elif ev == "fatal":
            e = msg["error"]
            self._fail(rebuild(e["type"], e["message"], e.get("traceback", ""), e.get("extra")))

    def _enqueue(self, cb, data) -> None:
        with self._cb_cond:
            self._cb_queue.append((cb, data))
            self._cb_cond.notify()

    def _dispatcher(self) -> None:
        while True:
            with self._cb_cond:
                while not self._cb_queue:
                    if self._closed:
                        return
                    self._cb_cond.wait(0.5)
                cb, data = self._cb_queue.pop(0)
            try:
                cb(data)
            except BaseException as exc:
                # CyberDog 层会包装用户回调，这里只会捕获到库内部的错误
                traceback.print_exc()
                self._fail(exc)

    def _pinger(self) -> None:
        while not self._closed_evt.wait(self._ping_interval):
            if self._closed or self._fatal is not None:
                return
            try:
                self._stream.send({"op": "ping"})
            except BaseException as exc:
                if not self._closed:
                    self._fail(ConnectionLost("发送心跳失败: %r" % exc))
                return

    def _request(self, op: str, args: dict, timeout: Optional[float]):
        if self._fatal is not None:
            raise self._fatal
        if self._closed:
            raise ConnectionLost("连接已关闭")
        req_id = next(self._ids)
        slot = {"event": threading.Event(), "result": None, "error": None}
        with self._lock:
            self._pending[req_id] = slot
        try:
            self._stream.send({"id": req_id, "op": op, "args": args})
        except BaseException as exc:
            with self._lock:
                self._pending.pop(req_id, None)
            self._fail(ConnectionLost("发送请求失败: %r" % exc))
            raise self._fatal
        wait = None if timeout is None else timeout + _NET_MARGIN
        if not slot["event"].wait(wait):
            with self._lock:
                self._pending.pop(req_id, None)
            raise ServiceTimeout("代理在 %.1fs 内没有响应 %s 请求" % (wait, op))
        if slot["error"] is not None:
            raise slot["error"]
        return slot["result"]

    # ------------------------------------------------------------ Backend
    def call_service(self, name, srv_type, request, timeout=5.0):
        return self._request("call_service", {"name": name, "type": srv_type, "request": request,
                                              "timeout": timeout}, timeout)

    def publish(self, topic, msg_type, data):
        self._request("publish", {"topic": topic, "type": msg_type, "data": data}, 5.0)

    def subscribe(self, topic, msg_type, callback):
        sub_id = self._request("subscribe", {"topic": topic, "type": msg_type}, 10.0)
        with self._lock:
            self._subs[sub_id] = callback
            early = self._early.pop(sub_id, [])
        for data in early:
            self._enqueue(callback, data)
        return sub_id

    def unsubscribe(self, sub_id):
        with self._lock:
            self._subs.pop(sub_id)
        self._request("unsubscribe", {"sub": sub_id}, 5.0)

    def send_goal(self, name, action_type, goal, feedback_cb=None, timeout=10.0):
        goal_id = self._request("send_goal", {"name": name, "type": action_type, "goal": goal,
                                              "feedback": feedback_cb is not None, "timeout": timeout}, timeout)
        with self._lock:
            early = self._early_feedback.pop(goal_id, [])
            if feedback_cb is not None:
                self._feedback[goal_id] = feedback_cb
        for data in early:
            self._enqueue(feedback_cb, data)
        return goal_id

    def wait_goal(self, goal_id, timeout=None):
        result = self._request("wait_goal", {"goal": goal_id, "timeout": timeout}, timeout)
        # 任务已经结束，ROS 动作语义下之后不会再有反馈，释放回调
        with self._lock:
            self._feedback.pop(goal_id, None)
            self._early_feedback.pop(goal_id, None)
        return result

    def cancel_goal(self, goal_id, timeout=5.0):
        self._request("cancel_goal", {"goal": goal_id, "timeout": timeout}, timeout)

    # 打开过哪些传感器由狗端代理记录，这样电脑崩溃或断网时代理也能把它们关掉
    def sensor_start(self, sensor_id, timeout=15.0):
        return self._request("sensor_start", {"sensor_id": int(sensor_id), "timeout": timeout}, timeout)

    def sensors_stop_started(self, timeout=15.0):
        return self._request("sensors_stop_started", {"timeout": timeout}, timeout)

    def servo_set(self, cmd):
        self._request("servo_set", {"cmd": cmd}, 2.0)

    def servo_stop(self):
        self._request("servo_stop", {}, 2.0)

    def latest_image(self, topic, timeout=3.0, jpeg=False):
        return Frame.from_dict(self._request("latest_image", {"topic": topic, "timeout": timeout, "jpeg": jpeg},
                                             timeout))

    def image_stream_start(self, topic, fps, jpeg, quality, callback):
        sid = self._request("image_stream_start", {"topic": topic, "fps": fps, "jpeg": jpeg,
                                                   "quality": quality}, 10.0)
        with self._lock:
            self._frame_handlers[sid] = callback
            early = self._early_frames.pop(sid, None)
        if early is not None:
            callback(Frame.from_dict(early))
        return sid

    def image_stream_stop(self, stream_id):
        with self._lock:
            self._frame_handlers.pop(stream_id)
            self._early_frames.pop(stream_id, None)
        self._request("image_stream_stop", {"stream": stream_id}, 5.0)

    def list_names(self):
        return self._request("list_names", {}, 10.0)

    @property
    def fatal_error(self):
        return self._fatal

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._closed_evt.set()
        try:
            self._stream.close()
        finally:
            with self._cb_cond:
                self._cb_cond.notify_all()
            if self._on_close is not None:
                self._on_close()

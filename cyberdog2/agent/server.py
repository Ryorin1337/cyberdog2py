"""狗端代理服务。

只监听 127.0.0.1，一次只服务一个客户端；客户端断开后 serve_one() 返回，进程随之退出。
看门狗：超过 watchdog 秒没有收到客户端的任何消息（包括 ping），就停止伺服并断开连接。
"""

import socket
import sys
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor

from ..backends.base import Backend
from ..backends.streaming import LatestSlot
from ..frame import Frame
from ..navigation import release_tasks
from ..protocol import PROTOCOL_VERSION, LineStream, error_payload


class AgentServer(object):
    def __init__(self, backend: Backend, host: str = "127.0.0.1", port: int = 0, watchdog: float = 1.0):
        self.backend = backend
        self.watchdog = watchdog
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((host, port))
        self._listener.listen(1)
        self.port = self._listener.getsockname()[1]
        self._subs = set()
        self._goals = set()
        self._streams = {}
        self.stopped_by_watchdog = False

    # ------------------------------------------------------------------
    def serve_one(self, accept_timeout: float = 30.0) -> None:
        self._listener.settimeout(accept_timeout)
        conn, _ = self._listener.accept()
        self._listener.close()
        conn.settimeout(None)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        stream = LineStream(conn)
        self._last_rx = time.monotonic()
        alive = threading.Event()
        alive.set()
        dog_thread = threading.Thread(target=self._watchdog_loop, args=(stream, alive), daemon=True)
        dog_thread.start()
        pool = ThreadPoolExecutor(max_workers=8)
        try:
            while True:
                try:
                    msg = stream.recv()
                except OSError:
                    msg = None
                if msg is None:
                    break
                self._last_rx = time.monotonic()
                op = msg.get("op")
                if op == "ping":
                    continue
                pool.submit(self._handle, stream, msg)
        finally:
            alive.clear()
            try:
                self._cleanup()
            finally:
                pool.shutdown(wait=False)
                stream.close()

    def _watchdog_loop(self, stream: LineStream, alive: threading.Event) -> None:
        while alive.is_set():
            time.sleep(self.watchdog / 4.0)
            fatal = self.backend.fatal_error
            if alive.is_set() and fatal is not None:
                # 后端出现致命错误（例如 ROS 执行器崩溃）：通知客户端后断开，let it crash
                try:
                    stream.send({"ev": "fatal", "error": error_payload(fatal)})
                finally:
                    stream.close()
                return
            if alive.is_set() and time.monotonic() - self._last_rx > self.watchdog:
                self.stopped_by_watchdog = True
                sys.stderr.write("[cyberdog2-agent] %.1fs 没有收到客户端消息，停止运动并断开\n" % self.watchdog)
                self.backend.servo_stop()
                stream.close()
                return

    def _cleanup(self) -> None:
        """客户端断开后的安全收尾。

        顺序：停止行走 → 停止本次会话启动过的导航/跟随任务 → 停推流 → 退订 → 关闭本次会话打开的传感器。
        每一步都会执行；某一步失败时打印 traceback 后继续，最后再把第一个错误抛出（let it crash），
        这样任何一步出错都不会让后面的安全步骤被跳过。
        """
        errors = []

        def attempt(fn, *args, **kw):
            try:
                fn(*args, **kw)
            except BaseException as exc:
                traceback.print_exc()
                errors.append(exc)

        attempt(self.backend.servo_stop)
        if self._goals:
            # 跟随类任务在 goal 被接受后就会在狗上持续运行，取消 goal 不一定能停下，
            # 所以先按狗的实际状态结束任务，再取消各个 goal
            attempt(release_tasks, self.backend, service_timeout=10.0)
            for goal_id in list(self._goals):
                attempt(self.backend.cancel_goal, goal_id, 5.0)
            self._goals.clear()
        for sid in list(self._streams):
            attempt(self._stop_stream, sid)
        for sub_id in list(self._subs):
            attempt(self.backend.unsubscribe, sub_id)
        self._subs.clear()
        attempt(self.backend.sensors_stop_started)
        if errors:
            raise errors[0]

    # ------------------------------------------------------------------ 推流
    def _start_stream(self, stream: LineStream, a: dict) -> int:
        """每个流配一个发送线程和一个只保留最新帧的槽。

        网络跟不上时，槽里的旧帧会被覆盖，所以 TCP 里不会积压大量过期画面。
        """
        slot = LatestSlot()
        stop = threading.Event()
        holder = {"id": None}

        def sender():
            while not stop.is_set():
                frame = slot.get(0.2)
                if frame is None or holder["id"] is None:
                    continue
                try:
                    stream.send({"ev": "frame", "stream": holder["id"], "data": frame.to_dict()})
                except OSError:
                    return  # 客户端已断开，主循环会负责收尾

        sid = self.backend.image_stream_start(a["topic"], a["fps"], a["jpeg"], a["quality"], slot.put)
        holder["id"] = sid
        self._streams[sid] = (stop, slot)
        threading.Thread(target=sender, name="agent-stream-%d" % sid, daemon=True).start()
        return sid

    def _stop_stream(self, sid: int) -> None:
        stop, slot = self._streams.pop(sid)
        stop.set()
        slot.wake()
        self.backend.image_stream_stop(sid)

    # ------------------------------------------------------------------
    def _handle(self, stream: LineStream, msg: dict) -> None:
        req_id = msg.get("id")
        try:
            result = self._dispatch(stream, msg["op"], msg.get("args") or {})
            reply = {"id": req_id, "ok": True, "result": result}
        except BaseException as exc:
            reply = {"id": req_id, "ok": False, "error": error_payload(exc)}
        try:
            stream.send(reply)
        except OSError:
            pass  # 客户端已经断开，主循环会负责收尾
        except Exception as exc:
            # 结果无法编码成 JSON：把编码错误作为这次请求的错误返回，客户端立即抛出，而不是傻等超时
            try:
                stream.send({"id": req_id, "ok": False, "error": error_payload(exc)})
            except OSError:
                pass

    def _dispatch(self, stream: LineStream, op: str, a: dict):
        b = self.backend
        if op == "info":
            return {"namespace": b.namespace, "protocol": PROTOCOL_VERSION}
        if op == "call_service":
            return b.call_service(a["name"], a["type"], a["request"], a.get("timeout", 5.0))
        if op == "publish":
            return b.publish(a["topic"], a["type"], a["data"])
        if op == "subscribe":
            holder = {}

            def forward(data, holder=holder):
                try:
                    stream.send({"ev": "sub", "sub": holder["id"], "data": data})
                except OSError:
                    pass  # 客户端断开时，主循环会负责退订
            # subscribe() 期间同步到达的消息（例如 latched）先暂存，拿到 id 后再转发；
            # 客户端会缓存还不认识的订阅 id 的事件
            pending = []
            holder["id"] = None

            def early(data, holder=holder, pending=pending):
                if holder["id"] is None:
                    pending.append(data)
                else:
                    forward(data)
            sub_id = b.subscribe(a["topic"], a["type"], early)
            holder["id"] = sub_id
            self._subs.add(sub_id)
            for d in pending:
                forward(d)
            return sub_id
        if op == "unsubscribe":
            self._subs.discard(a["sub"])
            return b.unsubscribe(a["sub"])
        if op == "send_goal":
            holder = {"id": None}

            def fb(data, holder=holder):
                if holder["id"] is not None:
                    stream.send({"ev": "feedback", "goal": holder["id"], "data": data})
            goal_id = b.send_goal(a["name"], a["type"], a["goal"], feedback_cb=fb if a.get("feedback") else None,
                                  timeout=a.get("timeout", 10.0))
            holder["id"] = goal_id
            self._goals.add(goal_id)
            return goal_id
        if op == "wait_goal":
            return b.wait_goal(a["goal"], a.get("timeout"))
        if op == "cancel_goal":
            return b.cancel_goal(a["goal"], a.get("timeout", 5.0))
        if op == "sensor_start":
            return b.sensor_start(a["sensor_id"], a.get("timeout", 15.0))
        if op == "sensors_stop_started":
            return b.sensors_stop_started(a.get("timeout", 15.0))
        if op == "servo_set":
            return b.servo_set(a["cmd"])
        if op == "servo_stop":
            return b.servo_stop()
        if op == "latest_image":
            frame = b.latest_image(a["topic"], a.get("timeout", 3.0), a.get("jpeg", False))
            return frame.to_dict()
        if op == "image_stream_start":
            return self._start_stream(stream, a)
        if op == "image_stream_stop":
            return self._stop_stream(a["stream"])
        if op == "list_names":
            return b.list_names()
        raise ValueError("未知操作: %r" % op)

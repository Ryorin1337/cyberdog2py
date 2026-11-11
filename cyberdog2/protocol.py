"""电脑与狗端代理之间的通信协议：每行一个 JSON。

请求   {"id": 1, "op": "call_service", "args": {...}}
响应   {"id": 1, "ok": true, "result": ...}
       {"id": 1, "ok": false, "error": {"type": "ServiceTimeout", "message": "...", "traceback": "...", "extra": {}}}
事件   {"ev": "sub", "sub": 3, "data": {...}}                订阅消息
       {"ev": "feedback", "goal": "abc", "data": {...}}       动作反馈
       {"ev": "frame", "stream": 2, "data": {帧 dict}}         视频流帧
       {"ev": "fatal", "error": {...}}                        代理端的致命错误
心跳   {"op": "ping"}                                          没有 id，代理不回复
bytes 编码为 {"__b64__": "..."}；NaN/Inf 编码为 {"__f__": "nan" | "inf" | "-inf"}。
"""

import base64
import json
import math
import socket
import threading
import traceback
from typing import Optional

PROTOCOL_VERSION = 2
READY_MARKER = "CYBERDOG2_AGENT_READY"
ERROR_MARKER = "CYBERDOG2_AGENT_ERROR"


def _encode(obj):
    if isinstance(obj, float) and not math.isfinite(obj):
        # JSON 不支持 NaN/Inf，而 Range 超量程等场景下 ROS 消息里会出现这些值，原样编码后在另一端还原
        return {"__f__": repr(obj)}
    if isinstance(obj, (bytes, bytearray)):
        return {"__b64__": base64.b64encode(bytes(obj)).decode("ascii")}
    if isinstance(obj, dict):
        return {str(k): _encode(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_encode(v) for v in obj]
    return obj


def _decode(obj):
    if isinstance(obj, dict):
        if len(obj) == 1 and "__b64__" in obj:
            return base64.b64decode(obj["__b64__"])
        if len(obj) == 1 and "__f__" in obj:
            return float(obj["__f__"])
        return {k: _decode(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_decode(v) for v in obj]
    return obj


def dumps(msg: dict) -> bytes:
    return (json.dumps(_encode(msg), ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")


def loads(line: bytes) -> dict:
    return _decode(json.loads(line.decode("utf-8")))


def error_payload(exc: BaseException) -> dict:
    extra = {}
    if hasattr(exc, "code"):
        extra["code"] = getattr(exc, "code")
    if hasattr(exc, "response"):
        extra["response"] = getattr(exc, "response")
    return {"type": type(exc).__name__, "message": str(exc),
            "traceback": "".join(traceback.format_exception(type(exc), exc, exc.__traceback__)),
            "extra": extra}


class LineStream(object):
    """在 socket 或 paramiko Channel 上收发 JSON 行。发送有锁保护，可以在多个线程里调用。"""

    def __init__(self, sock):
        self._sock = sock
        self._buf = b""
        self._send_lock = threading.Lock()

    def send(self, msg: dict) -> None:
        data = dumps(msg)
        with self._send_lock:
            self._sock.sendall(data)

    def recv(self) -> Optional[dict]:
        """读取一条消息。对端关闭时返回 None。"""
        while b"\n" not in self._buf:
            chunk = self._sock.recv(65536)
            if not chunk:
                return None
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return loads(line)

    def close(self) -> None:
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except (OSError, AttributeError):
            pass  # 对端已关闭，或者 paramiko Channel 没有 shutdown(how) 的这种用法
        self._sock.close()

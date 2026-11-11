"""伺服心跳循环。

motion_manager 要求两帧伺服指令的间隔不超过 200ms，否则会强制站立。
本循环以固定周期（默认 50ms）重复下发 SERVO_DATA；stop() 时发送一帧 SERVO_END。
如果发布过程中出错，会记录下来，下一次调用 set() 或 stop() 时抛出。
"""

import threading
import time
from typing import Callable, Optional

from ..constants import MOTION_ARRAY_SIZES, ServoCmdType


def fill_motion_arrays(cmd: dict) -> dict:
    """把运动指令中的数组按运控 LCM 的固定长度补零；超长直接抛 ValueError。"""
    out = dict(cmd)
    for key, size in MOTION_ARRAY_SIZES.items():
        values = [float(v) for v in out.get(key) or []]
        if len(values) > size:
            raise ValueError("%s 最多 %d 个元素，收到 %d 个" % (key, size, len(values)))
        out[key] = values + [0.0] * (size - len(values))
    return out


class ServoLoop(object):
    def __init__(self, publish: Callable[[dict], None], period: float = 0.05):
        self._publish = publish
        self._period = period
        self._lock = threading.Lock()
        self._target = None  # type: Optional[dict]
        self._thread = None  # type: Optional[threading.Thread]
        self._stop_evt = threading.Event()
        self._error = None  # type: Optional[BaseException]
        self._deadline = None  # type: Optional[float]
        self.expired_count = 0  # 因 expire_after 到期而自动停止的次数

    @property
    def active(self) -> bool:
        return self._thread is not None

    @property
    def error(self) -> Optional[BaseException]:
        return self._error

    def _raise_if_failed(self):
        if self._error is not None:
            raise self._error

    def set(self, cmd: dict) -> None:
        """设置伺服目标。

        cmd 里可以带 expire_after（秒）：从这次 set() 起，如果超过这个时间没有再次调用 set()，
        就自动发送 SERVO_END。这是防"程序卡死但进程没退出"的死人开关；不带则一直下发到 stop()。
        """
        self._raise_if_failed()
        cmd = dict(cmd)
        expire_after = cmd.pop("expire_after", None)
        if expire_after is not None and expire_after <= 0:
            raise ValueError("expire_after 必须为正数，收到 %r" % expire_after)
        frame = fill_motion_arrays(cmd)
        frame["cmd_type"] = int(ServoCmdType.DATA)
        with self._lock:
            self._target = frame
            self._deadline = None if expire_after is None else time.monotonic() + expire_after
            if self._thread is None:
                self._stop_evt.clear()
                self._thread = threading.Thread(target=self._run, name="cyberdog2-servo", daemon=True)
                self._thread.start()

    def stop(self) -> None:
        with self._lock:
            thread, target = self._thread, self._target
            self._thread = None
            self._target = None
        if thread is not None:
            self._stop_evt.set()
            thread.join()
            end = dict(target)
            end["cmd_type"] = int(ServoCmdType.END)
            end["vel_des"] = [0.0] * MOTION_ARRAY_SIZES["vel_des"]
            self._publish(end)
        self._raise_if_failed()

    def _run(self):
        try:
            while True:
                with self._lock:
                    frame = self._target
                    expired = self._deadline is not None and time.monotonic() >= self._deadline
                    if expired:
                        self._target = None
                        self._thread = None
                        self.expired_count += 1
                if frame is None:
                    return
                if expired:
                    end = dict(frame)
                    end["cmd_type"] = int(ServoCmdType.END)
                    end["vel_des"] = [0.0] * MOTION_ARRAY_SIZES["vel_des"]
                    self._publish(end)
                    return
                self._publish(frame)
                if self._stop_evt.wait(self._period):
                    return
        except BaseException as exc:  # 记录下来，由主线程下一次调用时抛出
            self._error = exc
            with self._lock:
                self._thread = None
                self._target = None

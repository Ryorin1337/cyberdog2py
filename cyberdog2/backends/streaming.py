"""推流用的小工具：帧率限制器和"只保留最新一帧"的槽。"""

import threading
import time
from typing import Optional


class RateLimiter(object):
    """限制每秒最多放行 fps 次。线程安全。"""

    def __init__(self, fps: float):
        if fps <= 0:
            raise ValueError("fps 必须为正数，收到 %r" % fps)
        self._interval = 1.0 / fps
        self._next = 0.0
        self._lock = threading.Lock()

    def allow(self, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        with self._lock:
            if now < self._next:
                return False
            # 以上一次计划时间为基准累加，避免帧率因为抖动逐渐偏低；落后太多时重新对齐
            self._next = max(self._next + self._interval, now)
            return True


class LatestSlot(object):
    """只保留最新的一项。消费者处理不过来时，旧的项会被覆盖丢弃，所以画面不会积压、越来越延迟。"""

    def __init__(self):
        self._cond = threading.Condition()
        self._item = None
        self._has = False
        self.received = 0
        self.dropped = 0

    def put(self, item) -> None:
        with self._cond:
            if self._has:
                self.dropped += 1
            self._item = item
            self._has = True
            self.received += 1
            self._cond.notify_all()

    def get(self, timeout: Optional[float]):
        """取出最新一项。timeout 秒内没有新项时返回 None。"""
        with self._cond:
            if not self._has:
                self._cond.wait(timeout)
            if not self._has:
                return None
            item, self._item, self._has = self._item, None, False
            return item

    def wake(self) -> None:
        with self._cond:
            self._cond.notify_all()

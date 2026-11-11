"""视频流测试。"""
import time

import pytest

from cyberdog2 import CallbackError, ConnectionLost, SensorTimeout
from cyberdog2.backends.streaming import LatestSlot, RateLimiter
from cyberdog2.frame import Frame

from conftest import RemotePair


def test_rate_limiter():
    lim = RateLimiter(10)
    allowed = [t for t in [i * 0.01 for i in range(100)] if lim.allow(t)]  # 1 秒内 100 次请求
    assert 9 <= len(allowed) <= 11


def test_rate_limiter_rejects_bad_fps():
    with pytest.raises(ValueError):
        RateLimiter(0)


def test_latest_slot_drops_old():
    slot = LatestSlot()
    for i in range(5):
        slot.put(i)
    assert slot.get(0) == 4
    assert slot.get(0) is None
    assert (slot.received, slot.dropped) == (5, 4)


def test_stream_mock(dog):
    with dog.sensors.camera.stream(fps=20) as s:
        frames = [s.read() for _ in range(5)]
    stamps = [f.stamp for f in frames]
    assert stamps == sorted(stamps) and len(set(stamps)) == 5
    assert dog.backend.active_streams == 0
    cmds = [c["command"] for c in dog.backend.calls_to("camera_service")]
    assert cmds == [9, 10]  # 自动开启和关闭相机


def test_stream_iter_and_slow_consumer_drops(dog):
    with dog.sensors.camera.stream(fps=30) as s:
        for i, frame in enumerate(s):
            time.sleep(0.15)  # 识别很慢
            if i == 3:
                break
        assert s.dropped > 0


def test_stream_timeout(dog):
    dog.backend.image = None
    with dog.sensors.camera.stream(fps=10, timeout=0.3) as s:
        with pytest.raises(SensorTimeout):
            s.read()


def test_stream_bad_params(dog):
    with pytest.raises(ValueError):
        dog.sensors.camera.stream(fps=60)
    with pytest.raises(ValueError):
        dog.sensors.camera.stream(quality=0)
    assert dog.backend.calls_to("camera_service") == []


def test_stream_raises_after_callback_crash(dog):
    dog.sensors.on_touch(lambda ev: 1 / 0)
    with dog.sensors.camera.stream(fps=20) as s:
        s.read()
        dog.backend.emit("touch_status", {"touch_state": 1, "timestamp": 0})
        with pytest.raises(CallbackError):
            s.read()
    assert dog.backend.active_streams == 0
    assert dog.backend.calls_to("camera_service")[-1]["command"] == 10  # 回调崩溃后相机仍被关闭


def test_stream_remote(remote):
    remote.mock.image = Frame(2, 2, "rgb8", bytes(range(12)), "raw")
    with remote.dog.sensors.camera.stream(fps=20, jpeg=False) as s:
        frames = [s.read() for _ in range(4)]
    assert all(f.data == bytes(range(12)) for f in frames)
    assert len({f.stamp for f in frames}) == 4
    time.sleep(0.1)
    assert remote.mock.active_streams == 0


def test_stream_remote_disconnect_raises():
    pair = RemotePair()
    s = pair.dog.sensors.camera.stream(fps=20, jpeg=False)
    s.read()
    pair.sock.close()
    with pytest.raises(ConnectionLost):
        for _ in range(50):
            s.read()
    pair.thread.join(3)
    assert pair.mock.active_streams == 0  # 代理断线收尾时停止了推流


def test_to_bgr():
    cv2 = pytest.importorskip("cv2")
    f = Frame(1, 1, "rgb8", bytes([255, 0, 0]), "raw")
    assert f.to_bgr()[0, 0].tolist() == [0, 0, 255]

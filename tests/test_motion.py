import time

import pytest

from cyberdog2 import CallbackError, MotionId, MotionRejected
from cyberdog2.constants import MOTION_ARRAY_SIZES


def test_do_sends_result_cmd(dog):
    dog.motion.do("sit")
    req = dog.backend.calls_to("motion_result_cmd")[-1]
    assert req["motion_id"] == 143
    assert req["cmd_source"] == 0
    for key, size in MOTION_ARRAY_SIZES.items():
        assert len(req[key]) == size


def test_do_accepts_enum_and_int(dog):
    dog.motion.do(MotionId.BALLET)
    dog.motion.do(152)
    ids = [r["motion_id"] for r in dog.backend.calls_to("motion_result_cmd")]
    assert ids == [151, 152]


def test_unknown_action_raises(dog):
    with pytest.raises(ValueError):
        dog.motion.do("fly")


def test_servo_id_in_do_raises(dog):
    with pytest.raises(ValueError):
        dog.motion.do(MotionId.WALK_SLOW)


def test_rejected_motion_raises(dog):
    dog.backend.set_response("motion_result_cmd", {"motion_id": 121, "result": False, "code": 3})
    with pytest.raises(MotionRejected) as ei:
        dog.motion.do("backflip")
    assert ei.value.code == 3


def test_rpy_out_of_range_raises(dog):
    with pytest.raises(ValueError):
        dog.motion.do("stand", rpy=[0.5, 0, 0])


def test_velocity_over_limit_raises_not_clamped(dog):
    with pytest.raises(ValueError):
        dog.motion.set_velocity(vx=1.2, gait="medium")  # 中速步态上限 1.0
    assert dog.backend.published_on("motion_servo_cmd") == []


def test_walk_heartbeat_and_end(dog):
    dog.motion.walk(vx=0.3, wz=0.2, duration=0.5)
    frames = dog.backend.published_on("motion_servo_cmd")
    data = [f for f in frames if f["cmd_type"] == 1]
    assert 7 <= len(data) <= 13  # 50ms 周期，0.5s 大约 10 帧
    assert data[0]["vel_des"] == [0.3, 0.0, 0.2]
    assert data[0]["motion_id"] == 308
    assert frames[-1]["cmd_type"] == 2
    assert frames[-1]["vel_des"] == [0.0, 0.0, 0.0]
    assert not dog.backend.servo_active


def test_heartbeat_interval_under_200ms(dog):
    dog.motion.set_velocity(0.2)
    time.sleep(0.6)
    dog.motion.stop()
    times = [t for (t, topic, d) in dog.backend.published if topic == "motion_servo_cmd"]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert max(gaps) < 0.2


def test_callback_exception_stops_motion_and_crashes_next_call(dog):
    def bad(ev):
        raise RuntimeError("boom")

    dog.motion.set_velocity(0.2)
    dog.sensors.on_touch(bad)
    dog.backend.emit("touch_status", {"touch_state": 1, "timestamp": 0})
    assert not dog.backend.servo_active
    assert dog.backend.published_on("motion_servo_cmd")[-1]["cmd_type"] == 2
    with pytest.raises(CallbackError) as ei:
        dog.motion.stand()
    assert isinstance(ei.value.__cause__, RuntimeError)


def test_close_stops_servo(dog):
    dog.motion.set_velocity(0.2)
    dog.close()
    assert dog.backend.published_on("motion_servo_cmd")[-1]["cmd_type"] == 2

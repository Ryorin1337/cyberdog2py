import time

import pytest

from cyberdog2 import CommandFailed, SensorTimeout
from cyberdog2.constants import SensorId, SensorOperation

from conftest import RemotePair

START = int(SensorOperation.START)
STOP = int(SensorOperation.STOP)


def silence(dog, cache, topic):
    """让这个传感器没有数据，并缩短探测时间。"""
    dog.backend.latched.pop(topic, None)
    cache._probe = 0.05


def ops(dog):
    return [(c["sensor_id"], c["operation"]) for c in dog.backend.calls_to("sensor_operation")]


def _wait_until(pred, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


def test_sensor_with_data_is_not_started(dog):
    dog.sensors.ultrasonic.get()
    assert ops(dog) == []


def test_silent_sensor_is_started_once(dog):
    silence(dog, dog.sensors.ultrasonic, "ultrasonic_payload")
    for _ in range(2):
        with pytest.raises(SensorTimeout):
            dog.sensors.ultrasonic.get(timeout=0.05)
    assert ops(dog) == [(int(SensorId.ULTRASONIC), START)]


def test_data_after_start_is_returned(dog):
    silence(dog, dog.sensors.scan, "scan")
    scan = {"ranges": [1.0], "angle_min": 0.0, "angle_increment": 0.1, "range_min": 0.1, "range_max": 10.0}

    def turn_on(request):
        dog.backend.emit("scan", scan)
        return {"success": True, "code": 0}

    dog.backend.set_handler("sensor_operation", turn_on)
    assert dog.sensors.scan.get(timeout=1.0)["ranges"] == [1.0]
    assert ops(dog) == [(int(SensorId.LIDAR), START)]


@pytest.mark.parametrize("attr,topic,sensor_id", [("scan", "scan", SensorId.LIDAR),
                                                  ("ultrasonic", "ultrasonic_payload", SensorId.ULTRASONIC),
                                                  ("head_tof", "head_tof_payload", SensorId.TOF),
                                                  ("rear_tof", "rear_tof_payload", SensorId.TOF),
                                                  ("gps", "gps_payload", SensorId.GPS)])
def test_every_sensor_autostarts(dog, attr, topic, sensor_id):
    cache = getattr(dog.sensors, attr)
    silence(dog, cache, topic)
    with pytest.raises(SensorTimeout):
        cache.get(timeout=0.05)
    assert ops(dog) == [(int(sensor_id), START)]


def test_head_and_rear_tof_start_once(dog):
    silence(dog, dog.sensors.head_tof, "head_tof_payload")
    silence(dog, dog.sensors.rear_tof, "rear_tof_payload")
    with pytest.raises(SensorTimeout):
        dog.sensors.tof(timeout=0.05)
    with pytest.raises(SensorTimeout):
        dog.sensors.rear_tof.get(timeout=0.05)
    assert ops(dog) == [(int(SensorId.TOF), START)]


def test_on_callback_also_starts(dog):
    silence(dog, dog.sensors.ultrasonic, "ultrasonic_payload")
    got = []
    dog.sensors.ultrasonic.on(got.append)
    assert ops(dog) == [(int(SensorId.ULTRASONIC), START)]
    dog.backend.emit("ultrasonic_payload", {"range": 0.5})
    assert got == [{"range": 0.5}]


def test_start_failure_raises(dog):
    silence(dog, dog.sensors.ultrasonic, "ultrasonic_payload")
    dog.backend.set_response("sensor_operation", {"success": False, "code": 7})
    with pytest.raises(CommandFailed, match="code=7"):
        dog.sensors.ultrasonic.get(timeout=0.05)


def test_close_stops_only_what_we_started(dog):
    dog.sensors.head_tof.get()  # 已经有数据，不是库打开的
    silence(dog, dog.sensors.ultrasonic, "ultrasonic_payload")
    with pytest.raises(SensorTimeout):
        dog.sensors.ultrasonic.get(timeout=0.05)
    dog.close()
    assert ops(dog) == [(int(SensorId.ULTRASONIC), START), (int(SensorId.ULTRASONIC), STOP)]


def test_close_without_autostart_touches_nothing(dog):
    dog.sensors.ultrasonic.get()
    dog.close()
    assert ops(dog) == []


def test_remote_autostart_and_stop_on_disconnect():
    pair = RemotePair()
    pair.mock.latched.pop("ultrasonic_payload")
    pair.dog.sensors.ultrasonic._probe = 0.05
    with pytest.raises(SensorTimeout):
        pair.dog.sensors.ultrasonic.get(timeout=0.05)
    assert [c["operation"] for c in pair.mock.calls_to("sensor_operation")] == [START]
    pair.sock.close()  # 模拟电脑崩溃或断网
    assert _wait_until(lambda: [c["operation"] for c in pair.mock.calls_to("sensor_operation")] == [START, STOP])
    pair.thread.join(3)


def camera_cmds(dog):
    return [c["command"] for c in dog.backend.calls_to("camera_service")]


def test_get_frame_uses_running_camera_without_starting(dog):
    dog.sensors.camera.get_frame()
    assert camera_cmds(dog) == []
    dog.close()
    assert camera_cmds(dog) == []


def test_get_frame_autostarts_camera_and_stops_on_close(dog):
    from cyberdog2.constants import CameraCmd

    image, dog.backend.image = dog.backend.image, None

    def turn_on(request):
        dog.backend.image = image
        return {"result": 0, "msg": "", "code": 0}

    dog.backend.set_handler("camera_service", turn_on)
    assert dog.sensors.camera.get_frame() is image
    assert camera_cmds(dog) == [int(CameraCmd.START_IMAGE_PUBLISH)]
    dog.sensors.camera.get_frame()
    assert len(camera_cmds(dog)) == 1
    dog.close()
    assert camera_cmds(dog) == [int(CameraCmd.START_IMAGE_PUBLISH), int(CameraCmd.STOP_IMAGE_PUBLISH)]


def test_manual_camera_start_is_not_stopped_on_close(dog):
    from cyberdog2.constants import CameraCmd

    dog.sensors.camera.start()
    dog.sensors.camera.get_frame()
    dog.close()
    assert camera_cmds(dog) == [int(CameraCmd.START_IMAGE_PUBLISH)]

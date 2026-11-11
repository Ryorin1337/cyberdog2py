import math

import pytest

from cyberdog2 import CommandFailed, GoalRejected, SensorTimeout
from cyberdog2.backends.ros import pick_namespace
from cyberdog2.errors import NamespaceNotFound


# ---------------------------------------------------------------- LED
def test_led_predefined(dog):
    dog.led.set("head", "red", "breath")
    req = dog.backend.calls_to("led_execute")[-1]
    assert (req["target"], req["mode"], req["effect"]) == (1, 0x01, 0xA4)


def test_led_rgb(dog):
    dog.led.set("tail", (10, 20, 30), "blink")
    req = dog.backend.calls_to("led_execute")[-1]
    assert (req["target"], req["mode"], req["effect"]) == (2, 0x02, 0x02)
    assert (req["r_value"], req["g_value"], req["b_value"]) == (10, 20, 30)


def test_led_invalid_combo_raises(dog):
    with pytest.raises(ValueError):
        dog.led.set("head", "green", "on")
    with pytest.raises(ValueError):
        dog.led.set("head", (300, 0, 0), "on")


def test_led_error_code_raises(dog):
    dog.backend.set_response("led_execute", {"code": 1125})
    with pytest.raises(CommandFailed, match="LOW_PRIORITY"):
        dog.led.set("head", "blue", "on")


# ---------------------------------------------------------------- 语音
def test_say(dog):
    dog.audio.say("你好")
    req = dog.backend.calls_to("speech_text_play")[-1]
    assert req["is_online"] is True and req["text"] == "你好"


def test_say_failure_raises(dog):
    dog.backend.set_response("speech_text_play", {"status": 1, "code": 0})
    with pytest.raises(CommandFailed):
        dog.audio.say("x")


def test_volume(dog):
    assert dog.audio.volume == 50
    dog.audio.volume = 70
    assert dog.backend.calls_to("audio_volume_set")[-1] == {"volume": 70}
    with pytest.raises(ValueError):
        dog.audio.volume = 101


# ---------------------------------------------------------------- 传感器
def test_battery(dog):
    assert dog.sensors.battery.soc == 80
    assert dog.sensors.battery.voltage == 25.0


def test_touch_kinds(dog):
    got = []
    dog.sensors.on_touch(got.append)
    for s in (1, 3, 7, 9):
        dog.backend.emit("touch_status", {"touch_state": s, "timestamp": 0})
    assert [e.kind for e in got] == ["single", "double", "long", None]


def test_sensor_timeout(dog):
    dog.backend.latched.pop("gps_payload")
    with pytest.raises(SensorTimeout):
        dog.sensors.gps.get(timeout=0.1)


def test_tof_matrix(dog):
    tof = dog.sensors.tof()
    assert len(tof["left_head"]) == 8 and len(tof["left_head"][0]) == 8


def test_attitude_level(dog):
    assert dog.sensors.attitude() == {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}


@pytest.mark.parametrize("axis,euler", [("x", "roll"), ("y", "pitch"), ("z", "yaw")])
def test_attitude_single_axis(dog, axis, euler):
    angle = math.radians(30)
    q = {"x": 0.0, "y": 0.0, "z": 0.0, "w": math.cos(angle / 2)}
    q[axis] = math.sin(angle / 2)
    dog.backend.latched["odom_out"] = _odom(q, {"x": 0.0, "y": 0.0, "z": 0.0})
    result = dog.sensors.attitude()
    for name, value in result.items():
        assert value == pytest.approx(angle if name == euler else 0.0, abs=1e-9)


@pytest.mark.parametrize("bad", [0.0, float("nan"), float("inf")])
def test_attitude_invalid_quaternion(dog, bad):
    q = {"x": bad, "y": 0.0, "z": 0.0, "w": 0.0}
    dog.backend.latched["odom_out"] = _odom(q, {"x": 0.0, "y": 0.0, "z": 0.0})
    with pytest.raises(ValueError):
        dog.sensors.attitude()


def test_attitude_stale_raises(dog):
    dog.sensors.attitude()
    with pytest.raises(SensorTimeout):
        dog.sensors.attitude(timeout=0.1, max_age=0.0)


def _odom(q, angular):
    return {"pose": {"pose": {"position": {"x": 1.0, "y": 2.0, "z": 0.3}, "orientation": q}},
            "twist": {"twist": {"linear": {"x": 0.5, "y": 0.0, "z": 0.0}, "angular": angular}}}


def test_odom_is_the_raw_message(dog):
    q = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    dog.backend.latched["odom_out"] = _odom(q, {"x": 0.0, "y": 0.0, "z": 0.1})
    pose = dog.sensors.odom.get()["pose"]["pose"]
    assert pose["position"] == {"x": 1.0, "y": 2.0, "z": 0.3}


def test_imu_is_computed_from_odom(dog):
    q = {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.9}
    angular = {"x": 0.4, "y": 0.5, "z": 0.6}
    dog.backend.latched["odom_out"] = _odom(q, angular)
    assert dog.sensors.imu.get() == {"orientation": q, "angular_velocity": angular}
    assert dog.sensors.imu.age is not None


def test_imu_shares_odom_subscription(dog):
    dog.sensors.imu.get()
    dog.sensors.odom.get()
    assert sum(1 for topic, _ in dog.backend._subs.values() if topic == "odom_out") == 1


def test_imu_callback_gets_computed_message(dog):
    got = []
    dog.sensors.imu.on(got.append)
    q = {"x": 0.0, "y": 0.0, "z": 1.0, "w": 0.0}
    angular = {"x": 0.0, "y": 0.0, "z": 0.2}
    dog.backend.emit("odom_out", _odom(q, angular))
    assert got[-1] == {"orientation": q, "angular_velocity": angular}


def test_camera_failure_raises(dog):
    dog.backend.set_response("camera_service", {"result": 4, "msg": "busy", "code": 0})
    with pytest.raises(CommandFailed):
        dog.sensors.camera.start()


def test_unregistered_service_raises(dog):
    del dog.backend.services["led_execute"]
    from cyberdog2 import ServiceUnavailable

    with pytest.raises(ServiceUnavailable):
        dog.led.set("head", "red")


# ---------------------------------------------------------------- 视觉
def test_vision_enable(dog):
    dog.vision.enable("face", "gesture")
    req = dog.backend.calls_to("algo_manager")[-1]
    assert req["algo_enable"] == [{"algo_module": 0}, {"algo_module": 2}]
    with pytest.raises(ValueError):
        dog.vision.enable("xray")


def test_gesture_event(dog):
    got = []
    dog.vision.on_gesture(got.append)
    dog.backend.emit("gesture_action_msg", {"id": 8})
    assert got[0].name == "thumb_up"


# ---------------------------------------------------------------- 导航
def test_follow_person_goal(dog):
    task = dog.nav.follow_person(distance=1.5, position="left")
    goal = dog.backend.goals[task.goal_id]["goal"]
    assert goal == {"nav_type": 13, "object_tracking": False, "keep_distance": 1.5, "relative_pos": 202}
    assert task.wait()["result"] == 0


def test_nav_failed_result_raises(dog):
    task = dog.nav.go_to(1.0, 2.0)
    dog.backend.goal_results[task.goal_id] = {"result": 3}
    with pytest.raises(GoalRejected, match="FAILED"):
        task.wait()


def test_nav_stop(dog):
    dog.nav.stop_mapping("home")
    assert dog.backend.calls_to("stop_algo_task")[-1] == {"task_id": 5, "map_name": "home"}


# ---------------------------------------------------------------- 命名空间
def test_pick_namespace():
    nodes = [("motion_manager", "/mi_desktop_48_b0_2d_7b_05_1d"), ("x", "/mi_desktop_48_b0_2d_7b_05_1d/sub"),
             ("cli", "/")]
    assert pick_namespace(nodes) == "/mi_desktop_48_b0_2d_7b_05_1d"


def test_pick_namespace_none_or_ambiguous():
    with pytest.raises(NamespaceNotFound):
        pick_namespace([("a", "/")])
    with pytest.raises(NamespaceNotFound):
        pick_namespace([("a", "/mi_a"), ("b", "/mi_b")])

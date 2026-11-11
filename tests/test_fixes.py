"""针对 docs/API.md 12.2 节所列问题的回归测试。"""
import math
import subprocess
import threading
import time

import pytest

from cyberdog2 import CallbackError, CommandFailed, CyberDogError, SensorTimeout
from cyberdog2.deploy import build_agent_command, env_inherit_loop
from cyberdog2.protocol import dumps, loads

from conftest import RemotePair


def _wait_until(pred, timeout=2.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


# 1. NaN / Inf
def test_protocol_roundtrips_non_finite_floats():
    out = loads(dumps({"a": float("inf"), "b": [float("-inf"), 1.5], "c": float("nan")}))
    assert out["a"] == math.inf and out["b"] == [-math.inf, 1.5] and math.isnan(out["c"])


def test_remote_sensor_with_inf_does_not_kill_agent(remote):
    remote.mock.latched["ultrasonic_payload"] = {"range": float("inf")}
    assert remote.dog.sensors.ultrasonic_distance() == math.inf
    remote.mock.set_response("motion_result_cmd", {"motion_id": 111, "result": True, "code": 0, "x": float("nan")})
    remote.dog.motion.stand()  # 响应里带 NaN 也能正常返回
    assert remote.backend.fatal_error is None


def test_unencodable_reply_fails_fast(remote):
    remote.mock.set_response("led_execute", {"code": 0, "bad": object()})
    t0 = time.time()
    with pytest.raises(Exception):
        remote.dog.led.set("head", "red")
    assert time.time() - t0 < 2.0  # 立即报错，而不是等到 timeout + 5s


# 2. 断线后停止导航任务
def test_disconnect_stops_nav_tasks():
    pair = RemotePair()
    pair.dog.nav.follow_person()
    pair.sock.close()
    assert _wait_until(lambda: pair.mock.calls_to("stop_algo_task"), 3.0)
    assert pair.mock.calls_to("stop_algo_task")[-1]["task_id"] == 13
    assert _wait_until(lambda: all(g["cancelled"] for g in pair.mock.goals.values()), 3.0)
    pair.thread.join(3)


def test_close_stops_nav_tasks(dog):
    dog.nav.follow_uwb()
    dog.close()
    assert dog.backend.calls_to("stop_algo_task")[-1]["task_id"] == 11


def test_close_during_mapping_uses_mapping_stop(dog):
    dog.nav.start_mapping()
    dog.close()
    assert dog.backend.calls_to("stop_algo_task")[-1] == {"task_id": 5, "map_name": ""}
    assert dog.backend.latched["algo_task_status"]["task_status"] == 101


def test_disconnect_during_mapping_uses_mapping_stop():
    pair = RemotePair()
    pair.dog.nav.start_mapping()
    pair.sock.close()
    assert _wait_until(lambda: pair.mock.calls_to("stop_algo_task"), 3.0)
    assert pair.mock.calls_to("stop_algo_task")[-1]["task_id"] == 5
    assert _wait_until(lambda: pair.mock.latched["algo_task_status"]["task_status"] == 101, 3.0)
    pair.thread.join(3)


def test_close_sends_no_stop_when_task_already_finished(dog):
    dog.nav.follow_uwb()
    dog.backend._set_task_status(101)
    dog.close()
    assert dog.backend.calls_to("stop_algo_task") == []


def test_release_returns_status_and_clears_started(dog):
    dog.nav.start_localization("room")
    assert dog.nav.release() == 7
    assert dog.backend.calls_to("stop_algo_task")[-1]["task_id"] == 7
    assert dog.nav.started is False


@pytest.mark.parametrize("status,task_id", [(5, 5), (15, 5), (7, 7), (17, 7), (1, 0), (21, 0), (8, 0), (18, 0),
                                             (9, 9), (11, 11), (13, 13), (3, 13)])
def test_release_uses_the_stop_that_dog_accepts(dog, status, task_id):
    dog.backend._set_task_status(status)
    assert dog.nav.release() == status
    assert dog.backend.calls_to("stop_algo_task")[-1]["task_id"] == task_id
    assert dog.backend.latched["algo_task_status"]["task_status"] == 101


def test_release_reports_status_it_cannot_stop(dog):
    from cyberdog2.errors import CyberDogError

    dog.backend._set_task_status(100)
    with pytest.raises(CyberDogError, match="100"):
        dog.nav.release()
    assert dog.backend.calls_to("stop_algo_task") == []


def test_release_failure_names_the_status(dog):
    from cyberdog2.errors import CommandFailed

    dog.backend._set_task_status(8)
    dog.backend.set_response("stop_algo_task", {"result": 1, "code": 0})
    with pytest.raises(CommandFailed, match="8"):
        dog.nav.release()


def test_release_waits_while_status_is_changing(dog):
    dog.backend._set_task_status(103)
    timer = threading.Timer(0.05, dog.backend.emit, ("algo_task_status", {"task_status": 101}))
    timer.start()
    assert dog.nav.release() == 101
    timer.join()
    assert dog.backend.calls_to("stop_algo_task") == []


def test_release_stops_all_then_raises_when_status_missing(dog, monkeypatch):
    from cyberdog2 import navigation
    from cyberdog2.errors import SensorTimeout

    monkeypatch.setattr(navigation, "RELEASE_STATUS_TIMEOUT", 0.05)
    dog.nav.follow_uwb()
    dog.backend.set_response("stop_algo_task", {"result": 0, "code": 0})
    del dog.backend.latched["algo_task_status"]
    with pytest.raises(SensorTimeout):
        dog.nav.release()
    assert dog.backend.calls_to("stop_algo_task")[-1]["task_id"] == 0
    dog.nav.started = False


def test_close_without_nav_does_not_call_stop(dog):
    dog.close()
    assert dog.backend.calls_to("stop_algo_task") == []


# 3. 未知步态
def test_unknown_gait_is_value_error(dog):
    with pytest.raises(ValueError):
        dog.motion.set_velocity(0.1, gait="moonwalk")


# 4. 回调崩溃后急停仍能发出
def test_estop_after_callback_crash(dog):
    dog.sensors.on_touch(lambda ev: 1 / 0)
    dog.backend.emit("touch_status", {"touch_state": 1, "timestamp": 0})
    with pytest.raises(CallbackError):
        dog.motion.stand()
    dog.motion.estop()
    assert dog.backend.calls_to("motion_result_cmd")[-1]["motion_id"] == 0


# 5~7. 校验
def test_nav_task_checks_fatal(dog):
    task = dog.nav.follow_person()
    dog.sensors.on_touch(lambda ev: 1 / 0)
    dog.backend.emit("touch_status", {"touch_state": 1, "timestamp": 0})
    with pytest.raises(CallbackError):
        task.wait()


def test_follow_uwb_distance(dog):
    with pytest.raises(ValueError):
        dog.nav.follow_uwb(distance=0)


def test_cancel_face_checks_result(dog):
    dog.backend.set_response("cyberdog_face_recognition_srv", {"result": 5901, "code": 0})
    with pytest.raises(CommandFailed):
        dog.vision.cancel_face()


def test_vision_requires_algos(dog):
    with pytest.raises(ValueError):
        dog.vision.enable()


def test_led_no_silent_ignore(dog):
    with pytest.raises(ValueError):
        dog.led.set("mini", "red", "three_circular")
    with pytest.raises(ValueError):
        dog.led.set("head", "off", "breath")
    dog.led.set("head", "off")
    dog.led.set("mini", None, "three_circular")


# 8. 传感器新鲜度
def test_topic_cache_max_age(dog):
    cache = dog.sensors.battery
    assert cache.get()["batt_soc"] == 80
    time.sleep(0.15)
    with pytest.raises(SensorTimeout, match="max_age"):
        cache.get(timeout=0.1, max_age=0.05)
    dog.backend.emit("bms_status", {"batt_soc": 70, "batt_volt": 24000})
    assert cache.get(timeout=0.1, max_age=0.05)["batt_soc"] == 70
    assert cache.age is not None and cache.age < 1.0


# 9. 远程导航反馈
def test_remote_feedback_delivered_and_released(remote):
    got = []
    task = remote.dog.nav.follow_person(feedback=got.append)
    remote.mock.goals[task.goal_id]["feedback_cb"]({"feedback_code": 503, "feedback_msg": "following"})
    assert _wait_until(lambda: got)
    assert got[0]["feedback_code"] == 503
    task.wait()
    assert task.goal_id not in remote.backend._feedback


# 13. 协议版本
def test_protocol_version_mismatch(monkeypatch):
    import cyberdog2.backends.remote as remote_mod

    monkeypatch.setattr(remote_mod, "PROTOCOL_VERSION", 999)
    with pytest.raises(CyberDogError, match="协议版本"):
        RemotePair()


# 15~16. 部署脚本
def test_env_inherit_handles_quotes(tmp_path):
    """在本机 bash 上实际执行继承环境变量那段脚本：值里带单引号、空格也能原样导出。"""
    bash = "bash"
    try:
        subprocess.run([bash, "-c", "true"], check=True, capture_output=True)
    except (OSError, subprocess.CalledProcessError):
        pytest.skip("没有可用的 bash")
    environ = tmp_path / "environ"
    environ.write_bytes(b"ROS_DOMAIN_ID=42\0CYCLONEDDS_URI=<a b='1'/>\0OTHER=x\0")
    loop = env_inherit_loop(environ.as_posix())
    assert "read -r -d" in build_agent_command("/x", None, [], "motion_manager", {}, 1.0)
    out = subprocess.run([bash, "-c", loop + '; printf "%s|%s|%s" "$ROS_DOMAIN_ID" "$CYCLONEDDS_URI" "$OTHER"'],
                         capture_output=True, check=True).stdout.decode("utf-8")
    assert out == "42|<a b='1'/>|"


def test_agent_command_python_and_real_namespace():
    from cyberdog2.backends.ros import pick_namespace

    ns = "/mi_desktop_48_b0_2d_7a_fb_a9"  # 实测机器的命名空间
    assert pick_namespace([("motion_manager", ns), ("cli", "/")]) == ns
    cmd = build_agent_command("/x", ns, [], None, {}, 1.0, python="python3.8")
    assert "exec python3.8 -u -m cyberdog2.agent" in cmd and ns in cmd
    assert "exec python3 -u" in build_agent_command("/x", None, [], None, {}, 1.0)


def test_agent_command_timeouts():
    cmd = build_agent_command("/x", None, [], None, {}, 1.0, discovery_timeout=12, accept_timeout=45)
    assert "--discovery-timeout 12.0" in cmd and "--accept-timeout 45.0" in cmd


# 真机上 touch_status 在代理就绪后才被发现，doctor 不能只查一次
def _doctor_names(ns):
    from cyberdog2.constants import REQUIRED_SERVICES, REQUIRED_TOPICS

    return {"services": [ns + "/" + n for n in REQUIRED_SERVICES],
            "topics": [ns + "/" + n for n in REQUIRED_TOPICS]}


def test_doctor_waits_for_late_discovery():
    from cyberdog2.doctor import wait_for_interfaces

    ns = "/mi_x"
    full = _doctor_names(ns)
    partial = {"services": full["services"], "topics": full["topics"][:-1]}
    replies = [partial, partial, full]
    names, missing = wait_for_interfaces(lambda: replies.pop(0), ns, timeout=5.0, poll=0.01)
    assert missing == [] and names is full and replies == []


def test_doctor_reports_missing_after_timeout():
    from cyberdog2.constants import REQUIRED_TOPICS
    from cyberdog2.doctor import wait_for_interfaces

    ns = "/mi_x"
    full = _doctor_names(ns)
    partial = {"services": full["services"], "topics": full["topics"][:-1]}
    _, missing = wait_for_interfaces(lambda: partial, ns, timeout=0.05, poll=0.01)
    assert missing == [ns + "/" + REQUIRED_TOPICS[-1]]


# 真机的 python3 没有 cv2，狗端压缩只能依赖 Pillow
@pytest.mark.parametrize("encoding,pixel", [("rgb8", b"\xff\x00\x00"), ("bgr8", b"\x00\x00\xff"),
                                            ("rgba8", b"\xff\x00\x00\xff"), ("bgra8", b"\x00\x00\xff\xff")])
def test_encode_jpeg_without_cv2_keeps_colors(monkeypatch, encoding, pixel):
    import io
    import sys

    from PIL import Image

    from cyberdog2.frame import Frame, encode_jpeg

    monkeypatch.setitem(sys.modules, "cv2", None)  # 让 import cv2 必然失败
    out = encode_jpeg(Frame(16, 16, encoding, pixel * 256, stamp=1.5), quality=90)
    assert (out.format, out.width, out.height, out.stamp) == ("jpeg", 16, 16, 1.5)
    r, g, b = Image.open(io.BytesIO(out.data)).convert("RGB").getpixel((8, 8))
    assert r > 240 and g < 15 and b < 15


def test_encode_jpeg_mono_and_bad_encoding():
    from cyberdog2.frame import Frame, encode_jpeg

    assert encode_jpeg(Frame(8, 8, "mono8", b"\x80" * 64)).data[:2] == b"\xff\xd8"
    with pytest.raises(ValueError, match="nv12"):
        encode_jpeg(Frame(8, 8, "nv12", b"\x00" * 96))
    with pytest.raises(ValueError):
        encode_jpeg(Frame(8, 8, "rgb8", b"\x00" * 10))  # 数据长度不对时直接报错


# 18. 伺服死人开关
def test_servo_expires_without_refresh(dog):
    dog.motion.set_velocity(0.2, timeout=0.2)
    frames = lambda: dog.backend.published_on("motion_servo_cmd")  # noqa: E731
    assert _wait_until(lambda: frames() and frames()[-1]["cmd_type"] == 2, 1.0)
    assert len(frames()) >= 3  # 到期前一直在正常下发
    assert not dog.backend.servo_active
    dog.motion.stop()  # 已经自动停止，再 stop 不报错


def test_servo_refresh_keeps_alive(dog):
    for _ in range(6):
        dog.motion.set_velocity(0.2, timeout=0.2)
        time.sleep(0.1)
    assert dog.backend.servo_active
    dog.motion.stop()


def test_walk_sets_deadman(remote):
    remote.dog.motion.walk(0.2, duration=0.2)
    assert remote.mock.published_on("motion_servo_cmd")[-1]["cmd_type"] == 2


# 20. 关闭后读取视频流
def test_read_closed_stream(dog):
    s = dog.sensors.camera.stream(fps=10)
    s.close()
    with pytest.raises(CyberDogError, match="已关闭"):
        s.read()

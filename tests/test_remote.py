"""代理与 RemoteBackend 在本机回环上的端到端测试（跳过 SSH 这一层）。"""
import ast
import os
import time

import pytest

from cyberdog2 import CallbackError, ConnectionLost, MotionRejected, ServiceUnavailable
from cyberdog2.deploy import build_agent_command, package_files, package_hash

from conftest import RemotePair


def _wait_until(pred, timeout=2.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


def test_namespace_and_motion(remote):
    assert remote.dog.namespace == "/mi_mock"
    remote.dog.motion.sit()
    assert remote.mock.calls_to("motion_result_cmd")[-1]["motion_id"] == 143


def test_remote_error_type_preserved(remote):
    remote.mock.set_response("motion_result_cmd", {"motion_id": 121, "result": False, "code": 7})
    with pytest.raises(MotionRejected) as ei:
        remote.dog.motion.do("backflip")
    assert ei.value.code == 7
    del remote.mock.services["led_execute"]
    with pytest.raises(ServiceUnavailable):
        remote.dog.led.set("head", "red")


def test_latched_and_events(remote):
    assert remote.dog.sensors.battery.soc == 80
    got = []
    remote.dog.sensors.on_touch(got.append)
    remote.mock.emit("touch_status", {"touch_state": 3, "timestamp": 0})
    assert _wait_until(lambda: got)
    assert got[0].kind == "double"


def test_servo_runs_on_agent_side(remote):
    remote.dog.motion.walk(vx=0.2, duration=0.4)
    frames = remote.mock.published_on("motion_servo_cmd")
    assert len([f for f in frames if f["cmd_type"] == 1]) >= 5
    assert frames[-1]["cmd_type"] == 2


def test_bytes_roundtrip(remote):
    remote.mock.image.data = bytes(range(36))
    frame = remote.dog.sensors.camera.get_frame(jpeg=False)
    assert frame.data == bytes(range(36)) and (frame.width, frame.height) == (4, 3)


def test_remote_callback_exception(remote):
    remote.dog.motion.set_velocity(0.2)
    remote.dog.sensors.on_touch(lambda ev: 1 / 0)
    remote.mock.emit("touch_status", {"touch_state": 1, "timestamp": 0})
    assert _wait_until(lambda: remote.mock.published_on("motion_servo_cmd")[-1:] and remote.mock.published_on("motion_servo_cmd")[-1]["cmd_type"] == 2)
    with pytest.raises(CallbackError):
        remote.dog.motion.stand()


def test_disconnect_triggers_servo_end():
    pair = RemotePair()
    pair.dog.motion.set_velocity(0.2)
    assert _wait_until(lambda: pair.mock.servo_active)
    pair.sock.close()  # 模拟电脑断网
    assert _wait_until(lambda: pair.mock.published_on("motion_servo_cmd")[-1:] and pair.mock.published_on("motion_servo_cmd")[-1]["cmd_type"] == 2, 3.0)
    assert not pair.mock.servo_active
    with pytest.raises(ConnectionLost):
        pair.dog.motion.stand()
    pair.thread.join(3)


def test_watchdog_when_pings_stop():
    pair = RemotePair(watchdog=0.5, ping_interval=10.0)  # 客户端几乎不发心跳，模拟网络卡死
    pair.dog.motion.set_velocity(0.2)
    assert _wait_until(lambda: pair.server.stopped_by_watchdog, 3.0)
    assert _wait_until(lambda: pair.mock.published_on("motion_servo_cmd")[-1:] and pair.mock.published_on("motion_servo_cmd")[-1]["cmd_type"] == 2)
    assert not pair.mock.servo_active
    assert _wait_until(lambda: pair.backend.fatal_error is not None)
    with pytest.raises(ConnectionLost):
        pair.dog.motion.stand()
    pair.thread.join(3)


def test_agent_command_script():
    cmd = build_agent_command("/home/mi/.cyberdog2/abc", None, ["/opt/ros2/galactic/setup.bash"],
                              "motion_manager", {"ROS_DOMAIN_ID": "42"}, 1.0)
    assert "[m]otion_manager" in cmd
    assert "ROS_DOMAIN_ID=42" in cmd
    assert "python3 -u -m cyberdog2.agent" in cmd
    with pytest.raises(ValueError):
        build_agent_command("/x", None, [], None, {"BAD NAME": "1"}, 1.0)


def test_package_hash_stable():
    files = package_files()
    names = [rel for rel, _ in files]
    assert "cyberdog2/agent/__main__.py" in names
    assert package_hash(files) == package_hash(package_files())


def test_dog_side_code_is_python36_syntax():
    # 实测机器为 Ubuntu 18.04（系统 python3 = 3.6），狗端代码必须能在 3.6 上运行
    for rel, data in package_files():
        src = data.decode("utf-8")
        ast.parse(src, filename=rel, feature_version=(3, 6))
        assert "from __future__ import annotations" not in src, rel  # 3.7+ 才支持


def _annotation_names(node):
    """注解里引用的名字（字符串形式的前向引用不会在定义时求值，跳过）。"""
    if node is None or (isinstance(node, ast.Constant) and isinstance(node.value, str)):
        return set()
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


def test_annotations_resolvable_at_def_time_on_py36():
    """3.6 在 def 执行时就求值注解：注解里不能出现尚未绑定的名字（未加引号的前向引用）。"""
    import builtins

    problems = []
    for rel, data in package_files():
        tree = ast.parse(data.decode("utf-8"))
        bound = []  # (名字, 绑定完成的行号)
        for stmt in tree.body:
            end = getattr(stmt, "end_lineno", stmt.lineno)
            if isinstance(stmt, (ast.Import, ast.ImportFrom)):
                bound += [((a.asname or a.name).split(".")[0], end) for a in stmt.names]
            elif isinstance(stmt, (ast.ClassDef, ast.FunctionDef)):
                bound.append((stmt.name, end))
            elif isinstance(stmt, ast.Assign):
                bound += [(t.id, end) for t in stmt.targets if isinstance(t, ast.Name)]
        for fn in ast.walk(tree):
            if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            args = fn.args.args + fn.args.kwonlyargs + fn.args.posonlyargs
            args += [a for a in (fn.args.vararg, fn.args.kwarg) if a]
            names = set().union(*[_annotation_names(a.annotation) for a in args], _annotation_names(fn.returns))
            for name in names:
                ok = hasattr(builtins, name) or any(n == name and end < fn.lineno for n, end in bound)
                if not ok:
                    problems.append("%s:%d %s() 注解中的 %s 在定义时未绑定" % (rel, fn.lineno, fn.name, name))
    assert not problems, "\n".join(problems)

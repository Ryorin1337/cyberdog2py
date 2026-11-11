"""房间建模应用的测试：纯雷达沿墙建图、定位、路径规划、SQLite、接口失败路径、完整流程。"""

import argparse
import json
import math
import threading
import time
import urllib.error
import urllib.request

import numpy as np
import pytest

from cyberdog2 import CyberDog
from roommapper import planner, slam
from roommapper.server import create_server
from roommapper.session import FAILED, IDLE, Session
from roommapper.sim import RAYS, SimBackend, SimRoom
from roommapper.store import NotFound, Store
from roommapper.wall_follow import Pose, ScanView, StuckError, WallFollower, wrap_angle

START = (1.0, 1.5, 0.0)


def truth_in_map(room):
    """地图坐标系以狗的出发点为原点。START 的朝向是 0，所以只差一个平移。"""
    return Pose(room.x - START[0], room.y - START[1], room.yaw - START[2])


def build_map(room, dt=0.1, limit=500.0):
    """不给算法任何真实位姿，只给 scan 和发出的速度指令。"""
    mapper, follower = slam.Mapper(), WallFollower()
    t, vx, wz = 0.0, 0.0, 0.0
    while t < limit:
        scan = room.scan_msg()
        pose = mapper.update(scan, vx, wz, dt if t else 0.0)
        cmd = follower.step(scan, pose, t)
        if cmd.done:
            return mapper, follower
        vx, wz = cmd.vx, cmd.wz
        room.advance(vx, wz, dt)
        t += dt
    raise AssertionError("%.0f 秒内没走完一圈，phase=%s traveled=%.1f" % (limit, follower.phase, follower.traveled))


@pytest.fixture(scope="module")
def mapped():
    room = SimRoom(start=START)
    mapper, follower = build_map(room)
    return room, mapper, follower


# ------------------------------------------------------------ 建图
def test_mapping_closes_loop_without_collision_or_drift(mapped):
    room, mapper, follower = mapped
    assert room.collisions == 0
    assert follower.traveled > 15.0
    truth = truth_in_map(room)
    assert math.hypot(mapper.pose.x - truth.x, mapper.pose.y - truth.y) < 0.25   # 走 24 m 的累计漂移
    assert abs(wrap_angle(mapper.pose.yaw - truth.yaw)) < math.radians(4)


def test_free_standing_obstacle_is_not_mistaken_for_the_room():
    room = SimRoom(start=(1.4, 2.7, 0.0))   # 正对柱子
    _, follower = build_map(room)
    assert room.collisions == 0
    assert follower.phase == "done" and follower.traveled > 15.0


def test_map_shows_known_walls(mapped):
    _, mapper, _ = mapped
    grid = mapper.export()
    cells = np.frombuffer(grid["cells"], dtype=np.int8).reshape((grid["height"], grid["width"]))
    # L 形房间在地图坐标下的左墙 x = -1.0，取中间一段检查
    col = int((-1.0 - grid["origin_x"]) / grid["resolution"])
    rows = [int((y - grid["origin_y"]) / grid["resolution"]) for y in np.arange(-0.5, 2.5, 0.25)]
    hits = [(cells[r, col - 3:col + 4] == 100).any() for r in rows]
    assert sum(hits) >= len(hits) - 1
    assert (cells == 0).sum() > 3000


def test_export_is_plannable_around_pillar(mapped):
    _, mapper, _ = mapped
    g = mapper.export()
    grid = planner.Grid(g["cells"], g["width"], g["height"], g["resolution"], g["origin_x"], g["origin_y"])
    # 柱子在房间坐标 x 2.0~2.6、y 2.4~3.0；从它左边到右边必须绕开
    points = planner.plan(grid, [(0.3, 1.2), (2.3, 1.2)])
    assert points[0] != points[-1] and len(points) >= 3


# ------------------------------------------------------------ 定位
def localizer_for(mapper):
    g = mapper.export()
    return slam.Localizer(g["cells"], g["width"], g["height"], g["resolution"], g["origin_x"], g["origin_y"])


def test_locate_corrects_a_rough_guess(mapped):
    room, mapper, _ = mapped
    expected = mapper.pose   # 定位结果是地图坐标，地图本身带建图时的漂移，所以和建图结束时的位姿比
    localizer = localizer_for(mapper)
    pose = localizer.locate(room.scan_msg(), Pose(expected.x + 0.3, expected.y - 0.25, expected.yaw + math.radians(15)))
    assert math.hypot(pose.x - expected.x, pose.y - expected.y) < 0.1
    assert abs(wrap_angle(pose.yaw - expected.yaw)) < math.radians(4)


def test_locate_rejects_wrong_position(mapped):
    room, mapper, _ = mapped
    truth = truth_in_map(room)
    with pytest.raises(slam.MatchLost):
        localizer_for(mapper).locate(room.scan_msg(), Pose(truth.x, truth.y, truth.yaw + math.pi))


def test_tracking_raises_after_sustained_mismatch(mapped):
    room, mapper, _ = mapped
    localizer = localizer_for(mapper)
    truth = truth_in_map(room)
    localizer.locate(room.scan_msg(), truth)
    far = Pose(truth.x, truth.y, truth.yaw + math.pi)
    localizer.pose = far
    with pytest.raises(slam.MatchLost):
        for _ in range(slam.MAX_LOW_QUALITY + 1):
            localizer.pose = far
            localizer.update(room.scan_msg(), 0.0, 0.0, 0.1)


def test_scan_rays_drop_zero_readings():
    scan = {"angle_min": 0.0, "angle_increment": 0.1, "range_min": 0.01, "range_max": 30.0,
            "ranges": [0.0, 1.0, float("nan"), 2.0, 11.0]}
    angles, ranges = slam.scan_rays(scan)
    assert ranges.tolist() == [1.0, 2.0] and angles.tolist() == pytest.approx([0.1, 0.3])
    with pytest.raises(ValueError):
        slam.scan_rays(dict(scan, ranges=[0.0] * 5))


# ------------------------------------------------------------ 沿墙
def fake_scan(ranges):
    n = len(ranges)
    return {"angle_min": -math.pi / 2, "angle_increment": math.pi / (n - 1), "range_min": 0.01, "range_max": 30.0,
            "ranges": list(ranges)}


def test_front_with_mostly_no_returns_counts_as_blocked():
    view = ScanView(fake_scan([3.0] * 20 + [0.0] * (RAYS - 20)))   # 只有视野最右边有回波
    assert view.coverage(0.0, math.radians(35)) == 0.0
    assert view.front_range() == 0.0
    open_view = ScanView(fake_scan([5.0] * RAYS))
    assert open_view.front_range() == 5.0


def test_seek_turns_when_nothing_in_view_then_gives_up():
    follower = WallFollower()
    scan = fake_scan([float("inf")] * RAYS)
    yaw = 0.0
    cmd = follower.step(scan, Pose(0.0, 0.0, yaw), 0.0)
    assert cmd.vx == 0.0 and cmd.wz > 0.0
    with pytest.raises(ValueError, match="转了一圈"):
        for t in range(40):
            yaw = wrap_angle(yaw + 0.3)
            follower.step(scan, Pose(0.0, 0.0, yaw), float(t))


def test_wall_follower_raises_when_stuck():
    room = SimRoom(start=START)
    follower = WallFollower()
    follower.phase = "follow"
    pose = Pose(room.x, room.y, room.yaw)
    follower.start = follower._last = follower._anchor = pose
    follower._trail = [pose]
    scan = room.scan_msg()
    follower.step(scan, pose, 1.0)
    with pytest.raises(StuckError):
        follower.step(scan, pose, 20.0)


# ------------------------------------------------------------ 规划
def make_grid(rows, inflate=0.0):
    """rows 从上到下书写：# 占据，. 空闲，? 未知。"""
    table = {"#": 100, ".": 0, "?": -1}
    flat = [table[c] for row in reversed(rows) for c in row]
    return planner.Grid(np.array(flat, dtype=np.int8).tobytes(), len(rows[0]), len(rows), 1.0, 0.0, 0.0, inflate=inflate)


def test_plan_routes_around_obstacle_and_keeps_waypoints():
    grid = make_grid(["...", ".#.", "..."])
    points = planner.plan(grid, [(0.5, 1.5), (2.5, 1.5)])
    assert points[0] == (0.5, 1.5) and points[-1] == (2.5, 1.5)
    assert len(points) >= 3
    for a, b in zip(points, points[1:]):
        assert grid.line_clear(grid.to_cell(*a), grid.to_cell(*b))


@pytest.mark.parametrize("point", [(1.5, 1.5), (0.5, 0.5), (99.0, 0.5)])
def test_plan_rejects_bad_waypoint(point):
    grid = make_grid(["...", ".#.", "?.."])
    with pytest.raises(planner.PlanError):
        planner.plan(grid, [point])


def test_plan_reports_unreachable():
    grid = make_grid(["..#..", "..#..", "..#.."])
    with pytest.raises(planner.PlanError, match="没有可通行"):
        planner.plan(grid, [(0.5, 1.5), (4.5, 1.5)])


def test_inflation_blocks_cells_near_walls():
    grid = make_grid([".....", ".....", "..#..", ".....", "....."], inflate=1.0)
    assert grid.is_blocked((1, 2)) and grid.is_blocked((2, 3))
    assert not grid.is_blocked((1, 1))


# ------------------------------------------------------------ 存储
def test_store_round_trip(tmp_path):
    store = Store(str(tmp_path / "t.db"))
    map_id = store.create_map("客厅")
    cells = np.array([-1, 0, 100, 0], dtype=np.int8).tobytes()
    store.save_grid(map_id, cells, 2, 2, 0.5, 1.0, 2.0)
    store.append_trajectory(map_id, [(0, 1.0, 0.1, 0.2, 0.0), (1, 2.0, 0.3, 0.4, 0.0)])
    store.finish_map(map_id, "complete")
    row = store.get_map(map_id)
    assert row["status"] == "complete" and row["origin_x"] == 1.0 and row["grid"] == cells
    assert store.get_trajectory(map_id) == [[0.1, 0.2], [0.3, 0.4]]
    path_id = store.save_path(map_id, "巡逻", [(0, 0), (1, 1)], [(0, 0), (1, 0), (1, 1)])
    assert store.get_path(path_id)["points"] == [[0, 0], [1, 0], [1, 1]]
    assert [p["id"] for p in store.list_paths(map_id)] == [path_id]
    store.close()


def test_store_rejects_bad_grid_and_missing_rows(tmp_path):
    store = Store(str(tmp_path / "t.db"))
    map_id = store.create_map("x")
    with pytest.raises(ValueError):
        store.save_grid(map_id, b"\x00", 2, 2, 0.5, 0.0, 0.0)
    with pytest.raises(NotFound):
        store.get_map(999)
    with pytest.raises(NotFound):
        store.get_path(999)
    store.close()


# ------------------------------------------------------------ 接口与完整流程
class App(object):
    def __init__(self, tmp_path, speed=4.0):
        self.backend = SimBackend(speed=speed, room=SimRoom(start=START))
        self.dog = CyberDog(self.backend)
        self.store = Store(str(tmp_path / "app.db"))
        self.session = Session(self.dog, self.store)
        self.server = create_server(self.session, self.store, "127.0.0.1", 0)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def request(self, path, body=None, headers=None):
        data = None if body is None else json.dumps(body).encode()
        hdrs = {"Content-Type": "application/json"} if body is not None else {}
        hdrs.update(headers or {})
        req = urllib.request.Request("http://127.0.0.1:%d%s" % (self.port, path), data=data, headers=hdrs)
        try:
            with urllib.request.urlopen(req) as res:
                return res.status, json.loads(res.read().decode())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read().decode())

    def wait_state(self, wanted, timeout=150.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            _, snap = self.request("/api/state")
            if snap["state"] in wanted:
                return snap
            time.sleep(0.05)
        raise AssertionError("等待状态 %s 超时，当前 %s" % (wanted, {k: snap[k] for k in ("state", "phase", "traveled", "pose")}))

    def close(self):
        self.session.shutdown()
        self.server.shutdown()
        self.server.server_close()
        self.dog.close()
        self.store.close()


@pytest.fixture
def app(tmp_path):
    a = App(tmp_path)
    yield a
    a.close()


def test_full_flow_map_plan_execute(app):
    status, res = app.request("/api/mapping/start", {"name": "L 房间"})
    assert status == 200
    map_id = res["map_id"]
    assert app.request("/api/mapping/start", {"name": "再来一次"})[0] == 409
    snap = app.wait_state({IDLE, FAILED})
    assert snap["state"] == IDLE, snap["error"]
    assert not app.backend.calls_to("stop_algo_task") and not app.backend.goals  # 没有用狗的导航服务

    status, detail = app.request("/api/maps/%d" % map_id)
    assert status == 200 and detail["status"] == "complete" and detail["grid_b64"]
    assert len(detail["trajectory"]) > 50
    assert app.backend.room.collisions == 0

    # 房间坐标里的路线，换到以出发点为原点的地图坐标
    route = [[1.0, 1.5], [3.5, 2.5], [1.0, 4.0]]
    waypoints = [[x - START[0], y - START[1]] for x, y in route]
    status, saved = app.request("/api/maps/%d/paths" % map_id, {"name": "巡逻", "waypoints": waypoints})
    assert status == 200, saved
    assert app.request("/api/paths/%d/execute" % saved["path_id"], {})[0] == 200
    snap = app.wait_state({IDLE, FAILED})
    assert snap["state"] == IDLE, snap["error"]
    assert snap["progress"] == {"done": len(saved["points"]), "total": len(saved["points"])}
    room = app.backend.room
    assert math.hypot(room.x - route[-1][0], room.y - route[-1][1]) < 0.35
    assert room.collisions == 0


def test_execute_requires_known_position(app):
    map_id = app.store.create_map("旧图")
    app.store.save_grid(map_id, np.zeros(4, dtype=np.int8).tobytes(), 2, 2, 1.0, 0.0, 0.0)
    app.store.finish_map(map_id, "complete")
    path_id = app.store.save_path(map_id, "p", [(0.5, 0.5)], [(0.5, 0.5)])
    status, res = app.request("/api/paths/%d/execute" % path_id, {})
    assert status == 409 and "设置狗的位置" in res["error"]


def test_set_pose_rejects_mismatch_and_bad_input(app):
    map_id = app.store.create_map("空图")
    app.store.save_grid(map_id, np.zeros(400, dtype=np.int8).tobytes(), 20, 20, 0.5, -5.0, -5.0)
    app.store.finish_map(map_id, "complete")
    status, res = app.request("/api/maps/%d/pose" % map_id, {"x": 0.0, "y": 0.0, "yaw": 0.0})
    assert status == 409 and "对不上" in res["error"]
    assert app.request("/api/maps/%d/pose" % map_id, {"x": "a", "y": 0, "yaw": 0})[0] == 400


def test_stop_during_mapping_keeps_partial_map(tmp_path):
    a = App(tmp_path, speed=2.0)
    try:
        map_id = a.request("/api/mapping/start", {"name": "中止"})[1]["map_id"]
        deadline = time.monotonic() + 20
        while a.store.get_map(map_id)["grid"] is None:
            assert time.monotonic() < deadline, "没有保存地图"
            time.sleep(0.05)
        assert a.request("/api/stop", {})[0] == 200
        snap = a.wait_state({IDLE, FAILED}, timeout=10.0)
        assert snap["state"] == IDLE, snap["error"]
        assert a.store.get_map(map_id)["status"] == "stopped"
        assert not a.backend.servo_active
    finally:
        a.close()


def test_worker_failure_stops_dog_and_blocks_new_tasks(app):
    app.backend.latched["bms_status"] = dict(app.backend.latched["bms_status"], batt_soc=5)
    map_id = app.request("/api/mapping/start", {"name": "低电"})[1]["map_id"]
    snap = app.wait_state({FAILED}, timeout=10.0)
    assert "低于" in snap["error"]
    assert app.store.get_map(map_id)["status"] == "failed"
    status, res = app.request("/api/mapping/start", {"name": "重来"})
    assert status == 409 and "重启" in res["error"]


def test_bad_requests_are_rejected(app):
    map_id = app.store.create_map("空")
    app.store.finish_map(map_id, "failed")
    assert app.request("/api/maps/%d/plan" % map_id, {"waypoints": [[1, 1]]})[0] == 400
    assert app.request("/api/maps/999")[0] == 404
    assert app.request("/api/paths/999/execute", {})[0] == 404
    assert app.request("/api/mapping/start", {"name": "  "})[0] == 400
    assert app.request("/api/maps/%d/plan" % map_id, {"waypoints": "x"})[0] == 400


def test_cross_site_post_is_refused(app):
    status, _ = app.request("/api/stop", {}, headers={"Origin": "http://evil.example"})
    assert status == 403
    status, _ = app.request("/api/stop", {}, headers={"Content-Type": "text/plain"})
    assert status == 415


# ------------------------------------------------------------ 终端输入
def prompt_answers(monkeypatch, host, user, password):
    from roommapper import __main__ as cli

    answers = iter([host, user])
    connected = []
    monkeypatch.setattr("builtins.input", lambda prompt: next(answers))
    monkeypatch.setattr(cli.getpass, "getpass", lambda prompt: password)
    monkeypatch.setattr(cli.CyberDog, "connect", lambda *a, **kw: connected.append((a, kw)) or "dog")
    return cli, connected


def test_real_dog_connection_details_come_from_terminal(monkeypatch):
    cli, connected = prompt_answers(monkeypatch, " 10.0.0.5 ", "mi", "secret")
    assert cli.open_dog(argparse.Namespace(mock=False)) == "dog"
    assert connected == [(("10.0.0.5",), {"user": "mi", "password": "secret"})]


@pytest.mark.parametrize("answers", [("", "mi", "x"), ("10.0.0.5", "", "x"), ("10.0.0.5", "mi", "")])
def test_empty_connection_details_exit(monkeypatch, answers):
    cli, connected = prompt_answers(monkeypatch, *answers)
    with pytest.raises(SystemExit):
        cli.open_dog(argparse.Namespace(mock=False))
    assert connected == []

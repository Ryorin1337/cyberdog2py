"""导航与跟随任务（start_algo_task 动作 / stop_algo_task 服务）。"""

import math
import threading
from typing import Callable, Optional, Sequence

from .constants import (FOLLOW_POSITIONS, NAV_OK_RESULTS, NAV_RESULTS, TASK_ACCEPTING, TASK_STATUS_NAMES,
                        TASK_STOP_BY_STATUS, TASK_TRANSITIONAL, Iface, NavType, StopTask)
from .errors import CommandFailed, CyberDogError, GoalRejected, SensorTimeout

RELEASE_STATUS_TIMEOUT = 5.0    # s，等狗报告任务状态的上限
RELEASE_SETTLE_TIMEOUT = 30.0   # s，等正在切换的状态结束（官方建图停止最长要 15 秒以上）


def _status_text(status: int) -> str:
    return "%d：%s" % (status, TASK_STATUS_NAMES.get(status, "未知状态"))


def _stop_task(backend, task: int, timeout: float, status: str) -> None:
    name, srv_type = Iface.NAV_STOP
    resp = backend.call_service(name, srv_type, {"task_id": int(task), "map_name": ""}, timeout=timeout)
    if resp.get("result", 0) != 0:
        raise CommandFailed("停止任务失败（狗的状态 %s，task_id=%d）: %s" % (status, int(task), resp),
                            resp.get("code", 0), resp)


def release_tasks(backend, service_timeout: float = 30.0) -> int:
    """按狗的实际状态结束任务，返回结束前的状态。

    狗上不同状态要用不同的 task_id，对应关系见 constants.TASK_STOP_BY_STATUS。
    读不到状态时，出于安全仍然发一次 StopTask.ALL，然后抛 SensorTimeout。未保存的地图会被丢弃。
    """
    seen = []
    cond = threading.Condition()

    def on_status(msg):
        with cond:
            seen.append(int(msg["task_status"]))
            cond.notify_all()

    name, msg_type = Iface.NAV_STATUS
    sub = backend.subscribe(name, msg_type, on_status)
    try:
        with cond:
            if cond.wait_for(lambda: seen, RELEASE_STATUS_TIMEOUT):
                settled = cond.wait_for(lambda: seen[-1] not in TASK_TRANSITIONAL, RELEASE_SETTLE_TIMEOUT)
            else:
                settled = False
            status = seen[-1] if seen else None
    finally:
        backend.unsubscribe(sub)
    if status is None:
        message = "%.1fs 内没有收到 %s，无法判断狗的任务状态" % (RELEASE_STATUS_TIMEOUT, name)
        try:
            _stop_task(backend, StopTask.ALL, service_timeout, "未知")
        except CommandFailed as exc:
            raise SensorTimeout(message + "；stop_algo_task(ALL) 也失败了") from exc
        raise SensorTimeout(message + "；已发送 stop_algo_task(ALL)")
    if not settled:
        raise SensorTimeout("狗 %.1fs 内一直处于状态 %s" % (RELEASE_SETTLE_TIMEOUT, _status_text(status)))
    if status in TASK_ACCEPTING:
        return status
    if status not in TASK_STOP_BY_STATUS:
        raise CyberDogError("狗的状态是 %s，没有对应的结束方式，可能需要重启狗上的服务" % _status_text(status))
    _stop_task(backend, TASK_STOP_BY_STATUS[status], service_timeout, _status_text(status))
    return status


class NavTask(object):
    """一个已被接受的导航或跟随任务。"""

    def __init__(self, nav, goal_id: str, nav_type: int):
        self._nav = nav
        self.goal_id = goal_id
        self.nav_type = nav_type

    def wait(self, timeout: Optional[float] = None) -> dict:
        """等待任务结束。result 不是 SUCCESS 或 ACCEPT 时抛 GoalRejected。"""
        self._nav._dog._check()
        res = self._nav._dog._backend.wait_goal(self.goal_id, timeout)
        code = res.get("result")
        if code not in NAV_OK_RESULTS:
            raise GoalRejected("任务 %s 结束: %s" % (NavType(self.nav_type).name, NAV_RESULTS.get(code, code)),
                               code or 0, res)
        return res

    def cancel(self) -> None:
        self._nav._dog._check()
        self._nav._dog._backend.cancel_goal(self.goal_id)


def pose_stamped(x: float, y: float, yaw: float = 0.0, frame_id: str = "map") -> dict:
    return {"header": {"frame_id": frame_id},
            "pose": {"position": {"x": float(x), "y": float(y), "z": 0.0},
                     "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}}}


class Navigation(object):
    def __init__(self, dog):
        self._dog = dog
        self.started = False  # 本次会话是否启动过导航/跟随任务；为 True 时 dog.close() 会自动 stop()

    def start(self, nav_type: int, feedback: Optional[Callable[[dict], None]] = None, **goal) -> NavTask:
        """通用入口：发送 Navigation 目标。goal 中的字段见 protocol/action/Navigation.action。"""
        name, action_type = Iface.NAV_START
        req = {"nav_type": int(nav_type)}
        req.update(goal)
        self._dog._check()
        cb = self._dog._wrap(feedback) if feedback else None
        self.started = True  # 先置位：即使 send_goal 超时，任务也可能已经在狗上启动，close() 时要停掉
        goal_id = self._dog._backend.send_goal(name, action_type, req, feedback_cb=cb, timeout=15.0)
        return NavTask(self, goal_id, int(nav_type))

    def follow_person(self, distance: float = 1.2, position: str = "behind", **kw) -> NavTask:
        """人体跟随。position 可选 auto、behind、left、right。"""
        return self._follow(False, distance, position, **kw)

    def follow_object(self, roi: Sequence[int], distance: float = 1.2, position: str = "behind", **kw) -> NavTask:
        """万物跟随。roi 是图像中目标框的 (x_offset, y_offset, width, height)。"""
        x, y, w, h = [int(v) for v in roi]
        return self._follow(True, distance, position,
                            tracking_roi={"x_offset": x, "y_offset": y, "width": w, "height": h}, **kw)

    def _follow(self, object_tracking, distance, position, **kw):
        if position not in FOLLOW_POSITIONS:
            raise ValueError("position 可选: %s" % list(FOLLOW_POSITIONS))
        if distance <= 0:
            raise ValueError("distance 必须为正数，收到 %s" % distance)
        return self.start(NavType.START_HUMAN_TRACKING, object_tracking=object_tracking,
                          keep_distance=float(distance), relative_pos=FOLLOW_POSITIONS[position], **kw)

    def follow_uwb(self, distance: float = 1.2, **kw) -> NavTask:
        """UWB 跟随（需要配套的 UWB 标签）。"""
        if distance <= 0:
            raise ValueError("distance 必须为正数，收到 %s" % distance)
        return self.start(NavType.START_UWB_TRACKING, keep_distance=float(distance), **kw)

    def start_mapping(self, outdoor: bool = False, **kw) -> NavTask:
        """开始建图。outdoor 为 False 时用激光建图，为 True 时用视觉建图。"""
        return self.start(NavType.START_MAPPING, outdoor=outdoor, **kw)

    def stop_mapping(self, map_name: str) -> None:
        """结束建图，并以 map_name 保存地图。"""
        self.stop(StopTask.MAPPING, map_name=map_name)

    def start_localization(self, map_name: str = "", **kw) -> NavTask:
        return self.start(NavType.START_LOCALIZATION, map_name=map_name, **kw)

    def go_to(self, x: float, y: float, yaw: float = 0.0, **kw) -> NavTask:
        """AB 点导航到地图坐标 (x, y, yaw)。需要先建图并完成定位。"""
        return self.start(NavType.START_AB, poses=[pose_stamped(x, y, yaw)], **kw)

    def stop(self, task: int = StopTask.ALL, map_name: str = "") -> None:
        self._stop(task, map_name, check=True)

    def _stop(self, task, map_name, check):
        name, srv_type = Iface.NAV_STOP
        req = {"task_id": int(task), "map_name": map_name}
        if check:
            resp = self._dog._call(name, srv_type, req, timeout=30.0)
        else:
            resp = self._dog._backend.call_service(name, srv_type, req, timeout=30.0)
        if resp.get("result", 0) != 0:
            raise CommandFailed("停止任务失败: %s" % resp, resp.get("code", 0), resp)
        if int(task) == int(StopTask.ALL):
            self.started = False

    def release(self) -> int:
        """结束狗上正在运行的任务（包括建图、定位），返回结束前的状态码。未保存的地图会被丢弃。

        这是收尾指令，和 motion.estop() 一样，回调崩溃后也照样下发。
        """
        self._dog._ensure_open()
        return self._release()

    def _release(self) -> int:
        status = release_tasks(self._dog._backend)
        self.started = False
        return status

    def on_status(self, callback: Callable[[dict], None]) -> int:
        name, msg_type = Iface.NAV_STATUS
        return self._dog._subscribe(name, msg_type, callback)

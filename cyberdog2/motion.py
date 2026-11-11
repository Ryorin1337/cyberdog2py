"""运动控制：结果指令（完整动作）与伺服指令（持续行走）。"""

import time
from typing import Optional, Sequence, Union

from .backends.servo import fill_motion_arrays
from .constants import GAIT_LIMITS, GAITS, POS_LIMIT, RPY_LIMIT, SERVO_MOTIONS, Iface, MotionId
from .errors import MotionRejected

ActionLike = Union[str, int, MotionId]


def resolve_motion(action: ActionLike) -> int:
    """把 "sit"、MotionId.SIT、143 统一转成 motion_id；未知动作直接抛 ValueError。"""
    if isinstance(action, str):
        key = action.strip().upper()
        if key not in MotionId.__members__:
            raise ValueError("未知动作 %r，可选: %s" % (action, ", ".join(m.lower() for m in MotionId.__members__)))
        return int(MotionId[key])
    if isinstance(action, bool) or not isinstance(action, int):
        raise TypeError("动作必须是字符串、MotionId 或整数，收到 %r" % (action,))
    return int(action)


def _check_vec(name: str, values: Sequence[float], limits: Sequence[float]) -> None:
    if len(values) != len(limits):
        raise ValueError("%s 需要 %d 个值，收到 %d 个" % (name, len(limits), len(values)))
    for v, lim in zip(values, limits):
        if abs(v) > lim:
            raise ValueError("%s=%s 超出范围（每个分量的上限为 %s）" % (name, list(values), list(limits)))


class Motion(object):
    def __init__(self, dog):
        self._dog = dog

    # ------------------------------------------------------------ 结果指令
    def do(self, action: ActionLike, *, rpy: Optional[Sequence[float]] = None,
           pos: Optional[Sequence[float]] = None, step_height: Optional[Sequence[float]] = None,
           duration: int = 0, value: int = 0, timeout: float = 30.0) -> dict:
        """执行一个完整动作，阻塞到 motion_manager 返回结果。

        动作被拒绝时抛 MotionRejected。rpy（rad，每个分量上限 0.4）和 pos（m，上限 0.2/0.2/0.3）
        只对支持姿态参数的动作生效，超出范围直接抛 ValueError。
        """
        motion_id = resolve_motion(action)
        if motion_id in SERVO_MOTIONS:
            raise ValueError("motion_id %d 是伺服步态，请用 set_velocity() 或 walk()" % motion_id)
        req = {"motion_id": motion_id, "cmd_source": int(self._dog.cmd_source),
               "duration": int(duration), "value": int(value)}
        if rpy is not None:
            _check_vec("rpy", rpy, (RPY_LIMIT,) * 3)
            req["rpy_des"] = list(rpy)
        if pos is not None:
            _check_vec("pos", pos, POS_LIMIT)
            req["pos_des"] = list(pos)
        if step_height is not None:
            req["step_height"] = list(step_height)
        return self._send_result(req, timeout, check=True)

    def _send_result(self, req: dict, timeout: float, check: bool) -> dict:
        motion_id = req["motion_id"]
        name, srv_type = Iface.MOTION_RESULT
        if check:
            resp = self._dog._call(name, srv_type, fill_motion_arrays(req), timeout=timeout)
        else:
            resp = self._dog._backend.call_service(name, srv_type, fill_motion_arrays(req), timeout=timeout)
        if not resp.get("result"):
            raise MotionRejected("动作 %d 被拒绝, code=%s" % (motion_id, resp.get("code")),
                                 resp.get("code", 0), resp)
        return resp

    def stand(self, **kw) -> dict:
        return self.do(MotionId.STAND, **kw)

    def get_down(self, **kw) -> dict:
        return self.do(MotionId.GET_DOWN, **kw)

    lie_down = get_down

    def sit(self, **kw) -> dict:
        return self.do(MotionId.SIT, **kw)

    def shake_hand(self, left: bool = False, **kw) -> dict:
        return self.do(MotionId.SHAKE_LEFT_HAND if left else MotionId.SHAKE_RIGHT_HAND, **kw)

    def estop(self) -> dict:
        """急停（motion_id 0）。会先结束伺服，再下发急停。

        这是安全指令，即使之前有回调崩溃（其他 API 都会抛 CallbackError），急停也照样下发。
        只要后端连接还在，就一定会尝试发出。
        """
        self._dog._ensure_open()
        try:
            self._dog._backend.servo_stop()
        finally:
            resp = self._send_result({"motion_id": int(MotionId.ESTOP), "cmd_source": int(self._dog.cmd_source),
                                      "duration": 0, "value": 0}, 5.0, check=False)
        return resp

    # ------------------------------------------------------------ 伺服指令
    def set_velocity(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0, *,
                     gait: Union[str, int] = "medium", step_height: float = 0.05,
                     timeout: Optional[float] = None) -> None:
        """设定行走速度（m/s，rad/s），立即返回；后台以 20Hz 持续下发，直到调用 stop()。

        速度超过所选步态的上限时直接抛 ValueError，不会自动截断。
        timeout（秒）是死人开关：超过这个时间没有再次调用 set_velocity() 就自动停下。
        实时控制循环里建议设成控制周期的几倍，例如 0.5，这样主程序卡死时狗也会停。
        """
        if isinstance(gait, str):
            if gait not in GAITS:
                raise ValueError("未知步态 %r，可选: %s" % (gait, list(GAITS)))
            motion_id = int(GAITS[gait])
        else:
            motion_id = int(gait)
        if motion_id not in GAIT_LIMITS:
            raise ValueError("未知步态 %r，可选: %s 或 %s" % (gait, list(GAITS), sorted(GAIT_LIMITS)))
        _check_vec("(vx, vy, wz)", (vx, vy, wz), GAIT_LIMITS[motion_id])
        if not 0.0 <= step_height <= 0.1:
            raise ValueError("step_height=%s 超出范围 [0, 0.1] m" % step_height)
        if timeout is not None and timeout <= 0:
            raise ValueError("timeout 必须为正数，收到 %r" % timeout)
        self._dog._check()
        cmd = {
            "motion_id": motion_id,
            "cmd_source": int(self._dog.cmd_source),
            "value": 0,
            "vel_des": [float(vx), float(vy), float(wz)],
            "step_height": [float(step_height), float(step_height)],
        }
        if timeout is not None:
            cmd["expire_after"] = float(timeout)
        self._dog._backend.servo_set(cmd)

    def stop(self) -> None:
        """结束伺服（发送 SERVO_END），狗会停下站立。"""
        self._dog._backend.servo_stop()
        self._dog._check()

    def walk(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0, duration: float = 1.0, **kw) -> None:
        """以给定速度行走 duration 秒后停止（阻塞）。中途出错也会先停止，再抛出异常。

        另外设置了 duration + 1 秒的死人开关：即使本线程卡住，狗也会在那之后自动停下。
        """
        if duration <= 0:
            raise ValueError("duration 必须为正数，收到 %r" % duration)
        kw.setdefault("timeout", duration + 1.0)
        self.set_velocity(vx, vy, wz, **kw)
        try:
            end = time.monotonic() + duration
            while True:
                left = end - time.monotonic()
                if left <= 0:
                    break
                time.sleep(min(left, 0.05))
                self._dog._check()
        finally:
            self._dog._backend.servo_stop()
        self._dog._check()

    def turn(self, wz: float, duration: float = 1.0, **kw) -> None:
        """原地转向，wz 为角速度（rad/s），正值为左转。"""
        self.walk(0.0, 0.0, wz, duration, **kw)

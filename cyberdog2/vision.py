"""视觉算法：人脸、人体、手势识别。"""

from typing import Callable

from .constants import ALGOS, GESTURES, Iface
from .errors import CommandFailed


class GestureEvent(object):
    def __init__(self, raw: dict):
        self.raw = raw
        self.id = int(raw["id"])
        self.name = GESTURES.get(self.id)  # 例如 thumb_up；未知 id 为 None

    def __repr__(self):
        return "GestureEvent(%r, id=%d)" % (self.name, self.id)


def _algo_list(names):
    out = []
    for n in names:
        if n not in ALGOS:
            raise ValueError("未知算法 %r，可选: %s" % (n, list(ALGOS)))
        out.append({"algo_module": ALGOS[n]})
    return out


class Vision(object):
    def __init__(self, dog):
        self._dog = dog

    # ---- 通用算法管理（algo_manager），结果从 person 话题发出
    def enable(self, *algos: str, open_age: bool = False, open_emotion: bool = False) -> None:
        """开启算法，可选 face、body、gesture、keypoints、reid、focus。"""
        if not algos:
            raise ValueError("至少指定一个算法，可选: %s" % list(ALGOS))
        self._manage(_algo_list(algos), [], open_age, open_emotion)

    def disable(self, *algos: str) -> None:
        if not algos:
            raise ValueError("至少指定一个算法，可选: %s" % list(ALGOS))
        self._manage([], _algo_list(algos), False, False)

    def _manage(self, enable, disable, open_age, open_emotion):
        name, srv_type = Iface.ALGO_MANAGER
        resp = self._dog._call(name, srv_type, {"algo_enable": enable, "algo_disable": disable,
                                                "open_age": open_age, "open_emotion": open_emotion}, timeout=30.0)
        if enable and resp.get("result_enable", 0) != 0:
            raise CommandFailed("开启算法失败: %s" % resp, resp.get("code", 0), resp)
        if disable and resp.get("result_disable", 0) != 0:
            raise CommandFailed("关闭算法失败: %s" % resp, resp.get("code", 0), resp)

    def on_person(self, callback: Callable[[dict], None]) -> int:
        """person 话题回调，参数是包含 face_info、body_info、track_res 字段的 dict。"""
        name, msg_type = Iface.PERSON
        return self._dog._subscribe(name, msg_type, callback)

    # ---- 手势识别
    def start_gesture(self, timeout: int = 60) -> None:
        """开启手势识别，持续 timeout 秒（有效范围 1~300）。"""
        if not 1 <= timeout <= 300:
            raise ValueError("timeout 必须在 1~300 秒之间")
        self._gesture(0, timeout)

    def stop_gesture(self) -> None:
        self._gesture(1, 60)

    def _gesture(self, command, timeout):
        name, srv_type = Iface.GESTURE
        resp = self._dog._call(name, srv_type, {"command": command, "timeout": int(timeout)}, timeout=30.0)
        if resp.get("code", 0) != 0:
            raise CommandFailed("手势识别命令失败: code=%s（1 表示忙）" % resp.get("code"), resp.get("code", 0), resp)

    def on_gesture(self, callback: Callable[[GestureEvent], None]) -> int:
        name, msg_type = Iface.GESTURE_MSG
        return self._dog._subscribe(name, msg_type, lambda d: callback(GestureEvent(d)))

    # ---- 人脸识别（需要先在 App 里录入人脸）
    def recognize_face(self, username: str = "", timeout: int = 60) -> None:
        """开始人脸识别。结果通过 on_face 回调返回。username 为空时识别所有已录入的人脸。"""
        if not 30 <= timeout <= 300:
            raise ValueError("timeout 必须在 30~300 秒之间")
        name, srv_type = Iface.FACE_REC
        req = {"command": 1 if username else 0, "username": username, "id": "", "timeout": int(timeout)}
        resp = self._dog._call(name, srv_type, req, timeout=30.0)
        if resp.get("result", 0) != 0:
            raise CommandFailed("人脸识别启动失败: %s" % resp, resp.get("code", 0), resp)

    def cancel_face(self) -> None:
        name, srv_type = Iface.FACE_REC
        resp = self._dog._call(name, srv_type, {"command": 2, "username": "", "id": "", "timeout": 60})
        if resp.get("result", 0) != 0:
            raise CommandFailed("取消人脸识别失败: %s" % resp, resp.get("code", 0), resp)

    def on_face(self, callback: Callable[[dict], None]) -> int:
        """人脸识别结果回调，参数是 FaceRecognitionResult 消息转成的 dict。"""
        name, msg_type = Iface.FACE_REC_MSG
        return self._dog._subscribe(name, msg_type, callback)

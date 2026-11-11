"""语音播放与音量。"""

from .constants import Iface
from .errors import CommandFailed

MODULE_NAME = "cyberdog2py"


class Audio(object):
    def __init__(self, dog):
        self._dog = dog

    def say(self, text: str, timeout: float = 30.0) -> None:
        """在线 TTS 播放一段文字，阻塞到播放完毕。

        需要狗已经联网并登录小米账号（App 中开启在线语音）。播放失败时抛 CommandFailed。
        """
        if not text:
            raise ValueError("text 不能为空")
        self._play(is_online=True, play_id=0, text=text, timeout=timeout)

    def play(self, play_id: int, timeout: float = 30.0) -> None:
        """离线播放系统内置语音。play_id 的取值见 protocol/msg/AudioPlay.msg，例如 9000 是整机就绪音效。"""
        self._play(is_online=False, play_id=int(play_id), text="", timeout=timeout)

    def _play(self, is_online, play_id, text, timeout):
        name, srv_type = Iface.SPEECH
        req = {"module_name": MODULE_NAME, "is_online": is_online,
               "speech": {"module_name": MODULE_NAME, "play_id": play_id}, "text": text}
        resp = self._dog._call(name, srv_type, req, timeout=timeout)
        if resp.get("status", 0) != 0:
            raise CommandFailed("语音播放失败: status=%s code=%s" % (resp.get("status"), resp.get("code")),
                                resp.get("code", 0), resp)

    @property
    def volume(self) -> int:
        name, srv_type = Iface.VOLUME_GET
        return int(self._dog._call(name, srv_type, {})["volume"])

    @volume.setter
    def volume(self, value: int) -> None:
        if not 0 <= int(value) <= 100:
            raise ValueError("音量范围是 0~100，收到 %r" % value)
        name, srv_type = Iface.VOLUME_SET
        resp = self._dog._call(name, srv_type, {"volume": int(value)})
        if not resp.get("success"):
            raise CommandFailed("设置音量失败: code=%s" % resp.get("code"), resp.get("code", 0), resp)

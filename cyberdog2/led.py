"""灯效控制（led_execute 服务）。"""

from typing import Sequence, Union

from .constants import (Iface, LED_CODES, LED_MINI_PREDEFINED, LED_MINI_USER_EFFECTS, LED_MODE_PREDEFINED,
                        LED_MODE_USER, LED_OFF, LED_PREDEFINED, LED_TARGETS, LED_USER_EFFECTS)
from .errors import CommandFailed

Color = Union[str, Sequence[int]]


class Led(object):
    def __init__(self, dog, client: str = "vp"):
        self._dog = dog
        # client 决定优先级，必须是 LedExecute.srv 里登记过的模块名之一：lowpower、bms、connector、vp、tracking、system
        self.client = client

    def set(self, target: str, color: Color = "red", effect: str = "on") -> None:
        """设置灯效。

        target 取值为 "head"、"tail" 或 "mini"。
        color 可以是 "red"、"blue"、"yellow"、"off"，也可以是 (r, g, b) 元组。
        effect 的取值：
          - 头灯和尾灯：on、blink、blink_fast、breath、breath_fast、one_by_one、one_by_one_fast；
            使用 RGB 元组时还可以用 back_and_forth、trailing_race。
          - mini 灯：使用 RGB 元组时可用 circular_breath、circular_ring；
            预定义灯效可用 rectangle_color、centre_color、three_circular、color_one_by_one，color 传 None 即可。
        组合不合法时直接抛 ValueError。
        """
        if target not in LED_TARGETS:
            raise ValueError("target 必须是 %s 之一，收到 %r" % (list(LED_TARGETS), target))
        mode, eff, rgb = self._resolve(target, color, effect)
        self._send(target, mode, eff, rgb, occupation=True)

    def off(self, target: str) -> None:
        if target == "mini":
            self._send(target, LED_MODE_PREDEFINED, LED_MINI_PREDEFINED["off"], (0, 0, 0), True)
        else:
            self.set(target, "off")

    def release(self, target: str) -> None:
        """释放对该灯的占用，让系统（或其他模块）的灯效接管。"""
        if target not in LED_TARGETS:
            raise ValueError("target 必须是 %s 之一，收到 %r" % (list(LED_TARGETS), target))
        self._send(target, LED_MODE_PREDEFINED, LED_OFF if target != "mini" else LED_MINI_PREDEFINED["off"],
                   (0, 0, 0), occupation=False)

    # ------------------------------------------------------------------
    @staticmethod
    def _resolve(target, color, effect):
        if target == "mini":
            if isinstance(color, str):
                raise ValueError("mini 灯不支持颜色名 %r：预定义灯效请传 color=None，自定义颜色请传 (r, g, b)；"
                                 "关灯用 led.off('mini')" % color)
            if color is None:
                if effect not in LED_MINI_PREDEFINED:
                    raise ValueError("mini 灯的预定义灯效可选: %s" % list(LED_MINI_PREDEFINED))
                return LED_MODE_PREDEFINED, LED_MINI_PREDEFINED[effect], (0, 0, 0)
            if effect not in LED_MINI_USER_EFFECTS:
                raise ValueError("mini 灯的 RGB 灯效可选: %s" % list(LED_MINI_USER_EFFECTS))
            return LED_MODE_USER, LED_MINI_USER_EFFECTS[effect], _rgb(color)
        if color is None:
            raise ValueError("头灯/尾灯必须指定 color：颜色名或 (r, g, b)")
        if isinstance(color, str):
            if color == "off":
                if effect != "on":
                    raise ValueError("color='off' 时不能指定灯效 %r" % effect)
                return LED_MODE_PREDEFINED, LED_OFF, (0, 0, 0)
            key = (color, effect)
            if key not in LED_PREDEFINED:
                raise ValueError("不支持的预定义组合 color=%r effect=%r；颜色可选 red/blue/yellow/off，"
                                 "其他颜色请传 (r, g, b)" % (color, effect))
            return LED_MODE_PREDEFINED, LED_PREDEFINED[key], (0, 0, 0)
        if effect not in LED_USER_EFFECTS:
            raise ValueError("RGB 模式的灯效可选: %s" % list(LED_USER_EFFECTS))
        return LED_MODE_USER, LED_USER_EFFECTS[effect], _rgb(color)

    def _send(self, target, mode, effect, rgb, occupation):
        name, srv_type = Iface.LED
        req = {"occupation": occupation, "client": self.client, "target": LED_TARGETS[target], "mode": mode,
               "effect": effect, "r_value": rgb[0], "g_value": rgb[1], "b_value": rgb[2]}
        resp = self._dog._call(name, srv_type, req)
        code = resp.get("code", 0)
        if code != 0:
            raise CommandFailed("LED 设置失败: %s (code=%d)" % (LED_CODES.get(code, "UNKNOWN"), code), code, resp)


def _rgb(color) -> tuple:
    values = tuple(int(c) for c in color)
    if len(values) != 3 or not all(0 <= c <= 255 for c in values):
        raise ValueError("RGB 必须是 3 个 0~255 的整数，收到 %r" % (color,))
    return values

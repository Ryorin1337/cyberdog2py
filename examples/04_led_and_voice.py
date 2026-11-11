"""灯效和语音。在线语音需要狗已联网并登录小米账号。"""
import time

from _common import open_dog

with open_dog() as dog:
    dog.led.set("head", "blue", "breath")
    dog.led.set("tail", (0, 255, 128), "trailing_race")
    dog.led.set("mini", None, "three_circular")
    print("当前音量:", dog.audio.volume)
    dog.audio.say("你好，我是铁蛋")
    time.sleep(3)
    for target in ("head", "tail", "mini"):
        dog.led.release(target)

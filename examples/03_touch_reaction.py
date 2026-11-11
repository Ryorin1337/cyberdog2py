"""摸头互动：单击坐下，双击握手，长按退出。"""
import queue
import sys

from _common import open_dog

events = queue.Queue()

with open_dog() as dog:
    dog.motion.stand()
    # 回调运行在后台线程里，只负责把事件放进队列；运动指令在主线程里发送
    dog.sensors.on_touch(events.put)
    if "--mock" in sys.argv or len(sys.argv) == 1:
        for state in (0x01, 0x03, 0x07):  # 模拟依次单击、双击、长按
            dog.backend.emit("touch_status", {"touch_state": state, "timestamp": 0})
    print("摸摸狗头：单击坐下，双击握手，长按退出")
    while True:
        ev = events.get()
        print("收到", ev)
        if ev.kind == "single":
            dog.motion.sit()
            dog.motion.stand()
        elif ev.kind == "double":
            dog.motion.shake_hand()
        elif ev.kind == "long":
            break
    dog.motion.get_down()

"""人体跟随 20 秒，期间打印任务状态。"""
import time

from _common import open_dog
from cyberdog2 import CyberDog

with (CyberDog.connect("192.168.1.139", password="123") as dog):
    dog.motion.stand()
    dog.nav.on_status(lambda s: print("任务状态:", s))
    task = dog.nav.follow_person(distance=1.2, position="behind",
                                 feedback=lambda fb: print("反馈:", fb.get("feedback_code"), fb.get("feedback_msg")))
    print("已接受，结果:", task.wait(timeout=30))
    time.sleep(20)
    dog.nav.stop()
    print("已停止跟随")

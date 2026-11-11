"""走一个正方形：直行 2 秒，左转 90 度，重复 4 次。请在空旷场地运行。"""
import math

from _common import open_dog
from cyberdog2 import CyberDog

with CyberDog.connect("192.168.1.139", password="123") as dog:
    dog.motion.stand()
    for i in range(4):
        print("第 %d 条边" % (i + 1))
        dog.motion.walk(vx=0.2, duration=2.0)
        dog.motion.turn(wz=0.8, duration=(math.pi / 2) / 0.8)
    dog.motion.get_down()
    dog.close()
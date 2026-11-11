"""站立、坐下、握手、趴下。"""
import time

from _common import open_dog
from cyberdog2 import CyberDog
with CyberDog.connect("192.168.1.139", password="123") as dog:
    print("电量:", dog.sensors.battery.soc, "%")
    dog.motion.stand()
    time.sleep(1)
    dog.motion.sit()
    dog.motion.shake_hand()
    dog.motion.stand()
    dog.motion.get_down()
    print("完成")


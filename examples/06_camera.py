"""开启 AI 相机，取一帧保存为 frame.jpg。"""
from _common import open_dog
from cyberdog2 import CyberDog

with CyberDog.connect("192.168.1.139", password="123") as dog:
    dog.sensors.camera.start(640, 480)
    try:
        frame = dog.sensors.camera.get_frame()
        print(frame)
        frame.save("frame.jpg")
        print("已保存 frame.jpg")
    finally:
        dog.sensors.camera.stop()

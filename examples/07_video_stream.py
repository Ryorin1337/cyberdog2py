"""连续视频流 + 实时人脸检测（OpenCV 自带的 Haar 级联分类器）。按 Esc 退出。

需要：pip install opencv-python numpy
把"识别"那一段换成你自己的算法即可，例如 YOLO 或 MediaPipe。
"""
import sys
import time

import cv2

from _common import open_dog

MOCK = "--mock" in sys.argv or len([a for a in sys.argv[1:] if not a.startswith("--")]) == 0
detector = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

with open_dog() as dog:
    with dog.sensors.camera.stream(width=640, height=480, fps=15, quality=70) as stream:
        t0, n = time.time(), 0
        for frame in stream:
            img = frame.to_bgr()

            # ================= 识别 =================
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = detector.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5)
            for (x, y, w, h) in faces:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # =======================================

            n += 1
            fps = n / (time.time() - t0)
            cv2.putText(img, "%.1f fps  dropped %d" % (fps, stream.dropped), (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("CyberDog", img)
            if cv2.waitKey(1) == 27 or (MOCK and n >= 30):
                break
    cv2.destroyAllWindows()
    print("共处理 %d 帧，丢弃 %d 帧" % (n, stream.dropped))

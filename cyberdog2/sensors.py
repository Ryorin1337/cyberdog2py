"""传感器读取：电池、触摸、TOF、超声、GPS、UWB、里程计、IMU、运动状态、相机。"""

import math
import threading
import time
import traceback
from typing import Callable, Iterator, Optional

from .backends.streaming import LatestSlot
from .constants import TOUCH_KINDS, CameraCmd, Iface, SensorId, SwitchStatus
from .errors import CommandFailed, CyberDogError, SensorTimeout
from .frame import Frame

PROBE_TIMEOUT = 1.0      # 第一次读取时等这么久还没有数据，就认为传感器没开（秒）
GPS_PROBE_TIMEOUT = 3.0  # GPS 发布得慢，要多等一会儿，免得把别的程序打开的 GPS 当成没开


class TopicCache(object):
    """惰性订阅一个话题并缓存最新的消息。第一次读取时才订阅。

    sensor_id 不为 None 时，第一次读取如果等不到数据，会自动打开这个传感器。已经有数据就不碰它，
    所以只有库自己打开的传感器才会在退出时关闭，别的程序（比如 App 的导航）正在用的不受影响。
    """

    def __init__(self, dog, iface, sensor_id: Optional[int] = None, probe: float = PROBE_TIMEOUT):
        self._dog = dog
        self.topic, self.msg_type = iface
        self._sensor_id = sensor_id
        self._probe = probe
        self._latest = None  # type: Optional[dict]
        self._received_at = 0.0
        self._cond = threading.Condition()
        self._sub = None
        self._sub_lock = threading.Lock()

    def _on_msg(self, data: dict) -> None:
        with self._cond:
            self._latest = data
            self._received_at = time.monotonic()
            self._cond.notify_all()

    @property
    def age(self) -> Optional[float]:
        """最新一条消息距今的秒数；还没收到过消息时为 None。"""
        with self._cond:
            return None if self._latest is None else time.monotonic() - self._received_at

    def _ensure_subscribed(self) -> None:
        with self._sub_lock:
            if self._sub is not None:
                return
            self._sub = self._dog._subscribe(self.topic, self.msg_type, self._on_msg, wrap=False)
            if self._sensor_id is None:
                return
            with self._cond:
                has_data = self._cond.wait_for(lambda: self._latest is not None, self._probe)
            if not has_data:
                self._dog._backend.sensor_start(self._sensor_id)

    def get(self, timeout: float = 3.0, max_age: Optional[float] = None) -> dict:
        """返回最新的一条消息（dict）。

        max_age 为 None 时，有缓存就直接返回；否则要求消息在 max_age 秒内收到，
        太旧的话会等新消息。超过 timeout 仍没有合格的消息就抛 SensorTimeout，不会返回过期数据。
        """
        self._dog._check()
        self._ensure_subscribed()
        deadline = time.monotonic() + timeout  # 打开传感器花的时间不算在 timeout 里

        def fresh():
            return self._latest is not None and (max_age is None or time.monotonic() - self._received_at <= max_age)

        with self._cond:
            while not fresh():
                left = deadline - time.monotonic()
                if left <= 0:
                    self._dog._check()
                    if self._latest is None:
                        raise SensorTimeout("%.1fs 内没有收到 %s 的消息" % (timeout, self.topic))
                    raise SensorTimeout("%s 最新的消息是 %.1fs 前收到的，超过 max_age=%.1fs"
                                        % (self.topic, time.monotonic() - self._received_at, max_age))
                self._cond.wait(left)
            return self._latest

    def on(self, callback: Callable[[dict], None]) -> int:
        """每收到一条消息就调用一次 callback(dict)。返回订阅 id，可传给 dog.unsubscribe()。"""
        if self._sensor_id is not None:
            self._dog._check()
            self._ensure_subscribed()
        return self._dog._subscribe(self.topic, self.msg_type, callback)


class Battery(TopicCache):
    def __init__(self, dog):
        super().__init__(dog, Iface.BMS)

    @property
    def soc(self) -> int:
        """电量百分比。"""
        return int(self.get()["batt_soc"])

    @property
    def voltage(self) -> float:
        """电压，单位 V（BmsStatus 里的原始值是 mV）。"""
        return self.get()["batt_volt"] / 1000.0

    @property
    def charging(self) -> bool:
        d = self.get()
        return bool(d.get("power_wired_charging") or d.get("power_wp_charging"))


class TouchEvent(object):
    def __init__(self, raw: dict):
        self.raw = raw
        self.state = int(raw["touch_state"])
        self.kind = TOUCH_KINDS.get(self.state)  # single / double / long；未知编码为 None

    def __repr__(self):
        return "TouchEvent(kind=%r, state=0x%02x)" % (self.kind, self.state)


class MotionState(TopicCache):
    def __init__(self, dog):
        super().__init__(dog, Iface.MOTION_STATUS)

    @property
    def motion_id(self) -> int:
        return int(self.get()["motion_id"])

    @property
    def switch_status(self) -> SwitchStatus:
        return SwitchStatus(int(self.get()["switch_status"]))


class Camera(object):
    """AI 相机。get_frame() 在相机没开时会自动打开；要指定分辨率就先调用 start()。"""

    def __init__(self, dog, topic: str = Iface.IMAGE[0]):
        self._dog = dog
        self.topic = topic
        self._publishing = False    # 库已经让相机开始发布图像
        self._auto_started = False  # 是 get_frame() 自动打开的，CyberDog 关闭时要关掉

    def _cmd(self, command: int, timeout: float = 10.0, cleanup: bool = False, **fields) -> dict:
        name, srv_type = Iface.CAMERA
        req = {"command": int(command), "args": "", "width": 0, "height": 0, "fps": 0}
        req.update(fields)
        if cleanup:
            # 收尾时绕过 _check()：回调崩溃后仍要能关掉相机，同时不重复抛出 CallbackError
            resp = self._dog._backend.call_service(name, srv_type, req, timeout=timeout)
        else:
            resp = self._dog._call(name, srv_type, req, timeout=timeout)
        if resp.get("result", 0) != 0:
            raise CommandFailed("相机命令 %s 失败: result=%s msg=%s" % (CameraCmd(command).name, resp.get("result"),
                                                                    resp.get("msg")), resp.get("code", 0), resp)
        if command == CameraCmd.START_IMAGE_PUBLISH:
            self._publishing = True
        elif command == CameraCmd.STOP_IMAGE_PUBLISH:
            self._publishing = False
            self._auto_started = False
        return resp

    def start(self, width: int = 640, height: int = 480, fps: int = 0) -> None:
        self._cmd(CameraCmd.START_IMAGE_PUBLISH, width=width, height=height, fps=fps)

    def stop(self) -> None:
        self._cmd(CameraCmd.STOP_IMAGE_PUBLISH)

    def take_picture(self) -> dict:
        """在狗端拍照，照片保存在狗的 /home/mi/Camera 目录。"""
        return self._cmd(CameraCmd.TAKE_PICTURE)

    def get_frame(self, timeout: float = 3.0, jpeg: Optional[bool] = None) -> Frame:
        """取最新一帧。jpeg 为 None 时由后端决定：远程连接默认压缩成 JPEG，本地默认不压缩。

        相机没有在发布时会自动以默认分辨率打开，CyberDog 关闭时再关掉。要指定分辨率请先调用 start()。
        """
        self._dog._check()
        if jpeg is None:
            jpeg = self._dog._remote
        backend = self._dog._backend
        if not self._publishing:
            try:
                return backend.latest_image(self.topic, timeout=PROBE_TIMEOUT, jpeg=jpeg)
            except SensorTimeout:
                self.start()
                self._auto_started = True
        return backend.latest_image(self.topic, timeout=timeout, jpeg=jpeg)

    def stream(self, width: int = 640, height: int = 480, fps: float = 15, quality: int = 70,
               jpeg: Optional[bool] = None, timeout: float = 5.0, start_camera: bool = True) -> "ImageStream":
        """打开连续视频流，用于在电脑上做实时视觉识别。

        fps      狗端限速，最多每秒推送多少帧（不超过 30）
        quality  JPEG 质量 1~100，越低越省带宽
        jpeg     是否在狗端压缩成 JPEG；为 None 时，远程连接压缩，本地运行不压缩
        timeout  read() 等待新帧的最长秒数，超时抛 SensorTimeout
        start_camera  为 True 时自动开启相机，关闭流时自动关闭相机
        """
        self._dog._check()
        if jpeg is None:
            jpeg = self._dog._remote
        return ImageStream(self, width, height, fps, quality, jpeg, timeout, start_camera)


class ImageStream(object):
    """连续视频流。用 camera.stream() 创建，推荐配合 with 使用：

        with dog.sensors.camera.stream(fps=15) as stream:
            for frame in stream:
                img = frame.to_bgr()

    电脑端只保留最新的一帧：识别代码比推流慢时，旧帧会被丢弃，画面不会越积越滞后。
    丢弃的帧数可以从 dropped 读取。超过 timeout 秒收不到新帧时抛 SensorTimeout。
    """

    def __init__(self, camera, width, height, fps, quality, jpeg, timeout, start_camera):
        if not 0 < fps <= 30:
            raise ValueError("fps 必须在 (0, 30] 之间，收到 %r" % fps)
        if not 1 <= quality <= 100:
            raise ValueError("quality 必须在 1~100 之间，收到 %r" % quality)
        self._camera = camera
        self._dog = camera._dog
        self.fps = fps
        self.timeout = timeout
        self._slot = LatestSlot()
        self._closed = False
        self._start_camera = start_camera
        if start_camera:
            camera.start(width, height)
        try:
            self._sid = self._dog._backend.image_stream_start(camera.topic, fps, jpeg, quality, self._slot.put)
        except BaseException:
            if start_camera:
                camera._cmd(CameraCmd.STOP_IMAGE_PUBLISH, cleanup=True)
            raise

    @property
    def received(self) -> int:
        """电脑端收到的帧数。"""
        return self._slot.received

    @property
    def dropped(self) -> int:
        """因为处理不过来，在电脑端被丢弃的帧数。"""
        return self._slot.dropped

    def read(self, timeout: Optional[float] = None) -> Frame:
        """阻塞到有一帧上次读取之后的新帧，返回这一帧。"""
        if self._closed:
            raise CyberDogError("视频流已关闭，不能再读取")
        timeout = self.timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while True:
            self._dog._check()
            frame = self._slot.get(min(0.1, max(0.0, deadline - time.monotonic())))
            if frame is not None:
                return frame
            if time.monotonic() >= deadline:
                self._dog._check()
                raise SensorTimeout("%.1fs 内没有收到新的图像帧（话题 %s）" % (timeout, self._camera.topic))

    def __iter__(self) -> Iterator[Frame]:
        while not self._closed:
            yield self.read()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if self._dog._backend.fatal_error is None:
                self._dog._backend.image_stream_stop(self._sid)
        finally:
            if self._start_camera and self._dog._backend.fatal_error is None:
                self._camera._cmd(CameraCmd.STOP_IMAGE_PUBLISH, cleanup=True)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.close()
        except Exception:
            if exc_type is None:
                raise
            traceback.print_exc()


class Imu(object):
    """从里程计算出的 IMU 信息。狗上默认没有 imu 话题，所以这里没有自己的订阅，共用 sensors.odom。

    只提供里程计里有的量：orientation 和 angular_velocity。里程计没有加速度，这里也不编造。
    """

    def __init__(self, odom):
        self._odom = odom

    @staticmethod
    def _from_odom(odom: dict) -> dict:
        return {"orientation": odom["pose"]["pose"]["orientation"], "angular_velocity": odom["twist"]["twist"]["angular"]}

    @property
    def age(self) -> Optional[float]:
        """最新一条里程计消息距今的秒数；还没收到过消息时为 None。"""
        return self._odom.age

    def get(self, timeout: float = 3.0, max_age: Optional[float] = None) -> dict:
        """返回 {"orientation": {x, y, z, w}, "angular_velocity": {x, y, z}}。"""
        return self._from_odom(self._odom.get(timeout, max_age))

    def on(self, callback: Callable[[dict], None]) -> int:
        """每收到一条里程计消息就调用一次 callback，参数与 get() 的返回值相同。返回订阅 id。"""
        return self._odom.on(lambda d: callback(self._from_odom(d)))


class Sensors(object):
    def __init__(self, dog):
        self._dog = dog
        self.battery = Battery(dog)
        self.motion_status = MotionState(dog)
        self.head_tof = TopicCache(dog, Iface.HEAD_TOF, SensorId.TOF)
        self.rear_tof = TopicCache(dog, Iface.REAR_TOF, SensorId.TOF)
        self.ultrasonic = TopicCache(dog, Iface.ULTRASONIC, SensorId.ULTRASONIC)
        self.gps = TopicCache(dog, Iface.GPS, SensorId.GPS, GPS_PROBE_TIMEOUT)
        self.uwb = TopicCache(dog, Iface.UWB)
        self.scan = TopicCache(dog, Iface.SCAN, SensorId.LIDAR)
        self.odom = TopicCache(dog, Iface.ODOM)
        self.imu = Imu(self.odom)
        self.camera = Camera(dog)

    def _release(self) -> None:
        """关闭本次会话自动打开的传感器和相机。每一项都会尝试，最后抛出第一个错误。"""
        try:
            self._dog._backend.sensors_stop_started()
        finally:
            if self.camera._auto_started:
                self.camera._cmd(CameraCmd.STOP_IMAGE_PUBLISH, cleanup=True)

    def on_touch(self, callback: Callable[[TouchEvent], None]) -> int:
        """头部触摸板事件回调，参数为 TouchEvent，kind 取值为 single、double 或 long。"""
        name, msg_type = Iface.TOUCH
        return self._dog._subscribe(name, msg_type, lambda d: callback(TouchEvent(d)))

    def ultrasonic_distance(self, timeout: float = 3.0) -> float:
        """前方超声测距，单位 m。"""
        return float(self.ultrasonic.get(timeout)["range"])

    def attitude(self, timeout: float = 3.0, max_age: Optional[float] = None) -> dict:
        """机身姿态，单位 rad，返回 {"roll", "pitch", "yaw"}（ROS 约定，ZYX 欧拉角）。

        roll 右侧下沉为正，pitch 低头为正，yaw 左转为正（相对值，不是指南针方向）。
        四元数无效（全 0、NaN、inf）时抛 ValueError。
        """
        q = self.imu.get(timeout, max_age)["orientation"]
        x, y, z, w = q["x"], q["y"], q["z"], q["w"]
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not (math.isfinite(norm) and norm > 0):
            raise ValueError("IMU 四元数无效: %s" % (q,))
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        # 只防归一化后的浮点误差让 asin 越界，输入本身已在上面检查
        sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        return {"roll": math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
                "pitch": math.asin(sin_pitch),
                "yaw": math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))}

    def tof(self, timeout: float = 3.0) -> dict:
        """四个 TOF 的 8×8 距离矩阵（单位 m），以 dict 返回：left_head、right_head、left_rear、right_rear。"""
        head = self.head_tof.get(timeout)
        rear = self.rear_tof.get(timeout)
        out = {}
        for key, src in (("left_head", head), ("right_head", head), ("left_rear", rear), ("right_rear", rear)):
            data = list(src[key]["data"])
            if len(data) != 64:
                raise ValueError("%s 的数据长度为 %d，不是 64" % (key, len(data)))
            out[key] = [data[r * 8:(r + 1) * 8] for r in range(8)]
        return out

# cyberdog2：小米 CyberDog 2 Python 开发库

在自己的电脑上（Windows、macOS、Linux 都可以）用几行 Python 控制铁蛋二代。**电脑上不需要安装 ROS。**

```python
from cyberdog2 import CyberDog

with CyberDog.connect("192.168.31.50", password="123") as dog:
    dog.motion.stand()
    dog.motion.walk(vx=0.3, duration=2)
    dog.led.set("head", "blue", "breath")
    dog.audio.say("你好")
```

> 完整接口文档与交接说明见 [docs/API.md](docs/API.md)。

## 工作原理
```
你的电脑                                     CyberDog 2
CyberDog API → RemoteBackend ══ SSH 隧道 ══> 狗端代理(127.0.0.1) → rclpy → motion_manager 等
```
`connect()` 会自动完成以下几步：
1. 通过 SSH 登录狗。
2. 把本库上传到 `~/.cyberdog2/<版本哈希>/`。代码有改动才会重新上传。
3. 在狗上启动代理。代理会继承 `motion_manager` 进程的 ROS 环境变量，并自动探测 `/mi_xxx` 命名空间。
4. 打开 SSH 隧道，把电脑和代理连起来。

代理只监听狗的本机回环地址，不对外开放端口。断开连接时代理自动退出。

**安全机制**
- 行走时的 20Hz 心跳由狗端代理发送。
- 电脑每 200ms 给代理发一次 ping。代理超过 1 秒没收到任何消息（比如断网、程序卡死），会立即停止行走并断开连接。

## 安装
```bash
pip install paramiko            # 远程连接需要
pip install numpy pillow        # 可选：图像转 numpy、保存图片
```
把 `cyberdog2py` 目录放进你的项目，或者执行 `pip install -e .[remote]`。

## 三种运行方式
| 方式 | 用途 |
|---|---|
| `CyberDog.connect(ip, password=...)` | 电脑远程控制（推荐） |
| `CyberDog.mock()` | **没有真机时开发和调试**。`dog.backend` 可以注入事件、检查发出的指令 |
| `CyberDog.local()` | 程序直接运行在狗上 |

## 错误处理：let it crash
本库不会悄悄吞掉错误，出问题就直接抛异常：
- 动作被拒绝时抛 `MotionRejected`；服务超时抛 `ServiceTimeout`；服务不存在抛 `ServiceUnavailable`。
- 参数越界时抛 `ValueError`，**不会自动截断**。例如中速步态 `vx=1.2` 会直接报错。
- 回调函数里抛出异常时，库会立即停止运动，下一次调用任何 API 时抛 `CallbackError`，原始异常保存在 `__cause__` 中。
- 连接断开后，下一次调用抛 `ConnectionLost`，异常信息里附带狗端代理的输出。
- 无论因为什么原因退出（`with` 块结束、异常、Ctrl+C），都会先停止运动。

## API 速查
### 运动 `dog.motion`
| 调用 | 说明 |
|---|---|
| `stand()` / `get_down()` / `sit()` / `shake_hand(left=False)` | 常用动作 |
| `do("ballet")` / `do(MotionId.BACKFLIP)` / `do(143)` | 任意动作，完整列表见 `MotionId` |
| `do("stand", rpy=[0, 0.2, 0])` | 带姿态参数的动作（rpy 每个分量上限 0.4 rad） |
| `walk(vx, vy, wz, duration, gait="medium")` | 走一段时间后自动停下 |
| `turn(wz, duration)` | 原地转向 |
| `set_velocity(vx, vy, wz)` + `stop()` | 持续行走，用于实时控制 |
| `estop()` | 急停 |

各步态速度上限（vx, vy, wz）：
- slow：0.65, 0.3, 1.25
- medium：1.0, 0.3, 1.25
- fast：1.6, 0.55, 2.5

### 传感器 `dog.sensors`
雷达、超声波、TOF、GPS 和相机取帧在第一次读取时，如果传感器没开会自动打开，退出时只关库自己打开的。

- `battery.soc` / `.voltage` / `.charging`
- `on_touch(cb)`：`cb` 收到的事件里，`ev.kind` 取值为 `single`、`double`、`long`
- `tof()`：4 个 8×8 距离矩阵，单位 m
- `ultrasonic_distance()`
- `gps.get()` / `uwb.get()` / `motion_status.switch_status`
- `camera.start()`、`camera.get_frame()`（返回 Frame，可调用 `.save("a.jpg")` 或 `.to_numpy()`）、`camera.stop()`
- `camera.stream(fps=15, quality=70)`：连续视频流，用于实时视觉识别，见下一节

### 视频流（实时视觉识别）
```python
with dog.sensors.camera.stream(width=640, height=480, fps=15, quality=70) as stream:
    for frame in stream:
        img = frame.to_bgr()        # OpenCV 格式的 BGR 数组，需要 opencv-python
        ...                         # 在这里写你的识别代码
        print(stream.dropped)       # 因为处理不过来而丢弃的帧数
```
- 狗端订阅相机话题，按 `fps` 限速、压缩成 JPEG 后主动推给电脑。电脑不需要逐帧请求。
- 狗端和电脑端都**只保留最新一帧**：识别比推流慢时会丢掉旧帧，画面不会越积越滞后。
- 默认自动开启相机，退出 `with` 时自动关闭相机。
- 超过 `timeout`（默认 5 秒）收不到新帧时抛 `SensorTimeout`；连接断开时抛 `ConnectionLost`。

### 灯效 `dog.led`
- `set("head" | "tail", "red" | "blue" | "yellow" | "off", "on" | "blink" | "breath" | ...)`
- `set("tail", (r, g, b), "trailing_race")`
- `set("mini", None, "three_circular")`
- `release(target)`：把灯还给系统控制

### 语音 `dog.audio`
- `say("文字")`：在线 TTS，需要联网并登录小米账号
- `play(play_id)`：离线播放内置语音
- `volume`：读写音量

### 视觉 `dog.vision`
- `enable("face", "body", ...)` 配合 `on_person(cb)`
- `start_gesture()` 配合 `on_gesture(cb)`，`ev.name` 例如 `thumb_up`
- `recognize_face()` 配合 `on_face(cb)`

### 导航 `dog.nav`
- `follow_person(distance=1.2)`、`follow_object(roi)`、`follow_uwb()`：返回一个任务对象，有 `.wait()` 和 `.cancel()`
- `start_mapping()` / `stop_mapping("map")` / `start_localization()` / `go_to(x, y, yaw)`
- `stop()`

### 其他
- `dog.list_names()`：列出狗上所有的话题、服务和动作
- `python -m cyberdog2.doctor <ip>`：连通性自检

## 回调的写法
回调函数在后台线程里执行。**不要在回调里直接发运动指令**，否则会和主线程的指令互相干扰。正确的做法是把事件放进队列，在主线程里处理。参考 `examples/03_touch_reaction.py`。

## 示例
```bash
python examples/01_basic_motion.py --mock                          # 用模拟狗运行
set CYBERDOG_PASSWORD=123
python examples/01_basic_motion.py 192.168.31.50                   # 在真机上运行
```
| 文件 | 内容 |
|---|---|
| 01_basic_motion | 站立、坐下、握手、趴下 |
| 02_walk_square | 走正方形 |
| 03_touch_reaction | 摸头互动 |
| 04_led_and_voice | 灯效和语音 |
| 05_follow_person | 人体跟随 |
| 06_camera | 拍一帧图像 |
| 07_video_stream | 视频流 + 实时人脸检测（需要 opencv-python） |
| 08_http_live | 把狗的视角转成 HTTP MJPEG 直播，VLC 或浏览器打开 `http://127.0.0.1:8080/stream.mjpg` |

## 测试
```bash
pip install pytest
python -m pytest
```
所有测试都基于模拟后端，覆盖以下内容：
- 指令字段
- 心跳间隔小于 200ms，结束时发送 END
- 参数越界时报错
- 回调崩溃后停止运动
- 代理与客户端的端到端通信
- 断线和看门狗触发时自动停止
- 狗端代码可在 Python 3.6 上运行（实测机器为 Ubuntu 18.04 + Galactic，系统 python3 是 3.6）

## 真机验证清单（拿到狗后按顺序执行）
1. 按 [docs/first_boot.md](docs/first_boot.md) 开通开发者权限，然后运行 `python -m cyberdog2.doctor <ip>`，确认全部通过。
2. 运行 `01_basic_motion`，确认站立和趴下正常。
3. 抬着狗或在空地上，用 `walk(vx=0.2, duration=1)` 走 1 秒后停下。
4. 让狗持续行走时关掉电脑的 WiFi，狗应在约 1 秒内停下。
5. 运行 `04_led_and_voice`、`03_touch_reaction` 和 `06_camera`。

## 接口来源与尚未核实的部分
接口定义来自 [MiRoboticsLab/bridges](https://github.com/MiRoboticsLab/bridges/tree/rolling/protocol/ros) 和 [官方开发者手册](https://miroboticslab.github.io/blogs/#/cn/developer_guide)。以下几项手册里没有写明，需要在真机上核实，接口名都集中写在 `cyberdog2/constants.py` 的 `Iface` 里：
- 运动状态话题 `motion_status`
- 相机图像话题 `image`
- 狗上 ROS 环境脚本的路径
- `motion_manager` 的进程名

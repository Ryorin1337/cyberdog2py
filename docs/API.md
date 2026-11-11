# cyberdog2 API 参考与交接文档

> 适用版本：`cyberdog2` 0.1.0（`cyberdog2/__init__.py` 中的 `__version__`）
>
> 本文档面向**从没接触过本项目**的接手同学：读完以后应该能用、能维护、能扩展这个库。
> 所有签名、默认值、接口名均以源码为准；如果以后源码改了而本文没改，**以源码为准**，并顺手更新本文。
>
> **重要前提：本库至今没有在真机上跑过。** 模拟后端和"代理 ⇄ 客户端"的通信链路有完整的自动化测试；
> 但 ROS 后端（`backends/ros.py`）和 SSH 部署（`deploy.py`）没有经过真机验证。
> 文中凡是标注 **【未核实】** 的地方，都需要拿到狗之后逐项确认，清单见第 12 节。

---

## 目录

1. [概述与架构](#1-概述与架构)
2. [安装与依赖](#2-安装与依赖)
3. [快速开始](#3-快速开始)
4. [API 参考](#4-api-参考)
5. [常量参考](#5-常量参考)
6. [异常参考](#6-异常参考)
7. [远程连接细节](#7-远程连接细节)
8. [代理通信协议](#8-代理通信协议protocolpy)
9. [doctor 自检工具](#9-doctor-自检工具)
10. [扩展指南](#10-扩展指南)
11. [测试说明](#11-测试说明)
12. [已知限制与待真机核实事项](#12-已知限制与待真机核实事项)
13. [示例索引](#13-示例索引)

---

## 1. 概述与架构

### 1.1 这个库是做什么的

`cyberdog2` 是控制小米 CyberDog 2（铁蛋二代）的 Python 库。CyberDog 2 的所有功能（运动、灯、语音、传感器、相机、视觉算法、导航）
都以 ROS 2（Galactic）的话题 / 服务 / 动作形式暴露在狗上的 `/mi_xxx` 命名空间下。

本库的目标是：**电脑上不装 ROS**，用几行 Python 就能控制狗。

```python
from cyberdog2 import CyberDog

with CyberDog.connect("192.168.31.50", password="123") as dog:
    dog.motion.stand()
    dog.motion.walk(vx=0.3, duration=2)
```

### 1.2 整体架构

```
 ┌──────────────────────────── 你的电脑（Windows / macOS / Linux，无需 ROS） ─────────────────────────────┐
 │                                                                                                    │
 │  用户代码                                                                                            │
 │    │                                                                                               │
 │    ▼                                                                                               │
 │  CyberDog（dog.py，门面）                                                                            │
 │    ├── motion  (Motion)        ├── led    (Led)        ├── vision (Vision)                         │
 │    ├── sensors (Sensors)       ├── audio  (Audio)      └── nav    (Navigation)                     │
 │    │                                                                                               │
 │    ▼  统一调用 Backend 接口（backends/base.py），消息全部是 dict                                        │
 │  RemoteBackend（backends/remote.py）                                                                │
 │    线程：cyberdog2-reader（收消息） / cyberdog2-ping（每 0.2s ping） / cyberdog2-callbacks（执行回调）   │
 │    │                                                                                               │
 │    │  JSON 行协议（protocol.py），走 paramiko 的 direct-tcpip 通道                                     │
 └────┼───────────────────────────────────────────────────────────────────────────────────────────────┘
      │  SSH 隧道（deploy.py: SshSession.open_tunnel）
 ┌────┼────────────────────────────────── CyberDog 2（Ubuntu + ROS 2 Galactic） ─────────────────────────┐
 │    ▼                                                                                               │
 │  狗端代理 python3 -m cyberdog2.agent（agent/server.py: AgentServer，只监听 127.0.0.1）                 │
 │    线程：主循环收消息 / 看门狗 / 8 线程请求池 / 每路视频流一个发送线程                                     │
 │    │                                                                                               │
 │    ▼                                                                                               │
 │  RosBackend（backends/ros.py，rclpy）                                                                │
 │    ServoLoop（backends/servo.py）：20Hz 伺服心跳在这里发                                               │
 │    │                                                                                               │
 │    ▼  /mi_xxx/motion_result_cmd、/mi_xxx/motion_servo_cmd、/mi_xxx/led_execute ...                  │
 │  狗上的系统节点：motion_manager、led、audio、camera、algo_manager、导航 ...                             │
 └────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

### 1.3 三种后端

`CyberDog` 本身不碰 ROS，它只调用 `Backend` 抽象接口（`backends/base.py`）。有三个实现：

| 工厂方法 | 后端类 | 运行位置 | 依赖 | 用途 |
|---|---|---|---|---|
| `CyberDog.connect(host, ...)` | `RemoteBackend` | 电脑 | `paramiko` | 推荐。SSH 登录狗，自动部署并启动代理，经 SSH 隧道通信 |
| `CyberDog.local(...)` | `RosBackend` | 狗本机 | 狗上的 `rclpy`、`protocol` 消息包 | 程序直接跑在狗上 |
| `CyberDog.mock(...)` | `MockBackend` | 任意 | 无 | 没有真机时开发、调试、单元测试 |

狗端代理内部用的也是 `RosBackend`（调试代理本身时可以用 `--mock` 换成 `MockBackend`）。

### 1.4 数据流

以 `dog.motion.sit()` 为例（远程模式）：

1. `Motion.do("sit")` → 解析成 `motion_id=143`，组装请求 dict，用 `fill_motion_arrays` 把数组字段补零到固定长度。
2. `CyberDog._call()` → 先 `_check()`（检查已关闭 / 回调崩溃 / 后端致命错误），再调 `backend.call_service("motion_result_cmd", "protocol/srv/MotionResultCmd", req, timeout=30.0)`。
3. `RemoteBackend._request("call_service", ...)` → 发送 `{"id": N, "op": "call_service", "args": {...}}\n`，在本地等待响应，最多等 `timeout + 5` 秒。
4. 代理 `AgentServer._handle` 在线程池中执行 → `RosBackend.call_service` → 拼上命名空间得到 `/mi_xxx/motion_result_cmd`，用 rclpy 调服务。
5. 响应转换成 dict（`to_plain`），回传 `{"id": N, "ok": true, "result": {...}}`；出错则回传 `{"ok": false, "error": {...}}`，客户端用 `errors.rebuild` 还原成同名异常再抛出。
6. `Motion.do` 检查 `result` 字段，为 false 时抛 `MotionRejected`。

伺服（持续行走）不同：`set_velocity()` 只把"目标指令"发给代理（`servo_set`），**20Hz 心跳由代理端的 `ServoLoop` 发出**，
这样网络抖动不会让心跳间隔超过 motion_manager 要求的 200ms。

订阅数据流：代理收到 ROS 消息 → 推送事件 `{"ev": "sub", "sub": id, "data": {...}}` → 客户端 reader 线程放进回调队列 → `cyberdog2-callbacks` 线程执行用户回调。

### 1.5 设计原则：let it crash

- 出错**立即抛异常**：不静默兜底、不自动重试、**不截断参数**（例如中速步态 `vx=1.2` 直接 `ValueError`，而不是悄悄改成 1.0）。
- 用户回调抛异常时：异常被记录，立即停止伺服（`servo_stop`），之后**下一次调用任何 API** 都抛 `CallbackError`（原始异常在 `__cause__`）。此后该回调层不会再调用其他用户回调。
- 后台线程出致命错误（连接断开、ROS 执行器崩溃、伺服发布失败）时，下一次 API 调用抛出该错误（通常是 `ConnectionLost`）。
- 无论怎么退出（`with` 结束、异常、Ctrl+C、解释器退出时的 `atexit`），都会先尝试停止运动；本次会话启动过导航 / 跟随任务时还会按狗的实际状态结束任务（`nav.release()`）。
- 安全收尾仍会执行：`close()`、`walk()` 的 `finally`、`ImageStream.close()`、`motion.estop()`、代理断线后的 `_cleanup()` 都会尽力停止伺服 / 推流 / 导航任务（即使回调崩溃过）。
- 死人开关：`set_velocity(timeout=...)` 指定的时间内没有再次调用，伺服自动发送 END 停下；`walk()` / `turn()` 默认带 `duration + 1` 秒的死人开关。

### 1.6 线程模型（哪些回调在哪个线程执行）

| 回调 / 处理函数 | mock | remote（`connect`） | local（`local`） |
|---|---|---|---|
| `on_touch`、`on_person`、`on_gesture`、`on_face`、`nav.on_status`、`TopicCache.on` 的用户回调 | 调用 `backend.emit()` 的那个线程（同步）；订阅 latched 话题时，**在 `subscribe` 调用线程里立即同步回调一次** | `cyberdog2-callbacks` 单线程，按到达顺序**串行**执行 | rclpy `MultiThreadedExecutor`（4 线程，`ReentrantCallbackGroup`），**可能并发** |
| 导航 `feedback` 回调 | 不会被调用（mock 只把它存在 `goals[goal_id]["feedback_cb"]`） | `cyberdog2-callbacks`（`send_goal` 返回前到达的反馈先缓存，返回时补发；`wait_goal` 返回后释放回调） | rclpy 执行器线程 |
| 视频流帧（`ImageStream` 内部写入 `LatestSlot`） | `mock-stream` 线程 | `cyberdog2-reader` 线程（不经回调队列） | rclpy 执行器线程 |
| 伺服心跳 | `cyberdog2-servo` | 代理端的 `cyberdog2-servo` | `cyberdog2-servo` |
| 心跳 ping | — | `cyberdog2-ping` | — |

**规则：**

1. **不要在回调里发运动指令**（`motion.*`、`nav.*` 等）。回调在后台线程里执行，会和主线程的指令互相干扰；在 remote 模式下回调是串行的，一个阻塞的回调还会卡住后面所有回调。正确做法是回调只把事件放进 `queue.Queue`，主线程取出后再处理（见 `examples/03_touch_reaction.py`）。
2. 回调要**快速返回**。
3. local 模式下回调可能并发，回调里访问共享数据要自己加锁。
4. 回调里抛异常 = 停止运动 + 后续 API 抛 `CallbackError`。这是故意的。

---

## 2. 安装与依赖

### 2.1 电脑端

- Python ≥ 3.6（`pyproject.toml`: `requires-python = ">=3.6"`）。
- 库本身**没有强制依赖**（`dependencies = []`），按需安装可选依赖：

| extra | 包 | 用途 |
|---|---|---|
| `remote` | `paramiko>=2.7` | `CyberDog.connect()`、`cyberdog2.doctor` 必需 |
| `image` | `numpy`、`pillow` | `Frame.to_numpy()`、`Frame.save()`（raw 帧）、解码 JPEG 到 numpy |
| `dev` | `pytest`、`paramiko>=2.7`、`numpy`、`pillow` | 跑测试 |
| （未列入 extra） | `opencv-python` | `Frame.to_bgr()`、`examples/07_video_stream.py` |

```bash
# 在仓库根目录（含 pyproject.toml）执行
pip install -e .[remote]          # 远程控制
pip install -e .[remote,image]    # 再加上图像处理
pip install -e .[dev]             # 开发 / 跑测试
pip install opencv-python         # 需要 to_bgr() 时
```

也可以不安装，直接把 `cyberdog2` 目录放进项目（示例和测试就是用 `sys.path.insert` 的方式导入的）。

### 2.2 狗端

**狗上不需要手动安装任何东西。** `connect()` 会把电脑上 `cyberdog2/` 包里所有 `.py` 文件通过 SFTP 上传到狗的 `~/.cyberdog2/<哈希>/` 目录。代理运行时需要狗上已有的：

- 一个能 `import rclpy` 的 Python 解释器，默认用 `python3`，可以通过 `connect(..., python="python3.8")` 或 doctor 的 `--python` 指定。
  **实测机器**：Ubuntu 18.04（lubuntu）、ROS 2 Galactic、命名空间 `/mi_desktop_48_b0_2d_7a_fb_a9`，系统自带的 `python3` 是 3.6。所以上传的代码必须兼容 **Python 3.6**（见 11.1 节）。
  rclpy 实际是用哪个 Python 版本编译的，以 doctor 中"导入 rclpy"这一步打印的结果为准。
- `rclpy`、`rosidl_runtime_py`、`protocol`（CyberDog 的消息包）、`sensor_msgs`
- `Pillow`（只有远程取图 / 推流要压缩 JPEG 时才需要）。实测狗上有 `python3-pil 5.1.0`，**没有 `cv2`**（只有 C++ 的 libopencv），所以狗端代码不能依赖 `cv2`；狗上的 `python3 -m pip` 也是坏的，不要指望在狗上 pip 安装
- `bash`、`pgrep`、`tr`、`grep`、`sed`（代理启动脚本用到）

狗必须开通**开发者权限**才能 SSH 登录，见 `docs/first_boot.md`。

### 2.3 延迟导入

- `paramiko` 只在 `deploy.SshSession` 内部 import；不用 `connect()` 就不需要它。
- `rclpy` 等 ROS 包只在 `RosBackend` 内部 import；电脑上 `import cyberdog2` 不会失败。
- `numpy`、`PIL`、`cv2` 只在 `Frame` 的转换方法里 import，`encode_jpeg` 只用 `PIL`；缺失时调用才抛 `ImportError`。

---

## 3. 快速开始

### 3.1 远程连接真机（`connect`）

```python
from cyberdog2 import CyberDog

with CyberDog.connect("192.168.31.50", user="mi", password="123") as dog:
    print("命名空间:", dog.namespace)
    print("电量:", dog.sensors.battery.soc, "%")
    dog.motion.stand()
    dog.motion.walk(vx=0.3, duration=2)        # 走 2 秒后自动停下
    dog.led.set("head", "blue", "breath")
    dog.audio.say("你好")
    dog.motion.get_down()
# 退出 with：停止伺服 → 关闭隧道 → 代理退出 → 关闭 SSH
```

第一次运行前建议先跑自检：`python -m cyberdog2.doctor 192.168.31.50`（见第 9 节）。

### 3.2 模拟狗（`mock`）

```python
from cyberdog2 import CyberDog, MotionRejected

dog = CyberDog.mock()
dog.motion.sit()
print(dog.backend.calls_to("motion_result_cmd")[-1]["motion_id"])   # 143

# 模拟一次头部单击
dog.sensors.on_touch(lambda ev: print("收到触摸:", ev.kind))
dog.backend.emit("touch_status", {"touch_state": 0x01, "timestamp": 0})   # 打印：收到触摸: single

# 模拟动作被拒绝
dog.backend.set_response("motion_result_cmd", {"motion_id": 121, "result": False, "code": 3})
try:
    dog.motion.do("backflip")
except MotionRejected as e:
    print("被拒绝, code =", e.code)

dog.close()
```

### 3.3 直接在狗上运行（`local`）

把代码拷到狗上，在已经 `source` 过 ROS 环境、且 DDS 设置（`ROS_DOMAIN_ID` 等）与狗的系统节点一致的终端里运行：

```python
from cyberdog2 import CyberDog

with CyberDog.local() as dog:               # 自动探测 /mi_xxx 命名空间，最多等 5 秒
    dog.motion.stand()
    dog.motion.turn(wz=0.5, duration=2)
```

> local 模式没有代理和看门狗：程序卡死时伺服心跳线程仍可能在发指令（它是 daemon 线程，进程退出时才停）。
> 持续行走时建议用 `set_velocity(..., timeout=0.5)` 这类死人开关（`walk()` / `turn()` 默认已带），超时没有刷新就自动停下。

---

## 4. API 参考

约定：

- "ROS 接口"一列写的是相对名（不带命名空间），后端会拼成 `/mi_xxx/<name>`。以 `/` 开头的名字原样使用。
- 所有经过 `CyberDog._check()` 的方法，在以下情况下会先抛异常：dog 已关闭（`CyberDogError("CyberDog 已关闭")`）、回调崩溃过（`CallbackError`）、后端有致命错误（通常是 `ConnectionLost`）。下文把这组异常简称为 **「通用前置异常」**。
- 所有服务调用都可能抛：`ServiceUnavailable`（服务在超时内没出现）、`ServiceTimeout`（服务没在超时内返回；远程模式下也可能是代理没在 `timeout+5` 秒内响应）。下文简称 **「通用服务异常」**。
- 远程模式下狗端发生的 `AttributeError`、`ImportError` 等非本库异常会以 `RemoteError` 抛出（见第 6 节）。

### 4.1 `CyberDog`（`cyberdog2/dog.py`）

```python
from cyberdog2 import CyberDog
```

#### 构造函数

```python
CyberDog(backend: Backend, cmd_source: CmdSource = CmdSource.APP, remote: bool = False)
```

一般不直接调用，用下面三个工厂方法。直接构造的场景：测试里手工组装后端（`tests/conftest.py`、`doctor.py`）。

| 参数 | 说明 |
|---|---|
| `backend` | 任意 `Backend` 实现 |
| `cmd_source` | 运动指令的来源，写入 `MotionResultCmd` / `MotionServoCmd` 的 `cmd_source` 字段。默认 `CmdSource.APP`（优先级最高）。构造后可以改 `dog.cmd_source` |
| `remote` | 是否远程连接。只影响相机取图时 `jpeg=None` 的默认值（远程默认压缩成 JPEG） |

公开属性：`motion`（`Motion`）、`sensors`（`Sensors`）、`led`（`Led`）、`audio`（`Audio`）、`vision`（`Vision`）、`nav`（`Navigation`）、`cmd_source`。

构造时会 `atexit.register` 一个弱引用回调：解释器退出时如果 dog 还没关闭，会自动 `close()`。

#### `CyberDog.connect`

```python
@classmethod
connect(cls, host: str, user: str = "mi", password: Optional[str] = None, *,
        key_file: Optional[str] = None, port: int = 22, namespace: Optional[str] = None,
        **agent_options) -> "CyberDog"
```

SSH 登录狗、部署并启动狗端代理、打开隧道，返回 `remote=True` 的 `CyberDog`。

| 参数 | 说明 |
|---|---|
| `host` | 狗的 IP 或主机名 |
| `user` | SSH 用户名，默认 `mi` |
| `password` | SSH 密码（出厂默认 `123`，见 `docs/first_boot.md`）。与 `key_file` 都不传时，paramiko 会尝试 ssh-agent 和 `~/.ssh` 下的密钥 |
| `key_file` | 私钥路径（仅关键字） |
| `port` | SSH 端口，默认 22（仅关键字） |
| `namespace` | 手动指定命名空间，例如 `"/mi_desktop_48_b0_2d_7b_05_1d"`；`None` 时由代理自动探测（仅关键字） |
| `**agent_options` | 原样传给 `RemoteBackend.connect_ssh`：`ssh_timeout` 由它自己取出（SSH 连接超时），其余传给 `SshSession.start_agent`：`setup_scripts`、`inherit_env_from`、`env`、`start_timeout`、`watchdog`、`discovery_timeout`、`accept_timeout`、`python`。详见第 7.1 节 |

异常：

- paramiko 的异常（`paramiko.AuthenticationException`、`socket.timeout`、`NoValidConnectionsError` 等）原样抛出——SSH 登录失败。
- `AgentStartError`：拿不到 HOME、创建目录失败、清理旧版本目录失败、代理启动失败或超时（异常信息附带代理输出）。
- `ServiceTimeout` / `ConnectionLost`：隧道建立后 `info` 请求失败。
- `CyberDogError`：代理返回的 `protocol` 版本与本地 `PROTOCOL_VERSION` 不一致（狗端运行的不是同一版本的代码）。
- `ImportError`：没装 paramiko。
- `TypeError`：`agent_options` 里有不认识的参数名。
- `ValueError`：`env` 里有非法的环境变量名。

```python
dog = CyberDog.connect("192.168.31.50", password="123",
                       inherit_env_from=None, env={"ROS_DOMAIN_ID": "42"})
```

#### `CyberDog.local`

```python
@classmethod
local(cls, namespace: Optional[str] = None, discovery_timeout: float = 5.0) -> "CyberDog"
```

在狗本机用 `RosBackend`。`namespace` 为 `None` 时，在 `discovery_timeout` 秒内反复扫描 ROS 图找唯一的 `/mi_*` 命名空间。

异常：`NamespaceNotFound`（找不到或找到多个）、`ImportError`（没有 rclpy）、rclpy 初始化相关异常。

#### `CyberDog.mock`

```python
@classmethod
mock(cls, **kw) -> "CyberDog"
```

`kw` 原样传给 `MockBackend(namespace: str = "/mi_mock", servo_period: float = 0.05)`。用 `dog.backend` 拿到 `MockBackend` 对象，见 4.13 节。

#### 属性与方法

| 成员 | 签名 | 说明 |
|---|---|---|
| `backend` | 属性 → `Backend` | 底层后端对象。mock 时可用于注入事件、检查指令 |
| `namespace` | 属性 → `str` | 当前命名空间，例如 `/mi_desktop_xx`；mock 为 `/mi_mock` |
| `list_names` | `list_names(self) -> Dict[str, list]` | 返回 `{"topics": [...], "services": [...], "actions": [...]}`，都是完整名称、已排序。会先 `_check()` |
| `unsubscribe` | `unsubscribe(self, sub_id: int) -> None` | 取消 `on_*` / `TopicCache.on` 返回的订阅。会先 `_check()`。未知 id 抛 `KeyError` |
| `close` | `close(self) -> None` | 停止运动并断开连接，可重复调用。若后端没有致命错误：先 `servo_stop()`；然后（无论成功与否）如果 `nav.started` 为真，调用 `nav.release()`（绕过 `_check()`，回调崩溃后也会执行）；最后（无论成功与否）`backend.close()`。停止导航失败时 `close()` 会抛出该异常（如 `CommandFailed`、`ServiceUnavailable`） |
| 上下文管理 | `__enter__` / `__exit__` | `with` 结束时调用 `close()`。如果 `with` 块本身已经在抛异常，`close()` 的异常只打印不抛出，避免盖掉原始异常 |

```python
names = dog.list_names()
print([s for s in names["services"] if "motion" in s])
```

### 4.2 `Motion`（`dog.motion`，`cyberdog2/motion.py`）

运动分两类：

- **结果指令**（`do` 及其快捷方法）：一个完整动作，调用服务 `motion_result_cmd`，阻塞到 motion_manager 返回结果。
- **伺服指令**（`set_velocity` / `walk` / `turn` / `stop`）：持续行走，发布话题 `motion_servo_cmd`，后端以 20Hz 心跳持续下发。

| 方法 | ROS 接口 | 类型 |
|---|---|---|
| `do` / `stand` / `get_down` / `lie_down` / `sit` / `shake_hand` / `estop` | `motion_result_cmd` | 服务 `protocol/srv/MotionResultCmd` |
| `set_velocity` / `walk` / `turn` / `stop` | `motion_servo_cmd` | 话题 `protocol/msg/MotionServoCmd` |

#### 模块函数 `resolve_motion`

```python
resolve_motion(action: ActionLike) -> int        # ActionLike = Union[str, int, MotionId]
```

把 `"sit"`（大小写不敏感、去首尾空格，按 `MotionId` 成员名匹配）、`MotionId.SIT`、`143` 统一转成 int。
未知字符串 → `ValueError`；`bool` 或非 int 类型 → `TypeError`。**整数不做范围检查**，直接透传给狗。

#### `do`

```python
do(self, action: ActionLike, *, rpy: Optional[Sequence[float]] = None,
   pos: Optional[Sequence[float]] = None, step_height: Optional[Sequence[float]] = None,
   duration: int = 0, value: int = 0, timeout: float = 30.0) -> dict
```

| 参数 | 单位 / 范围 | 说明 |
|---|---|---|
| `action` | — | 动作名 / `MotionId` / 整数。不能是伺服步态（301/302/303/305/308），否则 `ValueError` |
| `rpy` | rad，3 个分量，每个 \|x\| ≤ 0.4（`RPY_LIMIT`） | 姿态，写入 `rpy_des`；只对支持姿态参数的动作生效 |
| `pos` | m，3 个分量，上限分别 0.2 / 0.2 / 0.3（`POS_LIMIT`） | 位置，写入 `pos_des` |
| `step_height` | m，最多 2 个元素 | 写入 `step_height`，**不做范围检查** |
| `duration` | int | 写入 `duration`，含义见 MotionResultCmd 定义 |
| `value` | int | 写入 `value` |
| `timeout` | s | 服务调用超时 |

实际下发的请求：`{"motion_id", "cmd_source", "duration", "value", ["rpy_des"], ["pos_des"], ["step_height"]}`，再经
`fill_motion_arrays` 把 `vel_des(3) rpy_des(3) pos_des(3) acc_des(6) ctrl_point(3) foot_pose(6) step_height(2)` 全部补零到固定长度。

返回：服务响应 dict（mock 为 `{"motion_id": ..., "result": True, "code": 0}`）。

异常：

- `ValueError`：未知动作名、伺服步态 id、`rpy`/`pos` 个数不对或越界、数组超长。
- `TypeError`：`action` 类型不对。
- `MotionRejected`：响应 `result` 为假；`e.code` 是响应的 `code`，`e.response` 是完整响应。
- 通用前置异常、通用服务异常。

```python
from cyberdog2 import MotionId
dog.motion.do("ballet")
dog.motion.do(MotionId.BACKFLIP)
dog.motion.do(143)
dog.motion.do("stand", rpy=[0, 0.2, 0])     # 抬头 0.2 rad
```

#### 快捷动作

| 方法 | 等价于 |
|---|---|
| `stand(self, **kw) -> dict` | `do(MotionId.STAND, **kw)`（111） |
| `get_down(self, **kw) -> dict` | `do(MotionId.GET_DOWN, **kw)`（101） |
| `lie_down` | `get_down` 的别名 |
| `sit(self, **kw) -> dict` | `do(MotionId.SIT, **kw)`（143） |
| `shake_hand(self, left: bool = False, **kw) -> dict` | `left=True` → 141 握左手；否则 142 握右手 |

`**kw` 就是 `do` 的关键字参数。

#### `estop`

```python
estop(self) -> dict
```

急停：先 `servo_stop()` 结束伺服，再（`finally` 里，即使停伺服失败也会执行）下发 motion_id 0，服务超时 5 秒。

- 这是安全指令，**不经过 `_check()`**（内部用 `_send_result(check=False)` 直接调后端），所以回调崩溃过（其他 API 都抛 `CallbackError`）时仍会下发。
- 只检查 dog 是否已关闭（`_ensure_open()`，已关闭抛 `CyberDogError`）。后端已有致命错误（如 `ConnectionLost`）时，后端请求本身会抛该错误。
- 异常：`MotionRejected`（响应 `result` 为假）、通用服务异常；`servo_stop()` 抛出的错误在急停发出后再抛出。

#### `set_velocity`

```python
set_velocity(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0, *,
             gait: Union[str, int] = "medium", step_height: float = 0.05,
             timeout: Optional[float] = None) -> None
```

设定行走速度后**立即返回**；后台以 20Hz 持续下发 `SERVO_DATA`，直到调用 `stop()`（或 `close()`、回调崩溃、代理看门狗触发、死人开关到期）。
可以反复调用以实时改变速度（只替换目标，不重启心跳线程）。

| 参数 | 单位 / 范围 | 说明 |
|---|---|---|
| `vx` | m/s | 前进为正 |
| `vy` | m/s | 横移 |
| `wz` | rad/s | 偏航角速度，正值为左转 |
| `gait` | `"slow"` / `"medium"` / `"fast"` / `"jump"` / `"hop"`，或整数 303/308/305/301/302 | 步态。各步态上限见 5.2 节，**超限直接 `ValueError`，不截断** |
| `step_height` | m，`[0, 0.1]` | 抬腿高度，两个分量都用这个值 |
| `timeout` | s，`> 0` 或 `None`（仅关键字） | **死人开关**：从这次调用起超过 `timeout` 秒没有再次调用 `set_velocity()`，伺服循环自动发一帧 END 停下。`None`（默认）表示一直下发到 `stop()`。实时控制循环里建议设成控制周期的几倍（如 0.5），这样主程序卡死时狗也会停 |

发给后端的目标：`{"motion_id", "cmd_source", "value": 0, "vel_des": [vx, vy, wz], "step_height": [h, h]}`，
设置了 `timeout` 时再加 `"expire_after": timeout`。`ServoLoop.set()` 取出 `expire_after`（不会发到 ROS），其余经 `fill_motion_arrays` 补零并加上 `cmd_type=1` 下发。
远程模式下死人开关由**代理端**的 `ServoLoop` 计时，因此电脑端程序卡死（但 ping 线程仍在发）时也有效。

异常：`ValueError`（未知字符串 / 整数步态、速度超限、`step_height` 越界、`timeout <= 0`）、通用前置异常；后端伺服线程之前出过错时抛那个错误。参数校验在 `_check()` 之前完成。

#### `stop`

```python
stop(self) -> None
```

结束伺服：停止心跳，发送一帧 `cmd_type=2 (END)`、`vel_des=[0,0,0]`，狗停下站立。
**先停止、再 `_check()`**，因此即使已有回调错误，也一定会先停下来。没有在伺服时调用是安全的（不发任何东西）。

#### `walk`

```python
walk(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0, duration: float = 1.0, **kw) -> None
```

`set_velocity(vx, vy, wz, **kw)` 后阻塞 `duration` 秒（每 50ms 检查一次 `_check()`），然后停止。中途出任何错（包括 Ctrl+C）都会在 `finally` 里先 `servo_stop()` 再抛出。`**kw` 可传 `gait`、`step_height`、`timeout`。

- `duration <= 0` → `ValueError`。
- 默认 `timeout = duration + 1.0`（死人开关）：即使调用线程卡住，狗也会在约 `duration + 1` 秒后自动停下。显式传 `timeout=None` 可以关闭。

#### `turn`

```python
turn(self, wz: float, duration: float = 1.0, **kw) -> None
```

原地转向，等价于 `walk(0.0, 0.0, wz, duration, **kw)`。`wz` 正值左转。

```python
import math
dog.motion.walk(vx=0.3, duration=2.0)
dog.motion.turn(wz=0.8, duration=(math.pi / 2) / 0.8)   # 约左转 90 度
dog.motion.walk(vx=1.5, gait="fast", duration=1.0)

# 实时控制
dog.motion.set_velocity(0.2, timeout=0.5)
...                         # 主循环里每个周期再次 set_velocity(..., timeout=0.5)；超过 0.5s 不刷新就自动停
dog.motion.stop()
```

### 4.3 `Sensors`（`dog.sensors`，`cyberdog2/sensors.py`）

| 成员 | 类型 | ROS 接口 | 消息类型 |
|---|---|---|---|
| `battery` | `Battery` | 话题 `bms_status` | `protocol/msg/BmsStatus` |
| `motion_status` | `MotionState` | 话题 `motion_status` **【未核实】** | `protocol/msg/MotionStatus` |
| `head_tof` | `TopicCache` | 话题 `head_tof_payload` | `protocol/msg/HeadTofPayload` |
| `rear_tof` | `TopicCache` | 话题 `rear_tof_payload` | `protocol/msg/RearTofPayload` |
| `ultrasonic` | `TopicCache` | 话题 `ultrasonic_payload` | `sensor_msgs/msg/Range` |
| `gps` | `TopicCache` | 话题 `gps_payload` | `protocol/msg/GpsPayload` |
| `uwb` | `TopicCache` | 话题 `uwb_raw` | `protocol/msg/UwbRaw` |
| `scan` | `TopicCache` | 话题 `scan` | `sensor_msgs/msg/LaserScan`（真机确认：前方 180°，495 点，10 Hz，`0.0` 表示无回波） |
| `odom` | `TopicCache` | 话题 `odom_out` | `nav_msgs/msg/Odometry`，里程计原始消息（真机实测约 48 Hz） |
| `imu` | `Imu` | 由 `odom` 计算，不单独订阅 | `get()` 返回 `{"orientation": {x, y, z, w}, "angular_velocity": {x, y, z}}`，另有 `age` 和 `on(callback)`。真机默认没有 `imu` 话题。里程计没有加速度，所以**没有 `linear_acceleration`** |
| `camera` | `Camera` | 服务 `camera_service` + 话题 `image` **【未核实】** | `protocol/srv/CameraService` / `sensor_msgs/msg/Image` |
| `on_touch()` | 方法 | 话题 `touch_status` | `protocol/msg/TouchStatus` |

#### `on_touch`

```python
on_touch(self, callback: Callable[[TouchEvent], None]) -> int
```

头部触摸板事件。每条 `touch_status` 消息构造成 `TouchEvent` 传给 `callback`。返回订阅 id（可传给 `dog.unsubscribe()`）。
回调受 let-it-crash 保护（抛异常 → 停止运动 → 后续 `CallbackError`）。通用前置异常。

#### `ultrasonic_distance`

```python
ultrasonic_distance(self, timeout: float = 3.0) -> float
```

前方超声测距，单位 m，取 `Range.range`。超时抛 `SensorTimeout`。

#### `attitude`

```python
attitude(self, timeout: float = 3.0, max_age: Optional[float] = None) -> dict
```

机身姿态，返回 `{"roll", "pitch", "yaw"}`，单位 rad（ROS 约定的 ZYX 欧拉角：`roll` 右侧下沉为正，`pitch` 低头为正，`yaw` 左转为正）。由 `sensors.imu` 的 `orientation` 四元数换算（来自里程计）；四元数全 0、含 NaN 或 inf 时抛 `ValueError`，超时抛 `SensorTimeout`。

**真机实测**（趴着时手动抬起和翻转）：约 48 Hz，四元数顺序正确，`pitch` 方向与上述约定一致，悬空翻转时朝向也能跟上。`yaw` 是相对值，不是指南针方向；机身接近倒置时欧拉角会出现很大的 `roll` 和 `yaw`，这是欧拉角本身的特性。站立和行走时的数据、`angular_velocity` 的数值和坐标系**尚未核实**。

官方的 `imu_bridge`（`/opt/ros2/cyberdog/lib/motion_bridge/imu_bridge`）装在狗上但默认不运行，本库不使用它。

#### 传感器自动开启

`scan`（雷达）、`ultrasonic`、`head_tof` / `rear_tof`、`gps` 第一次读取（`get()` 或 `on()`）时，如果约 1 秒内没有数据（GPS 为 3 秒），库会调用 `sensor_manager` 的 `sensor_operation` 服务打开对应传感器，打开用的时间不计入 `timeout`。打开失败抛 `CommandFailed`。

`camera.get_frame()` 同理：相机没有在发布时以默认分辨率打开。要指定分辨率请先调用 `camera.start()`。

收尾规则：**只关库自己打开的**，已经有数据的（比如 App 的导航正在用）不碰。传感器的记录在狗端代理里，电脑崩溃或断网时代理也会关闭；相机只在 `CyberDog.close()` 时关闭。

**真机状态**：雷达的 `START` 已实测可用；`STOP` 和其他传感器的开关**未核实**。其余话题（电池、里程计、运动状态、UWB 等）不受 `sensor_manager` 管理，没有自动开启。

#### `tof`

```python
tof(self, timeout: float = 3.0) -> dict
```

返回 `{"left_head", "right_head", "left_rear", "right_rear"}` 四个 8×8 的二维 list（单位 m，行优先）。
分别读 `head_tof` 和 `rear_tof` 的最新消息，取 `msg[key]["data"]`。某个 `data` 长度不是 64 时抛 `ValueError`；超时抛 `SensorTimeout`。

```python
m = dog.sensors.tof()
print(min(min(row) for row in m["left_head"]))
```

### 4.4 `TopicCache`

```python
TopicCache(dog, iface)          # iface = (话题名, 消息类型)，例如 Iface.GPS
```

惰性订阅一个话题并缓存最新消息。公开属性 `topic`、`msg_type`。

| 成员 | 说明 |
|---|---|
| `get(self, timeout: float = 3.0, max_age: Optional[float] = None) -> dict` | 先 `_check()`（通用前置异常）。第一次调用时才订阅（订阅 **不** 经过回调包装；首次订阅有锁保护，多线程同时首次 `get()` 只会订阅一次）。`max_age=None`：有缓存就立即返回，没有则最多等 `timeout` 秒。`max_age` 为数值：要求最新消息在 `max_age` 秒内收到，太旧就继续等新消息。超时先 `_check()` 再抛 `SensorTimeout`（从未收到过消息 / 最新消息超过 `max_age`，两种情况信息不同），**不会返回过期数据** |
| `age` | 属性 → `Optional[float]`，最新一条消息距今的秒数（单调时钟，按电脑端收到的时刻算）；还没收到过为 `None` |
| `on(self, callback: Callable[[dict], None]) -> int` | 每条消息调用一次 `callback(dict)`，回调受 let-it-crash 保护。返回订阅 id |

`get()` 建立的内部订阅不会被取消，伴随 dog 的整个生命周期。`Battery` / `MotionState` 的属性以及 `ultrasonic_distance()` / `tof()` 都调用不带 `max_age` 的 `get()`，即话题停发后仍返回旧值；需要新鲜度时直接用 `get(max_age=...)`。

```python
gps = dog.sensors.gps.get()
print(gps["lat"], gps["lon"])
d = dog.sensors.ultrasonic.get(max_age=0.5)["range"]   # 只要 0.5 秒内的数据
print(dog.sensors.ultrasonic.age)
sid = dog.sensors.uwb.on(lambda d: print(d["dist"]))
dog.unsubscribe(sid)
```

### 4.5 `Battery`（继承 `TopicCache`，话题 `bms_status`）

| 属性 | 类型 | 说明 |
|---|---|---|
| `soc` | `int` | 电量百分比，`int(msg["batt_soc"])` |
| `voltage` | `float` | 电压 V，`msg["batt_volt"] / 1000.0`（原始值为 mV） |
| `charging` | `bool` | `power_wired_charging` 或 `power_wp_charging` 任一为真 |

每次访问属性都调用一次 `get()`（默认超时 3 秒），可能抛 `SensorTimeout`。也可以直接 `dog.sensors.battery.get()` 拿完整消息。

### 4.6 `TouchEvent`

```python
TouchEvent(raw: dict)
```

| 属性 | 说明 |
|---|---|
| `raw` | 原始消息 dict |
| `state` | `int(raw["touch_state"])` |
| `kind` | `"single"`（0x01）/ `"double"`（0x03）/ `"long"`（0x07）；未知编码为 `None` |

`repr` 形如 `TouchEvent(kind='single', state=0x01)`。

### 4.7 `MotionState`（继承 `TopicCache`，话题 `motion_status`【未核实】）

| 属性 | 类型 | 说明 |
|---|---|---|
| `motion_id` | `int` | 当前动作 id |
| `switch_status` | `SwitchStatus` | 状态切换标志，见 5.10 节。未知值会抛 `ValueError`（`IntEnum` 构造失败） |

```python
if dog.sensors.motion_status.switch_status == SwitchStatus.LOW_BAT:
    print("电量过低")
```

### 4.8 `Camera`（`dog.sensors.camera`）

```python
Camera(dog, topic: str = Iface.IMAGE[0])     # 默认话题 "image"【未核实】
```

公开属性 `topic`。所有相机命令都调用服务 `camera_service`（`protocol/srv/CameraService`），请求为
`{"command", "args": "", "width": 0, "height": 0, "fps": 0}` 加上各方法的字段；响应 `result != 0` 时抛 `CommandFailed`
（信息中含 `CameraCmd` 名称、`result`、`msg`）。

| 方法 | 说明 | 异常 |
|---|---|---|
| `start(self, width: int = 640, height: int = 480, fps: int = 0) -> None` | `CameraCmd.START_IMAGE_PUBLISH`（9），开启图像发布 | `CommandFailed`、通用前置/服务异常 |
| `stop(self) -> None` | `CameraCmd.STOP_IMAGE_PUBLISH`（10） | 同上 |
| `take_picture(self) -> dict` | `CameraCmd.TAKE_PICTURE`（1），照片保存在狗的 `/home/mi/Camera`。返回响应 dict | 同上 |
| `get_frame(self, timeout: float = 3.0, jpeg: Optional[bool] = None) -> Frame` | 取最新一帧。`jpeg=None` 时远程默认 `True`（狗端压成 JPEG），本地/mock 默认 `False` | `SensorTimeout`、通用前置异常；远程压缩时狗上缺 Pillow → `RemoteError` |
| `stream(self, width: int = 640, height: int = 480, fps: float = 15, quality: int = 70, jpeg: Optional[bool] = None, timeout: float = 5.0, start_camera: bool = True) -> ImageStream` | 打开连续视频流 | 见下 |

`get_frame` 细节：

- 必须先 `start()`，否则 ROS 后端会超时（提示"相机是否已用 camera.start() 开启？"）。
- ROS 后端第一次取图时订阅该话题并一直保留订阅；只接受接收时间不早于"请求时刻 − 1 秒"的帧，避免返回很旧的帧。
- `get_frame(jpeg=True)` 在狗端压缩时质量固定为 `encode_jpeg` 的默认 80。
- mock 直接返回 `backend.image`（不压缩，忽略 `jpeg`）；`backend.image` 为 `None` 时抛 `SensorTimeout`。

```python
cam = dog.sensors.camera
cam.start(640, 480)
try:
    frame = cam.get_frame()
    frame.save("frame.jpg")
finally:
    cam.stop()
```

`stream` 参数：

| 参数 | 说明 |
|---|---|
| `width`, `height` | 传给 `camera.start()`（仅 `start_camera=True` 时有用） |
| `fps` | 狗端限速，`(0, 30]`，否则 `ValueError` |
| `quality` | JPEG 质量 `1~100`，否则 `ValueError` |
| `jpeg` | 是否在狗端压缩；`None` 时远程压缩、本地不压缩。mock 始终返回 raw |
| `timeout` | `read()` 默认等待新帧的秒数 |
| `start_camera` | `True` 时创建流时自动 `camera.start(width, height)`，关闭流时自动 `STOP_IMAGE_PUBLISH` |

参数校验在开相机之前完成，校验失败不会发任何相机命令。如果开流失败，会尝试关闭刚打开的相机后再抛出。

### 4.9 `ImageStream`

由 `camera.stream()` 创建，推荐配合 `with`：

```python
with dog.sensors.camera.stream(width=640, height=480, fps=15, quality=70) as stream:
    for frame in stream:
        img = frame.to_bgr()          # 需要 opencv-python
        ...                           # 识别代码
        print(stream.received, stream.dropped)
```

机制：狗端（代理）和电脑端都**只保留最新一帧**（`LatestSlot`），处理慢时旧帧被覆盖丢弃，画面不会越积越滞后。

| 成员 | 说明 |
|---|---|
| `fps`, `timeout` | 创建时的参数 |
| `received` | 属性，电脑端收到的帧数 |
| `dropped` | 属性，电脑端因处理不过来被覆盖丢弃的帧数 |
| `read(self, timeout: Optional[float] = None) -> Frame` | 阻塞到有一帧"上次读取之后的新帧"。`timeout=None` 用 `self.timeout`。每 0.1s 检查一次 `_check()`。超时抛 `SensorTimeout`；流已关闭（调用时）抛 `CyberDogError("视频流已关闭，不能再读取")`；连接断开抛 `ConnectionLost`；回调崩溃抛 `CallbackError` |
| `__iter__` | `while not closed: yield self.read()`，因此超时会以 `SensorTimeout` 抛出而不是结束迭代 |
| `close(self) -> None` | 可重复调用。后端无致命错误时停止推流；`start_camera=True` 且后端无致命错误时关闭相机（绕过 `_check()`，因此回调崩溃后仍能关相机） |
| `__enter__` / `__exit__` | 同 `CyberDog`：正在处理异常时 `close()` 的异常只打印 |

### 4.10 `Frame`（`cyberdog2/frame.py`）

```python
Frame(width: int, height: int, encoding: str, data: bytes, format: str = "raw", stamp: float = 0.0)
```

| 属性 | 说明 |
|---|---|
| `width`, `height` | 像素 |
| `encoding` | ROS 图像编码，如 `rgb8`、`bgr8`、`mono8`、`rgba8`、`bgra8`（JPEG 帧也保留原始编码名） |
| `data` | `format="raw"` 时是未压缩像素；`format="jpeg"` 时是 JPEG 文件字节 |
| `format` | `"raw"` 或 `"jpeg"` |
| `stamp` | 时间戳（秒）。ROS 后端取 `header.stamp`；mock 推流取 `time.time()` |

| 方法 | 说明 | 依赖 / 异常 |
|---|---|---|
| `to_dict(self) -> dict` | `{"width","height","encoding","data","format","stamp"}`，用于协议传输 | — |
| `from_dict(cls, d: dict) -> Frame` | 类方法，`to_dict` 的逆操作，`stamp` 缺省为 0.0 | `KeyError` |
| `to_numpy(self)` | `(H, W, C)` 的 uint8 数组，**RGB 顺序**（bgr 编码会被换成 RGB）；JPEG 帧用 Pillow 解码为 RGB | numpy；JPEG 需 Pillow；不支持的编码 `ValueError` |
| `to_bgr(self)` | `(H, W, 3)` 的 BGR 数组，可直接给 cv2 用；mono 转成 3 通道，丢弃 alpha | opencv-python、numpy；JPEG 解码失败 `ValueError` |
| `save(self, path: str) -> None` | JPEG 帧直接写字节（**不管扩展名**）；raw 帧用 Pillow 按扩展名保存 | raw 需 Pillow + numpy |

模块函数（狗端使用）：`encode_jpeg(frame: Frame, quality: int = 80) -> Frame`，用 Pillow 把 raw 帧压成 JPEG；已是 JPEG 时原样返回；不支持的编码或数据长度不对 `ValueError`。

支持的 raw 编码（`_CHANNELS`）：`rgb8`(3)、`bgr8`(3)、`mono8`(1)、`rgba8`(4)、`bgra8`(4)。

### 4.11 `Led`（`dog.led`，`cyberdog2/led.py`）

ROS 接口：服务 `led_execute`（`protocol/srv/LedExecute`）。

```python
Led(dog, client: str = "vp")
```

`client` 决定优先级，必须是 LedExecute.srv 里登记过的模块名之一：`lowpower`、`bms`、`connector`、`vp`、`tracking`、`system`。
`dog.led` 使用默认的 `"vp"`；需要其他优先级时可以改 `dog.led.client = "..."`。

请求字段：`{"occupation", "client", "target", "mode", "effect", "r_value", "g_value", "b_value"}`。
响应 `code != 0` 时抛 `CommandFailed`，信息里带 `LED_CODES` 中的名称（如 `LOW_PRIORITY`），`e.code` 为该码。

#### `set`

```python
set(self, target: str, color: Color = "red", effect: str = "on") -> None      # Color = Union[str, Sequence[int]]
```

| `target` | `color` | 可用 `effect` | 模式 |
|---|---|---|---|
| `"head"` / `"tail"` | `"red"` / `"blue"` / `"yellow"` | `on`、`blink`、`blink_fast`、`breath`、`breath_fast`、`one_by_one`、`one_by_one_fast` | 系统预定义（mode 0x01） |
| `"head"` / `"tail"` | `"off"` | 只能是 `on`（默认），其他 → `ValueError` | 预定义，effect 0xA0 |
| `"head"` / `"tail"` | `(r, g, b)`，0~255 | 上面 7 个 + `back_and_forth`、`trailing_race` | 用户 RGB（mode 0x02） |
| `"mini"` | `None`（**字符串颜色 → `ValueError`**） | `off`、`rectangle_color`、`centre_color`、`three_circular`、`color_one_by_one` | 预定义 |
| `"mini"` | `(r, g, b)` | `circular_breath`、`circular_ring` | 用户 RGB |

`occupation=True`（占用该灯）。非法 target / 组合 / RGB 值、头灯 / 尾灯 `color=None`、mini 灯传颜色名 → `ValueError`（不会静默忽略参数）。还可能抛 `CommandFailed`、通用前置/服务异常。
注意 mini 灯的默认 `color="red"` 也是字符串，所以 mini 灯必须显式传 `color=None` 或 RGB 元组。

#### `off` / `release`

```python
off(self, target: str) -> None
release(self, target: str) -> None
```

- `off`：mini 灯发送预定义 `off`（0x32）；头灯 / 尾灯等价于 `set(target, "off")`。保持占用。
- `release`：以 `occupation=False` 发送关灯，**释放占用**，让系统或其他模块的灯效接管。非法 target → `ValueError`。

```python
dog.led.set("head", "blue", "breath")
dog.led.set("tail", (0, 255, 128), "trailing_race")
dog.led.set("mini", None, "three_circular")
for t in ("head", "tail", "mini"):
    dog.led.release(t)
```

### 4.12 `Audio`（`dog.audio`，`cyberdog2/audio.py`）

| 成员 | ROS 接口 | 类型 |
|---|---|---|
| `say` / `play` | 服务 `speech_text_play` | `protocol/srv/AudioTextPlay` |
| `volume`（读） | 服务 `audio_volume_get` | `protocol/srv/AudioVolumeGet` |
| `volume`（写） | 服务 `audio_volume_set` | `protocol/srv/AudioVolumeSet` |

模块常量 `MODULE_NAME = "cyberdog2py"`，作为请求里的 `module_name`。

```python
say(self, text: str, timeout: float = 30.0) -> None
play(self, play_id: int, timeout: float = 30.0) -> None
volume  # 属性：读返回 int；写接受 0~100
```

- `say`：在线 TTS，阻塞到播放完毕。需要狗已联网并登录小米账号（App 中开启在线语音）。`text` 为空 → `ValueError`。
- `play`：离线播放内置语音，`play_id` 取值见 bridges 的 `protocol/msg/AudioPlay.msg`（例如 9000 是整机就绪音效）。
- 请求：`{"module_name": "cyberdog2py", "is_online": bool, "speech": {"module_name": "cyberdog2py", "play_id": int}, "text": str}`。响应 `status != 0` → `CommandFailed`。
- `volume` 读：返回 `int(resp["volume"])`。写：超出 0~100 → `ValueError`；响应 `success` 为假 → `CommandFailed`。

```python
dog.audio.volume = 60
print(dog.audio.volume)
dog.audio.say("你好，我是铁蛋")
dog.audio.play(9000)
```

### 4.13 `Vision`（`dog.vision`，`cyberdog2/vision.py`）

| 方法 | ROS 接口 | 类型 |
|---|---|---|
| `enable` / `disable` | 服务 `algo_manager` | `protocol/srv/AlgoManager` |
| `on_person` | 话题 `person` | `protocol/msg/Person` |
| `start_gesture` / `stop_gesture` | 服务 `gesture_action_control` | `protocol/srv/GestureActionControl` |
| `on_gesture` | 话题 `gesture_action_msg` | `protocol/msg/GestureActionResult` |
| `recognize_face` / `cancel_face` | 服务 `cyberdog_face_recognition_srv` | `protocol/srv/FaceRec` |
| `on_face` | 话题 `face_rec_msg` | `protocol/msg/FaceRecognitionResult` |

```python
enable(self, *algos: str, open_age: bool = False, open_emotion: bool = False) -> None
disable(self, *algos: str) -> None
on_person(self, callback: Callable[[dict], None]) -> int
start_gesture(self, timeout: int = 60) -> None
stop_gesture(self) -> None
on_gesture(self, callback: Callable[[GestureEvent], None]) -> int
recognize_face(self, username: str = "", timeout: int = 60) -> None
cancel_face(self) -> None
on_face(self, callback: Callable[[dict], None]) -> int
```

- `enable` / `disable`：算法名取自 `ALGOS`（`face`、`body`、`gesture`、`keypoints`、`reid`、`focus`、`face_manager`），未知名 → `ValueError`；一个算法都不传 → `ValueError`。
  请求 `{"algo_enable": [{"algo_module": n}, ...], "algo_disable": [...], "open_age", "open_emotion"}`，服务超时 30s。
  `result_enable` / `result_disable` 非 0 → `CommandFailed`。结果从 `person` 话题发出，用 `on_person` 接收（dict，含 `face_info`、`body_info`、`track_res` 等字段）。
- `start_gesture(timeout)`：开启手势识别 `timeout` 秒（1~300，否则 `ValueError`），请求 `{"command": 0, "timeout"}`；`stop_gesture` 发 `{"command": 1, "timeout": 60}`。响应 `code != 0` → `CommandFailed`（1 表示忙）。
- `on_gesture`：回调参数是 `GestureEvent`。
- `recognize_face`：需要先在 App 里录入人脸。`timeout` 范围 30~300（否则 `ValueError`）。`username` 为空 → `command=0`（识别所有人），否则 `command=1`。响应 `result != 0` → `CommandFailed`。
- `cancel_face`：发 `command=2`（服务超时用 `_call` 默认的 10s），响应 `result != 0` → `CommandFailed`。
- `on_face`：回调参数为 `FaceRecognitionResult` 消息转成的 dict。
- 所有 `on_*` 返回订阅 id，回调受 let-it-crash 保护。

#### `GestureEvent`

```python
GestureEvent(raw: dict)
```

属性：`raw`、`id`（`int(raw["id"])`）、`name`（`GESTURES` 中的名字，如 `"thumb_up"`；未知 id 为 `None`）。`repr` 形如 `GestureEvent('thumb_up', id=8)`。

```python
import queue
q = queue.Queue()
dog.vision.on_gesture(q.put)
dog.vision.start_gesture(timeout=60)
ev = q.get(timeout=60)
print(ev.name)
dog.vision.stop_gesture()
```

### 4.14 `Navigation`（`dog.nav`，`cyberdog2/navigation.py`）

| 方法 | ROS 接口 | 类型 |
|---|---|---|
| `start` 及所有 `follow_*` / `start_*` / `go_to` | 动作 `start_algo_task` | `protocol/action/Navigation` |
| `stop` / `stop_mapping` | 服务 `stop_algo_task` | `protocol/srv/StopAlgoTask` |
| `on_status` | 话题 `algo_task_status` | `protocol/msg/AlgoTaskStatus` |

#### `start`（通用入口）

```python
start(self, nav_type: int, feedback: Optional[Callable[[dict], None]] = None, **goal) -> NavTask
```

发送 Navigation 目标 `{"nav_type": int(nav_type), **goal}`，goal 字段见 bridges 的 `protocol/action/Navigation.action`。
等到动作服务器接受后返回 `NavTask`（`send_goal` 超时固定 15 秒）。`feedback` 每条反馈调用一次（dict），受 let-it-crash 保护。

`_check()` 通过后、发送目标**之前**就把 `self.started` 置为 `True`（即使 `send_goal` 超时，任务也可能已经在狗上启动），之后 `dog.close()` 会自动 `release()`。

异常：`GoalRejected`（目标被拒绝）、`ServiceUnavailable`（动作服务器没出现）、`ServiceTimeout`、通用前置异常；goal 字段名写错时 ROS 端抛 `AttributeError`（远程为 `RemoteError`）。

#### 快捷方法

| 签名 | nav_type | goal 字段 |
|---|---|---|
| `follow_person(self, distance: float = 1.2, position: str = "behind", **kw) -> NavTask` | 13 `START_HUMAN_TRACKING` | `object_tracking=False`, `keep_distance`, `relative_pos` |
| `follow_object(self, roi: Sequence[int], distance: float = 1.2, position: str = "behind", **kw) -> NavTask` | 13 | `object_tracking=True`, `keep_distance`, `relative_pos`, `tracking_roi={"x_offset","y_offset","width","height"}` |
| `follow_uwb(self, distance: float = 1.2, **kw) -> NavTask` | 11 `START_UWB_TRACKING` | `keep_distance` |
| `start_mapping(self, outdoor: bool = False, **kw) -> NavTask` | 5 `START_MAPPING` | `outdoor`（False 激光建图，True 视觉建图） |
| `start_localization(self, map_name: str = "", **kw) -> NavTask` | 7 `START_LOCALIZATION` | `map_name` |
| `go_to(self, x: float, y: float, yaw: float = 0.0, **kw) -> NavTask` | 1 `START_AB` | `poses=[pose_stamped(x, y, yaw)]`；需要先建图并完成定位 |

- `position`：`auto`(200) / `behind`(201) / `left`(202) / `right`(203)，非法 → `ValueError`。`distance`（m）≤ 0 → `ValueError`（`follow_person` / `follow_object` / `follow_uwb` 都检查）。
- `roi`：图像中目标框 `(x_offset, y_offset, width, height)`，像素。
- `**kw` 会作为额外的 goal 字段传入；也可以传 `feedback=...`（会被 `start` 接住）。

#### `stop` / `stop_mapping` / `on_status`

```python
stop(self, task: int = StopTask.ALL, map_name: str = "") -> None
stop_mapping(self, map_name: str) -> None
on_status(self, callback: Callable[[dict], None]) -> int
```

- `stop`：请求 `{"task_id": int(task), "map_name": map_name}`，超时 30s；`result != 0` → `CommandFailed`。`task` 为 `StopTask.ALL` 且成功时把 `started` 清为 `False`（停单个任务不清）。
- 属性 `started`（`bool`）：本次会话是否启动过导航 / 跟随任务且还没有成功 `stop(ALL)`；为 `True` 时 `dog.close()` 会调用 `release()`。
- `release(self) -> int`：读话题 `algo_task_status`，按狗的实际状态结束任务并返回结束前的状态码：已空闲（101/6/16）不发指令，其余按 `constants.TASK_STOP_BY_STATUS` 选 task_id（建图 5、定位启动 7、导航和定位中 0、回充 9、UWB 11、跟随 13），状态 102/103 先等它结束，状态 100 或未知状态抛 `CyberDogError`。未保存的地图会被丢弃。和 `estop` 一样不经过 `_check()`。5 秒内读不到状态时先发一次 `StopTask.ALL`，再抛 `SensorTimeout`。成功后把 `started` 清为 `False`。
- `stop_mapping(map_name)`：`stop(StopTask.MAPPING, map_name=map_name)`，结束建图并以该名字保存地图。
- `on_status`：任务状态回调（dict）。

#### `NavTask`

```python
NavTask(nav, goal_id: str, nav_type: int)
```

| 成员 | 说明 |
|---|---|
| `goal_id` | 字符串。ROS 后端是 goal UUID 的 hex；mock 是 `"goal-N"` |
| `nav_type` | int |
| `wait(self, timeout: Optional[float] = None) -> dict` | 等待任务结束，返回结果 dict（ROS 后端额外带 `_status` 字段，即 action 的 GoalStatus）。`result` 不在 `{0 SUCCESS, 1 ACCEPT}` 中 → `GoalRejected`（信息里带 `NAV_RESULTS` 名称，如 `FAILED`）。`timeout=None` 无限等待；超时 → `ServiceTimeout`。开始等待前先 `_check()`（通用前置异常） |
| `cancel(self) -> None` | 先 `_check()`，再取消目标（后端默认超时 5 秒） |

#### 模块函数 `pose_stamped`

```python
pose_stamped(x: float, y: float, yaw: float = 0.0, frame_id: str = "map") -> dict
```

返回 `geometry_msgs/PoseStamped` 形状的 dict：`header.frame_id`、`pose.position (x, y, 0)`、`pose.orientation` 为绕 z 轴转 `yaw`（rad）的四元数。

```python
task = dog.nav.follow_person(distance=1.2, position="behind",
                             feedback=lambda fb: print(fb.get("feedback_code")))
...
dog.nav.stop()

dog.nav.start_mapping()
...                                    # 遥控狗走一圈
dog.nav.stop_mapping("home")
dog.nav.start_localization("home")
dog.nav.go_to(1.0, 2.0, yaw=1.57).wait(timeout=120)
```

### 4.15 `MockBackend` 测试辅助（`cyberdog2/backends/mock.py`）

```python
MockBackend(namespace: str = MOCK_NAMESPACE, servo_period: float = 0.05)    # MOCK_NAMESPACE = "/mi_mock"
```

通过 `CyberDog.mock(**kw)` 创建，`dog.backend` 访问。行为要点：

- 服务默认都返回成功（默认响应表见下）；**调用未登记的服务直接抛 `ServiceUnavailable`**，和真机找不到服务时一致。
- 订阅 `latched` 里有的话题时，会**在 `subscribe` 调用线程里立即同步**回调一次 latched 数据（所以 `battery.soc` 能立即读到 80）。
- 伺服心跳真实运行（`ServoLoop`），发布记录在 `published`。
- 推流按 fps 在 `mock-stream` 线程里发送 `self.image` 的副本（`stamp` 为当前时间），**忽略 `jpeg`**，始终是 raw。
- `fatal_error` 只反映伺服线程的错误。

#### 公开属性

| 属性 | 类型 | 说明 |
|---|---|---|
| `namespace` | `str` | 命名空间 |
| `services` | `Dict[str, Callable[[dict], dict]]` | 服务短名 → 处理函数。可以直接 `del dog.backend.services["led_execute"]` 模拟服务缺失 |
| `latched` | `Dict[str, dict]` | 话题短名 → 订阅时立即回放的消息。`pop` 掉可以模拟传感器无数据 |
| `published` | `List[(time, 话题短名, dict)]` | 所有 publish 记录（含伺服帧） |
| `calls` | `List[(服务短名, 请求 dict)]` | 所有服务调用记录 |
| `goals` | `Dict[goal_id, {"name", "goal", "feedback_cb", "cancelled"}]` | 所有发出的动作目标。可以手动调 `goals[gid]["feedback_cb"](...)` 模拟反馈 |
| `goal_results` | `Dict[goal_id, dict]` | 设置后 `wait_goal` 返回它；未设置返回 `{"result": 0}` |
| `reject_goals` | `bool` | 为 `True` 时 `send_goal` 抛 `GoalRejected` |
| `image` | `Optional[Frame]` | 相机帧，默认 4×3 全黑 `rgb8`。设为 `None` 模拟收不到图像 |
| `closed` | `bool` | 是否已关闭 |

#### 方法

| 签名 | 说明 |
|---|---|
| `short(self, name: str) -> str` | 把名字转成去掉命名空间的短名 |
| `set_response(self, service: str, response: dict) -> None` | 之后对该服务的调用都返回 `response` 的副本。**`service` 必须写短名**（直接作为 `services` 的键） |
| `set_handler(self, service: str, handler: Callable[[dict], dict]) -> None` | 自定义处理函数，参数是请求 dict。同样用短名 |
| `emit(self, topic: str, data: dict) -> None` | 模拟狗发布一条消息，**在调用线程里同步**分发给该话题的所有订阅者。不会更新 `latched` |
| `published_on(self, topic: str) -> List[dict]` | 某话题上发布过的所有消息 |
| `calls_to(self, service: str) -> List[dict]` | 某服务收到过的所有请求 |
| `servo_active` | 属性 → `bool`，伺服心跳是否在运行 |
| `active_streams` | 属性 → `int`，未停止的推流数 |

默认服务响应：

| 服务 | 默认响应 |
|---|---|
| `motion_result_cmd` | `{"motion_id": <请求的 id>, "result": True, "code": 0}` |
| `led_execute` | `{"code": 0}` |
| `speech_text_play` | `{"status": 0, "code": 0}` |
| `audio_volume_get` | `{"volume": 50, "code": 0}` |
| `audio_volume_set` | `{"success": True, "code": 0}` |
| `camera_service` | `{"result": 0, "msg": "", "code": 0}` |
| `algo_manager` | `{"result_enable": 0, "result_disable": 0, "code": 0}` |
| `cyberdog_face_recognition_srv` | `{"result": 0, "code": 0}` |
| `gesture_action_control` | `{"code": 0}` |
| `stop_algo_task` | `{"result": 0, "code": 0}` |

默认 latched 话题：`bms_status`（`batt_volt=25000`、`batt_soc=80` 等）、`motion_status`（`motion_id=111`）、`ultrasonic_payload`（`range=1.2`）、`head_tof_payload`（全 0.5）、`rear_tof_payload`（全 0.66）、`gps_payload`、`uwb_raw`。

`list_names()` 返回所有已登记服务、latched 话题加上 `motion_servo_cmd`、`touch_status`，动作只有 `start_algo_task`。

```python
import pytest
from cyberdog2 import CyberDog, CommandFailed, GoalRejected

def test_example():
    dog = CyberDog.mock()
    mock = dog.backend

    # 检查发出的指令
    dog.led.set("head", "red", "breath")
    assert mock.calls_to("led_execute")[-1]["effect"] == 0xA4

    # 模拟失败
    mock.set_response("led_execute", {"code": 1125})
    with pytest.raises(CommandFailed, match="LOW_PRIORITY"):
        dog.led.set("head", "blue")

    # 用 handler 按请求返回
    mock.set_handler("audio_volume_get", lambda req: {"volume": 33, "code": 0})
    assert dog.audio.volume == 33

    # 导航结果
    task = dog.nav.go_to(1.0, 2.0)
    mock.goal_results[task.goal_id] = {"result": 3}
    with pytest.raises(GoalRejected):
        task.wait()

    # 伺服
    dog.motion.set_velocity(0.2)
    assert mock.servo_active
    dog.motion.stop()
    assert mock.published_on("motion_servo_cmd")[-1]["cmd_type"] == 2
    dog.close()
```

### 4.16 后端接口（`Backend`，`cyberdog2/backends/base.py`）

扩展或排查问题时需要了解。所有消息都是 dict；名字不以 `/` 开头时由后端拼上命名空间（`resolve_name(namespace, name)`）。

| 方法 | 说明 |
|---|---|
| `call_service(self, name, srv_type, request, timeout=5.0) -> dict` | 调服务。`timeout` 是整个调用的总时长（`RosBackend`：等服务出现 + 等结果共用一个截止时间，`send_goal` 同理） |
| `publish(self, topic, msg_type, data) -> None` | 发布 |
| `subscribe(self, topic, msg_type, callback) -> int` | 订阅，返回 id |
| `unsubscribe(self, sub_id) -> None` | 退订 |
| `send_goal(self, name, action_type, goal, feedback_cb=None, timeout=10.0) -> str` | 发送动作目标，等到接受后返回 goal_id；被拒绝抛 `GoalRejected` |
| `wait_goal(self, goal_id, timeout=None) -> dict` | 等结果 |
| `cancel_goal(self, goal_id, timeout=5.0) -> None` | 取消 |
| `servo_set(self, cmd) -> None` | 设置伺服目标（MotionServoCmd 字段，不含 `cmd_type`；可选 `expire_after` 秒数作为死人开关，由 `ServoLoop` 取出），后端以 20Hz 下发 |
| `servo_stop(self) -> None` | 结束伺服 |
| `latest_image(self, topic, timeout=3.0, jpeg=False)` | 返回 `Frame`（三个后端默认值一致，都是 `jpeg=False`） |
| `image_stream_start(self, topic, fps, jpeg, quality, callback) -> int` | 开始推流，每帧以 `Frame` 调 `callback`（必须很快返回） |
| `image_stream_stop(self, stream_id) -> None` | 停止推流 |
| `list_names(self) -> Dict[str, List[str]]` | 列出接口 |
| `fatal_error` | 属性，后台致命错误，正常为 `None` |
| `close(self) -> None` | 关闭 |
| `namespace` | 类属性 / 实例属性 |

辅助：`ServoLoop(publish, period=0.05)`（`set` / `stop` / `active` / `error` / `expired_count`：因 `expire_after` 到期自动停止的次数；到期时自动发一帧 END，之后 `stop()` 不再发东西；`expire_after <= 0` → `ValueError`）、`fill_motion_arrays(cmd)`、`RateLimiter(fps)`、`LatestSlot()`、`ros.pick_namespace(nodes)`、`ros.load_type(type_str)`、`ros.to_plain(value)`、`ros.fill_message(msg, data)`。

---

## 5. 常量参考

均在 `cyberdog2/constants.py`。`MotionId`、`CmdSource`、`NavType`、`StopTask`、`SwitchStatus` 可直接 `from cyberdog2 import ...`。

### 5.1 `MotionId` 全表

结果指令（`motion.do()`）：

| id | 名称 | 中文说明 |
|---|---|---|
| 0 | `ESTOP` | 急停 |
| 101 | `GET_DOWN` | 高阻尼趴下 |
| 111 | `STAND` | 恢复站立 |
| 118 | `STAND_UP_STRONG` | 强化起身 |
| 121 | `BACKFLIP` | 后空翻 |
| 123 | `BOW` | 作揖 |
| 125 | `WALK_THE_DOG` | 遛狗 |
| 130 | `JUMP_TURN_LEFT_90` | 3D 跳跃左转 90 度 |
| 131 | `JUMP_TURN_RIGHT_90` | 3D 跳跃右转 90 度 |
| 132 | `JUMP_FORWARD_60CM` | 3D 跳跃前跳 60cm |
| 134 | `JUMP_LEFT_20CM` | 3D 跳跃左跳 20cm |
| 135 | `JUMP_RIGHT_20CM` | 3D 跳跃右跳 20cm |
| 140 | `DANCE` | 舞蹈集合 |
| 141 | `SHAKE_LEFT_HAND` | 握左手 |
| 142 | `SHAKE_RIGHT_HAND` | 握右手 |
| 143 | `SIT` | 坐下 |
| 144 | `BUTT_CIRCLE` | 屁股画圆 |
| 145 | `HEAD_CIRCLE` | 头画圆 |
| 146 | `STRETCH` | 伸展身体 |
| 151 | `BALLET` | 芭蕾舞 |
| 152 | `MOONWALK` | 太空步 |
| 174 | `PUSH_UP` | 俯卧撑 |
| 175 | `BOW_HEART` | 作揖比心 |

伺服步态（`motion.set_velocity(gait=...)`，`SERVO_MOTIONS = {301, 302, 303, 305, 308}`，不能用于 `do()`）：

| id | 名称 | 中文说明 | `gait` 字符串 |
|---|---|---|---|
| 301 | `JUMP_FRONT_BACK` | 前后足交替离地 | `"jump"` |
| 302 | `HOP` | 四足同时离地小跳 | `"hop"` |
| 303 | `WALK_SLOW` | 慢速行走 | `"slow"` |
| 305 | `WALK_FAST` | 快速行走 | `"fast"` |
| 308 | `WALK_MEDIUM` | 中速行走 | `"medium"`（默认） |

字符串动作名就是成员名的小写，例如 `"walk_the_dog"`、`"jump_turn_left_90"`。

### 5.2 步态速度上限（`GAIT_LIMITS`，来源：开发者手册）

| 步态 | id | vx 上限 (m/s) | vy 上限 (m/s) | wz 上限 (rad/s) |
|---|---|---|---|---|
| slow | 303 | 0.65 | 0.3 | 1.25 |
| medium | 308 | 1.0 | 0.3 | 1.25 |
| fast | 305 | 1.6 | 0.55 | 2.5 |
| hop | 302 | 0.25 | 0.1 | 0.5 |
| jump | 301 | 0.4 | 0.3 | 0.75 |

检查方式是"每个分量的绝对值 ≤ 上限"，超出抛 `ValueError`。

其他运动限制：

| 常量 | 值 | 用途 |
|---|---|---|
| `RPY_LIMIT` | 0.4 rad | `do(rpy=...)` 每个分量 |
| `POS_LIMIT` | (0.2, 0.2, 0.3) m | `do(pos=...)` |
| `step_height`（`set_velocity`） | [0, 0.1] m | 写在 `motion.py` 中 |
| `MOTION_ARRAY_SIZES` | `vel_des:3, rpy_des:3, pos_des:3, acc_des:6, ctrl_point:3, foot_pose:6, step_height:2` | 下发前补零的固定长度 |

`ServoCmdType`：`START=0`、`DATA=1`、`END=2`（本库只发 DATA 和 END）。

### 5.3 LED 映射与限制

`LED_TARGETS`：`head=1`、`tail=2`、`mini=3`。模式：`LED_MODE_PREDEFINED=0x01`、`LED_MODE_USER=0x02`。

头灯 / 尾灯预定义（`LED_PREDEFINED`，只支持红蓝黄三色）：

| effect | red | blue | yellow |
|---|---|---|---|
| on | 0xA1 | 0xA8 | 0xAF |
| blink | 0xA2 | 0xA9 | 0xB0 |
| blink_fast | 0xA3 | 0xAA | 0xB1 |
| breath | 0xA4 | 0xAB | 0xB2 |
| breath_fast | 0xA5 | 0xAC | 0xB3 |
| one_by_one | 0xA6 | 0xAD | 0xB4 |
| one_by_one_fast | 0xA7 | 0xAE | 0xB5 |

`LED_OFF = 0xA0`（头灯 / 尾灯关闭）。

头灯 / 尾灯 RGB 模式（`LED_USER_EFFECTS`）：`on 0x01`、`blink 0x02`、`blink_fast 0x03`、`breath 0x04`、`breath_fast 0x05`、`one_by_one 0x06`、`one_by_one_fast 0x07`、`back_and_forth 0x08`、`trailing_race 0x09`。

mini 灯：

| 类别 | 灯效 |
|---|---|
| RGB（`LED_MINI_USER_EFFECTS`） | `circular_breath 0x30`、`circular_ring 0x31` |
| 预定义（`LED_MINI_PREDEFINED`） | `off 0x32`、`rectangle_color 0x33`、`centre_color 0x34`、`three_circular 0x35`、`color_one_by_one 0x36` |

返回码 `LED_CODES`：`0 SUCCEED`、`1107 TIMEOUT`、`1121 TARGET_ERROR`、`1122 PRIORITY_ERROR`、`1123 MODE_ERROR`、`1124 EFFECT_ERROR`、`1125 LOW_PRIORITY`。

限制：预定义颜色只有 red/blue/yellow（其他颜色传 RGB 元组）；`back_and_forth` / `trailing_race` 只能用 RGB；RGB 每个分量 0~255 的整数；`client` 必须是 `lowpower`/`bms`/`connector`/`vp`/`tracking`/`system` 之一（本库不校验，由狗端判定）。

### 5.4 触摸编码（`TOUCH_KINDS`）

| `touch_state` | `kind` |
|---|---|
| 0x01 | `single` 单击 |
| 0x03 | `double` 双击 |
| 0x07 | `long` 长按 |
| 其他 | `None` |

### 5.5 手势编码（`GESTURES`）

| id | name | id | name |
|---|---|---|---|
| 0 | `none` | 6 | `slide_right` |
| 1 | `pull_in` | 7 | `stop` |
| 2 | `push_away` | 8 | `thumb_up` |
| 3 | `slide_up` | 9 | `zoom_in` |
| 4 | `slide_down` | 10 | `zoom_out` |
| 5 | `slide_left` | 11 | `thumb_down` |

### 5.6 视觉算法名（`ALGOS`）

`face 0`、`body 1`、`gesture 2`、`keypoints 3`、`reid 4`、`focus 5`、`face_manager 6`。

### 5.7 相机命令（`CameraCmd`）

`SET_PARAMETERS 0`、`TAKE_PICTURE 1`、`START_RECORDING 2`、`STOP_RECORDING 3`、`GET_STATE 4`、`DELETE_FILE 5`、`GET_ALL_FILES 6`、`START_LIVE_STREAM 7`、`STOP_LIVE_STREAM 8`、`START_IMAGE_PUBLISH 9`、`STOP_IMAGE_PUBLISH 10`。
本库只用到 1、9、10，其余可以用 `dog.sensors.camera._cmd(CameraCmd.X, ...)` 试验（私有方法）。

### 5.8 导航（`NavType` / `StopTask` / 跟随位置 / 结果码）

`NavType`：

| 值 | 名称 | 值 | 名称 |
|---|---|---|---|
| 1 | `START_AB` | 8 | `STOP_LOCALIZATION` |
| 2 | `STOP_AB` | 9 | `START_AUTO_DOCKING` |
| 3 | `START_FOLLOW` | 10 | `STOP_AUTO_DOCKING` |
| 4 | `STOP_FOLLOW` | 11 | `START_UWB_TRACKING` |
| 5 | `START_MAPPING` | 12 | `STOP_UWB_TRACKING` |
| 6 | `STOP_MAPPING` | 13 | `START_HUMAN_TRACKING` |
| 7 | `START_LOCALIZATION` | 14 | `STOP_HUMAN_TRACKING` |

`StopTask`：`ALL 0`、`AB 1`、`FOLLOW 3`、`MAPPING 5`、`LOCALIZATION 7`、`AUTO_DOCKING 9`、`UWB_TRACKING 11`、`HUMAN_TRACKING 13`。

`FOLLOW_POSITIONS`：`auto 200`、`behind 201`、`left 202`、`right 203`。

`NAV_RESULTS`：`0 SUCCESS`、`1 ACCEPT`、`2 UNAVAILABLE`、`3 FAILED`、`4 REJECT`、`5 CANCEL`；`NAV_OK_RESULTS = {0, 1}`。

自动回充（`START_AUTO_DOCKING`）没有快捷方法，可以用 `dog.nav.start(NavType.START_AUTO_DOCKING)`（goal 字段需对照 Navigation.action）。

### 5.9 `CmdSource`

`APP 0`（优先级最高，默认）、`AUDIO 1`、`VIS 2`、`BLUTELE 3`、`ALGO 4`。

### 5.10 `SwitchStatus`

| 值 | 名称 | 值 | 名称 |
|---|---|---|---|
| 0 | `NORMAL` | 8 | `ORI_ERR` |
| 1 | `TRANSITIONING` | 9 | `FOOTPOS_ERR` |
| 2 | `ESTOP` | 10 | `STAND_STUCK` |
| 3 | `EDAMP` | 11 | `MOTOR_OVER_HEAT` |
| 4 | `LIFTED` | 12 | `MOTOR_OVER_CURR` |
| 5 | `BAN_TRANS` | 13 | `MOTOR_ERR` |
| 6 | `OVER_HEAT` | 14 | `CHARGING` |
| 7 | `LOW_BAT` | | |

### 5.11 `Iface` 全表

来源：MiRoboticsLab/bridges（rolling 分支）`protocol/ros/` 与官方开发者手册。**只要接口名在真机上不一样，只改这里。**

| 常量 | 接口名 | 类型 | 种类 | 使用者 | 核实状态 |
|---|---|---|---|---|---|
| `MOTION_RESULT` | `motion_result_cmd` | `protocol/srv/MotionResultCmd` | 服务 | `Motion.do` | 文档有 |
| `MOTION_SERVO` | `motion_servo_cmd` | `protocol/msg/MotionServoCmd` | 话题（发布） | 伺服（后端里写死了同样的字符串） | 文档有 |
| `MOTION_SERVO_RESPONSE` | `motion_servo_response` | `protocol/msg/MotionServoResponse` | 话题 | **未使用** | 文档有 |
| `MOTION_STATUS` | `motion_status` | `protocol/msg/MotionStatus` | 话题 | `sensors.motion_status` | **【未核实】手册未写话题名** |
| `LED` | `led_execute` | `protocol/srv/LedExecute` | 服务 | `Led` | 文档有 |
| `SPEECH` | `speech_text_play` | `protocol/srv/AudioTextPlay` | 服务 | `Audio.say/play` | 文档有 |
| `VOLUME_GET` | `audio_volume_get` | `protocol/srv/AudioVolumeGet` | 服务 | `Audio.volume` | 文档有 |
| `VOLUME_SET` | `audio_volume_set` | `protocol/srv/AudioVolumeSet` | 服务 | `Audio.volume=` | 文档有 |
| `BMS` | `bms_status` | `protocol/msg/BmsStatus` | 话题 | `Battery` | 文档有 |
| `TOUCH` | `touch_status` | `protocol/msg/TouchStatus` | 话题 | `on_touch` | 文档有 |
| `HEAD_TOF` | `head_tof_payload` | `protocol/msg/HeadTofPayload` | 话题 | `head_tof` / `tof()` | 文档有 |
| `REAR_TOF` | `rear_tof_payload` | `protocol/msg/RearTofPayload` | 话题 | `rear_tof` / `tof()` | 文档有 |
| `ULTRASONIC` | `ultrasonic_payload` | `sensor_msgs/msg/Range` | 话题 | `ultrasonic` | 文档有 |
| `GPS` | `gps_payload` | `protocol/msg/GpsPayload` | 话题 | `gps` | 文档有 |
| `UWB` | `uwb_raw` | `protocol/msg/UwbRaw` | 话题 | `uwb` | 文档有 |
| `CAMERA` | `camera_service` | `protocol/srv/CameraService` | 服务 | `Camera` | 文档有 |
| `IMAGE` | `image` | `sensor_msgs/msg/Image` | 话题 | `Camera.get_frame/stream` | **【未核实】** |
| `ALGO_MANAGER` | `algo_manager` | `protocol/srv/AlgoManager` | 服务 | `Vision.enable/disable` | 文档有 |
| `PERSON` | `person` | `protocol/msg/Person` | 话题 | `on_person` | 文档有 |
| `FACE_REC` | `cyberdog_face_recognition_srv` | `protocol/srv/FaceRec` | 服务 | `recognize_face` / `cancel_face` | 文档有 |
| `FACE_REC_MSG` | `face_rec_msg` | `protocol/msg/FaceRecognitionResult` | 话题 | `on_face` | 文档有 |
| `GESTURE` | `gesture_action_control` | `protocol/srv/GestureActionControl` | 服务 | `start/stop_gesture` | 文档有 |
| `GESTURE_MSG` | `gesture_action_msg` | `protocol/msg/GestureActionResult` | 话题 | `on_gesture` | 文档有 |
| `NAV_START` | `start_algo_task` | `protocol/action/Navigation` | 动作 | `Navigation.start` 等 | 文档有 |
| `NAV_STOP` | `stop_algo_task` | `protocol/srv/StopAlgoTask` | 服务 | `Navigation.stop` | 文档有 |
| `NAV_STATUS` | `algo_task_status` | `protocol/msg/AlgoTaskStatus` | 话题 | `on_status` | 文档有 |
| `SCAN` | `scan` | `sensor_msgs/msg/LaserScan` | 话题 | `sensors.scan` | 真机已确认 |
| `ODOM` | `odom_out` | `nav_msgs/msg/Odometry` | 话题 | `sensors.odom`，并由它计算 `sensors.imu` / `attitude()` | 真机已确认朝向（趴着时手动抬起和翻转） |

> "文档有"只表示接口名来自 bridges / 手册，**所有接口都没有在真机上调用过**；消息字段名也同样待核实。

doctor 必查接口：`REQUIRED_SERVICES = ["motion_result_cmd", "led_execute", "speech_text_play"]`，`REQUIRED_TOPICS = ["motion_servo_cmd", "bms_status", "touch_status"]`。

---

## 6. 异常参考

所有本库异常都在 `cyberdog2/errors.py`，基类 `CyberDogError(Exception)`。除 `RemoteError` 外都可以 `from cyberdog2 import ...`；`RemoteError` 需 `from cyberdog2.errors import RemoteError`。

```
CyberDogError
├── ConnectionLost
├── AgentStartError
├── NamespaceNotFound
├── ServiceUnavailable
├── ServiceTimeout
├── SensorTimeout
├── CallbackError
├── RemoteError
└── CommandFailed            (.code: int, .response: dict)
    ├── MotionRejected
    └── GoalRejected
```

| 异常 | 含义 | 何时抛出 | 典型处理 |
|---|---|---|---|
| `CyberDogError` | 基类；直接抛出时表示使用了已关闭的对象或版本不一致 | `close()` 之后再调 API（`"CyberDog 已关闭"`）；`ImageStream` 关闭后再 `read()`；连接时代理 `protocol` 版本与本地不一致 | 不要在关闭后继续使用；版本不一致说明狗端跑的不是同一份代码 |
| `ConnectionLost` | 与代理的连接断开 | 读 / 写隧道失败、对端关闭、代理看门狗断开连接、发 ping 失败。信息里尽量附带代理输出的最后 200 行 | 检查网络 / 代理日志；重新 `connect()`。狗端已经自动停止伺服和本次会话的导航任务 |
| `AgentStartError` | 代理启动失败 | 拿不到 HOME、mkdir 失败、清理 `~/.cyberdog2` 旧版本目录失败、启动脚本出错（找不到 motion_manager、无权限读 environ 等）、`start_timeout` 内没打印 READY 行。信息里附带代理完整输出 | 阅读输出；按 `docs/first_boot.md` 的表格处理，或调整 `agent_options` |
| `NamespaceNotFound` | ROS 图里没有 `/mi_*` 命名空间，或有多个 | `RosBackend` 初始化（代理启动 / `local()`） | 多个时传 `namespace=`；没有时检查 DDS 域（`ROS_DOMAIN_ID` 等）、狗的服务是否启动完 |
| `ServiceUnavailable` | 服务 / 动作服务器在超时内没出现 | ROS 的 `wait_for_service` / `wait_for_server` 失败；mock 调用未登记的服务 | 用 `dog.list_names()` 核对接口名，修改 `constants.Iface` |
| `ServiceTimeout` | 已发现但没在超时内返回 | ROS future 超时；远程模式下代理在 `timeout + 5s` 内没响应；`NavTask.wait(timeout)` 超时 | 加大 `timeout`，或检查对应节点是否卡住 |
| `SensorTimeout` | 超时内没收到（足够新的）传感器数据 | `TopicCache.get`（含 `max_age` 内没有新消息）、`latest_image`、`ImageStream.read` | 检查话题名、传感器是否开启（相机需 `start()`） |
| `CommandFailed` | 服务返回失败码 | LED `code≠0`、语音 `status≠0`、音量 `success` 为假、相机 `result≠0`、算法开关失败、手势 `code≠0`、人脸识别 / 取消 `result≠0`、导航停止 `result≠0`（含 `close()` 时自动停止） | 看 `e.code`、`e.response`；LED 的 `LOW_PRIORITY` 说明被更高优先级模块占用 |
| `MotionRejected` | 运动指令被 motion_manager 拒绝（`result` 为假） | `Motion.do` 及快捷方法 | 看 `e.code`；通常是状态不允许（如未站立就做动作、电量低），先 `stand()` 再试 |
| `GoalRejected` | 动作目标被拒绝或任务失败 | `send_goal` 未被接受；mock `reject_goals=True`；`NavTask.wait` 的 `result ∉ {0,1}` | 看信息里的 `NAV_RESULTS` 名称；先满足前置条件（建图 / 定位） |
| `CallbackError` | 用户回调抛了异常 | 回调崩溃后的下一次 API 调用；原始异常在 `__cause__` | 修回调的 bug。该 `CyberDog` 实例已不可继续使用（只能 `close()`） |
| `RemoteError` | 代理端发生了本库没定义的异常 | 远程模式下狗端抛出 `AttributeError`、`ImportError`、`RuntimeError` 等 | `e.type_name`、`e.remote_message`、`e.remote_traceback` 看狗端 traceback |

另外还会出现的标准异常：`ValueError`（参数越界 / 非法组合，包括未知步态字符串，本库最常见的异常）、`TypeError`（动作类型错误）、`KeyError`（未知订阅 id / goal_id / 流 id）、`ImportError`（可选依赖缺失）。
远程模式下如果代理的响应无法编码成 JSON（例如含不支持的类型），代理会把编码错误作为这次请求的错误立即回传，客户端通常收到 `TypeError`（`json` 抛的类型），而不是等到 `timeout + 5s` 才 `ServiceTimeout`。

### 6.1 远程异常如何还原（`errors.rebuild`）

代理捕获异常后用 `protocol.error_payload(exc)` 序列化为：

```json
{"type": "MotionRejected", "message": "...", "traceback": "...", "extra": {"code": 3, "response": {...}}}
```

（`extra` 只在异常对象有 `code` / `response` 属性时才带。）客户端调用：

```python
rebuild(type_name: str, message: str, remote_traceback: str = "", extra: dict = None) -> Exception
```

还原规则（按顺序）：

1. `type_name` 是 `errors.py` 里的 `CommandFailed` 或其子类 → `cls(message, extra["code"], extra["response"])`，因此 `e.code` / `e.response` 保留。
2. 是 `errors.py` 里其他 `CyberDogError` 子类（`RemoteError` 除外）→ `cls(message)`。远程异常不带狗端 traceback。
3. 是 `ValueError` / `TypeError` / `KeyError` → 同类型本地异常。
4. 其他 → `RemoteError(type_name, message, remote_traceback)`。

因此远程模式与本地模式的异常类型对用户来说是一致的（测试 `test_remote_error_type_preserved` 覆盖）。

---

## 7. 远程连接细节

涉及文件：`dog.py`（`connect`）→ `backends/remote.py`（`RemoteBackend.connect_ssh`）→ `deploy.py`（`SshSession`）→ 狗上 `agent/__main__.py` / `agent/server.py`。

### 7.1 `connect_ssh` 与 `agent_options`

```python
RemoteBackend.connect_ssh(host: str, user: str = "mi", password: Optional[str] = None,
                          key_file: Optional[str] = None, port: int = 22,
                          namespace: Optional[str] = None, **agent_options) -> RemoteBackend
```

流程：从 `agent_options` 取出 `ssh_timeout` → `SshSession(..., timeout=ssh_timeout)` 登录 → `session.start_agent(namespace=namespace, **agent_options)` → `session.open_tunnel(agent.port)` → `RemoteBackend(sock, on_close=..., log_tail=agent.log_tail)`。`start_agent` / `open_tunnel` 失败会关闭 SSH 后再抛出。关闭时 `on_close` 依次关闭代理通道和 SSH。

`RemoteBackend.__init__` 发送 `info` 后检查返回的 `protocol`，与本地 `PROTOCOL_VERSION`（当前为 2）不一致时先 `close()` 再抛 `CyberDogError`。

除 `ssh_timeout` 外，`agent_options` 就是 `SshSession.start_agent` 的参数：

```python
start_agent(self, namespace: Optional[str] = None, setup_scripts: Sequence[str] = DEFAULT_SETUP_SCRIPTS,
            inherit_env_from: Optional[str] = "motion_manager", env: Optional[Dict[str, str]] = None,
            start_timeout: float = 30.0, watchdog: float = 1.0, discovery_timeout: float = 8.0,
            accept_timeout: float = 30.0) -> AgentProcess
```

| 选项 | 默认值 | 说明 |
|---|---|---|
| `setup_scripts` | `("/opt/ros2/galactic/setup.bash", "/opt/ros/galactic/setup.bash", "/opt/ros2/cyberdog/setup.bash")` **【未核实】** | 依次 source；**不存在的自动跳过** |
| `inherit_env_from` | `"motion_manager"` **【未核实进程名】** | 从该进程的 `/proc/<pid>/environ` 继承 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`、`RMW_IMPLEMENTATION`、`CYCLONEDDS_URI`、`FASTRTPS_DEFAULT_PROFILES_FILE`（`INHERITED_ENV`）。传 `None` 关闭 |
| `env` | `None`（即 `{}`） | 额外 `export` 的环境变量，在继承之后执行（可以覆盖继承值）。键必须匹配 `^[A-Za-z_][A-Za-z0-9_]*$`，否则 `ValueError` |
| `start_timeout` | `30.0` 秒 | 等待 READY 行的最长时间 |
| `watchdog` | `1.0` 秒 | 代理看门狗超时，传给代理的 `--watchdog` |
| `discovery_timeout` | `8.0` 秒 | 代理自动探测 `/mi_*` 命名空间的最长时间，传给代理的 `--discovery-timeout` |
| `accept_timeout` | `30.0` 秒 | 代理打印 READY 后等待电脑连入的最长时间，传给代理的 `--accept-timeout` |
| `ssh_timeout` | `10.0` 秒 | SSH 建立连接的超时（`SshSession(timeout=...)`），由 `connect_ssh` 取出，不传给 `start_agent` |
| `python` | `"python3"` | 狗上运行代理的解释器，必须是编译 rclpy 时用的那个版本，doctor 会帮你确认 |

SSH 本身的固定参数（`SshSession.__init__`）：`AutoAddPolicy`（首次连接自动信任主机密钥）、`load_system_host_keys()`、transport keepalive 5 秒；只有在 `password` 和 `key_file` 都没给时才启用 ssh-agent / 搜索 `~/.ssh` 密钥。

```python
dog = CyberDog.connect("192.168.31.50", password="123",
                       ssh_timeout=20, discovery_timeout=15, accept_timeout=60)
```

### 7.2 部署目录与哈希

`SshSession.deploy()`：

1. `package_files()`：遍历电脑上 `cyberdog2/` 包目录，收集所有 `.py`（跳过 `__pycache__`，按名字排序），路径形如 `cyberdog2/agent/__main__.py`。
2. `package_hash(files)`：对每个文件依次 `sha1.update(相对路径 + b"\0" + 内容)`，取十六进制前 **12** 位。
3. 目标目录 `<狗的 $HOME>/.cyberdog2/<哈希>/`（HOME 通过 `echo $HOME` 获取）。
4. 若 `<目录>/.complete` 存在则跳过上传；否则 `_upload()`：`mkdir -p` 所有子目录（失败 → `AgentStartError`），SFTP 写入每个文件，最后写 `.complete`（内容 `ok\n`）。
5. 清理旧版本：`find ~/.cyberdog2 -mindepth 1 -maxdepth 1 -type d ! -name <当前哈希> -exec rm -rf {} +`，只保留当前版本目录；命令失败 → `AgentStartError("清理狗上的旧版本目录失败: ...")`。

所以**代码有任何改动都会生成新目录**，并在同一次部署中删除其他版本的目录。
注意：如果同一只狗上同时有另一台电脑用**不同版本**的代码连着，它的部署目录也会被删掉（正在运行的代理进程不受影响，但下次启动需要重新上传）。

### 7.3 代理启动脚本（`build_agent_command`）

生成 `bash -c '<脚本>'`，在带 **pty** 的 SSH exec 通道上运行（SSH 断开时代理收到 SIGHUP 退出，不残留进程）。脚本内容依次为：

1. `set -e`
2. 对每个 `setup_scripts`：`if [ -f 脚本 ]; then set +e; . 脚本; set -e; fi`
3. 如果 `inherit_env_from` 不为空：
   - `PID=$(pgrep -o -f '[m]otion_manager' || true)`（取最老的匹配进程；`[m]` 写法避免匹配到自己）
   - 找不到 → 打印 `CYBERDOG2_AGENT_ERROR 找不到进程 ...` 并 `exit 3`
   - `/proc/$PID/environ` 不可读 → 打印 `CYBERDOG2_AGENT_ERROR 没有权限读取 ...` 并 `exit 3`
   - `env_inherit_loop("/proc/$PID/environ")` 生成的一行：
     `while IFS= read -r -d '' KV; do case "$KV" in ROS_DOMAIN_ID=*|...) export "$KV";; esac; done < /proc/$PID/environ`。
     逐条读取以 `\0` 分隔的 `名=值`，只 `export` `INHERITED_ENV` 里的变量；值原样作为一个参数传给 `export`，**值里有单引号、空格、换行都不会出错**
4. 对 `env` 的每一项（按键排序）`export K=<shlex.quote(V)>`
5. 打印诊断行：`CYBERDOG2_ENV ROS_DOMAIN_ID=... ROS_LOCALHOST_ONLY=... RMW_IMPLEMENTATION=...`
6. `cd <部署目录>`
7. `exec <python> -u -m cyberdog2.agent --watchdog <watchdog> --discovery-timeout <discovery_timeout> --accept-timeout <accept_timeout> [--namespace <ns>]`

`build_agent_command(base_dir, namespace, setup_scripts, inherit_env_from, env, watchdog, discovery_timeout=8.0, accept_timeout=30.0)`。

电脑端有个 `cyberdog2-agent-log` 线程把通道输出逐行存进最近 200 行的环形缓冲（`AgentProcess.log_tail()`），用于错误信息。

### 7.4 代理进程（`python3 -m cyberdog2.agent`）

命令行参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--port` | `0` | 监听端口，0 表示随机 |
| `--namespace` | 无 | 手动指定命名空间，不填自动探测 |
| `--discovery-timeout` | `8.0` | 自动探测命名空间的最长秒数 |
| `--watchdog` | `1.0` | 看门狗超时秒数 |
| `--accept-timeout` | `30.0` | 等待客户端连接的最长秒数，超时代理以 traceback 退出 |
| `--mock` | 关 | 用 `MockBackend`（调试代理本身） |

启动成功后打印 **READY 行**（格式固定）：

```
CYBERDOG2_AGENT_READY port=<端口> namespace=<命名空间>
```

电脑端识别以 `CYBERDOG2_AGENT_READY` 开头的第一行，按空格切分后把 `k=v` 解析成 dict，取 `port` 和 `namespace`。
如果通道关闭时都没出现 READY 行 → `AgentStartError("代理启动失败。输出: ...")`；超过 `start_timeout` → `AgentStartError("代理在 Ns 内没有启动完成 ...")`。

代理特点：只监听 `127.0.0.1`（不对外开放端口）；**只服务一个客户端**，accept 之后立即关闭监听 socket；客户端断开后执行 `_cleanup()`（见 7.6），无论它是否抛错都会关闭连接，然后 `serve_one()` 返回（`_cleanup` 出错时以 traceback 抛出），`backend.close()`，进程退出。

### 7.5 SSH 隧道

`SshSession.open_tunnel(port)` = `transport.open_channel("direct-tcpip", ("127.0.0.1", port), ("127.0.0.1", 0))`，返回 paramiko `Channel`（类 socket 对象），直接交给 `LineStream`。走的是同一条 SSH 连接，不需要额外开端口。

### 7.6 看门狗、心跳与超时参数

| 参数 | 值 | 位置 | 作用 |
|---|---|---|---|
| 伺服心跳周期 | 0.05 s（20Hz） | `ServoLoop(period=0.05)`，`RosBackend` / `MockBackend` 的 `servo_period` | motion_manager 要求伺服帧间隔 < 200ms，否则强制站立 |
| 客户端 ping 间隔 | 0.2 s | `RemoteBackend(ping_interval=0.2)` | 让代理知道客户端还活着 |
| 代理看门狗 | 1.0 s，每 `watchdog/4` 检查一次 | `AgentServer(watchdog=1.0)` / `--watchdog` | 超过该时间没有收到**任何**客户端消息 → `stopped_by_watchdog=True`，`servo_stop()`，关闭连接 |
| 代理致命错误检查 | 同上 | `_watchdog_loop` | 后端 `fatal_error` 非空 → 发 `fatal` 事件后断开 |
| 网络余量 `_NET_MARGIN` | 5.0 s | `remote.py` | 客户端等待 = 请求超时 + 5s。狗端 `RosBackend.call_service` / `send_goal` 的 `timeout` 是总时长（等服务 / 服务器出现 + 等结果共用一个截止时间，`_left(deadline)`），所以狗端一定先于客户端超时 |
| 伺服死人开关 | 调用方指定（`walk` 默认 `duration+1`） | `set_velocity(timeout=)` → `servo_set` 的 `expire_after` → `ServoLoop` | 超时没有新的 `servo_set` → 自动发 END；远程模式下在代理端计时 |
| SSH keepalive | 5 s | `SshSession` | |
| 断线收尾 | — | `AgentServer._cleanup` | 客户端断开后依次：`servo_stop()` → 若本次会话 `send_goal` 成功过：`release_tasks`（按狗的实际状态结束任务，服务超时 10s）并逐个 `cancel_goal`（超时 5s）→ 停止所有推流 → 退订所有订阅。每一步都会尝试，失败时打印 traceback 后继续，最后把第一个错误重新抛出 |

客户端各请求的超时（再加 5 秒余量）：`info` 10、`publish` 5、`subscribe` 10、`unsubscribe` 5、`servo_set` 2、`servo_stop` 2、`image_stream_start` 10、`image_stream_stop` 5、`list_names` 10；`call_service` / `send_goal` / `wait_goal` / `cancel_goal` / `latest_image` 用调用方传入的 `timeout`（`None` 表示无限等）。

安全链条总结：电脑断网 / 进程卡死 → 代理 1 秒内收不到 ping → 停止伺服并断开 → `_cleanup` 停导航任务 → 代理退出；程序主线程卡死但 ping 线程仍在 → 伺服死人开关（如果设置了 `timeout`）到期自动停下；SSH 断开 → pty 发 SIGHUP → 代理退出（SIGHUP 默认直接终止进程，`finally` 里的 `_cleanup` 不一定有机会执行，见 12.3 #7）。

---

## 8. 代理通信协议（`protocol.py`）

供以后用其他语言（如 C#、JS）写客户端参考。

### 8.1 帧格式

- 传输层：一条 TCP 流（这里是 SSH direct-tcpip 通道）。
- 每条消息是**一行 UTF-8 JSON**，以 `\n` 结尾（`json.dumps(..., ensure_ascii=False, allow_nan=False)`）。
- `bytes` / `bytearray` 编码为 `{"__b64__": "<base64>"}`；解码时**只有一个键且为 `__b64__`** 的对象才还原成 bytes。
- 非有限浮点（NaN、+Inf、-Inf）编码为 `{"__f__": "nan"}` / `{"__f__": "inf"}` / `{"__f__": "-inf"}`（即 `repr(x)`），解码时**只有一个键且为 `__f__`** 的对象用 `float(...)` 还原。JSON 本身仍用 `allow_nan=False`，所以线上不会出现裸 `NaN` / `Infinity`。（`sensor_msgs/Range` 超量程按 REP 117 用 ±Inf 表示，TOF / GPS 也可能有 NaN。）
- `dict` 的键一律转成字符串；`tuple` 编码为数组。
- `PROTOCOL_VERSION = 2`（1 → 2 的变化就是加入 `__f__` 编码），由 `info` 返回；客户端（`RemoteBackend.__init__`）发现不一致时抛 `CyberDogError`。

### 8.2 请求 / 响应

```
请求  {"id": 1, "op": "call_service", "args": {...}}
成功  {"id": 1, "ok": true,  "result": ...}
失败  {"id": 1, "ok": false, "error": {"type": "ServiceTimeout", "message": "...", "traceback": "...", "extra": {}}}
心跳  {"op": "ping"}            # 不带 id，代理不回复，只刷新看门狗
```

- `id` 由客户端生成（递增整数），响应按 `id` 匹配。代理用 8 线程池**并发**处理请求，响应顺序不保证与请求一致。
- 客户端本地超时后会丢弃迟到的响应。
- 未知 `op` → `ok: false`，`type: "ValueError"`。
- 结果无法编码成 JSON 时（`json` 抛非 `OSError` 的异常），代理改为回一条 `ok: false` 的错误响应（`error` 就是编码异常，通常 `type: "TypeError"`），客户端立即抛出，而不是等到超时。

### 8.3 全部 op

名字参数都可以是相对名（由代理拼命名空间）或以 `/` 开头的绝对名。

| op | args | result |
|---|---|---|
| `info` | `{}` | `{"namespace": str, "protocol": 2}` |
| `call_service` | `{"name", "type", "request": dict, "timeout": float=5.0}` | 响应 dict |
| `publish` | `{"topic", "type", "data": dict}` | `null` |
| `subscribe` | `{"topic", "type"}` | 订阅 id（int）。之后消息以 `sub` 事件推送 |
| `unsubscribe` | `{"sub": int}` | `null` |
| `send_goal` | `{"name", "type", "goal": dict, "feedback": bool, "timeout": float=10.0}` | goal_id（str）。`feedback=true` 时反馈以 `feedback` 事件推送 |
| `wait_goal` | `{"goal": str, "timeout": float 或 null}` | 结果 dict（ROS 后端附带 `_status`） |
| `cancel_goal` | `{"goal": str, "timeout": float=5.0}` | `null` |
| `servo_set` | `{"cmd": dict}`（MotionServoCmd 字段，不含 `cmd_type`；可选 `"expire_after": 秒` 死人开关） | `null` |
| `servo_stop` | `{}` | `null` |
| `latest_image` | `{"topic", "timeout": float=3.0, "jpeg": bool=false}` | 帧 dict：`{"width","height","encoding","data": bytes,"format","stamp"}` |
| `image_stream_start` | `{"topic", "fps", "jpeg", "quality"}` | 流 id（int）。之后帧以 `frame` 事件推送 |
| `image_stream_stop` | `{"stream": int}` | `null` |
| `list_names` | `{}` | `{"topics": [...], "services": [...], "actions": [...]}` |
| `ping` | （无 id、无 args） | 不回复 |

### 8.4 事件（代理主动推送，没有 `id`）

| `ev` | 格式 | 说明 |
|---|---|---|
| `sub` | `{"ev": "sub", "sub": <订阅 id>, "data": {...}}` | 订阅到的消息。可能在 `subscribe` 的响应**之前**到达（例如 latched 消息），客户端要先缓存未知 id 的事件 |
| `feedback` | `{"ev": "feedback", "goal": "<goal_id>", "data": {...}}` | 动作反馈（只在 `send_goal` 的 `feedback=true` 时发送）。可能早于 `send_goal` 的响应到达，客户端要先缓存，拿到 goal_id 后补发 |
| `frame` | `{"ev": "frame", "stream": <流 id>, "data": <帧 dict>}` | 视频帧。代理每个流只保留最新一帧，网络慢时丢旧帧。也可能早于 `image_stream_start` 的响应到达 |
| `fatal` | `{"ev": "fatal", "error": {...同错误格式...}}` | 代理后端出现致命错误，随后代理断开连接 |
### 8.5 会话生命周期

1. 客户端连接 → 先发 `info` 拿命名空间，并核对 `protocol` 版本。
2. 每 0.2s 发 `ping`；任何消息都会刷新看门狗。
3. 断开连接（或看门狗超时）→ 代理停伺服、（本次会话发过导航目标时）停止并取消导航任务、停推流、退订，进程退出。**一个代理进程只服务一次连接。**

---

## 9. doctor 自检工具

```bash
pip install paramiko
set CYBERDOG_PASSWORD=123                 # Windows cmd；bash 用 export CYBERDOG_PASSWORD=123
python -m cyberdog2.doctor 192.168.31.50
```

参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `host` | 必填 | 狗的 IP |
| `-u` / `--user` | `mi` | SSH 用户 |
| `--key` | 无 | 私钥路径；不填则用密码 |
| `--port` | 22 | SSH 端口 |
| `--namespace` | 无 | 手动指定命名空间 |
| `--no-inherit-env` | 关 | 不从狗的 ROS 进程继承 ROS 环境变量（对应 `inherit_env_from=None`），一般配合 `--env` 使用 |
| `--inherit-env-from NAME` | `motion_manager` | 从哪个进程继承 ROS 环境变量；同时决定"进程"那一步检查的进程名 |
| `--env KEY=VALUE` | 无，可重复 | 额外导出给代理的环境变量（对应 `env=`），例如 `--env ROS_DOMAIN_ID=42`。不含 `=` 时 argparse 报错退出（退出码 2）；变量名非法 → `ValueError` |
| `--wait` | 10 | 等待 `REQUIRED_*` 接口被发现的最长秒数，到齐就提前结束 |

密码：不用 `--key` 时，先读环境变量 `CYBERDOG_PASSWORD`，没有则交互输入。

```bash
python -m cyberdog2.doctor 192.168.31.50 --no-inherit-env --env ROS_DOMAIN_ID=42 --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python -m cyberdog2.doctor 192.168.31.50 --inherit-env-from cyberdog_motion
```

doctor 没有暴露 `ssh_timeout` / `discovery_timeout` / `accept_timeout` / `setup_scripts`，这些用默认值。

执行步骤与输出解读（每行以 `==> ` 开头）：

| 步骤 | 输出 | 怎么看 |
|---|---|---|
| `SSH 登录 mi@ip:22` | 无（失败直接抛 paramiko 异常） | 认证失败 → 没开开发者权限 / 密码错；连接失败 → IP 错，或 WiFi 不支持 SSH【未核实】，可改用 Type-C（`192.168.55.1`） |
| `狗端系统信息` | `uname -a`、`python3 --version`、`/etc/os-release` 前两行 | 确认 Python 版本（≥ 3.6 即可）和 Ubuntu 版本 |
| `用 <python> 导入 rclpy 和 protocol` | 先 source 环境脚本，再用 `--python` 指定的解释器（默认 `python3`）执行 `import rclpy, protocol.msg`，打印 Python 版本和两个包的路径 | 失败直接抛 `RuntimeError`。看 rclpy 安装路径里的 `pythonX.Y`，用 `--python python3.X` 重试；连接时也要传同样的 `python=` |
| `用 <python> 试压缩一张 JPEG` | 用狗上的 Pillow 压缩一张 8×8 的图，打印 Pillow 版本和压缩后的字节数 | 失败直接抛 `RuntimeError`，此时相机取帧和视频流用不了 |
| `ROS 环境脚本` | 每个 `DEFAULT_SETUP_SCRIPTS` 显示"存在 / 不存在" | **一个都不存在**说明路径不对，要找到真实路径并通过 `setup_scripts=` 传入，然后修改 `deploy.DEFAULT_SETUP_SCRIPTS` |
| `<进程名> 进程`（默认 `motion_manager 进程`） | `pgrep -a -f [m]otion_manager` 前 3 行（进程名来自 `--inherit-env-from`；即使给了 `--no-inherit-env` 也会执行这一步） | "没有找到" → 进程名不对，`ps aux \| grep -i motion` 查真名，再用 `--inherit-env-from` 指定 |
| `部署并启动狗端代理` | 代理日志：`CYBERDOG2_ENV ...` 行（继承到的 DDS 变量）和 READY 行 | 失败抛 `AgentStartError`，看附带输出 |
| `命名空间: /mi_xxx` | 探测到的命名空间 | `NamespaceNotFound` → DDS 域不一致或系统没启动完 |
| 接口统计 | ROS 发现是异步的，先轮询到 `REQUIRED_*` 到齐或 `--wait` 超时，再打印话题 / 服务 / 动作数量和每个接口的 `OK` 或 `缺失`（真机上 `touch_status` 曾在代理就绪后才被发现） | 等满 `--wait` 仍"缺失" → 对应节点没运行，或这版固件接口名不同，对照 `list_names()` 修改 `constants.Iface` |
| `读取电池` | `电量 N%，电压 X.XXV` | `SensorTimeout` → `bms_status` 收不到；也可能是 QoS 不匹配 |

最后有缺失接口时打印 `自检未通过，缺失的接口: [...]` 并以退出码 1 结束；全部正常打印 `自检通过`、退出码 0。其他任何错误都直接以完整 traceback 退出（let it crash）。

---

## 10. 扩展指南

### 10.1 新增一个 ROS 接口封装（以添加一个服务为例）

假设要封装一个"查询设备信息"的服务。下面的名字 `device_info_get` / `protocol/srv/DeviceInfoGet` 和字段都是**示意**，
实际请先用 `dog.list_names()` 在真机上确认，并在 bridges 的 `protocol/ros/srv/` 里查清请求 / 响应字段。

**第 1 步：在 `constants.Iface` 登记接口名和类型**

```python
class Iface:
    ...
    DEVICE_INFO = ("device_info_get", "protocol/srv/DeviceInfoGet")   # 示意，需真机核实
```

**第 2 步：在功能模块里写封装**（放进最相关的模块，或新建模块并挂到 `CyberDog.__init__`）

```python
# 例如在 cyberdog2/sensors.py 的 Sensors 类里
def device_info(self, timeout: float = 5.0) -> dict:
    """查询设备信息。"""
    name, srv_type = Iface.DEVICE_INFO
    resp = self._dog._call(name, srv_type, {"enables": [True]}, timeout=timeout)
    if resp.get("code", 0) != 0:
        raise CommandFailed("查询设备信息失败: code=%s" % resp.get("code"), resp.get("code", 0), resp)
    return resp
```

要点：

- 服务一律经 `self._dog._call(...)`（自动做 `_check()`）；订阅一律经 `self._dog._subscribe(...)`（自动包装回调）。
- 参数越界直接 `ValueError`，**不要截断、不要静默兜底**；失败码抛 `CommandFailed`（或子类）并带上 `code` 和 `response`。
- 不需要改后端：`call_service` / `subscribe` / `send_goal` 对任意类型通用，狗端用 `load_type("pkg/srv/Name")` 动态导入，字段由 `fill_message` 按 dict 填充（字段名写错会抛 `AttributeError`）。
- 新增常量（编码表等）也放进 `constants.py`。

**第 3 步：给 mock 加默认响应**（`backends/mock.py` 的 `_default_services()`；如果是话题，加到 `_default_latched()`）

```python
"device_info_get": lambda r: {"code": 0, "sn": "MOCK-SN"},
```

不加的话 mock 调用会抛 `ServiceUnavailable`。

**第 4 步：写测试**（`tests/test_modules.py`，用 `dog` fixture）

```python
def test_device_info(dog):
    assert dog.sensors.device_info()["sn"] == "MOCK-SN"
    assert dog.backend.calls_to("device_info_get")[-1] == {"enables": [True]}


def test_device_info_failure(dog):
    dog.backend.set_response("device_info_get", {"code": 5})
    with pytest.raises(CommandFailed):
        dog.sensors.device_info()
```

需要验证远程链路时用 `remote` fixture（`remote.dog` / `remote.mock`）。

**第 5 步**：如果这个接口是"必需的"，加入 `REQUIRED_SERVICES` / `REQUIRED_TOPICS`，doctor 会检查它；更新本文档第 4、5 节。

### 10.2 修正真机上发现的接口名

1. 运行 `python -m cyberdog2.doctor <ip>`，或：
   ```python
   with CyberDog.connect(ip, password="123") as dog:
       for kind, names in dog.list_names().items():
           print(kind); print("\n".join(names))
   ```
2. 找到真实名字后**只改 `constants.Iface`**（以及 `REQUIRED_*`）。
3. 注意：伺服话题名 `"motion_servo_cmd"` / `"protocol/msg/MotionServoCmd"` 在 `backends/ros.py` 和 `backends/mock.py` 的 `ServoLoop` 构造里是**写死的字符串**，没有引用 `Iface.MOTION_SERVO`，改名时这两处也要改。
4. mock 的 `_default_services()` / `_default_latched()` 的键名、`MockBackend.list_names()` 里写死的 `touch_status`，以及测试里用到的名字要同步修改。
5. 类型不对（例如图像是 `CompressedImage`）时不只是改名：`RosBackend.latest_image` / `image_stream_start` 里写死了 `sensor_msgs/msg/Image` 并直接读 `m.width/m.height/m.encoding/m.data`，需要改代码。
6. 改完跑 `python -m pytest`。因为部署目录按代码哈希区分，下一次 `connect()` 会自动上传新代码。

脚本路径 / 进程名不对时：临时用 `connect(..., setup_scripts=[...], inherit_env_from="真实进程名")`，确认后修改 `deploy.DEFAULT_SETUP_SCRIPTS` 和 `start_agent` 的默认值（doctor 的 `--inherit-env-from` 默认值 `"motion_manager"` 也要同步改；临时可以直接传 `--inherit-env-from 真实进程名`）。

### 10.3 新增一个后端方法（五处同步）

当现有的 `call_service / publish / subscribe / send_goal / servo_* / image_*` 组合不够用（例如需要在狗端做聚合运算、需要新的流式数据）时，要新增后端方法。以假想的 `get_param(node, name)`（读取 ROS 参数）为例：

| # | 文件 | 要做的事 |
|---|---|---|
| 1 | `backends/base.py` | 在 `Backend` 里加抽象方法 `def get_param(self, node: str, name: str, timeout: float = 5.0): raise NotImplementedError`，写清语义 |
| 2 | `backends/mock.py` | 实现一个可测试的版本（例如读 `self.params` 字典，并提供测试辅助） |
| 3 | `backends/ros.py` | 用 rclpy 实现。rclpy / ROS 包只能在函数内部 import；回调用 `self._guard(...)` 包装；阻塞等待用 `self._wait(future, timeout, what)` |
| 4 | `backends/remote.py` | `def get_param(self, node, name, timeout=5.0): return self._request("get_param", {"node": node, "name": name, "timeout": timeout}, timeout)` |
| 5 | `agent/server.py` | 在 `_dispatch` 里加 `if op == "get_param": return b.get_param(a["node"], a["name"], a.get("timeout", 5.0))`。如果有订阅 / 流之类的资源，记得在 `_cleanup` 里用 `attempt(...)` 释放（保证一步失败不跳过后面的安全步骤） |

然后：

- 在功能模块里经 `self._dog._check()` 后调用 `self._dog._backend.get_param(...)`。
- 更新本文第 8 节的 op 表（如果新增事件类型，还要改 `RemoteBackend._on_message`）。
- 返回值必须能被协议编码（dict / list / tuple / 基本类型 / bytes；NaN / Inf 会按 `__f__` 编码，可以出现）。其他类型（numpy 标量、自定义对象等）会让这次请求以 `TypeError` 失败，先用 `to_plain` 之类转换。
- 狗端代码必须兼容 **Python 3.6**：不要用 `from __future__ import annotations`、`list[int]` 之类的注解、`X | Y` 类型、`dataclasses`、`:=` 等（完整清单见 11.1 节）。
- 在 `tests/test_remote.py` 里用 `remote` fixture 写端到端测试。

---

## 11. 测试说明

```bash
pip install -e .[dev]          # 或 pip install pytest paramiko numpy pillow
python -m pytest               # pyproject.toml 中 testpaths = ["tests"]
python -m pytest tests/test_remote.py -v
```

当前状态（实测）：**134 passed**，约 80 秒（其中房间建模的完整流程约 60 秒）。没有安装 opencv-python 时，`test_to_bgr` 会被 `importorskip` 跳过。
**所有测试都基于 `MockBackend`，不需要真机，也不需要 ROS / SSH。**

| 文件 | 测试数 | 覆盖内容 |
|---|---|---|
| `tests/conftest.py` | — | fixture：`dog`（`CyberDog.mock()`）；`remote`（`RemotePair`：本机回环上起一个 `AgentServer(MockBackend)`，客户端用 `RemoteBackend` 直连，**跳过 SSH 层**，`ping_interval=0.1`） |
| `tests/test_motion.py` | 11 | `do` 请求字段与数组长度、枚举 / 整数动作、未知动作、伺服 id 用于 `do`、`MotionRejected` 与 `code`、`rpy` 越界、速度超限不截断、`walk` 心跳帧数与 END 帧、心跳间隔 < 200ms、回调崩溃停止运动并抛 `CallbackError`、`close()` 停伺服 |
| `tests/test_modules.py` | 20 | LED 预定义 / RGB / 非法组合 / 错误码；语音与失败；音量读写与越界；电池；触摸编码；`SensorTimeout`；TOF 矩阵；相机失败；未登记服务 `ServiceUnavailable`；算法开关；手势事件；跟随目标字段；导航失败结果；`stop_mapping`；`pick_namespace` 正常 / 缺失 / 多个 |
| `tests/test_remote.py` | 11 | 命名空间与运动经代理；远程异常类型与 `code` 保留；latched 与事件转发；伺服在代理端运行；bytes 往返；远程回调崩溃；**断开连接触发 SERVO_END**；**ping 停止时看门狗触发**；启动脚本内容（`[m]otion_manager`、env、非法变量名）；包哈希稳定；**狗端代码满足 Python 3.6 语法、注解在定义时可求值** |
| `tests/test_stream.py` | 11 | `RateLimiter`；`LatestSlot` 丢旧帧；mock 推流与自动开 / 关相机；慢消费者丢帧；读超时；参数校验不触发相机；回调崩溃后流抛 `CallbackError` 且相机被关闭；远程推流；远程断线抛 `ConnectionLost` 且代理停止推流；`to_bgr` 通道顺序 |
| `tests/test_fixes.py` | 42 | 原 12.2 节各问题的回归测试：NaN/Inf 协议往返、远程传感器含 Inf 不拖垮代理、无法编码的响应立即报错（< 2s）；断线 / `close()` 停止导航任务（以及没启动导航时不调 stop）；未知步态 `ValueError`；回调崩溃后 `estop` 仍下发；`NavTask` 检查 `_check()`；`follow_uwb` 距离校验；`cancel_face` 检查结果；`enable/disable` 必须给算法；LED 不再静默忽略参数；`TopicCache` 的 `max_age` / `age`；远程反馈提前到达仍送达且 `wait_goal` 后释放；协议版本不一致抛错；`env_inherit_loop` 处理引号（用本机 bash 实际执行，没有 bash 时 skip）；启动命令带 discovery / accept 超时；伺服死人开关到期 / 刷新保活；`walk` 设置死人开关（远程）；已关闭的流 `read()` 抛 `CyberDogError` |

`tests/test_roommapper.py`（30 个）覆盖 `roommapper` 应用：不给算法真实位姿，在模拟 L 形房间（180° 雷达、随机无回波、8% 打滑）走完一圈并检查漂移和碰撞；不把房间中间的柱子当成房间；地图上定位；A* 绕障与报错；SQLite；HTTP 接口失败路径；建图 → 规划 → 执行的完整流程。

### 11.1 Python 3.6 兼容性检查

实测的狗是 Ubuntu 18.04，系统自带的 `python3` 是 **3.6**。上传到狗上的是整个 `cyberdog2` 包，所以全部代码都要能在 3.6 上运行。`tests/test_remote.py` 里有两项检查：

- `test_dog_side_code_is_python36_syntax`：对每个文件执行 `ast.parse(..., feature_version=(3, 6))`，并禁止使用 `from __future__ import annotations`（这个写法 3.7 才支持）。
- `test_annotations_resolvable_at_def_time_on_py36`：3.6 会在 `def` 执行时就求值注解，所以注解里出现的名字必须在函数定义之前就已绑定。需要前向引用时要加引号，例如 `-> "Frame"`。

这两项都是静态检查，发现不了**运行时 API** 问题。例如以下写法在 3.6 上会出错：

- 注解或默认值里的 `list[int]` / `dict[str, int]`（3.9 才支持），一律改用 `typing.List` / `Dict` / `Optional`
- `X | Y` 类型写法、`dict | dict`、`str.removeprefix`、`functools.cache`、`dataclasses`（3.7）、`subprocess.run(capture_output=...)`（3.7）、`asyncio.run`（3.7）、`contextlib.nullcontext`（3.7）、海象运算符 `:=`（3.8）

新代码最好能用 3.6 解释器真正跑一遍（例如在狗上执行 `python3 -c "import cyberdog2.agent.server"`）。

---

## 12. 已知限制与待真机核实事项

### 12.1 待真机核实（按建议顺序执行）

| # | 事项 | 怎么核实 | 不对时怎么改 |
|---|---|---|---|
| 1 | WiFi 下能否直接 SSH（官方只写了 Type-C `192.168.55.1` 和网线 `192.168.44.1`） | `ssh mi@<WiFi IP>` | 改用 Type-C 连接；在 README / first_boot 里注明 |
| 2 | ROS 环境脚本路径 `DEFAULT_SETUP_SCRIPTS` | doctor 第 3 步；`ls /opt/ros*/*/setup.bash` | 修改 `deploy.DEFAULT_SETUP_SCRIPTS` |
| 3 | `motion_manager` 进程名，以及 `mi` 用户能否读其 `/proc/<pid>/environ` | doctor 第 4 步；`ps aux \| grep -i motion` | 改 `start_agent` 默认 `inherit_env_from` 与 doctor；无权限时用 `inherit_env_from=None, env={...}` |
| 4 | 命名空间自动探测（应恰好有一个 `/mi_*`） | doctor "命名空间"一行 | 多个时固定传 `namespace=` |
| 5 | `REQUIRED_*` 接口名 | doctor 接口检查 | 改 `constants.Iface` |
| 6 | `motion_status` 话题名与 `MotionStatus` 字段 `motion_id`、`switch_status` | `ros2 topic list \| grep status`；`ros2 interface show protocol/msg/MotionStatus` | 改 `Iface.MOTION_STATUS` / `MotionState` |
| 7 | 相机图像话题 `image`、类型是否是 `sensor_msgs/msg/Image`、编码（rgb8 / bgr8 / nv12？） | `camera.start()` 后 `ros2 topic list`、`ros2 topic info -v` | 改 `Iface.IMAGE`；若编码不在 `_CHANNELS` 里需扩展 `Frame` / `encode_jpeg` |
| 8 | 所有消息 / 服务 **字段名**（bridges rolling 分支可能和狗上固件版本不同），尤其是 `MotionServoCmd` / `MotionResultCmd` 是否都有 `acc_des`、`ctrl_point`、`foot_pose` 等 `fill_motion_arrays` 会补的字段、是否为定长数组、长度是否与 `MOTION_ARRAY_SIZES` 一致 | `ros2 interface show protocol/msg/MotionServoCmd` 等 | 字段不存在时 `fill_message` 抛 `AttributeError`；调整 `MOTION_ARRAY_SIZES` 或请求组装代码 |
| 9 | Galactic 版 `rosidl_runtime_py.set_message_fields` 能否给定长数组（numpy 类型）字段和**消息数组**（`go_to` 的 `poses` 是 PoseStamped 列表）赋值 | 真机上 `motion.sit()`、`set_velocity()`、`nav.go_to()` | 如不支持，需在 `ros.fill_message` 里自己实现递归赋值 |
| 10 | QoS：`RosBackend.subscribe` 对所有话题都用 `qos_profile_sensor_data`（best effort, depth 5）；若某发布者是 transient_local（latched）或必须可靠传输，可能收不到 / 丢消息 | `ros2 topic info -v <topic>` 看发布者 QoS | 为特定话题选择合适 QoS |
| 11 | 伺服心跳 20Hz 是否被 motion_manager 接受；`cmd_type` START 帧是否必需（本库直接发 DATA，结束发 END） | 抬起狗测 `walk(vx=0.2, duration=1)` | 修改 `ServoLoop` |
| 12 | `send_goal` 对跟随类任务的语义：动作是立即返回 ACCEPT，还是一直保持到 stop？`NavTask.wait()` 在跟随中是否会卡住 | 跑 `examples/05_follow_person.py` | 调整示例与 `NavTask.wait` 文档 |
| 13 | `audio.say` 的请求结构（`speech` 子消息字段）、`play_id` 取值 | 真机 `say("你好")`、`play(9000)` | 改 `audio.py` |
| 14 | 狗上能否压缩 JPEG（远程取图和推流默认压缩）。已实测：有 Pillow 5.1.0，没有 `cv2` | doctor 的"试压缩一张 JPEG"一步 | 失败时传 `jpeg=False`（带宽大） |
| 15 | 断网安全：持续行走时关掉电脑 WiFi，狗应在约 1 秒内停下 | README 真机验证清单第 4 步 | 调整 `watchdog` |
| 16 | `rclpy.action.get_action_names_and_types` 在 Galactic 的可用性（`list_names` 用到） | doctor 的 `list_names` | 改 import 路径 |
| 17 | `stop_algo_task(task_id=0 ALL)` 的语义：是否能停下跟随 / 建图 / 导航；建图中被 ALL 停止时地图是否丢失；是否也会停掉 App 启动的任务（`close()` 和代理断线收尾都会发它） | 跟随中直接关闭程序 / 拔掉网络，观察狗是否停止跟随 | 已改为按状态结束（`navigation.release_tasks`），见 #24；真机仍需确认 |
| 18 | 死人开关到期时发出的 END 帧（不带后续 DATA）能否让狗平稳停下 | 抬起狗测 `set_velocity(0.2, timeout=0.3)` 后不再调用 | 修改 `ServoLoop._run` 的到期处理 |
| 20 | 雷达零度方向与狗头是否一致 | 把狗放在房间中间，看页面上的雷达点是否贴着实际的墙 | `python -m roommapper --lidar-yaw <度>` |
| 21 | `roommapper` 的雷达匹配和沿墙控制只在模拟房间验证过 | 人在旁边、低速跑一圈 | 调 `roommapper/slam.py`、`wall_follow.py` 顶部的常量 |
| 24 | `start_algo_task` 拒绝目标的条件（读 MiRoboticsLab/cyberdog_nav2 的 `algorithm_manager` 得出，未在真机验证）：任务管理器状态必须是 101 / 6 / 16，且状态机为 Active / Protected。`stop(ALL)` 只接受状态 1/21/7/17/8/18，其余（含建图、跟随、回充、空闲）都返回失败；定位成功后的状态 8 要用 `stop(ALL)` 清，`stop(LOCALIZATION)` 不起作用 | 读话题 `algo_task_status` 的 `task_status` | `nav.release()`、`close()`、`_cleanup` 都已按状态结束任务 |

### 12.2 已修复的问题

上一版文档列出的 20 个源码问题都已修复，回归测试在 `tests/test_fixes.py`（测试名前的注释标了对应编号）。

| # | 原问题 | 修复 |
|---|---|---|
| 1 | NaN / Inf 让代理崩溃或请求卡到超时 | `protocol.py` 用 `{"__f__": ...}` 编码非有限浮点，`PROTOCOL_VERSION` 升到 2（8.1 节）；`AgentServer._handle` 响应无法编码时立即回错误（8.2 节） |
| 2 | 断线后导航任务不会被取消 | `_cleanup` 在发过目标时 `stop_algo_task(ALL)` + 逐个 `cancel_goal`，每步独立尝试、最后抛第一个错误（7.6 节）；`dog.close()` 在 `nav.started` 时绕过 `_check()` 停止导航（4.1 节） |
| 3 | 未知步态字符串抛 `KeyError` | `set_velocity` 改抛 `ValueError`（4.2 节） |
| 4 | 回调崩溃后 `estop()` 发不出 motion_id 0 | `estop()` 绕过 `_check()`（`_send_result(check=False)`），只检查 `_ensure_open()`（4.2 节） |
| 5 | `NavTask.wait/cancel` 不 `_check()`；`follow_uwb` 不校验距离 | 两者都先 `_check()`；`follow_uwb` 的 `distance <= 0` → `ValueError`（4.14 节） |
| 6 | `cancel_face` 不检查响应；`enable()` 空请求 | `cancel_face` 结果非 0 → `CommandFailed`；`enable/disable` 不给算法 → `ValueError`（4.13 节） |
| 7 | LED 静默忽略 mini 灯颜色名 / `off` 时的 effect | mini + 颜色名、头尾灯 `color=None`、`off` + 非 `on` 灯效都 → `ValueError`（4.11 节） |
| 8 | `TopicCache.get()` 不判断新鲜度、首次订阅非线程安全 | 新增 `max_age` 参数与 `age` 属性；首次订阅加锁；`get()` 先 `_check()`（4.4 节） |
| 9 | 远程导航反馈在 `send_goal` 返回前到达会丢失、`_feedback` 不清理 | 提前到达的反馈先缓存再补发；`wait_goal` 返回后释放回调（1.6、8.4 节） |
| 10 | 狗端服务调用最长约 `2×timeout`，客户端先超时 | `RosBackend.call_service` / `send_goal` 用同一截止时间（`_left()`），总时长 ≤ `timeout`（7.6 节） |
| 11 | `RosBackend.latest_image` 注释与逻辑不符 | 注释已更正；图像订阅保留是有意的缓存（4.8 节） |
| 12 | `RemoteBackend.latest_image` 默认 `jpeg=True` | 改为 `jpeg=False`，与基类一致（4.16 节） |
| 13 | 客户端不检查协议版本 | `RemoteBackend.__init__` 不一致时抛 `CyberDogError`（7.1 节） |
| 14 | 部署目录不清理旧版本 | `deploy()` 删除其他哈希目录，失败 → `AgentStartError`；上传拆到 `_upload()`（7.2 节） |
| 15 | 继承环境变量的 `sed` 遇到单引号出错 | 改用 `deploy.env_inherit_loop()`（`while read -d ''` + `export "$KV"`）（7.3 节） |
| 16 | `connect()` 不能设置 SSH / 探测 / accept 超时 | 新增 `agent_options`：`ssh_timeout`、`discovery_timeout`、`accept_timeout`（7.1 节） |
| 17 | doctor 的 `--no-inherit-env` 没有配套 env | 新增 `--inherit-env-from NAME`、可重复的 `--env KEY=VALUE`（第 9 节） |
| 18 | local 模式卡死时伺服一直发当前速度 | `set_velocity(timeout=...)` 死人开关（`ServoLoop` 的 `expire_after` / `expired_count`），`walk()` 默认 `duration + 1`（4.2 节）。**需要调用方设置 `timeout` 才生效**，见 12.3 #1 |
| 19 | `protocol.py` 头注释漏了 `frame` 事件 | 已补上 |
| 20 | 已关闭的流 `read()` 抛 `SensorTimeout` | 改抛 `CyberDogError`（4.9 节） |

### 12.3 仍未解决的问题 / 限制

以下是阅读当前源码时确认仍然存在、或修复后新出现的问题，**均未修改代码**：

1. **死人开关默认关闭**：`set_velocity()` 默认 `timeout=None`，local 模式下程序卡死（进程没退出）时伺服仍一直下发当前速度；远程模式下如果只有主线程卡死、ping 线程还活着，代理看门狗也不会触发。只有 `walk()` / `turn()` 默认带死人开关。
2. **`connect_ssh` 在 `info` 失败时泄漏 SSH 会话**：`try` 只包住 `start_agent` / `open_tunnel`；`RemoteBackend(...)` 构造时 `info` 请求失败（`ServiceTimeout` / `ConnectionLost`）不会调用 `close()`，SSH 连接、代理通道和 reader / ping / callbacks 三个线程都留着（只有协议版本不一致这条路径会先 `close()`）。doctor 里直接构造 `RemoteBackend` 也一样（不过 doctor 外层 `finally` 会关 SSH）。
3. **代理端反馈转发没有捕获 `OSError`**：`_dispatch` 里 `send_goal` 的 `fb` 直接 `stream.send(...)`（订阅转发 `forward` 会捕获 `OSError`）。客户端断开时如果正好有反馈，`OSError` 在 ROS 回调里被 `RosBackend._guard` 记成后端致命错误，此后该后端的所有 ROS 回调都被忽略（`_cleanup` 的服务调用不受影响）。
4. **断线收尾只覆盖代理端 `send_goal` 成功返回的目标**：代理端 `send_goal` 超时（但目标其实已被接受）时不会进入 `self._goals`，断线时不会触发 `stop_algo_task(ALL)`。客户端正常 `close()` 有 `nav.started` 兜底，断线时没有。
5. **停止导航的范围偏大**：代理端 `_goals` 和客户端 `nav.started` 都不会因为任务正常结束而清除（`nav.started` 只在 `stop(ALL)` 成功时清除），所以只要本次会话发过目标，断线 / `close()` 时就会按当时的状态结束任务，可能把 App 或其他程序之后启动的任务也停掉。这是偏安全的取舍，但真机上需要确认 `ALL` 的语义（见 12.1 #17）。
6. **`close()` 可能变慢 / 抛错**：`nav.started` 为真时 `close()` 会调用 `release()`（读状态最多 5s，服务超时 30s）。local 模式下该服务不可用时 `close()` 要等最长 30 秒后抛 `ServiceUnavailable`；`with` 块正常结束时这个异常会抛给调用方。
7. **SSH 突然断开时 `_cleanup` 不一定执行**：代理在带 pty 的通道上运行，SSH 断开时收到 SIGHUP，而 Python 对 SIGHUP 是默认动作（直接终止进程），`serve_one` 的 `finally` 不会执行。伺服会因 motion_manager 200ms 收不到伺服帧而停下，但导航任务不会被停止。通常代理看门狗（1s）会先于 SSH 断线触发，因此多数情况下没问题，需要真机确认。
8. **`NavTask.wait(timeout=None)` 只在开始时 `_check()` 一次**：等待期间回调崩溃不会打断等待（远程连接断开会通过 `ConnectionLost` 唤醒；local 模式下 ROS 执行器崩溃也不会唤醒）。
9. **远程反馈回调的释放不完整**：只有 `wait_goal` 成功返回才释放 `_feedback` / `_early_feedback`；跟随类任务通常不 `wait`，或 `wait` 超时 / 抛错时，回调一直保留到 `close()`。客户端本地 `send_goal` 超时后才到达的反馈会永远留在 `_early_feedback` 里。
10. **`TopicCache` 的内部订阅永不取消**；`Battery`、`MotionState`、`ultrasonic_distance()`、`tof()` 都不带 `max_age`，话题停发后仍返回旧值。
11. **`estop()` 的错误可能被覆盖**：`servo_stop()` 抛错后，`finally` 里发送急停如果也抛错，`servo_stop()` 的原始错误会丢失（Python `finally` 中的异常覆盖原异常）。
12. **`ImageStream.read()` 只在进入时检查是否已关闭**：另一个线程在 `read()` 等待期间 `close()`，`read()` 会等到超时后抛 `SensorTimeout`。
13. **部署清理会删掉其他版本目录**：多台电脑用不同版本的代码连同一只狗时，会互相删除对方的部署目录（不影响已在运行的代理，下次连接时重新上传）。
14. `Backend.servo_set` 的 docstring（`backends/base.py`）没有提到可选的 `expire_after` 键。
15. doctor 即使给了 `--no-inherit-env` 也会执行"`<进程名>` 进程"这一步（只是展示，不影响结果）；doctor 不能设置 `ssh_timeout` / `discovery_timeout` / `accept_timeout` / `setup_scripts`。

---

## 13. 示例索引

所有示例都通过 `examples/_common.py` 的 `open_dog()` 打开狗（`08` 除外）：命令行**不带 IP 或带 `--mock`** 用模拟狗；带 IP 时连真机，密码从环境变量 `CYBERDOG_PASSWORD` 读取（未设置会退出）。

```bash
python examples/01_basic_motion.py --mock            # 模拟狗
set CYBERDOG_PASSWORD=123                            # Windows cmd（bash: export CYBERDOG_PASSWORD=123）
python examples/01_basic_motion.py 192.168.31.50     # 真机
```

| 文件 | 内容 | 用到的 API | 备注 |
|---|---|---|---|
| `01_basic_motion.py` | 打印电量，站立 → 坐下 → 握手 → 站立 → 趴下 | `battery.soc`、`stand/sit/shake_hand/get_down` | |
| `02_walk_square.py` | 直行 2 秒、左转 90 度，重复 4 次走正方形 | `walk`、`turn` | 需要空旷场地 |
| `03_touch_reaction.py` | 摸头互动：单击坐下、双击握手、长按退出 | `on_touch` + `queue.Queue` | 演示**回调只入队、主线程发指令**的正确写法；mock 下自动注入单击 / 双击 / 长按 |
| `04_led_and_voice.py` | 头灯蓝色呼吸、尾灯 RGB 跑马、mini 灯三圈，读音量，在线 TTS，最后释放三个灯 | `led.set/release`、`audio.volume/say` | 在线语音需联网并登录小米账号 |
| `05_follow_person.py` | 人体跟随，打印任务状态和反馈，20 秒后停止 | `nav.on_status`、`follow_person(feedback=...)`、`task.wait(timeout=30)`、`nav.stop` | `wait` 在真机上的行为【未核实】（见 12.1 #12） |
| `06_camera.py` | 开相机，取一帧保存为 `frame.jpg`，关相机 | `camera.start/get_frame/stop`、`Frame.save` | mock 的 raw 帧保存需要 Pillow + numpy |
| `07_video_stream.py` | 视频流 + OpenCV Haar 人脸检测，窗口显示 fps 和丢帧数，Esc 退出；mock 下处理 30 帧自动退出 | `camera.stream`、`Frame.to_bgr`、`stream.dropped` | 需要 `opencv-python numpy`（顶层就 import cv2，mock 也需要） |
| `roommapper/`（应用，不在 examples 里） | 在仓库根目录运行 `python -m roommapper --mock`（模拟房间）或 `python -m roommapper`（真机，在终端里输入 IP、账号和密码）。WebUI 默认 `http://127.0.0.1:8090/`，沿墙建图、数据存 `room_maps.db`、点途经点规划并执行路径。只用雷达，不依赖狗的导航服务 | `sensors.scan`、`set_velocity`、`estop` | 建图前要关门；待真机核实项见 12.1 #20、#21 |
| `08_http_live.py` | 把狗的视角转成 HTTP MJPEG 直播：`http://127.0.0.1:8080/stream.mjpg`（VLC）或 `http://127.0.0.1:8080/`（浏览器） | `camera.stream`、`Frame.to_numpy` | **不使用 `_common`**：IP / 用户 / 密码写死在文件顶部 `DOG_IP = "192.168.1.55"`、`DOG_PASSWORD = "123"`，用前要改。参数 `--mock`、`--bind`（局域网共享用 `0.0.0.0`）、`--port 8080`、`--fps 15`、`--quality 70`、`--width 640`、`--height 480`；mock 下需要 Pillow |

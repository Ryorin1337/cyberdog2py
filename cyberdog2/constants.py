"""CyberDog 2 接口常量。

来源：MiRoboticsLab/bridges（rolling 分支）protocol/ros/ 下的 msg/srv/action 定义，
以及官方开发者手册 https://miroboticslab.github.io/blogs/#/cn/developer_guide
"""

from enum import IntEnum


# ---------------------------------------------------------------- 接口名与类型
class Iface:
    """ROS 接口名（不带命名空间）和类型。"""

    MOTION_RESULT = ("motion_result_cmd", "protocol/srv/MotionResultCmd")
    MOTION_SERVO = ("motion_servo_cmd", "protocol/msg/MotionServoCmd")
    MOTION_SERVO_RESPONSE = ("motion_servo_response", "protocol/msg/MotionServoResponse")
    MOTION_STATUS = ("motion_status", "protocol/msg/MotionStatus")  # 话题名手册里没写，需要真机核实
    LED = ("led_execute", "protocol/srv/LedExecute")
    SPEECH = ("speech_text_play", "protocol/srv/AudioTextPlay")
    VOLUME_GET = ("audio_volume_get", "protocol/srv/AudioVolumeGet")
    VOLUME_SET = ("audio_volume_set", "protocol/srv/AudioVolumeSet")
    BMS = ("bms_status", "protocol/msg/BmsStatus")
    TOUCH = ("touch_status", "protocol/msg/TouchStatus")
    HEAD_TOF = ("head_tof_payload", "protocol/msg/HeadTofPayload")
    REAR_TOF = ("rear_tof_payload", "protocol/msg/RearTofPayload")
    ULTRASONIC = ("ultrasonic_payload", "sensor_msgs/msg/Range")
    GPS = ("gps_payload", "protocol/msg/GpsPayload")
    UWB = ("uwb_raw", "protocol/msg/UwbRaw")
    CAMERA = ("camera_service", "protocol/srv/CameraService")
    IMAGE = ("image", "sensor_msgs/msg/Image")
    ALGO_MANAGER = ("algo_manager", "protocol/srv/AlgoManager")
    PERSON = ("person", "protocol/msg/Person")
    FACE_REC = ("cyberdog_face_recognition_srv", "protocol/srv/FaceRec")
    FACE_REC_MSG = ("face_rec_msg", "protocol/msg/FaceRecognitionResult")
    GESTURE = ("gesture_action_control", "protocol/srv/GestureActionControl")
    GESTURE_MSG = ("gesture_action_msg", "protocol/msg/GestureActionResult")
    NAV_START = ("start_algo_task", "protocol/action/Navigation")
    NAV_STOP = ("stop_algo_task", "protocol/srv/StopAlgoTask")
    NAV_STATUS = ("algo_task_status", "protocol/msg/AlgoTaskStatus")
    SENSOR_OPERATION = ("sensor_operation", "protocol/srv/SensorOperation")  # sensor_manager 的传感器开关
    SCAN = ("scan", "sensor_msgs/msg/LaserScan")  # 已真机确认：前方 180°，495 点，0.0 表示无回波
    ODOM = ("odom_out", "nav_msgs/msg/Odometry")  # 真机实测约 48Hz。真机默认没有 imu 话题，sensors.imu 由它计算


# 自检（doctor）时必须存在的接口
REQUIRED_SERVICES = ["motion_result_cmd", "led_execute", "speech_text_play"]
REQUIRED_TOPICS = ["motion_servo_cmd", "bms_status", "touch_status"]


# ---------------------------------------------------------------- 运动
class CmdSource(IntEnum):
    APP = 0      # 优先级最高
    AUDIO = 1
    VIS = 2
    BLUTELE = 3
    ALGO = 4


class SensorId(IntEnum):
    """sensor_operation 服务的 sensor_id。"""

    LIDAR = 1
    ULTRASONIC = 2
    TOF = 3
    GPS = 4


class SensorOperation(IntEnum):
    """sensor_operation 服务的 operation。"""

    START = 2
    STOP = 3


class ServoCmdType(IntEnum):
    START = 0
    DATA = 1
    END = 2


class MotionId(IntEnum):
    """结果指令（motion_result_cmd）和伺服指令（motion_servo_cmd）可用的 motion_id。"""

    ESTOP = 0                 # 急停
    GET_DOWN = 101            # 高阻尼趴下
    STAND = 111               # 恢复站立
    STAND_UP_STRONG = 118     # 强化起身
    BACKFLIP = 121            # 后空翻
    BOW = 123                 # 作揖
    WALK_THE_DOG = 125        # 遛狗
    JUMP_TURN_LEFT_90 = 130   # 3D 跳跃左转 90 度
    JUMP_TURN_RIGHT_90 = 131  # 3D 跳跃右转 90 度
    JUMP_FORWARD_60CM = 132   # 3D 跳跃前跳 60cm
    JUMP_LEFT_20CM = 134      # 3D 跳跃左跳 20cm
    JUMP_RIGHT_20CM = 135     # 3D 跳跃右跳 20cm
    DANCE = 140               # 舞蹈集合
    SHAKE_LEFT_HAND = 141     # 握左手
    SHAKE_RIGHT_HAND = 142    # 握右手
    SIT = 143                 # 坐下
    BUTT_CIRCLE = 144         # 屁股画圆
    HEAD_CIRCLE = 145         # 头画圆
    STRETCH = 146             # 伸展身体
    BALLET = 151              # 芭蕾舞
    MOONWALK = 152            # 太空步
    PUSH_UP = 174             # 俯卧撑
    BOW_HEART = 175           # 作揖比心
    # 以下为伺服（持续控制）步态
    JUMP_FRONT_BACK = 301     # 前后足交替离地
    HOP = 302                 # 四足同时离地小跳
    WALK_SLOW = 303           # 慢速行走
    WALK_FAST = 305           # 快速行走
    WALK_MEDIUM = 308         # 中速行走


SERVO_MOTIONS = {301, 302, 303, 305, 308}

# 步态名 -> motion_id
GAITS = {
    "slow": MotionId.WALK_SLOW,
    "medium": MotionId.WALK_MEDIUM,
    "fast": MotionId.WALK_FAST,
    "jump": MotionId.JUMP_FRONT_BACK,
    "hop": MotionId.HOP,
}

# motion_id -> (vx 上限, vy 上限, wz 上限)，单位 m/s 和 rad/s，来源为开发者手册
GAIT_LIMITS = {
    303: (0.65, 0.3, 1.25),
    308: (1.0, 0.3, 1.25),
    305: (1.6, 0.55, 2.5),
    302: (0.25, 0.1, 0.5),
    301: (0.4, 0.3, 0.75),
}

# 运控 LCM 命令里各数组的固定长度，下发时按这个长度补零
MOTION_ARRAY_SIZES = {
    "vel_des": 3,
    "rpy_des": 3,
    "pos_des": 3,
    "acc_des": 6,
    "ctrl_point": 3,
    "foot_pose": 6,
    "step_height": 2,
}

RPY_LIMIT = 0.4                 # rad
POS_LIMIT = (0.2, 0.2, 0.3)     # m


class SwitchStatus(IntEnum):
    NORMAL = 0
    TRANSITIONING = 1
    ESTOP = 2
    EDAMP = 3
    LIFTED = 4
    BAN_TRANS = 5
    OVER_HEAT = 6
    LOW_BAT = 7
    ORI_ERR = 8
    FOOTPOS_ERR = 9
    STAND_STUCK = 10
    MOTOR_OVER_HEAT = 11
    MOTOR_OVER_CURR = 12
    MOTOR_ERR = 13
    CHARGING = 14


# ---------------------------------------------------------------- LED
LED_TARGETS = {"head": 1, "tail": 2, "mini": 3}
LED_MODE_PREDEFINED = 0x01
LED_MODE_USER = 0x02

# 自定义 RGB 模式下，头灯和尾灯可用的灯效
LED_USER_EFFECTS = {
    "on": 0x01,
    "blink": 0x02,
    "blink_fast": 0x03,
    "breath": 0x04,
    "breath_fast": 0x05,
    "one_by_one": 0x06,
    "one_by_one_fast": 0x07,
    "back_and_forth": 0x08,
    "trailing_race": 0x09,
}

# 系统预定义模式下，头灯和尾灯的 (颜色, 灯效) -> effect 值
_PRE_BASE = {"red": 0xA1, "blue": 0xA8, "yellow": 0xAF}
_PRE_ORDER = ["on", "blink", "blink_fast", "breath", "breath_fast", "one_by_one", "one_by_one_fast"]
LED_PREDEFINED = {}
for _color, _base in _PRE_BASE.items():
    for _i, _eff in enumerate(_PRE_ORDER):
        LED_PREDEFINED[(_color, _eff)] = _base + _i
LED_OFF = 0xA0

# 头部 mini LED 的灯效
LED_MINI_USER_EFFECTS = {"circular_breath": 0x30, "circular_ring": 0x31}
LED_MINI_PREDEFINED = {
    "off": 0x32,
    "rectangle_color": 0x33,
    "centre_color": 0x34,
    "three_circular": 0x35,
    "color_one_by_one": 0x36,
}

LED_CODES = {
    0: "SUCCEED",
    1107: "TIMEOUT",
    1121: "TARGET_ERROR",
    1122: "PRIORITY_ERROR",
    1123: "MODE_ERROR",
    1124: "EFFECT_ERROR",
    1125: "LOW_PRIORITY",
}


# ---------------------------------------------------------------- 触摸 / 相机 / 视觉
TOUCH_KINDS = {0x01: "single", 0x03: "double", 0x07: "long"}


class CameraCmd(IntEnum):
    SET_PARAMETERS = 0
    TAKE_PICTURE = 1
    START_RECORDING = 2
    STOP_RECORDING = 3
    GET_STATE = 4
    DELETE_FILE = 5
    GET_ALL_FILES = 6
    START_LIVE_STREAM = 7
    STOP_LIVE_STREAM = 8
    START_IMAGE_PUBLISH = 9
    STOP_IMAGE_PUBLISH = 10


ALGOS = {"face": 0, "body": 1, "gesture": 2, "keypoints": 3, "reid": 4, "focus": 5, "face_manager": 6}

GESTURES = {
    0: "none",
    1: "pull_in",
    2: "push_away",
    3: "slide_up",
    4: "slide_down",
    5: "slide_left",
    6: "slide_right",
    7: "stop",
    8: "thumb_up",
    9: "zoom_in",
    10: "zoom_out",
    11: "thumb_down",
}


# ---------------------------------------------------------------- 导航
class NavType(IntEnum):
    START_AB = 1
    STOP_AB = 2
    START_FOLLOW = 3
    STOP_FOLLOW = 4
    START_MAPPING = 5
    STOP_MAPPING = 6
    START_LOCALIZATION = 7
    STOP_LOCALIZATION = 8
    START_AUTO_DOCKING = 9
    STOP_AUTO_DOCKING = 10
    START_UWB_TRACKING = 11
    STOP_UWB_TRACKING = 12
    START_HUMAN_TRACKING = 13
    STOP_HUMAN_TRACKING = 14


class StopTask(IntEnum):
    ALL = 0
    AB = 1
    FOLLOW = 3
    MAPPING = 5
    LOCALIZATION = 7
    AUTO_DOCKING = 9
    UWB_TRACKING = 11
    HUMAN_TRACKING = 13


FOLLOW_POSITIONS = {"auto": 200, "behind": 201, "left": 202, "right": 203}

# 狗上任务管理器的状态（话题 algo_task_status 的 task_status），来源：cyberdog_nav2/algorithm_manager
TASK_STATUS_NAMES = {100: "未初始化", 101: "空闲", 102: "正在启动依赖节点", 103: "正在停止任务",
                     1: "激光 AB 导航中", 21: "视觉 AB 导航中", 3: "跟随中", 5: "激光建图中", 15: "视觉建图中",
                     6: "激光定位失败", 16: "视觉定位失败", 7: "激光定位启动中", 17: "视觉定位启动中",
                     8: "激光定位中", 18: "视觉定位中", 9: "自动回充中", 11: "UWB 跟随中", 13: "人体跟随中"}
TASK_ACCEPTING = {101, 6, 16}   # 只有这些状态下，狗才接受建图、定位等新任务
TASK_TRANSITIONAL = {102, 103}  # 正在切换，要等它结束
# 状态 -> 结束它要用的 task_id。StopTask.ALL 只接受 1/21/7/17/8/18，其他状态用它会返回失败
TASK_STOP_BY_STATUS = {
    5: StopTask.MAPPING, 15: StopTask.MAPPING,
    7: StopTask.LOCALIZATION, 17: StopTask.LOCALIZATION,
    1: StopTask.ALL, 21: StopTask.ALL, 8: StopTask.ALL, 18: StopTask.ALL,
    9: StopTask.AUTO_DOCKING, 11: StopTask.UWB_TRACKING,
    13: StopTask.HUMAN_TRACKING, 3: StopTask.HUMAN_TRACKING,
}

NAV_RESULTS = {0: "SUCCESS", 1: "ACCEPT", 2: "UNAVAILABLE", 3: "FAILED", 4: "REJECT", 5: "CANCEL"}
NAV_OK_RESULTS = {0, 1}

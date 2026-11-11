"""cyberdog2 的异常类型。

设计原则是 let it crash：出了问题直接抛异常，不静默兜底、不自动重试、不截断参数。
"""


class CyberDogError(Exception):
    """库内所有异常的基类。"""


class ConnectionLost(CyberDogError):
    """与狗端代理的连接断开。"""


class AgentStartError(CyberDogError):
    """狗端代理启动失败。异常信息里附带代理的输出。"""


class NamespaceNotFound(CyberDogError):
    """在 ROS 图里找不到 /mi_xxx 命名空间，或者找到了多个、无法确定用哪个。"""


class ServiceUnavailable(CyberDogError):
    """在超时时间内没有发现目标 ROS 服务或动作服务器。"""


class ServiceTimeout(CyberDogError):
    """服务已发现，但调用在超时时间内没有返回。"""


class SensorTimeout(CyberDogError):
    """在超时时间内没有收到传感器数据。"""


class CommandFailed(CyberDogError):
    """服务返回了表示失败的结果码。"""

    def __init__(self, message: str, code: int = 0, response: dict = None):
        super().__init__(message)
        self.code = code
        self.response = response or {}


class MotionRejected(CommandFailed):
    """运动指令被 motion_manager 拒绝（result 为 false）。"""


class GoalRejected(CommandFailed):
    """动作目标被拒绝，或任务执行失败。"""


class CallbackError(CyberDogError):
    """用户回调抛出了异常。原始异常保存在 __cause__ 中。"""


class RemoteError(CyberDogError):
    """代理端发生了本库没有定义的异常类型。"""

    def __init__(self, type_name: str, message: str, remote_traceback: str = ""):
        super().__init__("%s: %s\n--- 狗端 traceback ---\n%s" % (type_name, message, remote_traceback))
        self.type_name = type_name
        self.remote_message = message
        self.remote_traceback = remote_traceback


def rebuild(type_name: str, message: str, remote_traceback: str = "", extra: dict = None) -> Exception:
    """把代理传回来的错误还原成本地的异常对象。"""
    extra = extra or {}
    cls = globals().get(type_name)
    if isinstance(cls, type) and issubclass(cls, CommandFailed):
        return cls(message, extra.get("code", 0), extra.get("response"))
    if isinstance(cls, type) and issubclass(cls, CyberDogError) and cls is not RemoteError:
        return cls(message)
    if type_name in ("ValueError", "TypeError", "KeyError"):
        return {"ValueError": ValueError, "TypeError": TypeError, "KeyError": KeyError}[type_name](message)
    return RemoteError(type_name, message, remote_traceback)

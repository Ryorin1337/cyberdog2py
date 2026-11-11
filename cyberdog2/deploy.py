"""SSH 部署与启动狗端代理（只在电脑端使用，依赖 paramiko）。"""

import collections
import hashlib
import os
import posixpath
import re
import shlex
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

from .errors import AgentStartError
from .protocol import ERROR_MARKER, READY_MARKER

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))

# 狗上可能存在的 ROS 环境脚本，存在哪个就依次 source 哪个
DEFAULT_SETUP_SCRIPTS = (
    "/opt/ros2/galactic/setup.bash",
    "/opt/ros/galactic/setup.bash",
    "/opt/ros2/cyberdog/setup.bash",
)

# 从狗的 ROS 进程继承的环境变量，保证代理和狗的 ROS 在同一个 DDS 域里
INHERITED_ENV = ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI",
                 "FASTRTPS_DEFAULT_PROFILES_FILE")


def package_files() -> List[Tuple[str, bytes]]:
    """收集需要上传到狗上的 .py 文件，返回 (相对路径, 内容) 列表。"""
    out = []
    for root, dirs, files in os.walk(PACKAGE_DIR):
        dirs[:] = sorted(d for d in dirs if d != "__pycache__")
        for f in sorted(files):
            if f.endswith(".py"):
                full = os.path.join(root, f)
                rel = os.path.relpath(full, os.path.dirname(PACKAGE_DIR)).replace(os.sep, "/")
                with open(full, "rb") as fh:
                    out.append((rel, fh.read()))
    return out


def package_hash(files: Sequence[Tuple[str, bytes]]) -> str:
    h = hashlib.sha1()
    for rel, data in files:
        h.update(rel.encode())
        h.update(b"\0")
        h.update(data)
    return h.hexdigest()[:12]


def env_inherit_loop(environ_path: str) -> str:
    """生成一行 bash：从 environ 文件里导出 INHERITED_ENV 中列出的变量。

    逐条读取以 \\0 分隔的 "名=值"，直接 export "$KV"，所以值里有引号、空格、换行都不会出错。
    """
    patterns = "|".join("%s=*" % n for n in INHERITED_ENV)
    return ("while IFS= read -r -d '' KV; do case \"$KV\" in %s) export \"$KV\";; esac; done < %s"
            % (patterns, environ_path))


def source_setup_lines(setup_scripts: Sequence[str]) -> List[str]:
    """依次 source 存在的 ROS 环境脚本（不存在的跳过）。"""
    lines = []
    for script in setup_scripts:
        q = shlex.quote(script)
        # setup.bash 内部会引用未定义的变量，source 时临时关掉 -e
        lines.append("if [ -f %s ]; then set +e; . %s; set -e; fi" % (q, q))
    return lines


def build_agent_command(base_dir: str, namespace: Optional[str], setup_scripts: Sequence[str],
                        inherit_env_from: Optional[str], env: Dict[str, str], watchdog: float,
                        discovery_timeout: float = 8.0, accept_timeout: float = 30.0,
                        python: str = "python3") -> str:
    """生成在狗上启动代理的 bash 脚本。任何一步失败都打印 ERROR_MARKER 并以非零码退出。

    python 是狗上运行代理的解释器，必须是编译 rclpy 时用的那个（狗上 rclpy 所在的
    site-packages 目录名里的 pythonX.Y 即为版本）。
    """
    lines = ["set -e"] + source_setup_lines(setup_scripts)
    if inherit_env_from:
        # 用 [m]otion_manager 这种写法，避免 pgrep 匹配到当前这条命令本身
        pattern = "[%s]%s" % (inherit_env_from[0], inherit_env_from[1:])
        lines += [
            "PID=$(pgrep -o -f %s || true)" % shlex.quote(pattern),
            'if [ -z "$PID" ]; then echo "%s 找不到进程 %s，无法继承 ROS 环境；'
            '可以传 inherit_env_from=None 并用 env= 手动指定"; exit 3; fi' % (ERROR_MARKER, inherit_env_from),
            'if [ ! -r /proc/$PID/environ ]; then echo "%s 没有权限读取 /proc/$PID/environ（进程 %s）；'
            '可以传 inherit_env_from=None 并用 env= 手动指定"; exit 3; fi' % (ERROR_MARKER, inherit_env_from),
            env_inherit_loop("/proc/$PID/environ"),
        ]
    for k, v in sorted(env.items()):
        if not re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", k):
            raise ValueError("非法的环境变量名: %r" % k)
        lines.append("export %s=%s" % (k, shlex.quote(str(v))))
    lines.append('echo "CYBERDOG2_ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-} '
                 'RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"')
    lines.append("cd %s" % shlex.quote(base_dir))
    cmd = ("exec %s -u -m cyberdog2.agent --watchdog %s --discovery-timeout %s --accept-timeout %s"
           % (shlex.quote(python), float(watchdog), float(discovery_timeout), float(accept_timeout)))
    if namespace is not None:
        cmd += " --namespace %s" % shlex.quote(namespace)
    lines.append(cmd)
    return "bash -c %s" % shlex.quote("\n".join(lines))


class AgentProcess(object):
    """狗上正在运行的代理进程（对应一个 SSH exec 通道）。"""

    def __init__(self, channel, port: int, namespace: str, log: collections.deque):
        self.channel = channel
        self.port = port
        self.namespace = namespace
        self._log = log

    def log_tail(self) -> str:
        return "".join(self._log)

    def close(self) -> None:
        # 通道使用了 pty，关闭通道时代理会收到 SIGHUP 并退出
        self.channel.close()


class SshSession(object):
    def __init__(self, host: str, user: str = "mi", password: Optional[str] = None, key_file: Optional[str] = None,
                 port: int = 22, timeout: float = 10.0):
        import paramiko

        self.host = host
        self.client = paramiko.SSHClient()
        self.client.load_system_host_keys()
        # 机器狗属于局域网设备，首次连接时自动信任它的主机密钥
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect(host, port=port, username=user, password=password, key_filename=key_file,
                            timeout=timeout, allow_agent=key_file is None and password is None,
                            look_for_keys=key_file is None and password is None)
        self.transport = self.client.get_transport()
        self.transport.set_keepalive(5)

    # ---------------------------------------------------------- 工具
    def run(self, command: str, timeout: float = 30.0) -> Tuple[int, str]:
        """执行命令，返回 (退出码, stdout+stderr)。"""
        chan = self.transport.open_session()
        chan.settimeout(timeout)
        chan.set_combine_stderr(True)
        chan.exec_command(command)
        out = b""
        while True:
            chunk = chan.recv(65536)
            if not chunk:
                break
            out += chunk
        code = chan.recv_exit_status()
        chan.close()
        return code, out.decode("utf-8", "replace")

    def home(self) -> str:
        code, out = self.run("echo $HOME")
        if code != 0 or not out.strip().startswith("/"):
            raise AgentStartError("无法获取狗上的 HOME 目录: %r" % out)
        return out.strip()

    # ---------------------------------------------------------- 部署
    def deploy(self) -> str:
        """把 cyberdog2 包上传到 ~/.cyberdog2/<哈希>/。目录已存在且完整时跳过。返回该目录路径。"""
        files = package_files()
        digest = package_hash(files)
        root = posixpath.join(self.home(), ".cyberdog2")
        base = posixpath.join(root, digest)
        marker = posixpath.join(base, ".complete")
        code, _ = self.run("test -f %s" % shlex.quote(marker))
        if code != 0:
            self._upload(files, base, marker)
        # 删除旧版本目录，只保留当前版本，避免 ~/.cyberdog2 越积越大
        code, out = self.run("find %s -mindepth 1 -maxdepth 1 -type d ! -name %s -exec rm -rf {} +"
                             % (shlex.quote(root), shlex.quote(digest)))
        if code != 0:
            raise AgentStartError("清理狗上的旧版本目录失败: %s" % out)
        return base

    def _upload(self, files, base: str, marker: str) -> None:
        sftp = self.client.open_sftp()
        try:
            dirs = sorted({posixpath.dirname(posixpath.join(base, rel)) for rel, _ in files})
            code, out = self.run("mkdir -p " + " ".join(shlex.quote(d) for d in dirs))
            if code != 0:
                raise AgentStartError("在狗上创建目录失败: %s" % out)
            for rel, data in files:
                with sftp.open(posixpath.join(base, rel), "wb") as fh:
                    fh.write(data)
            with sftp.open(marker, "wb") as fh:
                fh.write(b"ok\n")
        finally:
            sftp.close()

    def start_agent(self, namespace: Optional[str] = None, setup_scripts: Sequence[str] = DEFAULT_SETUP_SCRIPTS,
                    inherit_env_from: Optional[str] = "motion_manager", env: Optional[Dict[str, str]] = None,
                    start_timeout: float = 30.0, watchdog: float = 1.0, discovery_timeout: float = 8.0,
                    accept_timeout: float = 30.0, python: str = "python3") -> AgentProcess:
        base = self.deploy()
        command = build_agent_command(base, namespace, setup_scripts, inherit_env_from, env or {}, watchdog,
                                      discovery_timeout, accept_timeout, python)
        chan = self.transport.open_session()
        chan.get_pty()  # 有了 pty，SSH 断开时代理会收到 SIGHUP 并退出，不会残留进程
        chan.exec_command(command)

        log = collections.deque(maxlen=200)
        ready = {}
        got = threading.Event()

        def pump():
            buf = b""
            while True:
                chunk = chan.recv(4096)
                if not chunk:
                    got.set()
                    return
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("utf-8", "replace").rstrip("\r")
                    log.append(line + "\n")
                    if line.startswith(READY_MARKER) and not ready:
                        ready.update(dict(kv.split("=", 1) for kv in line.split()[1:]))
                        got.set()

        threading.Thread(target=pump, name="cyberdog2-agent-log", daemon=True).start()
        deadline = time.monotonic() + start_timeout
        while not got.is_set():
            if time.monotonic() > deadline:
                chan.close()
                raise AgentStartError("代理在 %.0fs 内没有启动完成。输出:\n%s" % (start_timeout, "".join(log)))
            got.wait(0.1)
        if not ready:
            time.sleep(0.2)
            chan.close()
            raise AgentStartError("代理启动失败。输出:\n%s" % "".join(log))
        return AgentProcess(chan, int(ready["port"]), ready["namespace"], log)

    def open_tunnel(self, port: int):
        """打开到狗上 127.0.0.1:port 的 SSH 隧道，返回一个类似 socket 的对象。"""
        return self.transport.open_channel("direct-tcpip", ("127.0.0.1", port), ("127.0.0.1", 0))

    def close(self) -> None:
        self.client.close()

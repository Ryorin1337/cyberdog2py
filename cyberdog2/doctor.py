"""连通性自检：python -m cyberdog2.doctor <狗的IP> [-u mi] [--key 私钥路径]

密码从环境变量 CYBERDOG_PASSWORD 读取；没有设置时会提示输入。
每一步出错都会直接抛出完整的异常（let it crash）。最后一步会列出缺失的接口，有缺失时以退出码 1 结束。
"""

import argparse
import getpass
import os
import shlex
import sys
import time

from .constants import REQUIRED_SERVICES, REQUIRED_TOPICS
from .deploy import DEFAULT_SETUP_SCRIPTS, SshSession, source_setup_lines


def step(msg: str) -> None:
    print("==> " + msg, flush=True)


def wait_for_interfaces(list_names, ns: str, timeout: float, poll: float = 0.5):
    """ROS 的发现是异步的，代理刚启动时列表可能还不全，所以轮询到关键接口到齐或超时。

    返回 (最后一次的名字列表, 仍然缺失的完整接口名)。
    """
    required = [("services", n) for n in REQUIRED_SERVICES] + [("topics", n) for n in REQUIRED_TOPICS]
    deadline = time.monotonic() + timeout
    while True:
        names = list_names()
        missing = [ns + "/" + n for kind, n in required if ns + "/" + n not in names[kind]]
        if not missing or time.monotonic() >= deadline:
            return names, missing
        time.sleep(poll)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="cyberdog2.doctor", description="检查电脑与 CyberDog 2 的连接")
    p.add_argument("host")
    p.add_argument("-u", "--user", default="mi")
    p.add_argument("--key", default=None, help="SSH 私钥路径；不填则使用密码登录")
    p.add_argument("--port", type=int, default=22)
    p.add_argument("--namespace", default=None)
    p.add_argument("--no-inherit-env", action="store_true", help="不从狗的 ROS 进程继承 ROS 环境变量")
    p.add_argument("--inherit-env-from", default="motion_manager", help="从哪个进程继承 ROS 环境变量")
    p.add_argument("--python", default="python3", help="狗上运行代理的解释器，须与 rclpy 的编译版本一致")
    p.add_argument("--env", action="append", default=[], metavar="KEY=VALUE",
                   help="额外导出给代理的环境变量，可重复，例如 --env ROS_DOMAIN_ID=42")
    p.add_argument("--wait", type=float, default=10.0, help="等待关键接口被发现的最长秒数")
    args = p.parse_args(argv)
    env = {}
    for item in args.env:
        if "=" not in item:
            p.error("--env 的格式应为 KEY=VALUE，收到 %r" % item)
        k, v = item.split("=", 1)
        env[k] = v

    password = None
    if args.key is None:
        password = os.environ.get("CYBERDOG_PASSWORD") or getpass.getpass("%s@%s 的密码: " % (args.user, args.host))

    step("SSH 登录 %s@%s:%d" % (args.user, args.host, args.port))
    ssh = SshSession(args.host, user=args.user, password=password, key_file=args.key, port=args.port)
    try:
        step("狗端系统信息")
        for cmd in ("uname -a", "python3 --version", "cat /etc/os-release | head -n 2"):
            code, out = ssh.run(cmd)
            print("    $ %s\n    %s" % (cmd, out.strip().replace("\n", "\n    ")))
        step("ROS 环境脚本")
        for s in DEFAULT_SETUP_SCRIPTS:
            code, _ = ssh.run("test -f %s" % s)
            print("    %s %s" % ("存在  " if code == 0 else "不存在", s))
        step("用 %s 导入 rclpy 和 protocol（source 上述环境脚本后）" % args.python)
        probe = ("import sys, rclpy, protocol.msg; print(sys.version.split()[0]); print(rclpy.__file__); "
                 "print(protocol.msg.__file__)")

        def run_probe(source):
            script = "\n".join(source_setup_lines(DEFAULT_SETUP_SCRIPTS)
                               + ["%s -c %s" % (shlex.quote(args.python), shlex.quote(source))])
            result, output = ssh.run("bash -c %s" % shlex.quote(script))
            print("    " + output.strip().replace("\n", "\n    "))
            return result

        code = run_probe(probe)
        if code != 0:
            raise RuntimeError("%s 无法导入 rclpy/protocol（退出码 %d）。rclpy 所在目录名里的 pythonX.Y 就是它需要的"
                               "解释器版本，用 --python python3.X 指定" % (args.python, code))

        step("用 %s 试压缩一张 JPEG（相机取帧和视频流需要 Pillow）" % args.python)
        code = run_probe("import io, PIL; from PIL import Image; b = io.BytesIO(); "
                         "Image.new('RGB', (8, 8)).save(b, 'JPEG', quality=70); "
                         "print('Pillow', PIL.__version__, len(b.getvalue()), 'bytes')")
        if code != 0:
            raise RuntimeError("%s 无法用 Pillow 压缩 JPEG（退出码 %d），相机取帧和视频流用不了" % (args.python, code))

        proc = args.inherit_env_from
        step("%s 进程" % proc)
        code, out = ssh.run("pgrep -a -f %s | head -n 3" % shlex.quote("[%s]%s" % (proc[0], proc[1:])))
        print("    " + (out.strip() or "（没有找到 %s 进程）" % proc).replace("\n", "\n    "))

        step("部署并启动狗端代理")
        from .backends.remote import RemoteBackend

        agent = ssh.start_agent(namespace=args.namespace,
                                inherit_env_from=None if args.no_inherit_env else proc, env=env,
                                python=args.python)
        print("    " + agent.log_tail().strip().replace("\n", "\n    "))
        backend = RemoteBackend(ssh.open_tunnel(agent.port), on_close=agent.close, log_tail=agent.log_tail)
        from .dog import CyberDog

        dog = CyberDog(backend, remote=True)
        try:
            step("命名空间: %s" % dog.namespace)
            ns = dog.namespace.rstrip("/")
            names, missing = wait_for_interfaces(dog.list_names, ns, args.wait)
            print("    话题 %d 个，服务 %d 个，动作 %d 个" % (len(names["topics"]), len(names["services"]),
                                                       len(names["actions"])))
            for n in REQUIRED_SERVICES + REQUIRED_TOPICS:
                full = ns + "/" + n
                print("    %s %s" % ("缺失" if full in missing else "OK  ", full))
            step("读取电池")
            print("    电量 %d%%，电压 %.2fV" % (dog.sensors.battery.soc, dog.sensors.battery.voltage))
        finally:
            dog.close()
    finally:
        ssh.close()
    if missing:
        print("\n自检未通过，缺失的接口: %s" % missing)
        return 1
    print("\n自检通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""示例公用：命令行里传了 IP 就连真机，不传或带 --mock 就用模拟狗。

    python examples/01_basic_motion.py --mock
    set CYBERDOG_PASSWORD=123 && python examples/01_basic_motion.py 192.168.31.50
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from cyberdog2 import CyberDog  # noqa: E402


def open_dog() -> CyberDog:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if "--mock" in sys.argv or not args:
        print("[示例] 使用模拟狗（mock）")
        return CyberDog.mock()
    host = args[0]
    password = os.environ.get("CYBERDOG_PASSWORD")
    if password is None:
        raise SystemExit("请先设置环境变量 CYBERDOG_PASSWORD（狗的 SSH 密码）")
    print("[示例] 连接 %s ..." % host)
    return CyberDog.connect(host, password=password)

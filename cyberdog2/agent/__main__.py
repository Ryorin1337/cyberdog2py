"""狗端代理入口：python3 -m cyberdog2.agent

启动成功后打印一行 `CYBERDOG2_AGENT_READY port=<端口> namespace=<命名空间>`，然后等待一个客户端连接。
任何启动错误都直接以 traceback 退出（let it crash），由电脑端把输出原样展示给用户。
"""

import argparse
import sys

from ..protocol import READY_MARKER
from .server import AgentServer


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="cyberdog2.agent")
    p.add_argument("--port", type=int, default=0)
    p.add_argument("--namespace", default=None, help="手动指定命名空间，例如 /mi_desktop_xx；不填则自动探测")
    p.add_argument("--discovery-timeout", type=float, default=8.0)
    p.add_argument("--watchdog", type=float, default=1.0)
    p.add_argument("--accept-timeout", type=float, default=30.0)
    p.add_argument("--mock", action="store_true", help="使用模拟后端（调试代理本身时使用）")
    args = p.parse_args(argv)

    if args.mock:
        from ..backends.mock import MockBackend

        backend = MockBackend()
    else:
        from ..backends.ros import RosBackend

        backend = RosBackend(namespace=args.namespace, discovery_timeout=args.discovery_timeout)
    try:
        server = AgentServer(backend, port=args.port, watchdog=args.watchdog)
        print("%s port=%d namespace=%s" % (READY_MARKER, server.port, backend.namespace), flush=True)
        server.serve_one(accept_timeout=args.accept_timeout)
    finally:
        backend.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

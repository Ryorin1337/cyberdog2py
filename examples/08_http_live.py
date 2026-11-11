"""把狗的视角转成 HTTP 直播流（MJPEG），可以用 VLC 或浏览器观看。

运行：
    python examples/08_http_live.py              # 连接 192.168.1.55
    python examples/08_http_live.py --mock       # 没有真机时用模拟狗测试

观看：
    VLC：媒体 → 打开网络串流 → 输入 http://127.0.0.1:8080/stream.mjpg
    浏览器：打开 http://127.0.0.1:8080/
    想让同一局域网里的其他设备也能看，加上 --bind 0.0.0.0，然后把 127.0.0.1 换成本机的 IP。

按 Ctrl+C 停止。
"""
import argparse
import io
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from cyberdog2 import CyberDog  # noqa: E402

DOG_IP = "192.168.1.139"
DOG_USER = "mi"
DOG_PASSWORD = "123"

BOUNDARY = "cyberdogframe"


class LatestJpeg(object):
    """主线程放入最新的 JPEG；每个 HTTP 观众各自等待下一帧。观众多少不影响拉流。"""

    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self.error = None  # 拉流线程崩溃时记录在这里，观众连接随之结束

    def put(self, jpeg: bytes) -> None:
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    def fail(self, exc: BaseException) -> None:
        with self._cond:
            self.error = exc
            self._cond.notify_all()

    def wait_next(self, last_seq: int, timeout: float = 10.0):
        with self._cond:
            if not self._cond.wait_for(lambda: self._seq != last_seq or self.error is not None, timeout):
                raise TimeoutError("%.0fs 内没有新帧" % timeout)
            if self.error is not None:
                raise self.error
            return self._seq, self._jpeg


def to_jpeg(frame) -> bytes:
    """远程连接时帧已经在狗端压成了 JPEG；mock 返回的是 raw 帧，这里用 Pillow 压缩。"""
    if frame.format == "jpeg":
        return frame.data
    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(frame.to_numpy()).convert("RGB").save(buf, "JPEG", quality=80)
    return buf.getvalue()


def make_handler(latest: LatestJpeg):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                body = ('<html><body style="margin:0;background:#000">'
                        '<img src="/stream.mjpg" style="width:100%"></body></html>').encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != "/stream.mjpg":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=" + BOUNDARY)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.end_headers()
            seq = 0
            try:
                while True:
                    seq, jpeg = latest.wait_next(seq)
                    self.wfile.write(("--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                      % (BOUNDARY, len(jpeg))).encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass  # 观众关掉了播放器，这是正常情况

        def log_message(self, fmt, *args):
            sys.stderr.write("[http] %s %s\n" % (self.address_string(), fmt % args))

    return Handler


def main():
    p = argparse.ArgumentParser(description="CyberDog 2 视角 HTTP 直播")
    p.add_argument("--mock", action="store_true", help="使用模拟狗")
    p.add_argument("--bind", default="127.0.0.1", help="HTTP 监听地址，局域网共享时用 0.0.0.0")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--fps", type=float, default=15)
    p.add_argument("--quality", type=int, default=70)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    args = p.parse_args()

    latest = LatestJpeg()
    server = ThreadingHTTPServer((args.bind, args.port), make_handler(latest))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, name="http", daemon=True).start()
    shown = "127.0.0.1" if args.bind == "0.0.0.0" else args.bind
    print("直播地址: http://%s:%d/stream.mjpg   （浏览器打开 http://%s:%d/）" % (shown, args.port, shown, args.port))

    if args.mock:
        dog = CyberDog.mock()
    else:
        print("连接 %s@%s ..." % (DOG_USER, DOG_IP))
        dog = CyberDog.connect(DOG_IP, user=DOG_USER, password=DOG_PASSWORD)

    try:
        with dog:
            with dog.sensors.camera.stream(width=args.width, height=args.height, fps=args.fps,
                                           quality=args.quality) as stream:
                print("开始拉流，按 Ctrl+C 停止")
                t0, n = time.time(), 0
                for frame in stream:
                    latest.put(to_jpeg(frame))
                    n += 1
                    if n % 50 == 0:
                        print("已推送 %d 帧，平均 %.1f fps，丢帧 %d" % (n, n / (time.time() - t0), stream.dropped))
    except KeyboardInterrupt:
        print("已停止")
    except BaseException as exc:
        latest.fail(exc)  # 让正在观看的连接也结束
        raise             # let it crash：拉流出错直接带着完整异常退出
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()

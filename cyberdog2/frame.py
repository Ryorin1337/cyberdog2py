"""相机图像帧。"""

_CHANNELS = {"rgb8": 3, "bgr8": 3, "mono8": 1, "rgba8": 4, "bgra8": 4}
_PIL_MODES = {"rgb8": ("RGB", "RGB"), "bgr8": ("RGB", "BGR"), "mono8": ("L", "L"),
              "rgba8": ("RGBA", "RGBA"), "bgra8": ("RGBA", "BGRA")}


class Frame(object):
    """一帧图像。

    format 为 "raw" 时，data 是未压缩的像素字节，排列方式由 encoding 决定（如 rgb8、bgr8）。
    format 为 "jpeg" 时，data 是 JPEG 文件字节，可以直接写入 .jpg 文件。
    """

    def __init__(self, width: int, height: int, encoding: str, data: bytes, format: str = "raw", stamp: float = 0.0):
        self.width = int(width)
        self.height = int(height)
        self.encoding = encoding
        self.data = bytes(data)
        self.format = format
        self.stamp = stamp

    def __repr__(self):
        return "Frame(%dx%d %s %s, %d bytes)" % (self.width, self.height, self.encoding, self.format, len(self.data))

    def to_dict(self) -> dict:
        return {"width": self.width, "height": self.height, "encoding": self.encoding,
                "data": self.data, "format": self.format, "stamp": self.stamp}

    @classmethod
    def from_dict(cls, d: dict) -> "Frame":
        return cls(d["width"], d["height"], d["encoding"], d["data"], d["format"], d.get("stamp", 0.0))

    def to_numpy(self):
        """返回 numpy 数组，形状为 (高, 宽, 通道)，通道顺序为 RGB。需要安装 numpy；解码 JPEG 还需要 Pillow。"""
        import numpy as np

        if self.format == "jpeg":
            import io
            from PIL import Image

            return np.asarray(Image.open(io.BytesIO(self.data)).convert("RGB"))
        if self.encoding not in _CHANNELS:
            raise ValueError("不支持的图像编码: %s" % self.encoding)
        ch = _CHANNELS[self.encoding]
        arr = np.frombuffer(self.data, dtype=np.uint8).reshape(self.height, self.width, ch)
        if self.encoding.startswith("bgr"):
            arr = arr[..., [2, 1, 0] + ([3] if ch == 4 else [])]
        return arr

    def to_bgr(self):
        """返回 OpenCV 习惯的 BGR numpy 数组，形状为 (高, 宽, 3)，可以直接交给 cv2 使用。需要 opencv-python。"""
        import cv2
        import numpy as np

        if self.format == "jpeg":
            img = cv2.imdecode(np.frombuffer(self.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                raise ValueError("JPEG 解码失败（%d 字节）" % len(self.data))
            return img
        arr = self.to_numpy()
        if arr.shape[2] == 1:
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        return cv2.cvtColor(np.ascontiguousarray(arr[..., :3]), cv2.COLOR_RGB2BGR)

    def save(self, path: str) -> None:
        """保存为图片文件。JPEG 帧直接写入字节；raw 帧需要 Pillow 转换。"""
        if self.format == "jpeg":
            with open(path, "wb") as f:
                f.write(self.data)
            return
        from PIL import Image

        arr = self.to_numpy()
        Image.fromarray(arr.squeeze() if arr.shape[2] == 1 else arr).save(path)


def encode_jpeg(frame: Frame, quality: int = 80) -> Frame:
    """在狗端把 raw 帧压缩成 JPEG。用 Pillow（实测狗上有 python3-pil 5.1.0，没有 cv2）；缺少时直接抛 ImportError。"""
    if frame.format == "jpeg":
        return frame
    import io

    from PIL import Image

    if frame.encoding not in _PIL_MODES:
        raise ValueError("不支持的图像编码: %s" % frame.encoding)
    mode, rawmode = _PIL_MODES[frame.encoding]
    img = Image.frombytes(mode, (frame.width, frame.height), frame.data, "raw", rawmode)
    if mode == "RGBA":
        img = img.convert("RGB")  # JPEG 不支持透明通道
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=quality)
    return Frame(frame.width, frame.height, frame.encoding, buf.getvalue(), "jpeg", frame.stamp)

"""ROS Depth のカメラモデルとメートル単位の画素入力を組み立てる。"""
from array import array
import math
import sys

from .planner import DepthProjection


def depth_metres(message):
    """32FC1 は m、16UC1 は mm。行のパディングとバイト順を処理する。"""
    formats = {"32FC1": ("f", 4), "16UC1": ("H", 2)}
    if message.encoding not in formats:
        raise ValueError("Depth encoding は 32FC1 または 16UC1 が必要です")
    kind, size = formats[message.encoding]
    packed_step = message.width * size
    if (message.width <= 0 or message.height <= 0 or message.step < packed_step
            or len(message.data) != message.height * message.step):
        raise ValueError("Depth の寸法・step・データ長が一致しません")
    raw = memoryview(message.data)
    pixels = array(kind)
    if message.step == packed_step:
        pixels.frombytes(raw)
    else:
        for offset in range(0, len(raw), message.step):
            pixels.frombytes(raw[offset:offset + packed_step])
    if bool(message.is_bigendian) != (sys.byteorder == "big"):
        pixels.byteswap()
    if kind == "H":
        return array("f", (value * .001 for value in pixels))
    return pixels


def optical_axes(rotation):
    x, y, z, w = rotation
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or norm == 0.:
        raise ValueError("Depth optical frame の回転が不正です")
    x, y, z, w = (component / norm for component in rotation)
    return (
        (1 - 2 * (y*y + z*z), 2 * (x*y + z*w), 2 * (x*z - y*w)),
        (2 * (x*y - z*w), 1 - 2 * (x*x + z*z), 2 * (y*z + x*w)),
        (2 * (x*z + y*w), 2 * (y*z - x*w), 1 - 2 * (x*x + y*y)),
    )


def projection(width, height, intrinsics, translation, rotation, rear_axle_height, body_height, *, allow_far_clip=False):
    fx, fy, cx, cy = intrinsics
    x, y, z = translation
    if min(width, height) <= 0 or min(fx, fy) <= 0 or not all(
        math.isfinite(value) for value in (*intrinsics, *translation)
    ):
        raise ValueError("整列済み Depth の CameraInfo または取付位置が不正です")
    return DepthProjection(width, height, fx, fy, cx, cy, (x, y, z + rear_axle_height),
                           optical_axes(rotation), body_height=body_height, allow_far_clip=allow_far_clip)

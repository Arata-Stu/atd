"""Gazebo の後輪軸姿勢をシミュレーション時刻で補間し、初期姿勢で map に合わせる。"""

from bisect import bisect_left
from dataclasses import dataclass
from decimal import Decimal
import json
import math
from pathlib import Path
from typing import NamedTuple

from atd_mapping.frames import rotate


class PlanarPose(NamedTuple):
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Trace:
    stamps: tuple[int, ...]
    poses: tuple[PlanarPose, ...]

    @classmethod
    def from_file(cls, path):
        path = Path(path)
        stamps, poses = [], []
        with path.open(encoding='utf-8') as stream:
            for number, line in enumerate(stream, 1):
                try:
                    sample = json.loads(line, parse_float=Decimal)
                    seconds = sample['sim_time_s']
                    if type(seconds) not in (int, Decimal) or not 0 <= seconds < 2**31:
                        raise ValueError('sim_time_s が ROS 時刻の範囲外です')
                    stamp = round(seconds * 1_000_000_000)
                    if stamps and stamp <= stamps[-1]:
                        raise ValueError('sim_time_s が単調増加していません')
                    coordinates = [sample[field] for field in ('x_m', 'y_m', 'yaw_rad')]
                    if any(type(v) not in (int, Decimal) for v in coordinates):
                        raise ValueError('姿勢には有限の数値が必要です')
                    x, y, yaw = map(float, coordinates)
                    if not all(map(math.isfinite, (x, y, yaw))):
                        raise ValueError('姿勢には有限の数値が必要です')
                    stamps.append(stamp)
                    poses.append(PlanarPose(x, y, math.remainder(yaw, math.tau)))
                except (ValueError, TypeError, KeyError, OverflowError) as exc:
                    raise ValueError(f'{path}:{number}: Gazebo 軌跡の読み取りエラー: {exc}') from exc
        if len(stamps) < 2:
            raise ValueError(f'{path}: Gazebo 軌跡には 2 点以上必要です')
        return cls(tuple(stamps), tuple(poses))

    def at(self, stamp_ns: int) -> PlanarPose | None:
        index = bisect_left(self.stamps, stamp_ns)
        if index < len(self.stamps) and self.stamps[index] == stamp_ns:
            return self.poses[index]
        if index == 0 or index == len(self.stamps):
            return None
        previous, following = self.poses[index-1:index+1]
        start, end = self.stamps[index-1:index+1]
        fraction = (stamp_ns - start) / (end - start)
        delta_yaw = math.remainder(following.yaw - previous.yaw, math.tau)
        return PlanarPose(
            previous.x + fraction * (following.x - previous.x),
            previous.y + fraction * (following.y - previous.y),
            math.remainder(previous.yaw + fraction * delta_yaw, math.tau),
        )


def _compose_yaw(q, yaw):
    """Quaternion product q * Rz(yaw), in ROS xyzw order."""
    x, y, z, w = q
    s, c = math.sin(yaw / 2), math.cos(yaw / 2)
    return (c*x + s*y, c*y - s*x, c*z + s*w, c*w - s*z)


@dataclass(frozen=True)
class Alignment:
    world_origin: PlanarPose
    map_origin: tuple[float, float, float]
    map_from_world: tuple[float, float, float, float]

    @classmethod
    def from_anchor(cls, trace_pose, position_xyz, orientation_xyzw):
        position, orientation = tuple(position_xyz), tuple(orientation_xyzw)
        if len(position) != 3 or not all(map(math.isfinite, position)):
            raise ValueError('cuVSLAM の初期位置が不正です')
        if len(orientation) != 4:
            raise ValueError('cuVSLAM の初期姿勢が不正です')
        norm = math.hypot(*orientation)
        if not math.isfinite(norm) or norm < 1e-9:
            raise ValueError('cuVSLAM の初期姿勢が不正です')
        unit = tuple(v / norm for v in orientation)
        return cls(trace_pose, position, _compose_yaw(unit, -trace_pose.yaw))

    def apply(self, trace_pose):
        # The Gazebo link has constant height. Its z displacement from the anchor is zero.
        displacement = (trace_pose.x - self.world_origin.x, trace_pose.y - self.world_origin.y, 0.0)
        delta = rotate(displacement, self.map_from_world)
        position = tuple(origin + d for origin, d in zip(self.map_origin, delta))
        return position, _compose_yaw(self.map_from_world, trace_pose.yaw)

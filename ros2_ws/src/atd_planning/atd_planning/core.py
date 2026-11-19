"""保存済み速度プロファイルとの対応・連続性を保って後輪軸をライン上に投影する。"""
from bisect import bisect_right
from dataclasses import dataclass
import math

LINE_KEYS = ('raceline', 'centerline', 'custom')
MAX_AGE_S = .5
DEFAULT_HORIZON_M = 2.


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def yaw_from_quaternion(qx, qy, qz, qw):
    values = (qx, qy, qz, qw)
    if not all(map(_number, values)):
        raise ValueError('車体姿勢に非有限値があります')
    norm = math.hypot(*values)
    if abs(norm - 1.) > .01:
        raise ValueError('車体姿勢のクォータニオンが正規化されていません')
    x, y, z, w = (v / norm for v in values)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def invalid(reason):
    return dict(valid=False, reason=reason, progress_m=None, lateral_error_m=None, target_speed_mps=None,
                estimated_speed_mps=None, trajectory=[], trajectory_speeds_mps=[])


@dataclass(frozen=True)
class Segment:
    x: float
    y: float
    ux: float
    uy: float
    length: float
    start_s: float


class ReferenceLine:
    def __init__(self, points, speeds, closed):
        self.closed = closed
        self.speeds = speeds
        self.segments = []
        self.stations = []
        station = 0.
        for i in range(len(points) if closed else len(points) - 1):
            (x, y), (next_x, next_y) = points[i], points[(i + 1) % len(points)]
            dx, dy = next_x - x, next_y - y
            length = math.hypot(dx, dy)
            if not math.isfinite(length) or length < 1e-8:
                raise ValueError('保存ラインに重複点または不正な線分があります')
            self.stations.append(station)
            self.segments.append(Segment(x, y, dx / length, dy / length, length, station))
            station += length
        if not math.isfinite(station):
            raise ValueError('ライン全長が有限値ではありません')
        self.length = station
        if not closed:
            self.stations.append(station)

    @classmethod
    def from_document(cls, doc, line):
        if line not in LINE_KEYS:
            raise ValueError('ラインには raceline / centerline / custom を指定してください')
        if not isinstance(doc, dict) or doc.get('frame') != 'map' or type(doc.get('closed')) is not bool:
            raise ValueError('map 座標系の HDMap と開閉路設定が必要です')
        saved = doc.get('lines')
        saved = saved.get(line) if isinstance(saved, dict) else None
        if not isinstance(saved, dict):
            raise ValueError(f'保存済みの {line} がありません')
        points, rows = saved.get('points'), saved.get('profile')
        if (not isinstance(points, list) or len(points) < (3 if doc['closed'] else 2)
                or not isinstance(rows, list) or len(rows) != len(points)):
            raise ValueError(f'{line} の点列と速度プロファイルを生成・保存してください')
        speeds = []
        for xy, row in zip(points, rows):
            if (not isinstance(xy, (list, tuple)) or len(xy) != 2 or not all(map(_number, xy))
                    or not isinstance(row, (list, tuple)) or len(row) != 7 or not all(map(_number, row))
                    or row[5] < 0 or not all(math.isclose(a, b, rel_tol=0, abs_tol=1e-6)
                                               for a, b in zip(xy, row[1:3]))):
                raise ValueError(f'{line} の点列と速度プロファイルの値が不正です')
            speeds.append(float(row[5]))
        if 'speeds' in saved and (not isinstance(saved['speeds'], list) or len(saved['speeds']) != len(speeds)
                or any(not _number(a) or not math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-6)
                       for a, b in zip(saved['speeds'], speeds))):
            raise ValueError(f'{line} の保存速度と速度プロファイルが一致しません')
        route = cls(points, speeds, doc['closed'])
        if not all(math.isclose(row[0], s, rel_tol=1e-6, abs_tol=1e-6) for row, s in zip(rows, route.stations)):
            raise ValueError(f'{line} の弧長と速度プロファイルが一致しません')
        return route

    def _index(self, station):
        return min(len(self.segments) - 1, max(0, bisect_right(self.stations, station) - 1))

    def _sample(self, station):
        s = station % self.length if self.closed else min(self.length, max(0., station))
        index = self._index(s)
        edge = self.segments[index]
        ds = min(edge.length, max(0., s - edge.start_s))
        ratio = ds / edge.length
        v0, v1 = self.speeds[index], self.speeds[(index + 1) % len(self.speeds)]
        # profile() uses constant acceleration on each edge: v²(s) varies linearly.
        speed = math.hypot(math.sqrt(1 - ratio) * v0, math.sqrt(ratio) * v1)
        return [edge.x + ds * edge.ux, edge.y + ds * edge.uy], speed

    def preview(self, progress_m, horizon_m=DEFAULT_HORIZON_M):
        if not _number(progress_m) or not _number(horizon_m) or horizon_m <= 0:
            raise ValueError('有限の進捗と正の前方距離 [m] が必要です')
        start = progress_m % self.length if self.closed else min(self.length, max(0., progress_m))
        end = min(start + horizon_m, start + self.length if self.closed else self.length)
        points, speeds = [], []
        point, speed = self._sample(start)
        points.append(point)
        speeds.append(speed)
        i = self._index(start) + 1
        count = len(self.stations)
        while True:
            lap, j = divmod(i, count)
            if lap and not self.closed:
                break
            vertex = self.stations[j] + self.length * lap
            if vertex >= end - 1e-9:
                break
            edge = self.segments[j]
            points.append([edge.x, edge.y])
            speeds.append(self.speeds[j])
            i += 1
        if end > start + 1e-9:
            point, speed = self._sample(end)
            points.append(point)
            speeds.append(speed)
        return points, speeds

    def nearby(self, station, radius):
        # Use arc length to bound a normal update; initial localization searches the full line.
        count = len(self.segments)
        if self.closed and 2 * radius >= self.length:
            return range(count)
        if self.closed:
            first = self._index((station - radius) % self.length)
            last = self._index((station + radius) % self.length)
            if (station - radius) // self.length != (station + radius) // self.length:
                return (i % count for i in range(first, min(first + count, count + last + 1)))
        else:
            first = self._index(station - radius)
            last = self._index(station + radius)
        return range(first, last + 1)

    def delta(self, station, previous):
        difference = station - previous
        if self.closed:
            return (difference + self.length * .5) % self.length - self.length * .5
        return difference


class Planner:
    def __init__(self, reference, *, horizon_m=DEFAULT_HORIZON_M, max_offset_m=.75):
        if not all(_number(v) and v > 0 for v in (horizon_m, max_offset_m)):
            raise ValueError('前方距離と投影距離の上限は正の有限値 [m] が必要です')
        self.reference = reference
        self.horizon_m = horizon_m
        self.max_offset_m = max_offset_m
        self.reset()

    def reset(self):
        self.previous = None
        self.source = None
        self.stamp_ns = None
        self.last_speed = 0.
        self.last_seen_ns = None

    def update(self, *, stamp_ns, x, y, yaw, speed_mps, session_id, epoch, include_trajectory=True):
        if (type(stamp_ns) is not int or stamp_ns <= 0 or not isinstance(session_id, str) or not session_id
                or type(epoch) is not int or epoch < 0 or not all(map(_number, (x, y, yaw, speed_mps)))):
            self.reset()
            return invalid('自己位置の時刻・姿勢・速度・追跡世代が不正です')
        source = (session_id, epoch)
        if source != self.source:
            self.reset()
        self.source = source
        if self.last_seen_ns is not None and stamp_ns < self.last_seen_ns:
            return invalid('自己位置の観測時刻が逆行しました')
        # Only a gap in actual observations or a new localization epoch permits a global reacquisition.
        if self.last_seen_ns is not None and stamp_ns - self.last_seen_ns > MAX_AGE_S * 1e9:
            self.previous = self.stamp_ns = None
        self.last_seen_ns = stamp_ns
        dt = 0. if self.stamp_ns is None else min(MAX_AGE_S, (stamp_ns - self.stamp_ns) * 1e-9)
        previous = self.previous
        route = self.reference
        predicted = .5 * (self.last_speed + speed_mps) * dt
        radius = .25 + dt * (1. + max(abs(speed_mps), abs(self.last_speed)))
        indices = range(len(route.segments)) if previous is None else route.nearby(previous, radius)
        hx, hy = math.cos(yaw), math.sin(yaw)
        candidates = []
        for i in indices:
            edge = route.segments[i]
            dot = hx * edge.ux + hy * edge.uy
            if dot < .25:  # Reverse-facing neighbouring lanes are not forward candidates.
                continue
            dx, dy = x - edge.x, y - edge.y
            along = min(edge.length, max(0., dx * edge.ux + dy * edge.uy))
            distance = math.hypot(dx - along * edge.ux, dy - along * edge.uy)
            if distance > self.max_offset_m:
                continue
            station = edge.start_s + along
            if route.closed:
                station %= route.length
            ds = 0. if previous is None else route.delta(station, previous)
            if previous is not None and abs(ds) > radius:
                continue
            score = distance + .15 * (1. - dot)
            if previous is not None:
                score += .15 * abs(predicted - ds)
            lateral = edge.ux * dy - edge.uy * dx
            candidates.append((score, station, lateral))
        if not candidates:
            return invalid('車両の位置・向き・前回の進捗と整合するライン区間がありません')
        best, station, lateral = min(candidates, key=lambda candidate: candidate[0])
        # Similar geometry can belong to different parts of the course even with matching headings.
        if any(score < best + .035 and abs(route.delta(other, station)) > .35
               for score, other, _ in candidates):
            return invalid('近接する異なるライン区間を識別できません')
        self.previous, self.stamp_ns, self.last_speed = station, stamp_ns, speed_mps
        _, target = route._sample(station)
        points, speeds = route.preview(station, self.horizon_m) if include_trajectory else ([], [])
        return dict(valid=True, reason='', progress_m=station, lateral_error_m=lateral,
                    estimated_speed_mps=speed_mps, target_speed_mps=target, trajectory=points,
                    trajectory_speeds_mps=speeds)

"""左右境界に基づく参照線。曲率低減は最短ラップや実車性能を保証しない。"""
from __future__ import annotations

import math
from shapely.geometry import LineString, LinearRing, Polygon


DEFAULTS = dict(spacing=0.1, width=0.19, margin=0.05, max_speed=1.5,
                lateral_accel=2.5, accel=1.5, decel=2.5)


def points(raw, minimum=0):
    if not isinstance(raw, list) or not minimum <= len(raw) <= 2000:
        raise ValueError(f'点列は {minimum}〜2000 点で指定してください')
    result = []
    for point in raw:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError('点は [x, y] で指定してください')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or abs(v) > 1e6 for v in point):
            raise ValueError('座標は有限のメートル単位の数値で指定してください')
        p = [float(v) for v in point]
        if result and math.dist(result[-1], p) < 1e-7:
            raise ValueError('連続する同一点を除いてください')
        result.append(p)
    if len(result) > 2 and math.dist(result[0], result[-1]) < 1e-7:
        raise ValueError('閉路の終点を始点に重ねず closed を指定してください')
    return result


def settings(raw):
    if not isinstance(raw, dict) or set(raw) - DEFAULTS.keys():
        raise ValueError('未知の生成設定です')
    output = dict(DEFAULTS, **raw)
    ranges = dict(spacing=(0.03, 2), width=(0.01, 3), margin=(0, 2), max_speed=(0.01, 30),
                  lateral_accel=(0.01, 30), accel=(0.01, 30), decel=(0.01, 30))
    for key, (lo, hi) in ranges.items():
        value = output[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f'{key} は {lo}〜{hi} の数値で指定してください')
    return output


def corridor(document):
    closed = document['closed']
    minimum = 3 if closed else 2
    left, right = points(document['left'], minimum), points(document['right'], minimum)
    if closed:
        a, b = Polygon(left), Polygon(right)
        if not a.is_valid or not b.is_valid or not LinearRing(left).is_simple or not LinearRing(right).is_simple:
            raise ValueError('境界が自己交差しています')
        if a.contains(b):
            region = Polygon(left, [right])
        elif b.contains(a):
            region = Polygon(right, [left])
        else:
            raise ValueError('閉路の内外境界が交差しているか、包含関係がありません')
    else:
        region = Polygon(left + right[::-1])
    if not region.is_valid or region.area <= 1e-8:
        raise ValueError('左右境界で有効な走行領域を作れません。方向・交差を確認してください')
    return region


def line_shape(line, closed):
    return LineString(line + [line[0]] if closed else line)


def resample(line, spacing, closed):
    shape = line_shape(line, closed)
    count = max(3 if closed else 2, math.ceil(shape.length / spacing) + (not closed))
    if count > 2000:
        raise ValueError('生成点数が2000を超えます。点間隔を広げてください')
    divisor = count if closed else count - 1
    return [list(shape.interpolate(i / divisor, normalized=True).coords[0]) for i in range(count)]


def speeds(line, closed, config):
    n = len(line)
    result = []
    for i in range(n):
        if not closed and i in (0, n - 1):
            result.append(0.0)
            continue
        a, b, c = line[(i - 1) % n], line[i], line[(i + 1) % n]
        denom = math.dist(a, b) * math.dist(b, c) * math.dist(a, c)
        cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        curvature = 2 * cross / denom if denom > 1e-12 else 0
        result.append(min(config['max_speed'], math.sqrt(config['lateral_accel'] / curvature) if curvature > 1e-9 else config['max_speed']))
    distances = [math.dist(line[i], line[(i + 1) % n]) for i in range(n if closed else n - 1)]
    for _ in range(n + 1 if closed else 2):
        previous = result[:]
        for i, distance in enumerate(distances):
            j = (i + 1) % n
            result[j] = min(result[j], math.sqrt(result[i] ** 2 + 2 * config['accel'] * distance))
        for i in reversed(range(len(distances))):
            j = (i + 1) % n
            result[i] = min(result[i], math.sqrt(result[j] ** 2 + 2 * config['decel'] * distances[i]))
        if max(abs(a - b) for a, b in zip(previous, result)) < 1e-8:
            break
    return result


def generate(document, kind, raw_settings, check=lambda: None):
    if kind not in ('centerline', 'raceline', 'custom'):
        raise ValueError('未知のライン種別です')
    config, closed = settings(raw_settings), document['closed']
    region = corridor(document)
    usable = region.buffer(-(config['width'] / 2 + config['margin']))
    if usable.is_empty:
        raise ValueError('車幅と余裕を満たす走行領域がありません')
    def contained(shape):
        if closed:
            return usable.buffer(1e-8).covers(shape)
        clearance = config['width'] / 2 + config['margin']
        return region.buffer(1e-8).covers(shape) and all(shape.distance(LineString(document[side])) + 1e-8 >= clearance for side in ('left', 'right'))
    if kind == 'custom':
        line = resample(points(document['custom'], 3 if closed else 2), config['spacing'], closed)
        method = 'custom_arc_length'
    else:
        left, right = document['left'], document['right']
        if closed:
            # 同一方向に統一し、始点の位相だけ合わせる。点の並び替えで交差を修正しない。
            if LinearRing(left).is_ccw != LinearRing(right).is_ccw:
                right = right[::-1]
            index = min(range(len(right)), key=lambda i: math.dist(left[0], right[i]))
            right = right[index:] + right[:index]
        a, b = line_shape(left, closed), line_shape(right, closed)
        count = max(3 if closed else 2, math.ceil(max(a.length, b.length) / config['spacing']) + (not closed))
        if count > 2000:
            raise ValueError('生成点数が2000を超えます。点間隔を広げてください')
        divisor = count if closed else count - 1
        line = []
        for i in range(count):
            p, q = a.interpolate(i / divisor, normalized=True), b.interpolate(i / divisor, normalized=True)
            line.append([(p.x + q.x) / 2, (p.y + q.y) / 2])
        method = 'paired_boundary_midpoints'
        if kind == 'raceline':
            method = 'constrained_local_smoothing'
            for _ in range(30):
                check()
                candidate = []
                for i, p in enumerate(line):
                    if not closed and i in (0, len(line) - 1):
                        candidate.append(p)
                    else:
                        candidate.append([0.5 * p[k] + 0.25 * (line[i - 1][k] + line[(i + 1) % len(line)][k]) for k in (0, 1)])
                shape = line_shape(candidate, closed)
                if shape.is_simple and contained(shape):
                    line = candidate
                else:
                    break
    shape = line_shape(line, closed)
    if not shape.is_simple:
        raise ValueError('生成ラインが自己交差しています')
    # 開路の始終点は境界の端を通る。横方向の境界余裕を検査し、端に仮の延長を加えない。
    if closed:
        valid = usable.buffer(1e-8).covers(shape)
    else:
        clearance = config['width'] / 2 + config['margin']
        valid = region.buffer(1e-8).covers(shape) and all(shape.distance(LineString(document[side])) + 1e-8 >= clearance for side in ('left', 'right'))
    if not valid:
        raise ValueError('ラインが走行領域外、または車幅・余裕を満たしていません')
    check()
    return dict(points=line, speeds=speeds(line, closed, config), method=method, settings=config,
                length=shape.length, closed=closed, limitations='オフライン参照。車両の操舵応答・実制動・最短ラップは未検証')

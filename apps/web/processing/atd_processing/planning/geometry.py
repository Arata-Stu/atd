# Ported from kart/tools/app/kart_studio/geometry.py; paired centerline removed.
"""Planar geometry: corridor validation and sampled paths."""

import math


def number(value, low, high, label):
    result = float(value)
    if not math.isfinite(result) or not low <= result <= high:
        raise ValueError(f"{label}: {low}〜{high}で指定してください")
    return result


def points(values, minimum=2, maximum=2000):
    if not isinstance(values, list) or not minimum <= len(values) <= maximum:
        raise ValueError(f"点は{minimum}〜{maximum}個必要です")
    out = []
    for p in values:
        if not isinstance(p, (list, tuple)) or len(p) < 2:
            raise ValueError("XY座標が必要です")
        out.append([number(p[0], -1e5, 1e5, "X"), number(p[1], -1e5, 1e5, "Y")])
    if any(math.dist(a, b) < 1e-6 for a, b in zip(out, out[1:])):
        raise ValueError("連続した点が重なっています")
    return out


def edges(line, closed=False):
    return list(zip(line, line[1:] + (line[:1] if closed else [])))


def distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    t = max(0, min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den)) if den else 0
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def intersects(a, b, c, d):
    if cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0:
        return True
    return (
        min(distance(a, c, d), distance(b, c, d), distance(c, a, b), distance(d, a, b))
        < 1e-8
    )


def segment_distance(a, b, c, d):
    return (
        0
        if intersects(a, b, c, d)
        else min(
            distance(a, c, d), distance(b, c, d), distance(c, a, b), distance(d, a, b)
        )
    )


def simple(line, closed):
    es = edges(line, closed)
    # Sweep candidate pairs on x. Keep the same endpoint tolerance as intersects().
    boxes = [(min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]))
             for a, b in es]
    active = []
    crossing = None
    for a, b in es:
        if math.dist(a, b) < 1e-6:
            raise ValueError(
                "境界の始終点が重複しています。閉路では終点を重ねないでください"
            )
    for i in sorted(range(len(es)), key=lambda k: boxes[k][0]):
        xmin, _, ymin, ymax = boxes[i]
        active = [j for j in active if boxes[j][1] + 1e-8 >= xmin]
        for j in active:
            first, last = sorted((i, j))
            if last - first < 2 or (closed and first == 0 and last == len(es) - 1):
                continue
            if boxes[j][2] > ymax + 1e-8 or boxes[j][3] + 1e-8 < ymin:
                continue
            if intersects(*es[first], *es[last]) and (crossing is None or (first, last) < crossing):
                crossing = first, last
        active.append(i)
    if crossing is not None:
        i, j = crossing
        raise PathValidationError(
            f"ラインが自己交差しています（区間{i + 1}と区間{j + 1}）",
            *es[i], i, reason="self_intersection", other_segment=j + 1,
            other_points=[list(p) for p in es[j]],
        )


def inside(p, polygon):
    result = False
    low_y, high_y = p[1] - 1e-8, p[1] + 1e-8
    for a, b in edges(polygon, True):
        if (a[1] < low_y and b[1] < low_y) or (a[1] > high_y and b[1] > high_y):
            continue
        if distance(p, a, b) < 1e-8:
            return True
        if (a[1] > p[1]) != (b[1] > p[1]) and p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (
            b[1] - a[1]
        ) + a[0]:
            result = not result
    return result


def resample(line, count, closed):
    es = edges(line, closed)
    lengths = [math.dist(a, b) for a, b in es]
    total = sum(lengths)
    if total < 0.1:
        raise ValueError("ラインが短すぎます（最低0.1 m）")
    result, index, previous = [], 0, 0.0
    for i in range(count):
        target = total * i / (count if closed else count - 1)
        while index < len(es) - 1 and previous + lengths[index] < target:
            previous += lengths[index]
            index += 1
        t = (target - previous) / max(lengths[index], 1e-12)
        a, b = es[index]
        result.append([a[k] + t * (b[k] - a[k]) for k in range(2)])
    return result


class PathValidationError(ValueError):
    """JSON-safe location of a rejected candidate; never a publishable line."""
    def __init__(self, message, a, b, index, **details):
        self.diagnostic = dict(segment=index + 1, points=[list(a), list(b)], **details)
        super().__init__(f"{message}（区間{index + 1}, "
                         f"XY=({a[0]:.3f}, {a[1]:.3f}) → ({b[0]:.3f}, {b[1]:.3f})）")


class Corridor:
    def __init__(self, left, right, closed):
        self.left, self.right, self.closed = left, right, closed
        for line in (left, right):
            simple(line, closed)
        self.boundaries = edges(left, closed) + edges(right, closed)
        self._boundary_boxes = [
            (min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]), a, b)
            for a, b in self.boundaries
        ]
        for a, b in edges(left, closed):
            if any(intersects(a, b, c, d) for c, d in edges(right, closed)):
                raise ValueError("左右の境界が交差しています")
        if closed:
            if inside(left[0], right) == inside(right[0], left):
                raise ValueError("閉路は内側と外側の境界を描いてください")
        else:
            self.polygon = left + list(reversed(right))
            simple(self.polygon, True)

    def contains(self, p):
        return (
            (inside(p, self.left) != inside(p, self.right))
            if self.closed
            else inside(p, self.polygon)
        )

    def segment_ok(self, a, b, clearance):
        if (
            not self.contains(a)
            or not self.contains(b)
            or not self.contains([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2])
        ):
            return False
        limit = max(clearance, 1e-7)
        xmin, xmax = min(a[0], b[0]) - limit, max(a[0], b[0]) + limit
        ymin, ymax = min(a[1], b[1]) - limit, max(a[1], b[1]) + limit
        return all(
            segment_distance(a, b, c, d) >= limit
            for x0, x1, y0, y1, c, d in self._boundary_boxes
            if x0 <= xmax and xmin <= x1 and y0 <= ymax and ymin <= y1
        )

    def validate(self, line, clearance=0):
        simple(line, self.closed)
        for i, (a, b) in enumerate(edges(line, self.closed)):
            if not self.segment_ok(a, b, clearance):
                nearest = min(self.boundaries, key=lambda edge: segment_distance(a, b, *edge))
                gap = segment_distance(a, b, *nearest)
                outside = any(not self.contains(p) for p in (a,b,[(a[0]+b[0])/2,(a[1]+b[1])/2]))
                reason = "生成線がコース境界の外です" if outside else "生成線と境界の距離が不足しています"
                raise PathValidationError(
                    f"{reason}: 境界まで{gap:.3f} m / 必要{clearance:.3f} m", a,b,i,
                    boundary=[list(p) for p in nearest], distance_m=gap,
                    required_clearance_m=clearance, reason="outside" if outside else "clearance")


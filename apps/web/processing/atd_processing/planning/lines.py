"""Kart と同じ領域中心線・固定 helper 最適化・車体/曲率/速度検査。"""
import math
from . import footprint
from .geometry import PathValidationError, Corridor, edges, points
from .speed_sections import sample, sections, speed_cap

def curvature(a, b, c):
    ab, bc, ac = math.dist(a, b), math.dist(b, c), math.dist(a, c)
    return (
        2
        * ((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]))
        / max(ab * bc * ac, 1e-12)
    )


def profile(line, closed, p, limits=(), step=False):
    n = len(line)
    k = [
        curvature(line[(i - 1) % n], line[i], line[(i + 1) % n])
        if closed or 0 < i < n - 1
        else 0
        for i in range(n)
    ]
    v = [
        p["max_speed"] if step else min(p["max_speed"], math.sqrt(p["lateral_accel"] / max(abs(c), 1e-8)))
        for c in k
    ]
    station = 0.0
    for i in range(n):
        if step:
            # Half-open intervals: the new target takes effect at its boundary.
            hits=[r["speed"] for r in limits if (r["start"] <= station+1e-8 < r["end"] if r["start"]<r["end"] else station+1e-8>=r["start"] or station+1e-8<r["end"])]
            v[i]=min(hits) if hits else p["max_speed"]
        else:
            v[i] = min(v[i], speed_cap(station, limits, p["max_speed"]))
        if i < n - 1:
            station += math.dist(line[i], line[i + 1])
    if not closed:
        v[-1] = 0.0
        if not step: v[0] = 0.0
    es = edges(line, closed)
    # Propagate squared-speed upper bounds around a closed loop until converged.
    for _ in range(0 if step else n + 1 if closed else 2):
        old = v[:]
        for i, (a, b) in enumerate(es):
            j = (i + 1) % n
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * p["accel"] * math.dist(a, b)))
        for i in reversed(range(len(es))):
            j = (i + 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * p["decel"] * math.dist(*es[i])))
        if max(abs(a - b) for a, b in zip(old, v)) < 1e-8:
            break
    s, rows = 0.0, []
    for i, point in enumerate(line):
        nxt = line[(i + 1) % n] if closed or i < n - 1 else line[i]
        prev = line[(i - 1) % n] if closed or i else line[i]
        heading = math.atan2(nxt[1] - prev[1], nxt[0] - prev[0])
        ds = math.dist(point, nxt)
        ax = (v[(i + 1) % n] ** 2 - v[i] ** 2) / (2 * ds) if ds else 0
        rows.append([s, *point, heading, k[i], v[i], ax])
        s += ds
    return rows


def validate_raceline_curvature(line, p, kappa=None):
    if kappa is None:
        kappa = [curvature(line[i - 1], line[i], line[(i + 1) % len(line)])
                 for i in range(len(line))]
    index = max(range(len(kappa)), key=lambda i: abs(kappa[i]))
    if abs(kappa[index]) > p["curvature_limit"] * 1.01:
        raise PathValidationError(
            f"補間後の曲率が上限を超えています: 最大{abs(kappa[index]):.3f} 1/m / "
            f"上限{p['curvature_limit']:.3f} 1/m。設定または境界を確認してください",
            line[index], line[(index + 1) % len(line)], index,
            stage="raceline_optimized_output", reason="curvature",
            curvature_abs=abs(kappa[index]), curvature_limit=p["curvature_limit"],
        )


def generate(document, kind, p, check=lambda: None):
    closed = document['closed']
    region_info = reference_info = optimizer_info = None
    if kind == 'centerline':
        from .region_centerline import extract
        line, widths, corridor, region_info = extract(
            document['left'], document['right'], closed, p['spacing'],
            p['region_resolution'], check, max_points=p['centerline_max_points'])
        method = 'region_equal_distance_v1'
    elif kind == 'raceline':
        if not closed:
            raise ValueError('Raceline の mincurv / mincurv_iqp は閉路専用です。別方式へ fallback しません')
        from .raceline_reference import build
        _, corridor, reference_info = build(document, p['spacing'])
        from .optimizer import optimize
        optimized = optimize(document, p, check)
        line = optimized['points']
        optimizer_info = optimized['optimizer_info']
        method = 'tph_' + p['optimizer']
    elif kind == 'custom':
        control = points(document.get('custom', []), 3 if closed else 2)
        line = sample(control, closed, p['spacing'], [])
        corridor = Corridor(points(document['left'], 3 if closed else 2),
                            points(document['right'], 3 if closed else 2), closed)
        method = 'resampled_user_polyline'
    else:
        raise ValueError('未知のライン種別です')
    check()
    clearance = p['vehicle_width'] / 2 + p['margin']
    corridor.validate(line, clearance)
    rows = profile(line, closed, p)
    footprint_corridor = footprint.corridor_for(document['left'], document['right'], closed, p)
    footprint.validate_line(line, closed, p, corridor=footprint_corridor,
                            yaws=[row[3] for row in rows], check=check)
    if kind == 'raceline':
        validate_raceline_curvature(line, p, [row[4] for row in rows])
    result = dict(points=line, profile=rows, speeds=[row[5] for row in rows], method=method,
                  optimizer_info=optimizer_info, reference_info=reference_info,
                  collision_model=footprint.MODEL)
    if region_info is not None:
        result.update(widths=widths, region_info=region_info)
    return result


def curvature_metrics(line):
    """Closed-route metrics independent of sample count; arc integrals use trapezoids."""
    count = len(line)
    kappa = [curvature(line[i-1], line[i], line[(i+1) % count]) for i in range(count)]
    lengths = [math.dist(line[i], line[(i+1) % count]) for i in range(count)]
    return dict(point_count=count, length_m=sum(lengths),
                mean_squared_curvature_1_per_m2=sum(k*k for k in kappa)/count,
                squared_curvature_integral_1_per_m=sum(
                    .5*(kappa[i]**2 + kappa[(i+1) % count]**2)*lengths[i] for i in range(count)),
                signed_curvature_integral_rad=sum(
                    .5*(kappa[i] + kappa[(i+1) % count])*lengths[i] for i in range(count)),
                absolute_curvature_integral_rad=sum(
                    .5*(abs(kappa[i]) + abs(kappa[(i+1) % count]))*lengths[i] for i in range(count)))

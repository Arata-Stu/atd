"""後輪軸を原点とする速度適応 Pure Pursuit。ROS と bag 解析で共用する。"""
from dataclasses import dataclass
import math


CONTROLLER = 'pure_pursuit'
EPSILON_M = 1e-8
STANDSTILL_MPS = .05


def forward_control_speed(measured, low_speed_drive):
    """低速自動走行の制御計算だけで停止帯の VO の揺らぎをゼロと扱う。"""
    return 0. if low_speed_drive and abs(measured) <= STANDSTILL_MPS else measured


def _finite(value):
    try:
        return type(value) in (float, int) and math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class LookaheadConfig:
    min_lookahead_m: float = 0.35
    lookahead_time_s: float = 0.4
    max_lookahead_m: float = 1.5

    def __post_init__(self):
        if not (all(_finite(v) for v in (self.min_lookahead_m, self.max_lookahead_m, self.lookahead_time_s))
                and 0 < self.min_lookahead_m <= self.max_lookahead_m and self.lookahead_time_s >= 0):
            raise ValueError('先読み距離は 0 < 最小 ≤ 最大 [m]、先読み時間は 0 以上 [s] の有限値を指定してください')

    def distance(self, speed_mps):
        return min(self.max_lookahead_m, self.min_lookahead_m + abs(speed_mps) * self.lookahead_time_s)


def unavailable(reason):
    return dict(valid=False, reason=reason, controller=CONTROLLER, target=None, requested_lookahead_m=None,
                actual_lookahead_m=None, curvature_per_m=None, steering_rad=None, preview_limited=False)


def _forward_intersection(start, end, radius):
    """進行順の線分と後輪軸を中心とする円の交点を車体座標で求める。"""
    x, y = start
    dx, dy = end[0] - x, end[1] - y
    length = math.hypot(dx, dy)
    if not math.isfinite(length) or length <= EPSILON_M:
        return None
    ux, uy = dx / length, dy / length
    # 後輪軸を無限直線に射影した位置と、直線から軸までの垂直距離。
    along = -(x * ux + y * uy)
    perpendicular = abs(x * uy - y * ux)
    if perpendicular > radius:
        return None
    half_chord = radius * math.sqrt(max(0., 1. - (perpendicular / radius) ** 2))
    for distance in (along - half_chord, along + half_chord):
        if -EPSILON_M <= distance <= length + EPSILON_M:
            d = min(length, max(0., distance))
            point = x + ux * d, y + uy * d
            if point[0] > EPSILON_M:
                return point
    return None


def pure_pursuit(trajectory, *, x, y, yaw, speed_mps, wheelbase_m=None, config=LookaheadConfig(), low_speed_drive=False):
    """map の軌道を進行順に検索し、基準曲率と任意の等価前輪舵角を返す。左正。"""
    if not all(_finite(v) for v in (x, y, yaw, speed_mps)):
        return unavailable('後輪軸の姿勢または推定速度が不正です')
    speed_mps = forward_control_speed(speed_mps, low_speed_drive)
    if speed_mps < 0:
        return unavailable('後退中です。前進用の追従計算を休止しています')
    if wheelbase_m is not None and (not _finite(wheelbase_m) or wheelbase_m <= 0):
        return unavailable('ホイールベースには正の有限値 [m] が必要です')
    if not isinstance(trajectory, (list, tuple)) or len(trajectory) < 2:
        return unavailable('目標を選べる前方の軌道区間がありません')
    radius = config.distance(speed_mps)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    points = []
    for point in trajectory:
        if not isinstance(point, (tuple, list)) or len(point) != 2 or not all(map(_finite, point)):
            return unavailable('前方軌道の座標が不正です')
        dx, dy = point[0] - x, point[1] - y
        body = cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy
        if not all(map(math.isfinite, body)):
            return unavailable('車体座標への変換結果が不正です')
        points.append(body)

    limited = False
    for start, end in zip(points, points[1:]):
        target = _forward_intersection(start, end, radius)
        if target is not None:
            break
    else:
        # 短い開路末端や preview の終端。手前の任意の頂点を代用しない。
        target = points[-1]
        if (target[0] <= EPSILON_M or any(math.hypot(*p) > radius for p in points)
                or not any(math.dist(p, target) > EPSILON_M for p in points[:-1])):
            return unavailable('先読み円と交わる前方軌道、または円内の前方終端がありません')
        limited = True

    tx, ty = target
    actual = math.hypot(tx, ty)
    curvature = 2. * (ty / actual) / actual
    angle = None if wheelbase_m is None else math.atan(wheelbase_m * curvature)
    return dict(valid=True, reason='', controller=CONTROLLER,
                target=[x + cos_yaw * tx - sin_yaw * ty, y + sin_yaw * tx + cos_yaw * ty],
                requested_lookahead_m=radius, actual_lookahead_m=actual, curvature_per_m=curvature,
                steering_rad=angle, preview_limited=limited)

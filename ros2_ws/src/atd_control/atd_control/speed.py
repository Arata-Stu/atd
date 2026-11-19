"""前進校正表と時刻付き速度観測を使う、CH2 raw 単位の速度制御プレビュー。"""
from bisect import bisect_right
from dataclasses import dataclass
import math
import re

from .core import forward_control_speed
from .manual_inputs import PROPO_MAX_AGE_NS, manual_reading


PID_MAX_GAP_NS = 500_000_000
DRIVE_FEEDBACK_TTL_NS = 300_000_000
DEFAULT_FEEDFORWARD_PATH = '/data/results/runs/vehicle/throttle_feedforward.json'


def _finite(value):
    try:
        return type(value) in (float, int) and math.isfinite(value)
    except OverflowError:
        return False


def limited_target(target, limit):
    """0 は監視用の無制限。無効な目標を上限値に置き換えない。"""
    if not _finite(limit) or not 0 <= limit <= 1.:
        raise ValueError('低速走行の速度上限は 0 超〜1.0 m/s、監視の制限なしは 0 を指定してください')
    return min(target, limit) if limit and _finite(target) else target


class DriveFeedback:
    """自動制御中の連続した送信セッションだけで PID の履歴を共有する。"""
    def __init__(self):
        self.session = None
        self.stamp = self.received = 0
        self.route = None

    def matches(self, route, wall_ns, steady_ns):
        return (self.route is not None and self.route == route
                and 0 <= wall_ns - self.stamp < DRIVE_FEEDBACK_TTL_NS
                and 0 <= steady_ns - self.received < DRIVE_FEEDBACK_TTL_NS)

    def update(self, data, wall_ns, steady_ns):
        """True のときだけ直前の制御履歴を引き継ぐ。再配信で期限を延ばさない。"""
        if not isinstance(data, dict) or not isinstance(data.get('session_id'), str) or not data['session_id']:
            self.route = None
            return False
        session = data['session_id']
        stamp = data.get('stamp_ns')
        previous_route = self.route
        continuous = self.matches(previous_route, wall_ns, steady_ns) and self.session == session
        if self.session != session:
            self.session, self.stamp = session, 0
        valid_stamp = type(stamp) is int and stamp > 0 and 0 <= wall_ns - stamp < DRIVE_FEEDBACK_TTL_NS
        if not valid_stamp or stamp < self.stamp:
            self.route = None
            return False
        route = ((data.get('map_id'), data.get('line')) if data.get('active') is True
                 and data.get('braking') is False and data.get('map_id') and data.get('line') else None)
        if stamp == self.stamp:
            if route != previous_route or not continuous:
                self.route = None
            return self.route is not None
        self.stamp, self.received, self.route = stamp, steady_ns, route
        return continuous and route is not None and previous_route == route


@dataclass(frozen=True)
class FeedforwardTable:
    run_id: str
    proposal_sha256: str
    speeds: tuple
    raw: tuple
    direction: int

    @classmethod
    def from_document(cls, doc):
        if not isinstance(doc, dict) or doc.get('schema') != 'atd.throttle_feedforward.v1':
            raise ValueError('速度 FF 表の形式を確認してください')
        if (doc.get('usable') is not True or doc.get('input_unit') != 'm/s'
                or doc.get('output_unit') != 'sbus_raw_ch2' or doc.get('interpolation') != 'linear'
                or doc.get('extrapolation') != 'none'):
            raise ValueError('速度 FF 表には使用可能な CH2 raw の前進校正結果が必要です')
        run_id, proposal = doc.get('run_id'), doc.get('proposal_sha256')
        if (not isinstance(run_id, str) or not run_id.strip() or not isinstance(proposal, str)
                or not re.fullmatch(r'[0-9a-f]{64}', proposal)):
            raise ValueError('速度 FF 表の校正 run ID または SHA256 が不正です')
        points = doc.get('points')
        if not isinstance(points, list) or len(points) < 2:
            raise ValueError('速度 FF 表には2点以上の観測が必要です')
        speeds, raw = [], []
        for p in points:
            if not isinstance(p, dict):
                raise ValueError('速度 FF 表の校正点が不正です')
            speed, throttle = p.get('speed_mps'), p.get('throttle_raw')
            if (not _finite(speed) or speed <= 0 or type(throttle) is not int or not 0 <= throttle <= 2047
                    or speeds and speed <= speeds[-1]):
                raise ValueError('校正点には昇順の正の速度と 0〜2047 の CH2 raw が必要です')
            speeds.append(float(speed))
            raw.append(throttle)
        if doc.get('direction') not in ('increasing', 'decreasing'):
            raise ValueError('CH2 の増減方向が不明です')
        direction = 1 if doc['direction'] == 'increasing' else -1
        if any((b - a) * direction <= 0 for a, b in zip(raw, raw[1:])):
            raise ValueError('CH2 の校正点が申告された増減方向と一致しません')
        if (not all(_finite(doc.get(k)) for k in ('speed_min_mps', 'speed_max_mps'))
                or (doc['speed_min_mps'], doc['speed_max_mps']) != (speeds[0], speeds[-1])):
            raise ValueError('速度 FF 表の有効速度域が端点と一致しません')
        return cls(run_id, proposal, tuple(speeds), tuple(raw), direction)

    def interpolate(self, target):
        if not _finite(target) or not self.speeds[0] <= target <= self.speeds[-1]:
            return None
        index = min(bisect_right(self.speeds, target) - 1, len(self.raw) - 2)
        a, b = self.speeds[index:index + 2]
        ratio = (target - a) / (b - a)
        lo, hi = self.raw[index:index + 2]
        return lo + (hi - lo) * ratio

    def forward_raw_bounds(self, max_target_speed_mps):
        """0 は全校正域。それ以外は低速上限で実測した定常スロットルまで。"""
        top = limited_target(self.speeds[-1], max_target_speed_mps)
        raw_limit = self.interpolate(top)
        if raw_limit is None:
            return None
        return tuple(sorted((self.raw[0], raw_limit)))


@dataclass(frozen=True)
class PIDConfig:
    kp: float = 40.
    ki: float = 8.
    kd: float = 1.

    def __post_init__(self):
        if not all(_finite(v) and v >= 0 for v in (self.kp, self.ki, self.kd)):
            raise ValueError('速度 PID ゲインには 0 以上の有限値が必要です')


def speed_unavailable(reason, target_speed_mps=None, estimated_speed_mps=None):
    target = target_speed_mps if _finite(target_speed_mps) and target_speed_mps >= 0 else None
    measured = estimated_speed_mps if _finite(estimated_speed_mps) else None
    error = target - measured if target is not None and measured is not None else None
    return dict(valid=False, reason=reason, target_speed_mps=target, estimated_speed_mps=measured,
                speed_error_mps=error if _finite(error) else None, feedforward_raw=None, p_raw=None,
                i_raw=None, d_raw=None, calculated_raw=None, saturated=False)


class SpeedController:
    """観測ごとに一回更新する PID。積分は台形則、微分は推定速度に適用する。"""
    def __init__(self, table, config=PIDConfig(), *, max_target_speed_mps=0.):
        self.table = table
        self.config = config
        self.speed_limit = max_target_speed_mps
        self.bounds = table.forward_raw_bounds(max_target_speed_mps)
        self.reset()

    def reset(self):
        self.stamp_ns = None
        self.identity = None
        self.last = None

    def update(self, stamp_ns, target_speed_mps, estimated_speed_mps, identity=None):
        target, measured = limited_target(target_speed_mps, self.speed_limit), estimated_speed_mps

        def invalid(reason):
            self.reset()
            return speed_unavailable(reason, target, measured)

        if type(stamp_ns) is not int or stamp_ns <= 0 or not _finite(target) or target < 0 or not _finite(measured):
            return invalid('速度観測または元時刻が不正です')
        pid_measured = forward_control_speed(measured, self.speed_limit > 0)
        if pid_measured < 0:
            return invalid('後退中は前進の速度校正表を適用できません')
        ff = self.table.interpolate(target)
        if ff is None or self.bounds is None:
            return invalid('目標速度が前進 FF 表の校正速度域外です')
        if identity != self.identity:
            self.reset()
        if self.stamp_ns is not None and stamp_ns <= self.stamp_ns:
            if (stamp_ns == self.stamp_ns and target == self.last['target_speed_mps']
                    and measured == self.last['estimated_speed_mps']):
                return self.last.copy()
            return invalid('速度観測の元時刻が逆行または重複しています')
        error = target - pid_measured
        sign = self.table.direction
        p = sign * self.config.kp * error
        i = d = 0.
        lo, hi = self.bounds
        if self.stamp_ns is not None and stamp_ns - self.stamp_ns <= PID_MAX_GAP_NS:
            dt = (stamp_ns - self.stamp_ns) * 1e-9
            previous_measured = forward_control_speed(self.last['estimated_speed_mps'], self.speed_limit > 0)
            d = -sign * self.config.kd * (pid_measured - previous_measured) / dt
            i = self.last['i_raw']
            previous_error = self.last['target_speed_mps'] - previous_measured
            step = sign * self.config.ki * .5 * (error + previous_error) * dt
        else:
            step = 0.
        base = ff + p + d + i
        unconstrained = base + step
        if not all(map(math.isfinite, (base, step, unconstrained))):
            return invalid('速度 PID の計算が有限範囲を超えました')
        # 積分の増分のみを制限し、飽和を深める蓄積を止める。逆方向の積分は許容する。
        if step >= 0:
            i += min(step, max(0., hi - base))
        else:
            i += max(step, min(0., lo - base))
        output = max(lo, min(hi, ff + p + i + d))
        self.last = dict(valid=True, reason='', target_speed_mps=target, estimated_speed_mps=measured,
                         speed_error_mps=target - measured, feedforward_raw=ff, p_raw=p, i_raw=i, d_raw=d,
                         calculated_raw=float(output), saturated=not lo <= unconstrained <= hi)
        self.stamp_ns, self.identity = stamp_ns, identity
        return self.last.copy()

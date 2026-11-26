"""最後に地図上で確定した姿勢から、観測済み車体ジャイロで平面運動を予測する。"""
from bisect import bisect_left, bisect_right
import math
from operator import itemgetter
from typing import NamedTuple

from .orb_gap_motion import HISTORY_NS, MAX_GYRO_HOLD_NS, advance_planar, estimate_motion


class Prediction(NamedTuple):
    x: float
    y: float
    yaw: float
    speed_mps: float
    age_s: float
    yaw_rate_radps: float


class GyroGapMotion:
    """入力の時計は画像・IMU の計測時刻。速度を推定する履歴には ORB の確定姿勢だけを使う。"""

    def __init__(self, max_gap_s=2.0):
        if not math.isfinite(max_gap_s) or max_gap_s <= 0:
            raise ValueError('補完の上限時間には正の有限値が必要です')
        self.max_gap_ns = round(max_gap_s * 1e9)
        # 画像処理中に先行して届く IMU も保持し、遅れて届く画像の計測時刻で積分する。
        self._retention_ns = self.max_gap_ns + 2_000_000_000
        self.reset()

    def reset(self):
        self._gyro = []
        self._epoch = None
        self._stamps = []
        self._poses = []
        self._speed = None
        self._cached = None

    def add_gyro(self, stamp_ns, yaw_rate_radps):
        if type(stamp_ns) is not int or stamp_ns < 0 or not math.isfinite(yaw_rate_radps):
            return False
        gyro = self._gyro
        if gyro and stamp_ns <= gyro[-1][0]:
            if stamp_ns < gyro[-1][0] - self._retention_ns:
                return False
            index = bisect_left(gyro, stamp_ns, key=itemgetter(0))
            if gyro[index][0] == stamp_ns:
                return False
            gyro.insert(index, (stamp_ns, yaw_rate_radps))
        else:
            gyro.append((stamp_ns, yaw_rate_radps))
        if self._cached and stamp_ns < self._cached[0]:
            self._cached = None
        cutoff = gyro[-1][0] - self._retention_ns
        if gyro[0][0] < cutoff - HISTORY_NS:
            # 保持期間の先頭で有効だった角速度を 1 点だけ残す。
            del gyro[:max(0, bisect_left(gyro, cutoff, key=itemgetter(0)) - 1)]
        return True

    def observe(self, stamp_ns, x, y, yaw, epoch):
        if (type(stamp_ns) is not int or stamp_ns < 0 or epoch is None
                or not all(map(math.isfinite, (x, y, yaw)))
                or (self._stamps and stamp_ns <= self._stamps[-1])):
            return False
        if epoch != self._epoch:
            self._stamps.clear()
            self._poses.clear()
            self._epoch = epoch
        first = bisect_left(self._stamps, stamp_ns - HISTORY_NS)
        del self._stamps[:first]
        del self._poses[:first]
        self._stamps.append(stamp_ns)
        self._poses.append((x, y, yaw))
        self._speed = None
        self._cached = None
        return True

    def predict(self, stamp_ns):
        if not self._stamps:
            return None, 'no_observation'
        anchor = self._stamps[-1]
        if type(stamp_ns) is not int or stamp_ns <= anchor:
            return None, 'not_after_observation'
        if len(self._stamps) < 2:
            return None, 'insufficient_history'
        age_ns = stamp_ns - anchor
        if age_ns > self.max_gap_ns:
            return None, 'horizon_exceeded'
        if self._speed is None:
            self._speed, _ = estimate_motion(self._stamps, self._poses)

        t, pose = anchor, self._poses[-1]
        if self._cached and self._cached[0] <= stamp_ns:
            t, pose = self._cached
        # サンプル時刻までは直前の角速度を保持する。終点以降の角速度は読まない。
        gyro = self._gyro
        index = bisect_right(gyro, t, key=itemgetter(0)) - 1
        if index < 0:
            return None, 'insufficient_gyro_coverage'
        while t < stamp_ns:
            sample_ns, rate = gyro[index]
            end = min(gyro[index+1][0], stamp_ns) if index+1 < len(gyro) else stamp_ns
            if end - sample_ns > MAX_GYRO_HOLD_NS:
                return None, 'insufficient_gyro_coverage'
            dt = (end - t) * 1e-9
            pose = advance_planar(pose, self._speed, dt, rate * dt)
            t, index = end, index + 1
        self._cached = stamp_ns, pose
        yaw_rate = gyro[bisect_right(gyro, stamp_ns, key=itemgetter(0)) - 1][1]
        return Prediction(*pose, self._speed, age_ns * 1e-9, yaw_rate), 'predicted'

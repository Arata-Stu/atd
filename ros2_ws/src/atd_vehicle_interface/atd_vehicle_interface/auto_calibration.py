"""計測した操舵・前進入力の対応を Rev6 HOST 指令に変換する。"""
from bisect import bisect_left
from dataclasses import dataclass
import json
import math
from pathlib import Path

import yaml
from atd_control.speed import FeedforwardTable


def _number(value, lo, hi, field):
    if type(value) not in (int, float) or not lo <= value <= hi:
        raise ValueError(f'{field} は {lo}〜{hi} の数値を指定してください')
    return float(value)


def _points(doc, key, xfield, xrange, yfield, yrange):
    points = doc.get(key)
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError(f'{key} に実測点を指定してください')
    xs, ys = [], []
    for point in points:
        if not isinstance(point, dict):
            raise ValueError(f'{key} の実測点が不正です')
        x = _number(point.get(xfield), *xrange, xfield)
        y = _number(point.get(yfield), *yrange, yfield)
        if not y.is_integer() or (xfield == 'ch2_raw' and not x.is_integer()):
            raise ValueError(f'{key} の PWM パルス幅と S.BUS raw は整数で指定してください')
        xs.append(x)
        ys.append(y)
    if not all(a < b for a, b in zip(xs, xs[1:])):
        raise ValueError(f'{key} は {xfield} の昇順で指定してください')
    if not (all(a < b for a, b in zip(ys, ys[1:])) or all(a > b for a, b in zip(ys, ys[1:]))):
        raise ValueError(f'{key} の {yfield} が単調に変化していません')
    return tuple(xs), tuple(ys)


def _interpolate(curve, x, field):
    xs, ys = curve
    _number(x, xs[0], xs[-1], field)
    hi = max(1, bisect_left(xs, x))
    weight = (x - xs[hi-1]) / (xs[hi] - xs[hi-1])
    return ys[hi-1] + weight * (ys[hi] - ys[hi-1])


@dataclass(frozen=True)
class AutoCalibration:
    steering: tuple
    throttle: tuple
    table: FeedforwardTable
    brake_level: float
    brake_hold_s: float

    @classmethod
    def from_document(cls, doc, table):
        if not isinstance(doc, dict) or doc.get('schema') != 'atd.auto_drive.v1':
            raise ValueError('実測した atd.auto_drive.v1 の校正ファイルを指定してください')
        angles, servo = _points(doc, 'steering', 'angle_deg', (-89.99, 89.99), 'servo_us', (1000, 2000))
        if 0. not in angles or not angles[0] < 0 < angles[-1]:
            raise ValueError('操舵には右・直進（0度）・左の実測点が必要です')
        raw, esc = _points(doc, 'forward_throttle', 'ch2_raw', (0, 2047), 'esc_us', (1000, 1500))
        neutral_index = 0 if table.direction > 0 else -1
        if esc[neutral_index] != 1500:
            raise ValueError('前進校正の中立 1500 µs と FF 表の CH2 増減方向が一致しません')
        if any((count - raw[neutral_index]) * table.direction <= 0 for count in table.raw):
            raise ValueError('FF 表にプロポの中立または後退側の CH2 値があります')
        level = _number(doc.get('brake_level'), 0, 1, 'brake_level')
        if round(level * 1000) < 2:
            raise ValueError('brake_level が Rev6 のパルス幅分解能を下回っています')
        hold = _number(doc.get('brake_hold_s'), .1, 2., 'brake_hold_s')
        return cls((tuple(math.radians(angle) for angle in angles), servo), (raw, esc), table, level, hold)

    def commands(self, steering_rad, speed_status, *, max_speed_mps):
        """同じ FF 表で生成された前進出力のみ許可。実測点間で線形補間する。"""
        if (not speed_status.valid or not speed_status.has_speeds or
                (speed_status.calibration_run_id, speed_status.proposal_sha256) !=
                (self.table.run_id, self.table.proposal_sha256)):
            raise ValueError('速度制御が無効か、起動時に採用した FF 表と異なります')
        limit = _number(max_speed_mps, 0., 1., '低速上限 [m/s]')
        if not 0 < speed_status.target_speed_mps <= limit:
            raise ValueError('前進目標速度が低速上限を超えたか停止目標です')
        expected_ff = self.table.interpolate(speed_status.target_speed_mps)
        if expected_ff is None or not math.isclose(expected_ff, speed_status.feedforward_raw, rel_tol=1e-10):
            raise ValueError('目標速度に対応する校正済み FF 値が一致しません')
        low, high = sorted((self.table.raw[0], self.table.raw[-1]))
        raw = _number(speed_status.calculated_raw, low, high, '速度制御の CH2 raw')
        bounds = self.table.forward_raw_bounds(limit)
        if bounds is None:
            raise ValueError('低速上限と FF 表の校正速度域が重なりません')
        raw = min(max(raw, bounds[0]), bounds[1])
        servo = _interpolate(self.steering, steering_rad, '等価前輪舵角 [rad]')
        esc = _interpolate(self.throttle, raw, '前進校正の CH2 raw')
        # Rev6 HOST: サーボは正で長いパルス、ESC の前進は正で短いパルス。
        return (servo - 1500.) / 500., (1500. - esc) / 500.


def load_calibration(path, feedforward_path):
    try:
        with Path(feedforward_path).open(encoding='utf-8') as f:
            table = FeedforwardTable.from_document(json.load(f))
        with Path(path).open(encoding='utf-8') as f:
            return AutoCalibration.from_document(yaml.safe_load(f), table)
    except (OSError, RecursionError, ValueError, yaml.YAMLError) as exc:
        raise ValueError(f'前進用の実車校正を読み込めません。{exc}') from exc

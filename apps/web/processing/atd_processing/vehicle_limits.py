"""手動走行の有効な保持区間から Raceline 制限と前進速度 → CH2 の候補を求める。"""
from collections import defaultdict
from itertools import groupby
from math import isfinite
from statistics import median
from typing import NamedTuple


# 推定手法 v1 の選択値。車両の仕様値・実測限界ではない。
LIMIT_FRACTION = 0.8
LOW_SPEED_MAX_MPS = 1.0
STEADY_SPEED_SPAN_MPS = 0.1
STEADY_SPEED_SPAN_FRACTION = 0.05
REQUIRED_COLUMNS = {
    'stamp', 'valid', 'motion_valid', 'sustained', 'episode_id', 'segment',
    'steering_raw', 'throttle_raw', 'vo_vx_mps', 'vo_ax_mps2', 'vo_yaw_rate_rps',
    'turn_regime', 'longitudinal_regime',
}


class Sample(NamedTuple):
    stamp: float
    vx: float
    ax: float
    yaw: float
    steering: int
    throttle: int


def _finite(value):
    if isinstance(value, bool):
        raise ValueError('有限の数値が必要です')
    number = float(value)
    if not isfinite(number):
        raise ValueError('有限の数値が必要です')
    return number


def _flag(value):
    if value is True or value == 'True':
        return True
    if value is False or value == 'False':
        return False
    raise ValueError('校正 CSV の有効性フラグが不正です')


def _read_rows(summary, rows, minimum_speed):
    total = valid = 0
    previous_valid_stamp = None
    for row in rows:
        if total == 0 and not REQUIRED_COLUMNS.issubset(row):
            raise ValueError('校正 CSV に候補算出用の列がありません')
        total += 1
        if not _flag(row['valid']):
            yield None, None
            continue
        valid += 1
        try:
            episode, segment = int(row['episode_id']), int(row['segment'])
            sample = Sample(_finite(row['stamp']), _finite(row['vo_vx_mps']), _finite(row['vo_ax_mps2']),
                            _finite(row['vo_yaw_rate_rps']), int(row['steering_raw']), int(row['throttle_raw']))
            if (not _flag(row['motion_valid']) or episode < 0 or segment < 0
                    or not 0 <= sample.steering <= 2047 or not 0 <= sample.throttle <= 2047
                    or previous_valid_stamp is not None and sample.stamp <= previous_valid_stamp):
                raise ValueError('運動量・時刻・区間・入力を確認してください')
            previous_valid_stamp = sample.stamp
            key = (episode, segment, row['turn_regime'], row['longitudinal_regime'],
                   _flag(row['sustained']), sample.vx >= minimum_speed)
        except (TypeError, ValueError) as error:
            raise ValueError(f'校正 CSV の有効試料が不正です（データ行 {total}）') from error
        yield key, sample
    if total != summary['timeseries_samples'] or valid != summary['eligible_samples']:
        raise ValueError('校正 CSV の記録件数と summary.json が一致しません')


def _summary_settings(summary):
    try:
        thresholds = summary['analysis']['thresholds']
        if summary['schema_version'] != 1:
            raise ValueError
        minimum_speed = _finite(thresholds['min_speed_mps'])
        duration = _finite(thresholds['min_regime_duration_s'])
        gap = _finite(thresholds['max_gap_s'])
        tolerance = thresholds['raw_stability_tolerance']
        bin_width = summary['analysis']['binning']['raw_bin_width']
        if (min(minimum_speed, duration, gap) <= 0 or type(tolerance) is not int or tolerance < 0
                or type(bin_width) is not int or not 1 <= bin_width <= 2048
                or type(summary['eligible_samples']) is not int or type(summary['timeseries_samples']) is not int
                or not 0 <= summary['eligible_samples'] <= summary['timeseries_samples']):
            raise ValueError
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('校正結果に時系列の件数・区間判定・入力ビン設定が必要です') from error
    return minimum_speed, duration, gap, tolerance, bin_width


def _measured_intervals(samples, duration, max_gap, tolerance):
    previous_maneuver = None
    maneuver_id = 0
    for key, group in groupby(samples, key=lambda pair: pair[0]):
        if key is None:
            continue
        episode, _, side, regime, sustained, forward = key
        if not sustained:
            continue
        block = [sample for _, sample in group]
        elapsed = block[-1].stamp - block[0].stamp
        if (elapsed + 1e-9 < duration or len(block) < 3
                or any(b.stamp - a.stamp > max_gap + 1e-9 for a, b in zip(block, block[1:]))):
            continue
        throttle = [s.throttle for s in block]
        steering = [s.steering for s in block]
        if max(throttle) - min(throttle) > tolerance or max(steering) - min(steering) > tolerance:
            continue
        # 別の持続した操作を観測して初めて別試行とする。CH の微調整、単発の
        # 閾値横断や記録欠落で同じ旋回・直進加速を複数回に数えない。
        maneuver = (forward, side, regime if side == 'straight' else None)
        if maneuver != previous_maneuver:
            maneuver_id += 1
            previous_maneuver = maneuver
        if not forward:
            continue
        speeds = [s.vx for s in block]
        evidence = dict(episode_id=episode, maneuver_id=maneuver_id, side=side, start_s=block[0].stamp,
                        end_s=block[-1].stamp, samples=len(block), speed_min_mps=min(speeds),
                        speed_max_mps=max(speeds), throttle_raw=median(throttle),
                        throttle_min_raw=min(throttle), throttle_max_raw=max(throttle))
        if side == 'straight':
            if regime in ('accelerating', 'decelerating'):
                key = 'accel' if regime == 'accelerating' else 'decel'
                value = median(s.ax for s in block) * (1 if key == 'accel' else -1)
                if value > 0:
                    yield key, dict(evidence, value=value)
            elif regime == 'steady':
                speed = median(speeds)
                if max(speeds) - min(speeds) <= max(STEADY_SPEED_SPAN_MPS, STEADY_SPEED_SPAN_FRACTION * speed):
                    yield 'feedforward', dict(evidence, value=speed)
        elif side in ('left', 'right') and regime == 'steady':
            if any((s.yaw if side == 'left' else -s.yaw) <= 0 for s in block):
                continue
            yield 'lateral_accel', dict(evidence, value=median(abs(s.vx * s.yaw) for s in block))
            if max(speeds) <= LOW_SPEED_MAX_MPS:
                yield 'curvature_limit', dict(evidence, value=median(abs(s.yaw / s.vx) for s in block))


def _limit(episodes, name, unit):
    # 同じ操作の連続部分では最大の保持区間中央値を1つだけ使う。2試行以上で
    # 到達した大きさ（2番目）を取り、旋回は左・右の小さい方を採る。
    representatives = {}
    for episode in episodes:
        maneuver = episode['maneuver_id']
        if maneuver not in representatives or representatives[maneuver]['value'] < episode['value']:
            representatives[maneuver] = episode
    sides = ('left', 'right') if name in ('curvature_limit', 'lateral_accel') else ('straight',)
    evidence, repeatable = [], []
    for side in sides:
        ranked = sorted((ep for ep in representatives.values() if ep['side'] == side),
                        key=lambda ep: -ep['value'])
        evidence.extend(ranked[:2])
        if len(ranked) >= 2:
            repeatable.append(ranked[1]['value'])
    value = LIMIT_FRACTION * min(repeatable) if len(repeatable) == len(sides) else None
    if sides == ('straight',):
        condition = '前進の直進加速' if name == 'accel' else '前進の直進減速（惰行・ESC 制動の区別なし）'
        reason = f'{condition}を2回以上観測した大きさの80%。'
        if value is None:
            reason = f'{condition}の保持区間が別の操作を挟んで2回必要です。'
    else:
        condition = '1.0 m/s 以下の前進・定常旋回' if name == 'curvature_limit' else '前進・定常旋回の |前後速度 × ヨーレート|'
        reason = f'{condition}で左右それぞれ2回観測した値の弱い側の80%。'
        if value is None:
            reason = f'{condition}が左・右それぞれ2回必要です。'
    return dict(value=value, unit=unit, reason=reason, episodes=len(representatives),
                samples=sum(ep['samples'] for ep in representatives.values()), evidence=evidence)


def _feedforward(episodes, bin_width):
    groups = defaultdict(list)
    for ep in episodes:
        groups[int(ep['throttle_raw'] // bin_width)].append(ep)
    points = []
    for _, group in sorted(groups.items()):
        # 重みは区間に均等。多数の連続フレームが一試行を過大に重くしないようにする。
        speeds = [ep['value'] for ep in group]
        points.append(dict(speed_mps=median(speeds), throttle_raw=round(median(ep['throttle_raw'] for ep in group)),
                           speed_min_mps=min(ep['speed_min_mps'] for ep in group),
                           speed_max_mps=max(ep['speed_max_mps'] for ep in group),
                           throttle_min_raw=min(ep['throttle_min_raw'] for ep in group),
                           throttle_max_raw=max(ep['throttle_max_raw'] for ep in group), episodes=len(group),
                           samples=sum(ep['samples'] for ep in group), evidence=group))
    direction = None
    reason = '異なる CH2 入力帯の前進・直進・定常保持が2点以上必要です。'
    if len(points) >= 2:
        adjacent = list(zip(points, points[1:]))
        raw_separated = all(a['throttle_max_raw'] < b['throttle_min_raw'] for a, b in adjacent)
        if raw_separated and all(a['speed_max_mps'] < b['speed_min_mps'] for a, b in adjacent):
            direction = 'increasing'
        elif raw_separated and all(a['speed_min_mps'] > b['speed_max_mps'] for a, b in adjacent):
            direction = 'decreasing'
        if direction is None:
            reason = '観測した CH2 値・定常速度の範囲が隣り合う入力帯で重なるか、対応が単調になっていません。'
        else:
            reason = '前進の直進定常区間。観測した速度範囲内を線形補間します。'
    points.sort(key=lambda point: point['speed_mps'])
    return dict(usable=direction is not None, reason=reason, points=points, direction=direction,
                input_unit='m/s', output_unit='sbus_raw_ch2', interpolation='linear', extrapolation='none',
                speed_min_mps=points[0]['speed_mps'] if direction else None,
                speed_max_mps=points[-1]['speed_mps'] if direction else None)


def derive_candidates(summary, rows):
    """校正 v1 の summary と同じ走行の CSV 行から制限候補・速度別入力表を作る。"""
    minimum_speed, duration, gap, tolerance, bin_width = _summary_settings(summary)
    intervals = defaultdict(list)
    for key, episode in _measured_intervals(_read_rows(summary, rows, minimum_speed), duration, gap, tolerance):
        intervals[key].append(episode)
    units = dict(curvature_limit='1/m', accel='m/s²', decel='m/s²', lateral_accel='m/s²')
    method = dict(version=1, limit_fraction=LIMIT_FRACTION, required_maneuvers_per_side=2,
                  low_speed_max_mps=LOW_SPEED_MAX_MPS, min_speed_mps=minimum_speed, min_duration_s=duration,
                  max_gap_s=gap, raw_stability_tolerance=tolerance, raw_bin_width=bin_width,
                  steady_speed_span_mps=STEADY_SPEED_SPAN_MPS, steady_speed_span_fraction=STEADY_SPEED_SPAN_FRACTION)
    return dict(schema_version=1, method=method, limits={key: _limit(intervals[key], key, unit) for key, unit in units.items()},
                feedforward=_feedforward(intervals['feedforward'], bin_width))

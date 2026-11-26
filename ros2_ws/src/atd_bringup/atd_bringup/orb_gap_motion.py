"""固定地図の姿勢出力が途切れた区間の終点誤差を、観測時刻に基づいて比較する。"""
from bisect import bisect_right
from itertools import pairwise
import math
from statistics import fmean, median

from .replay_result import GAP_NS

HISTORY_NS = 200_000_000
MAX_GYRO_HOLD_NS = 20_000_000
METHODS = ('hold', 'straight', 'turn', 'gyro')


def _sinc(angle):
    return math.sin(angle) / angle if angle else 1.


def advance_planar(pose, speed, seconds, yaw_change):
    """平面の定速円弧。旋回率ゼロのときも同じ式で直進する。"""
    x, y, yaw = pose
    distance = speed * seconds * _sinc(yaw_change * .5)
    mid_yaw = yaw + yaw_change * .5
    return (x + distance * math.cos(mid_yaw), y + distance * math.sin(mid_yaw),
            math.remainder(yaw + yaw_change, math.tau))


def _pose(row, prefix):
    pose = (row[f'{prefix}_x_m'], row[f'{prefix}_y_m'], row[f'{prefix}_yaw_rad'])
    if not all(map(math.isfinite, pose)):
        raise ValueError('比較対象の姿勢に非有限値があります')
    return pose


def estimate_motion(stamps, poses):
    """前進速度とヨー角速度を直前の位置・方位履歴の最小二乗勾配から得る。"""
    distances, angles = [0.], [0.]
    for (x0, y0, yaw0), (x1, y1, yaw1) in pairwise(poses):
        angle = math.remainder(yaw1 - yaw0, math.tau)
        yaw_mid = yaw0 + angle * .5
        arc = ((x1 - x0) * math.cos(yaw_mid) + (y1 - y0) * math.sin(yaw_mid)) / _sinc(angle * .5)
        distances.append(distances[-1] + arc)
        angles.append(angles[-1] + angle)
    times = [(s - stamps[0]) * 1e-9 for s in stamps]
    mean_t = fmean(times)
    offsets = [t - mean_t for t in times]
    denominator = sum(t*t for t in offsets)
    return tuple(sum(t*v for t, v in zip(offsets, values)) / denominator for values in (distances, angles))


def _gyro_endpoint(start, end, pose, speed, stamps, rates):
    if not stamps or start < stamps[0] or end > stamps[-1]:
        return None
    i = bisect_right(stamps, start) - 1
    t = start
    while t < end:
        stop = min(stamps[i+1], end)
        if stop - stamps[i] > MAX_GYRO_HOLD_NS:
            return None
        seconds = (stop - t) * 1e-9
        pose = advance_planar(pose, speed, seconds, rates[i] * seconds)
        i += 1
        t = stop
    return pose


def _error(estimate, truth):
    return dict(position_error_m=math.hypot(estimate[0] - truth[0], estimate[1] - truth[1]),
                abs_heading_error_deg=abs(math.degrees(math.remainder(estimate[2] - truth[2], math.tau))))


def _paired_metrics(rows):
    if not rows:
        return dict(count=0, median_position_error_m=None, rms_position_error_m=None,
                    median_abs_heading_error_deg=None)
    position = [r['position_error_m'] for r in rows]
    return dict(count=len(rows), median_position_error_m=median(position),
                rms_position_error_m=math.sqrt(fmean(x*x for x in position)),
                median_abs_heading_error_deg=median(r['abs_heading_error_deg'] for r in rows))


def evaluate_gaps(trajectory_stamps, epochs, samples, gyro_stamps, yaw_rates):
    """trajectory は全姿勢の画像時刻。samples は正解と照合できた時刻だけを含む。"""
    for stamps in (trajectory_stamps, gyro_stamps):
        if any(type(t) is not int or t < 0 for t in stamps) or any(b <= a for a, b in pairwise(stamps)):
            raise ValueError('観測時刻には単調増加する非負の整数 ns が必要です')
    if len(gyro_stamps) != len(yaw_rates) or not all(map(math.isfinite, yaw_rates)):
        raise ValueError('ジャイロ時刻と有限の角速度が対応していません')

    gaps = []
    for i, (start, end) in enumerate(pairwise(trajectory_stamps)):
        if end - start <= GAP_NS:
            continue
        duration = (end - start) * 1e-9
        gap = dict(start_stamp_ns=start, end_stamp_ns=end, duration_s=duration, errors={})
        gaps.append(gap)
        if start not in samples or end not in samples:
            gap['status'] = 'missing_gt_sample'
            continue
        pose = _pose(samples[start], 'est')
        truth = _pose(samples[end], 'gt')
        gap['errors']['hold'] = _error(pose, truth)

        first = i
        epoch = epochs.get(start)
        while first > 0 and epoch is not None:
            earlier = trajectory_stamps[first-1]
            if start - earlier > HISTORY_NS or earlier not in samples or epochs.get(earlier) != epoch:
                break
            first -= 1
        gap['history_count'] = i - first + 1
        if first == i:
            gap['status'] = 'insufficient_history'
            continue
        history = trajectory_stamps[first:i+1]
        speed, turn = estimate_motion(history, [_pose(samples[t], 'est') for t in history])
        gap.update(forward_mps=speed, yaw_rate_radps=turn)
        gap['errors']['straight'] = _error(advance_planar(pose, speed, duration, 0.), truth)
        gap['errors']['turn'] = _error(advance_planar(pose, speed, duration, turn * duration), truth)
        predicted = _gyro_endpoint(start, end, pose, speed, gyro_stamps, yaw_rates)
        if predicted is None:
            gap['status'] = 'insufficient_gyro_coverage'
            continue
        gap['errors']['gyro'] = _error(predicted, truth)
        gap['status'] = 'evaluated'

    paired = [g['errors'] for g in gaps if g['status'] == 'evaluated']
    return dict(gap_threshold_s=GAP_NS / 1e9, history_window_s=HISTORY_NS / 1e9,
                max_gyro_hold_s=MAX_GYRO_HOLD_NS / 1e9, gap_count=len(gaps), paired_count=len(paired),
                longest_gap=max(gaps, key=lambda g: g['duration_s'], default=None),
                paired_metrics={name: _paired_metrics([r[name] for r in paired]) for name in METHODS}, gaps=gaps)

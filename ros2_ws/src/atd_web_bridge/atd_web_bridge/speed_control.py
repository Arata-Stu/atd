"""同じ planner 観測に対応する速度、計算 CH2、手動入力とその経路を公開する。"""
from collections import deque
import math
import re

from .planning import source_status


CALCULATED = ('feedforward_raw', 'p_raw', 'i_raw', 'd_raw', 'calculated_raw')
SPEEDS = ('target_speed_mps', 'estimated_speed_mps', 'speed_error_mps')
LINEAGE = ('map_id', 'map_sha256', 'hdmap_sha256', 'session_id', 'epoch', 'line')
MANUAL_SOURCES = ('propo', 'host_joy', 'host', 'standby', 'fault', 'unknown')


def _empty(status, reason):
    return dict(status, **dict.fromkeys((*SPEEDS, *CALCULATED)), valid=False, reason=reason,
                saturated=False, propo_raw=None, propo_age_s=None, propo_reason=reason, manual_source='unknown',
                joy_throttle=None, joy_age_s=None, joy_enable_requested=None, joy_reason=reason, remaining_s=0.)


def observation(message):
    status = source_status(message)
    status.update(calibration_run_id=message.calibration_run_id, proposal_sha256=message.proposal_sha256)
    for name in ('kp', 'ki', 'kd'):
        gain = getattr(message, name)
        status[name] = gain if math.isfinite(gain) and gain >= 0 else None
    sample = _empty(status, message.reason)
    if not message.has_speeds:
        return sample
    if (message.header.frame_id != 'map' or not message.map_id or not message.session_id
            or message.line not in ('raceline', 'centerline', 'custom') or not status['validity_s']
            or status['validity_s'] <= 0 or not all(re.fullmatch('[0-9a-f]{64}', status[k])
                                                     for k in ('map_sha256', 'hdmap_sha256'))
            or not all(math.isfinite(getattr(message, k)) for k in SPEEDS) or message.target_speed_mps < 0):
        return _empty(status, '速度観測の入力・時刻または地図情報が不正です')
    sample.update({k: getattr(message, k) for k in SPEEDS})
    if message.has_propo and 0 <= message.propo_raw <= 2047 and 0 <= message.propo_age_s <= .05:
        sample.update(propo_raw=message.propo_raw, propo_age_s=message.propo_age_s, propo_reason='')
    else:
        sample['propo_reason'] = message.propo_reason or '同時刻のプロポ手動入力がありません'
    source = message.manual_source
    sample['manual_source'] = source if source in MANUAL_SOURCES else 'unknown'
    if message.has_joy and -1. <= message.joy_throttle <= 1. and 0. <= message.joy_age_s <= .25:
        sample.update(joy_throttle=message.joy_throttle, joy_age_s=message.joy_age_s,
                      joy_enable_requested=message.joy_enable_requested, joy_reason='')
    else:
        sample['joy_reason'] = message.joy_reason or '同時刻の有効な DualSense 入力がありません'
        if source == 'host_joy':
            sample['manual_source'] = 'host'
    if message.valid:
        if (all(math.isfinite(getattr(message, k)) for k in CALCULATED)
                and 0 <= message.feedforward_raw <= 2047 and 0 <= message.calculated_raw <= 2047
                and status['calibration_run_id'] and re.fullmatch('[0-9a-f]{64}', status['proposal_sha256'])
                and all(status[k] is not None for k in ('kp', 'ki', 'kd'))):
            sample.update({k: getattr(message, k) for k in CALCULATED},
                          valid=True, saturated=message.saturated)
        else:
            sample['reason'] = '速度制御の計算結果または校正表の出典が不正です'
    return sample


def live_speed(cache, publishers, plan, ros_now):
    sample = cache.get('speed_control')
    if sample is None:
        return None
    if publishers != 1:
        return _empty(dict(sample, stale=True), '速度制御の配信元が一つに定まりません')
    if sample['target_speed_mps'] is None:
        return sample
    if not plan or not plan['valid'] or any(sample[k] != plan[k] for k in LINEAGE):
        return _empty(sample, '現在の地図・ライン・追跡区間に対応する速度観測を待っています')
    age = ros_now - sample['stamp']
    remaining = min(sample['validity_s'], cache.fresh_s) - max(age, sample['age_s'])
    if sample['stale'] or age < -.05 or remaining <= 0:
        return _empty(sample, '速度とスロットルの観測期限が切れました')
    return dict(sample, remaining_s=min(remaining, plan['remaining_s']))


class SpeedWindow:
    """Notebook の HTTP ポーリングの間に受信した観測も、元時刻で比較する。"""

    def __init__(self, cache):
        self.cache = cache
        self.source = None
        self.points = deque(maxlen=1024)

    def receive(self, message):
        sample = observation(message)
        source = tuple(sample[k] for k in (*LINEAGE, 'proposal_sha256', 'kp', 'ki', 'kd'))
        stamp = sample['stamp']
        with self.cache.lock:
            self.cache.put('speed_control', sample)
            if source != self.source or (self.points and stamp < self.points[-1]['time_s']):
                self.points.clear()
            self.source = source
            if sample['target_speed_mps'] is None:
                self.points.clear()
                return
            while self.points and stamp - self.points[0]['time_s'] > 5.:
                self.points.popleft()
            if self.points and stamp == self.points[-1]['time_s']:
                self.points.pop()
            self.points.append(dict(time_s=stamp, **{k: sample[k] for k in
                                    ('target_speed_mps', 'estimated_speed_mps', 'calculated_raw',
                                     'propo_raw', 'joy_throttle', 'manual_source')}))

    def current(self, publishers, plan, ros_now):
        with self.cache.lock:
            sample = live_speed(self.cache, publishers, plan, ros_now)
            if sample is None:
                return None
            if sample['remaining_s'] <= 0:
                self.points.clear()
            return dict(sample, history=list(self.points))

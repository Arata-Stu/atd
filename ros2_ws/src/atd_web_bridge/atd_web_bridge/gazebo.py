"""Gazebo の制御出力をシミュレーション時刻で保持する。"""
from collections import deque
import math
import re
from threading import Lock
import time


def ground_truth_sample(data, stream_id, gid):
    if (not gid or not stream_id or not isinstance(data, dict) or data.get('stream_id') != stream_id
            or data.get('frame') != 'map' or not isinstance(data.get('map_id'), str) or not data['map_id']):
        return None
    if not all(isinstance(data.get(k), str) and re.fullmatch('[0-9a-f]{64}', data[k])
               for k in ('map_sha256', 'hdmap_sha256')):
        return None
    dims = data.get('footprint')
    keys = ('vehicle_length', 'vehicle_width', 'rear_axle_to_rear', 'margin')
    if not isinstance(dims, dict):
        return None
    values = [data.get(k) for k in ('x', 'y', 'yaw', 'stamp_s')] + [dims.get(k) for k in keys]
    if not all(type(v) in (int, float) and -math.inf < v < math.inf for v in values):
        return None
    if not (0 < data['stamp_s'] < 2**31 and 0 <= dims['rear_axle_to_rear'] < dims['vehicle_length']
            and dims['vehicle_width'] > 0 and dims['margin'] >= 0):
        return None
    sample = {k: data[k] for k in ('frame', 'map_id', 'map_sha256', 'hdmap_sha256', 'x', 'y', 'yaw', 'stamp_s')}
    return dict(sample, footprint={k: dims[k] for k in keys}, publisher_gid=gid)


def vehicle_hold_sample(data, gid):
    if not gid or not isinstance(data, dict):
        return None
    held, stamp, speed = data.get('held'), data.get('stamp_ns'), data.get('speed_mps')
    if (type(held) is not bool or type(stamp) is not int or not 0 <= stamp < 2**63
            or type(speed) not in (int, float) or not 0 <= speed < math.inf or held and speed != 0.):
        return None
    return dict(held=held, stamp_s=stamp / 1e9, speed_mps=speed, publisher_gid=gid)


def live_gazebo_sample(cache, key, now_s, gid):
    sample = cache.get(key)
    if not sample or not gid or sample.pop('publisher_gid') != gid:
        return None
    sim_age = now_s - sample['stamp_s']
    remaining = cache.fresh_s - max(sim_age, sample.pop('age_s'))
    if sim_age < -.05 or remaining <= 0. or sample.pop('stale'):
        return None
    return dict(sample, remaining_s=remaining)


class GazeboDriveWindow:
    def __init__(self, freshness_s):
        self.freshness_s = freshness_s
        self.lock = Lock()
        self.history = deque(maxlen=1500)
        self.gid = None
        self.received = 0.

    def receive(self, data, gid):
        if not isinstance(data, dict) or not gid or type(data.get('stamp_ns')) is not int or data['stamp_ns'] < 0:
            return
        if data.get('state') not in ('waiting', 'waiting_observation', 'driving', 'paused', 'stopped'):
            return
        if not all(isinstance(data.get(key), str) for key in ('line', 'map_id', 'reason')):
            return
        row = {key: data[key] for key in ('line', 'map_id', 'state', 'reason')}
        stamp = data['stamp_ns'] / 1e9
        row['stamp_s'] = stamp
        for key, source in [('target_mps', 'target_speed_mps'), ('estimated_mps', 'estimated_speed_mps'),
                            ('steering_rad', 'steering_rad'), ('throttle', 'throttle')]:
            value = data.get(source) if row['state'] == 'driving' else None
            if row['state'] == 'driving' and (type(value) not in (int, float) or not math.isfinite(value)):
                return
            row[key] = value
        with self.lock:
            if gid != self.gid or self.history and stamp < self.history[-1]['stamp_s']:
                self.history.clear()
            if self.history and stamp == self.history[-1]['stamp_s']:
                self.history.pop()
            while self.history and stamp - self.history[0]['stamp_s'] > 15.:
                self.history.popleft()
            self.gid, self.received = gid, time.monotonic()
            self.history.append(row)

    def snapshot(self, now_s, gid):
        with self.lock:
            rows = ([row for row in self.history if -.05 <= now_s - row['stamp_s'] <= 15.]
                    if gid and gid == self.gid else [])
            current = rows[-1] if rows and now_s - rows[-1]['stamp_s'] <= self.freshness_s and (
                time.monotonic() - self.received <= self.freshness_s) else None
            return dict(now_s=now_s, current=current, history=rows)

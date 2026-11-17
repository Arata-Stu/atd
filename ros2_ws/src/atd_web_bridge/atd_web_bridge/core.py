"""ROS-independent cache and physical-profile persistence."""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time

from atd_joy_tools import profile as profiles


class Cache:
    def __init__(self, fresh_s=2.5):
        self.fresh_s = fresh_s
        self.lock = threading.RLock()
        self.samples = {}

    def put(self, key, value):
        with self.lock:
            self.samples[key] = (time.monotonic(), copy.deepcopy(value))

    def invalidate(self, *keys):
        with self.lock:
            for key in keys:
                self.samples.pop(key, None)

    def get(self, key):
        with self.lock:
            found = self.samples.get(key)
            if found is None:
                return None
            received, value = found
            age = max(0.0, time.monotonic() - received)
            return {**copy.deepcopy(value), 'age_s': age, 'stale': age > self.fresh_s}

    def input_status(self):
        raw, normalized, status = (self.get(k) for k in ('raw', 'normalized', 'status'))
        fresh = bool(status and not status['stale'])
        # Raw and status must describe the same physical device generation.
        generation_ok = bool(raw and status and raw.get('frame') == 'atd_joy_raw_' + str(status.get('generation')))
        return dict(raw=raw, normalized=normalized, status=status,
                    device_connected=bool(fresh and status.get('connected') and raw and not raw['stale'] and generation_ok),
                    ready=bool(fresh and status.get('ready') and generation_ok and raw and not raw['stale'] and normalized and not normalized['stale']),
                    configuring=status.get('configuring') if fresh else None,
                    availability={k: 'missing' if v is None else 'stale' if v['stale'] else 'fresh'
                                  for k, v in (('raw', raw), ('normalized', normalized), ('status', status))})


def revision(profile):
    return hashlib.sha256(json.dumps(profile, sort_keys=True, allow_nan=False).encode()).hexdigest()


class ProfileStore:
    def __init__(self, path):
        self.path = Path(path).expanduser()
        self.lock = threading.RLock()

    def read(self):
        with self.lock:
            saved = self.path.exists()
            value = profiles.load(self.path) if saved else profiles.default_profile()
            return {'profile': value, 'revision': revision(value), 'saved': saved,
                    'path': str(self.path), 'axes': list(profiles.AXES), 'buttons': list(profiles.BUTTONS)}

    def write(self, value, expected, state):
        with self.lock:
            status = state.get('status')
            if not status or status.get('stale') or status.get('configuring') is not True:
                raise ValueError('新鮮な Joy 設定モード状態が必要です。先に設定開始を実行してください')
            if Path(status.get('profile_path', '')).expanduser().resolve() != self.path.resolve():
                raise ValueError('Joy node と bridge の profile_path が一致しません')
            current = self.read()
            if expected != current['revision']:
                raise ValueError('入力配置が変更されています。再取得してください')
            profiles.save(self.path, value)
            return {**self.read(), 'applied': False, 'message': '保存しました。再読込と入力出力再開は個別に実行してください'}


def validate_range(value, descriptor):
    """Validate numeric ROS descriptor ranges before an atomic service request."""
    for key, integral in (('integer_range', True), ('floating_point_range', False)):
        ranges = descriptor.get(key, [])
        if not ranges:
            continue
        if isinstance(value, bool) or (type(value) is not int if integral else type(value) not in (int, float)):
            raise ValueError('パラメータの数値型が一致しません')
        match = False
        for item in ranges:
            low, high, step = item['from_value'], item['to_value'], item['step']
            if low <= value <= high and (not step or abs((value-low)/step - round((value-low)/step)) < 1e-7):
                match = True
        if not match:
            raise ValueError('パラメータの範囲または step に一致しません')


def prepare_parameter_value(value, descriptor):
    """Keep the current ROS type when JSON drops the decimal notation."""
    if descriptor['type'] == 3 and type(value) is int:
        value = float(value)
    if descriptor['type'] == 8 and isinstance(value, list) and all(type(v) in (int, float) for v in value):
        value = [float(v) for v in value]
    numbers = value if isinstance(value, list) else [value]
    if any(type(v) in (int, float) and not math.isfinite(v) for v in numbers):
        raise ValueError('有限値を指定してください')
    if descriptor['type'] in (2, 3):
        validate_range(value, descriptor)
    return value

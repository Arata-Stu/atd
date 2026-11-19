"""自動走行の実送信・車体速度・基板報告を、各信号の元時刻で Notebook に公開する。"""
from collections import deque
import sys
from threading import RLock

from .drive_monitor import CDC0_TIMEOUT_S, ch3_position

NS = 1_000_000_000
STATUS_TTL = .5
HISTORY_S = 15.
STATES = ('driving', 'braking', 'latched_stop', 'arming', 'propo', 'standby', 'disconnected')


def _integer(value, lo, hi):
    return type(value) is int and lo <= value <= hi


def _number(value, lo, hi):
    return type(value) in (int, float) and lo <= value <= hi


def _name(value):
    return isinstance(value, str) and 0 < len(value) <= 128


def _stamp(doc, now, ttl):
    stamp = doc.get('stamp_ns') if isinstance(doc, dict) else None
    return stamp if _integer(stamp, 1, now) and now - stamp < ttl * NS else None


def _pico(doc, now):
    stamp = _stamp(doc, now, CDC0_TIMEOUT_S)
    if not stamp or not _name(doc.get('serial')):
        return None
    pulse, path, selector, faults = (doc.get(k) for k in ('rx_mode_us', 'active_path', 'selector', 'fault_bits'))
    if (not _integer(pulse, 0, 2000) or 0 < pulse < 1000 or not _integer(path, 0, 3)
            or not _integer(selector, 0, 1) or not _integer(faults, 0, 0xffffffff)
            or not all(_integer(doc.get(k), 1000, 2000) for k in ('servo_us', 'esc_us'))):
        return None
    enabled = not bool(faults & 1)
    return dict(stamp_s=stamp / NS, serial=doc['serial'], ch3=ch3_position(pulse), rx_mode_us=pulse or None,
                active_path=path, selector=selector, fault_bits=faults, pwm_enabled=enabled,
                servo_us=doc['servo_us'] if enabled else None, esc_us=doc['esc_us'] if enabled else None)


def _command(doc, now):
    stamp = _stamp(doc, now, CDC0_TIMEOUT_S)
    if not stamp or not _integer(doc.get('sequence'), 0, 0xffffffff):
        return None
    kind = doc.get('kind')
    steer, throttle = doc.get('steering'), doc.get('throttle')
    servo, esc = doc.get('servo_us'), doc.get('esc_us')
    if (kind not in ('drive', 'arm', 'disable', 'brake') or not _number(steer, -1, 1)
            or not _number(throttle, -1, 1) or kind == 'drive' and throttle < 0
            or kind == 'brake' and throttle >= 0 or kind in ('arm', 'disable') and (steer or throttle)):
        return None
    if kind == 'disable':
        if servo is not None or esc is not None:
            return None
    elif not all(_integer(v, 1000, 2000) for v in (servo, esc)):
        return None
    return dict(stamp_s=stamp / NS, kind=kind, steering=steer, throttle=throttle, servo_us=servo, esc_us=esc)


def _speed(doc, now):
    if not isinstance(doc, dict) or not _number(doc.get('validity_s'), 1e-9, .25):
        return None
    stamp = _stamp(doc, now, doc['validity_s'])
    if (not stamp or not _name(doc.get('tracking_session_id')) or not _integer(doc.get('epoch'), 0, 0xffffffff)
            or not _number(doc.get('target_mps'), 0, sys.float_info.max)
            or not _number(doc.get('estimated_mps'), -sys.float_info.max, sys.float_info.max)):
        return None
    return dict(stamp_s=stamp / NS, **{k: doc[k] for k in ('tracking_session_id', 'epoch', 'validity_s',
                                                           'target_mps', 'estimated_mps')})


def _source(board, state, command):
    if board is None:
        return 'unknown'
    path = board['active_path']
    if board['fault_bits'] or path == 3:
        return 'fault'
    if path == 1 and board['selector'] == 0:
        return 'propo'
    if path == 2 and board['selector'] == 1:
        active_kind = {'braking': 'brake', 'driving': 'drive'}.get(state)
        return 'autonomous' if command and command['kind'] == active_kind else 'host'
    return 'standby' if path == 0 else 'unknown'


def _frame(data, wall_ns):
    stamp = _stamp(data, wall_ns, STATUS_TTL)
    if not stamp:
        return None
    state = data.get('state')
    if (state not in STATES or type(data.get('reason')) is not str or not _name(data.get('session_id'))
            or not _name(data.get('map_id')) or data.get('line') not in ('centerline', 'raceline', 'custom')
            or not _number(data.get('max_speed_mps'), 0., 1.) or data['max_speed_mps'] == 0
            or data.get('active') is not (state == 'driving') or data.get('braking') is not (state == 'braking')):
        return None
    board = _pico(data.get('pico'), stamp)
    command = _command(data.get('command'), stamp)
    return dict(stamp_s=stamp / NS, state=state, reason=data['reason'][:192], map_id=data['map_id'],
                line=data['line'], max_speed_mps=data['max_speed_mps'], board=board, source=_source(board, state, command),
                command=command, speed=_speed(data.get('speed'), stamp))


class AutoDriveWindow:
    """20 Hz の基板報告、実 USB 送信、短い停止遷移を HTTP の読取間隔から独立して保持する。"""
    def __init__(self):
        self.lock = RLock()
        self.history = deque(maxlen=800)
        self.clear()

    def clear(self):
        with self.lock:
            self.history.clear()
            self.key = self.last_stop = None
            self.stamp_ns = self.steady_ns = self.wall_ns = 0
            self.expiry = {}

    def receive(self, data, gid, wall_ns, steady_ns):
        frame = _frame(data, wall_ns)
        with self.lock:
            if not gid or frame is None:
                self.clear()
                return
            key = (gid, data['session_id'], data['map_id'], data['line'])
            stamp = data['stamp_ns']
            if key != self.key or wall_ns < self.wall_ns:
                self.clear()
            if stamp <= self.stamp_ns:
                return
            stop = data.get('last_stop')
            if (isinstance(stop, dict) and _integer(stop.get('stamp_ns'), 1, stamp)
                    and isinstance(stop.get('reason'), str) and stop['reason']):
                self.last_stop = dict(stamp_s=stop['stamp_ns'] / NS, reason=stop['reason'][:192])
            self.key = key
            self.stamp_ns, self.wall_ns, self.steady_ns = stamp, wall_ns, steady_ns
            self.expiry = {
                field: data[source]['stamp_ns'] + round(frame[field].get('validity_s', CDC0_TIMEOUT_S) * NS)
                for field, source in (('board', 'pico'), ('command', 'command'), ('speed', 'speed')) if frame[field]}
            while self.history and frame['stamp_s'] - self.history[0]['stamp_s'] > HISTORY_S:
                self.history.popleft()
            self.history.append(frame)

    def current(self, gid, wall_ns, steady_ns, publishers=None):
        count = int(bool(gid)) if publishers is None else publishers
        with self.lock:
            now = wall_ns / NS
            result = dict(publishers=count, session_id=None, now_s=now, current=None, history=[], last_stop=None, issue='')
            if not gid or count != 1 or not self.key or gid != self.key[0] or wall_ns < self.wall_ns:
                self.clear()
                if count == 0:
                    result['issue'] = '自動走行ノードは未起動です'
                elif count > 1:
                    result['issue'] = '自動走行の配信元が複数あります'
                else:
                    result['issue'] = 'Jetson の自動走行ノードの状態を待機中'
                return result
            # ROS/実時間が止まっても、受信からの単調時計を使って現在値を失効させる。
            now_ns = max(wall_ns, self.wall_ns + steady_ns - self.steady_ns)
            now = now_ns / NS
            result.update(session_id=self.key[1], last_stop=self.last_stop, now_s=now,
                          history=[row for row in self.history if now - row['stamp_s'] <= HISTORY_S])
            if now_ns - self.stamp_ns >= STATUS_TTL * NS:
                return dict(result, issue='自動走行状態の更新が途絶えています')
            current = self.history[-1].copy()
            for field, deadline in self.expiry.items():
                if now_ns >= deadline:
                    current[field] = None
            current['source'] = _source(current['board'], current['state'], current['command'])
            result['current'] = current
            return result

"""自動走行の送信・基板報告・速度観測をそれぞれの時刻で公開する。"""
import math
import re

# 画像90 Hzで指令の最長有効期間250 msを照合できる件数。
MAX_PENDING_FRAMES = 32


class DriveSpeeds:
    """同じ追跡観測の速度対を表示する。制御器が出力できない停止目標も表示する。"""

    def __init__(self, map_id, line, max_age):
        self.map_id, self.line, self.max_age = map_id, line, max_age
        self.floor = 0
        self.identity = None
        self.rows = {'localization': {}, 'speed': {}}

    def clear(self, stamp):
        self.floor = max(self.floor, stamp)
        for rows in self.rows.values():
            rows.clear()

    def receive(self, kind, msg, wall, steady):
        if kind not in self.rows:
            return
        stamp = msg.header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        valid_stamp = ns > 0 and 0 <= stamp.nanosec < 1_000_000_000 and 0 <= wall - ns < self.max_age
        if (not valid_stamp or msg.header.frame_id != 'map' or msg.map_id != self.map_id or not msg.session_id
                or not all(re.fullmatch('[0-9a-f]{64}', v) for v in (msg.map_sha256, msg.hdmap_sha256))):
            self.clear(wall)
            return
        identity = (msg.map_sha256, msg.hdmap_sha256, msg.session_id, msg.epoch)
        if identity != self.identity:
            if self.identity is not None:
                self.clear(ns - 1)
            self.identity = identity
        if ns <= self.floor:
            return
        if kind == 'localization':
            if msg.state != 'tracking' or not math.isfinite(msg.speed_mps):
                self.clear(ns)
                return
            values, ttl = (msg.speed_mps,), self.max_age
        else:
            if (msg.line != self.line or not msg.has_speeds or not math.isfinite(msg.target_speed_mps)
                    or msg.target_speed_mps < 0 or not math.isfinite(msg.estimated_speed_mps)
                    or not math.isfinite(msg.validity_s) or msg.validity_s <= 0):
                self.clear(ns)
                return
            values = (msg.estimated_speed_mps, msg.target_speed_mps)
            ttl = min(self.max_age, round(min(1., msg.validity_s) * 1e9))
        rows = self.rows[kind]
        if rows and ns <= max(rows):
            if ns != max(rows) or rows[ns][:2] != (values, ttl):
                self.clear(max(rows))
            return
        if len(rows) >= MAX_PENDING_FRAMES:
            del rows[min(rows)]
        rows[ns] = (values, ttl, steady + ttl - (wall - ns))

    def latest(self, wall, steady):
        loc, speed = self.rows['localization'], self.rows['speed']
        stamp = max(loc.keys() & speed.keys(), default=0)
        if not stamp:
            return None
        (estimated,), loc_ttl, loc_end = loc[stamp]
        (reference, target), speed_ttl, speed_end = speed[stamp]
        ttl = min(loc_ttl, speed_ttl)
        if estimated != reference or not 0 <= wall - stamp < ttl or steady >= min(loc_end, speed_end):
            return None
        return dict(stamp_ns=stamp, target_mps=target, estimated_mps=estimated, validity_s=ttl * 1e-9,
                    tracking_session_id=self.identity[2], epoch=self.identity[3])


def snapshot(gate, board, session_id, reason, wall_ns, steady_ns):
    report = board.latest_report() if board else None
    pico = None
    if report:
        stamp, sample = report
        pico = {field: sample[field] for field in ('sequence', 'selector', 'active_path', 'rx_mode_us',
                                                    'servo_us', 'esc_us', 'fault_bits')}
        pico.update(stamp_ns=stamp, serial=board.serial)
    braking = bool(board and board.braking)
    if braking:
        state = 'braking'
    elif gate.active:
        state = 'driving'
    elif gate.fault:
        state = 'latched_stop'
    elif gate.requested:
        state = 'arming'
    elif pico and pico['active_path'] == 1:
        state = 'propo'
    else:
        state = 'standby' if pico else 'disconnected'
    return dict(state=state, reason=reason, active=gate.active, braking=braking, pico=pico, map_id=gate.map_id,
                line=gate.line, max_speed_mps=gate.max_speed, session_id=session_id, stamp_ns=wall_ns,
                command=board.latest_command(wall_ns, steady_ns) if board else None,
                speed=gate.speeds.latest(wall_ns, steady_ns), last_stop=gate.last_stop)

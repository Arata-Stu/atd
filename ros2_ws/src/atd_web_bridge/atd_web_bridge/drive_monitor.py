"""Jetson が受けた操作と Pico の出力報告を、独立した元時刻・配信元で照合する。"""
import math

JOY_TIMEOUT_S = .25
CDC0_TIMEOUT_S = .15
CDC1_DEFAULT_TIMEOUT_S = .3


def publisher_gid(info):
    gid = (info.get('publisher_gid') or {}).get('data', b'')
    return bytes(gid).hex() if gid and any(gid) else None


def _sample(message, info):
    stamp = message.header.stamp
    seconds = stamp.sec + stamp.nanosec / 1e9 if stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000 else 0.
    return dict(gid=publisher_gid(info), stamp=seconds)


def joy_sample(msg, info):
    data = _sample(msg, info)
    if (msg.header.frame_id != 'atd_gamepad' or msg.source != 'joy' or not msg.valid
            or not all(math.isfinite(v) and -1 <= v <= 1 for v in (msg.steering, msg.throttle))):
        return dict(data, reason='有効な DualSense の操作入力を待機中')
    return dict(data, reason='', steering=msg.steering, throttle=msg.throttle,
                enable_requested=msg.enable_requested)


def _path(path, faults, joy_active=False):
    if faults or path == 3:
        return 'fault'
    return {0: 'standby', 1: 'propo', 2: 'dualsense' if joy_active else 'host'}.get(path, 'unknown')


def ch3_position(pulse):
    if not pulse:
        return 'unknown'
    if pulse <= 1300:
        return 'propo'
    return 'jetson' if pulse >= 1700 else 'middle'


def feedback_sample(msg, info):
    data = dict(_sample(msg, info), serial=msg.serial)
    if not msg.has_report:
        return dict(data, reason=msg.reason or 'Pico CDC0 の状態報告を待機中')
    if (msg.header.frame_id != 'pico' or not msg.session_id or not msg.serial
            or msg.selector not in (0, 1) or msg.active_path not in (0, 1, 2, 3)
            or not (msg.rx_mode_us == 0 or 1000 <= msg.rx_mode_us <= 2000)
            or not all(1000 <= v <= 2000 for v in (msg.servo_us, msg.esc_us))):
        return dict(data, reason='Pico CDC0 の状態報告が不正です')
    pulse = msg.rx_mode_us or None
    ch3 = ch3_position(pulse)
    enabled = not bool(msg.fault_bits & 1)
    joy_active = msg.joy_active and msg.selector == 1 and ch3 == 'jetson'
    return dict(data, source='cdc0', ch3=ch3, rx_mode_us=pulse, ch3_raw=None,
                active_path=msg.active_path, control_source=_path(msg.active_path, msg.fault_bits, joy_active),
                fault_bits=msg.fault_bits, pwm_enabled=enabled, servo_us=msg.servo_us if enabled else None,
                esc_us=msg.esc_us if enabled else None, reason='', driver_reason=msg.reason)


def receive_feedback(cache, msg, info):
    sample = feedback_sample(msg, info)
    with cache.lock:
        cache.put('manual_feedback', sample)
        if not sample['reason'] and sample['stamp'] > 0 and sample['gid']:
            # 次の切断通知は stamp=0。直前の基板報告の時刻は別に保持する。
            cache.put('last_manual_report', {key: sample[key] for key in ('stamp', 'gid', 'serial')})


def _current(sample, gid, now, limit, name):
    if not gid or not sample or sample['gid'] != gid:
        return None, f'{name} の配信元・入力を待機中'
    if sample.get('reason'):
        return None, sample['reason']
    stamp = sample['stamp']
    if not math.isfinite(stamp) or stamp <= 0 or stamp > now:
        return None, f'{name} の元時刻が不正です'
    age = max(now - stamp, sample['age_s'])
    if sample['stale'] or age >= limit:
        return None, f'{name} の観測期限が切れました'
    return {k: v for k, v in dict(sample, age_s=age).items() if k not in ('gid', 'stale', 'reason')}, ''


def _cdc1_board(cache, gid, serial, now, last_report):
    # CDC1 には実出力の µs も、CH 別の校正・反転を適用した CH3 パルス幅も含まれない。
    board = cache.propo_status()['board']
    if not gid or not board or board.get('publisher_gid') != gid or not board['streaming']:
        return None
    t = board['telemetry']
    stamp, path, faults = t.get('stamp'), t.get('active_path'), t.get('fault_bits')
    if (not isinstance(t.get('serial'), str) or not t['serial']
            or (serial and serial.casefold() != t['serial'].casefold()) or type(stamp) not in (int, float)
            or not math.isfinite(stamp) or not 0 < stamp <= now
            or type(path) is not int or path not in (0, 1, 2, 3)
            or type(faults) is not int or not 0 <= faults <= 0xffffffff):
        return None
    age = max(now - stamp, t['age_s'])
    if age >= board.get('telemetry_timeout_s', CDC1_DEFAULT_TIMEOUT_S):
        return None
    if (last_report and last_report['serial'].casefold() == t['serial'].casefold()
            and stamp <= last_report['stamp'] <= now):
        return None
    return dict(source='cdc1', serial=t['serial'], stamp=stamp, age_s=age, ch3='unknown', rx_mode_us=None,
                ch3_raw=t['channels'][2] if t['rc_valid'] else None, active_path=path, control_source=_path(path, faults),
                fault_bits=faults, pwm_enabled=None, servo_us=None, esc_us=None)


def drive_monitor(cache, joy_gid, feedback_gid, pico_gid, now):
    with cache.lock:
        joy, joy_reason = _current(cache.get('manual_joy'), joy_gid, now, JOY_TIMEOUT_S, 'DualSense')
        feedback = cache.get('manual_feedback')
        board, reason = _current(feedback, feedback_gid, now, CDC0_TIMEOUT_S, 'Pico CDC0')
        if board is None:
            serial = feedback['serial'] if feedback and feedback_gid and feedback['gid'] == feedback_gid else None
            last_report = cache.get('last_manual_report')
            if last_report and feedback_gid and last_report['gid'] != feedback_gid:
                last_report = None
            board = _cdc1_board(cache, pico_gid, serial, now, last_report)
        else:
            reason = board.pop('driver_reason')
        return dict(joy=joy, joy_reason=joy_reason, board=board, board_reason=reason)

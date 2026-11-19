"""車載の元時刻で判定するライブ監視のテスト。"""
import json

from atd_web_bridge.auto_drive import AutoDriveWindow, NS


START = 1_780_000_000 * NS
MS = 1_000_000


def packet(stamp, state='driving'):
    return dict(stamp_ns=stamp, state=state, reason='低速前進', session_id='run', map_id='circuit',
                line='custom', max_speed_mps=.5, active=state == 'driving', braking=state == 'braking',
                pico=dict(stamp_ns=stamp - 2 * MS, sequence=2, selector=1, active_path=2,
                          rx_mode_us=1900, serial='abc', servo_us=1489, esc_us=1451, fault_bits=0),
                command=dict(stamp_ns=stamp - MS, sequence=3, kind='drive', steering=-.02, throttle=.1,
                             servo_us=1490, esc_us=1450),
                speed=dict(stamp_ns=stamp - 3 * MS, target_mps=.5, estimated_mps=-.02, validity_s=.25,
                           tracking_session_id='loc', epoch=2), last_stop=None)


def test_original_times_units_and_expiry_survive_heartbeats_and_paused_wall_clock():
    window = AutoDriveWindow()
    data = packet(START)
    window.receive(data, 'gid', START, 0)
    frame = window.current('gid', START, 0)['current']
    assert frame['speed']['estimated_mps'] == -.02
    assert (frame['command']['throttle'], frame['command']['esc_us'], frame['board']['esc_us']) == (.1, 1450, 1451)
    assert frame['speed']['stamp_s'] < frame['board']['stamp_s'] < frame['command']['stamp_s'] < frame['stamp_s']
    data['stamp_ns'] += 100 * MS
    window.receive(data, 'gid', START + 100 * MS, 100 * MS)
    # wall time is frozen at 100 ms, but the same Pico report expires 148 ms into the run.
    at = lambda ms: window.current('gid', START + 100 * MS, ms * MS)['current']
    assert at(147)['source'] == 'autonomous'
    assert at(148)['board'] is None and at(148)['source'] == 'unknown'
    assert at(148)['command'] and at(149)['command'] is None
    assert at(246)['speed'] and at(247)['speed'] is None
    assert at(600) is None


def test_ch3_handoff_standby_drive_brake_fault_and_short_latch_between_polls():
    window = AutoDriveWindow()
    rows = [('propo', 1900, 0, 1, 0, 'jetson', 'propo'), ('arming', 1550, 1, 0, 0, 'middle', 'standby'),
            ('arming', 1900, 1, 2, 0, 'jetson', 'host'), ('driving', 1900, 1, 2, 0, 'jetson', 'autonomous'),
            ('braking', 1900, 1, 2, 0, 'jetson', 'autonomous'), ('latched_stop', 0, 0, 3, 1, 'unknown', 'fault'),
            ('propo', 1100, 0, 1, 0, 'propo', 'propo')]
    stop = dict(stamp_ns=START + 4 * 20 * MS, reason='自己位置の追跡を失いました')
    for i, (state, pulse, selector, path, faults, ch3, source) in enumerate(rows):
        stamp = START + i * 20 * MS
        data = packet(stamp, state)
        data['pico'].update(rx_mode_us=pulse, selector=selector, active_path=path, fault_bits=faults)
        if state in ('latched_stop', 'braking'):
            data['last_stop'] = stop
        if state == 'braking':
            data['command'].update(kind='brake', steering=0., throttle=-.4, servo_us=1500, esc_us=1700)
        elif state == 'latched_stop':
            data['command'].update(kind='disable', steering=0., throttle=0., servo_us=None, esc_us=None)
        window.receive(data, 'driver', stamp, stamp - START)
    report = window.current('driver', stamp + MS, stamp - START + MS)
    assert [(r['state'], r['board']['ch3'], r['source']) for r in report['history']] == [
        (state, ch3, source) for state, _, _, _, _, ch3, source in rows]
    assert report['history'][4]['command']['throttle'] == -.4
    assert report['history'][5]['board']['pwm_enabled'] is False
    assert report['history'][5]['board']['esc_us'] is None and report['history'][5]['command']['esc_us'] is None
    assert report['current']['source'] == 'propo'
    assert report['last_stop'] == {'reason': stop['reason'], 'stamp_s': stop['stamp_ns']/NS}


def test_reject_bad_or_stale_fields_independently():
    window = AutoDriveWindow()
    data = packet(START)
    data['pico']['stamp_ns'] -= 150 * MS
    data['command']['throttle'] = float('nan')
    data['speed']['target_mps'] = .75  # Preserve the observed cause of an over-limit stop.
    window.receive(data, 'gid', START, 0)
    frame = window.current('gid', START, 0)['current']
    assert frame['command'] is None and frame['source'] == 'unknown' and frame['speed']['target_mps'] == .75
    data = packet(START + MS)
    data['speed']['stamp_ns'] = START + 2 * MS
    window.receive(data, 'gid', START + MS, MS)
    frame = window.current('gid', START + MS, MS)['current']
    assert frame['speed'] is None and frame['command'] and frame['board']
    data = packet(START + 2 * MS)
    data['command']['stamp_ns'] -= 140 * MS
    window.receive(data, 'gid', START + 2 * MS, 2 * MS)
    frame = window.current('gid', START + 12 * MS, 12 * MS)['current']
    assert frame['source'] == 'host' and frame['board']['ch3'] == 'jetson' and frame['command'] is None


def test_publisher_session_clock_and_history_boundaries():
    window = AutoDriveWindow()
    for i in range(800):
        stamp = START + i * 20 * MS
        window.receive(packet(stamp), 'gid', stamp, stamp - START)
    report = window.current('gid', stamp, stamp - START)
    assert len(report['history']) == 751
    assert len(json.dumps(report, ensure_ascii=False).encode()) < 1024 * 1024
    older = packet(stamp - MS)
    window.receive(older, 'gid', stamp, stamp - START)
    assert window.current('gid', stamp, stamp - START)['history'] == report['history']
    other = dict(packet(stamp), session_id='next')
    window.receive(other, 'gid', stamp, stamp - START)
    assert len(window.current('gid', stamp, stamp - START)['history']) == 1
    assert not window.current(None, stamp, stamp - START, 2)['history']
    assert not window.current('gid', stamp, stamp - START)['current']
    window.receive(other, 'new', stamp, stamp - START)
    assert not window.current('new', stamp - MS, stamp - START)['history']

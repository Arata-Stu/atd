from types import SimpleNamespace as NS

import pytest

from atd_vehicle_interface.auto_drive import AutoDriveGate, MAX_AGE_NS

T = 10_000_000_000
RC = dict(host_selected=False, active_path=1, selector=0, rx_mode_us=1100, fault_bits=0)
HOST = dict(host_selected=True, active_path=0, selector=1, rx_mode_us=1900, fault_bits=0,
            servo_us=1500, esc_us=1500)
ARMED = dict(HOST, active_path=2)


def message(kind, t, *, measured=.02, steer=.3, **fields):
    sec, nanosec = divmod(t, 10**9)
    data = dict(header=NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id='map'), map_id='test',
                map_sha256='1'*64, hdmap_sha256='2'*64, session_id='session', epoch=0, reason='')
    if kind == 'localization':
        data.update(state='tracking', speed_mps=measured)
    else:
        data.update(valid=True, validity_s=.4, line='centerline')
        if kind == 'steering':
            data.update(controller='pure_pursuit', has_steering=True, steering_rad=steer)
        else:
            data.update(has_speeds=True, feedforward_raw=910., calculated_raw=890., target_speed_mps=.4, estimated_speed_mps=measured,
                        calibration_run_id='run', proposal_sha256='a'*64)
    return NS(**(data | fields))


class Board:
    def __init__(self):
        self.writes = []

    def arm(self):
        self.writes.append('arm')
        return True

    def drive(self, *axes):
        self.writes.append(axes)
        return True

    def disarm(self):
        self.writes.append('disable')

    def stop_forward(self):
        self.writes.append('brake')


class Calibration:
    def commands(self, steering, speed, **kwargs):
        if speed.calibration_run_id != 'run':
            raise ValueError('FF 不一致')
        return steering, .2


class Run:
    def __init__(self):
        self.board = Board()
        self.gate = AutoDriveGate('test', 'centerline', .5, Calibration())

    def sample(self, t, **kwargs):
        for kind in ('speed', 'steering', 'localization'):
            self.gate.receive(kind, message(kind, t, **kwargs), t, t)

    def step(self, t, status):
        return self.gate.step(self.board, status, t, t)

    def start(self):
        self.step(T, RC)
        self.step(T+1, HOST)
        self.sample(T+2)
        self.step(T+3, HOST)
        assert self.board.writes == ['arm']
        self.step(T+4, ARMED)
        assert self.board.writes[-1] == (.3, .2)


def test_start_needs_physical_rc_host_edge_stationary_and_new_matched_samples():
    run = Run()
    run.sample(T)
    run.step(T+1, HOST)
    assert run.board.writes == []
    run.step(T+2, RC)
    run.step(T+3, HOST)
    run.step(T+4, HOST)
    run.gate.receive('steering', message('steering', T+5), T+5, T+5)
    run.gate.receive('speed', message('speed', T+6), T+6, T+6)
    run.gate.receive('localization', message('localization', T+7), T+7, T+7)
    run.step(T+8, HOST)
    assert run.board.writes == []
    run.sample(T+9, measured=.2)
    run.step(T+10, HOST)
    assert run.board.writes == [] and run.gate.fault
    run.sample(T+11)
    run.step(T+12, HOST)
    assert run.board.writes == []


@pytest.mark.parametrize(('kind', 'fields'), [
    ('localization', dict(state='lost')), ('localization', dict(measured=.6)),
    ('steering', dict(valid=False)), ('steering', dict(controller='other')),
    ('speed', dict(target_speed_mps=1.5)), ('speed', dict(map_id='other')),
    ('speed', dict(line='custom')), ('speed', dict(epoch=2)),
])
def test_loss_or_mismatch_stops_and_needs_new_ch3_edge(kind, fields):
    run = Run()
    run.start()
    run.sample(T+10, measured=.2)
    run.step(T+11, ARMED)
    run.gate.receive(kind, message(kind, T+12, **fields), T+12, T+12)
    # A recovered observation before the next 20 ms tick must not hide a loss.
    run.sample(T+13, measured=.2)
    run.step(T+14, ARMED)
    assert run.board.writes[-1] == 'brake'
    count = len(run.board.writes)
    for t in range(15, 20):
        run.sample(T+t)
        run.step(T+t, ARMED)
    assert len(run.board.writes) == count
    run.step(T+21, RC)
    run.step(T+22, HOST)
    run.sample(T+23)
    run.step(T+24, HOST)
    assert run.board.writes[-1] == 'arm'


def test_no_backward_brake_at_rest_and_no_auto_on_ch3_return():
    run = Run()
    run.start()
    run.gate.invalidate('tracking lost')
    run.step(T+5, ARMED)
    assert run.board.writes[-1] == 'disable'
    run.step(T+6, RC)
    for t in range(7, 15):
        run.sample(T+t)
        run.step(T+t, RC)
    assert run.board.writes.count('arm') == 1


def test_clock_gap_and_unchanged_stamp_do_not_refresh_drive():
    run = Run()
    run.start()
    run.sample(T+10, measured=.3)
    run.step(T+11, ARMED)
    for kind in ('speed', 'steering', 'localization'):
        run.gate.receive(kind, message(kind, T+10, measured=.3), T+11+MAX_AGE_NS//2, T+11+MAX_AGE_NS//2)
    run.step(T+10+MAX_AGE_NS, ARMED)
    assert run.board.writes[-1] == 'brake'
    assert not run.gate.active


@pytest.mark.parametrize('hz', (30, 90))
def test_new_camera_frame_ready_once_at_camera_rate(hz):
    run = Run()
    run.start()
    for index in range(1, hz + 1):
        stamp = T + index * 10**9 // hz
        for kind in ('steering', 'localization', 'speed', 'speed'):
            ready = run.gate.receive(kind, message(kind, stamp), stamp, stamp)
            if ready:
                run.step(stamp, ARMED)
        assert len(run.board.writes) == 2 + index


def test_90hz_buffer_can_match_a_still_valid_late_speed_command():
    run = Run()
    run.start()
    for index in range(1, 21):
        stamp = T + index * 10**9 // 90
        for kind in ('steering', 'localization'):
            run.gate.receive(kind, message(kind, stamp), stamp, stamp)
    first = T + 10**9 // 90
    assert run.gate.receive('speed', message('speed', first), stamp, stamp)
    run.step(stamp, ARMED)
    assert run.gate.active and run.gate.speeds.latest(stamp, stamp)['stamp_ns'] == first


def test_ch3_rc_directly_releases_without_braking():
    run = Run()
    run.start()
    run.sample(T+10, measured=.3)
    run.step(T+11, ARMED)
    run.step(T+12, RC)
    assert run.board.writes[-1] == 'disable'
    assert not run.gate.active


def test_retune_requires_new_matched_frame_and_keeps_physical_speed_ceiling():
    run = Run()
    run.start()
    run.gate.retune('raceline', .25, T+10)
    # Old samples, including messages delivered after the switch, cannot drive.
    run.sample(T+9)
    run.step(T+11, ARMED)
    assert run.board.writes[-1] == (0., 0.) and not run.gate.fault
    run.sample(T+12, line='raceline', measured=.32, target_speed_mps=.25)
    run.step(T+13, ARMED)
    assert run.board.writes[-1] == (.3, .2) and not run.gate.switching
    run.gate.receive('localization', message('localization', T+14, measured=.51), T+14, T+14)
    run.step(T+15, ARMED)
    assert run.board.writes[-1] == 'brake'


def test_repeated_retuning_cannot_extend_the_last_command_or_rearm_after_loss():
    run = Run()
    run.start()
    run.sample(T+10, measured=.2)
    run.step(T+11, ARMED)
    for offset in (20, 180_000_000):
        run.gate.retune('centerline', .4, T+offset)
        run.step(T+offset, ARMED)
        assert run.board.writes[-1] == (0., 0.)
    run.step(T+10+MAX_AGE_NS, ARMED)
    assert not run.gate.active and '期限内' in run.gate.fault
    run.gate.retune('raceline', .5, T+11+MAX_AGE_NS)
    run.sample(T+12+MAX_AGE_NS, line='raceline')
    run.step(T+13+MAX_AGE_NS, ARMED)
    assert run.board.writes.count('arm') == 1


def test_ch3_release_and_tracking_loss_during_switch_take_priority():
    for manual in (True, False):
        run = Run()
        run.start()
        run.gate.retune('raceline', .4, T+10)
        if manual:
            run.step(T+11, RC)
        else:
            run.gate.receive('localization', message('localization', T+10, state='lost'), T+11, T+11)
            run.step(T+11, ARMED)
        assert not run.gate.active and run.board.writes[-1] == 'disable'


def test_leaving_host_for_middle_requires_observed_rc_before_rearm():
    run = Run()
    run.start()
    middle = dict(RC, active_path=3, rx_mode_us=1500, fault_bits=8)
    run.step(T+10, middle)
    assert run.board.writes[-1] == 'disable'
    run.step(T+11, HOST)
    run.sample(T+12)
    run.step(T+13, HOST)
    assert run.board.writes.count('arm') == 1
    run.step(T+14, RC)
    # 通常の RC → 中間 → HOST の操作を受け付ける。
    run.step(T+15, middle)
    run.step(T+16, HOST)
    run.sample(T+17)
    run.step(T+18, HOST)
    assert run.board.writes.count('arm') == 2


def test_standstill_noise_preserves_original_velocity_without_requesting_reverse():
    run = Run()
    run.step(T, RC)
    run.step(T+1, HOST)
    run.sample(T+2, measured=-.003)
    run.step(T+3, HOST)
    run.step(T+4, ARMED)
    assert run.board.writes == ['arm', (.3, .2)]
    run.sample(T+5, measured=-.06)
    run.step(T+6, ARMED)
    assert run.board.writes[-1] == 'disable' and not run.gate.active
    assert all(entry[1] >= 0 for entry in run.board.writes if isinstance(entry, tuple))


def test_steering_speed_and_localization_must_agree_on_sample():
    run = Run()
    run.start()
    t = T+8
    run.gate.receive('localization', message('localization', t, measured=.22), t, t)
    run.gate.receive('steering', message('steering', t), t, t)
    run.gate.receive('speed', message('speed', t, measured=.20), t, t)
    run.step(t, ARMED)
    assert run.board.writes[-1] == 'brake'
    assert '不一致' in run.gate.fault


def test_speed_telemetry_in_propo_uses_original_matched_stamps_and_includes_stop_target():
    run = Run()
    run.step(T, RC)
    # FF 表外やキネマティクスのため駆動できなくても、停止目標と実速度は有効な観測。
    waiting = message('speed', T, target_speed_mps=0., measured=-.14, valid=False, validity_s=.1)
    run.gate.receive('speed', waiting, T, T)
    assert run.gate.speeds.latest(T, T) is None
    run.gate.receive('localization', message('localization', T, measured=-.14), T, T)
    pair = run.gate.speeds.latest(T, T)
    assert pair == dict(stamp_ns=T, tracking_session_id='session', epoch=0, target_mps=0.,
                        estimated_mps=-.14, validity_s=.1)
    run.gate.receive('speed', waiting, T+90_000_000, T+90_000_000)
    assert run.gate.speeds.latest(T+95_000_000, T+95_000_000) == pair
    assert run.gate.speeds.latest(T+100_000_000, T) is None
    assert run.gate.speeds.latest(T, T+100_000_000) is None
    assert not run.board.writes


@pytest.mark.parametrize(('state', 'reason'), [('lost', 'VO 追跡喪失'), ('stale', '新しい左 CameraInfo を待っています')])
def test_loss_clears_speed_plot_and_preserves_first_stop_after_ch3_takeover(state, reason):
    run = Run()
    run.start()
    run.sample(T+10, measured=.23)
    assert run.gate.speeds.latest(T+10, T+10)['estimated_mps'] == .23
    run.gate.receive('localization', message('localization', T+12, epoch=1, state=state, reason=reason), T+12, T+12)
    run.gate.receive('steering', message('steering', T+12, epoch=1, valid=False, reason='計画無効'), T+12, T+12)
    run.step(T+13, ARMED)
    assert run.gate.speeds.latest(T+13, T+13) is None
    # 遅れて届いた喪失前の速度では再有効化しない。
    run.sample(T+10, measured=.23)
    assert run.gate.speeds.latest(T+14, T+14) is None
    run.step(T+15, ARMED)
    run.step(T+16, RC)
    assert run.gate.last_stop == dict(stamp_ns=T+13, reason=reason)


def test_loss_from_replaced_localization_session_does_not_claim_previous_cause():
    run = Run()
    run.start()
    loss = message('localization', T+12, session_id='new', epoch=1, state='lost', reason='追跡喪失')
    run.gate.receive('localization', loss, T+12, T+12)
    run.step(T+13, ARMED)
    assert run.gate.last_stop['reason'] == '地図または追跡セッションが変わりました'


def test_speed_plot_does_not_join_other_epochs_or_conflicting_same_stamp():
    run = Run()
    run.gate.receive('speed', message('speed', T), T, T)
    run.gate.receive('localization', message('localization', T, epoch=1), T, T)
    assert run.gate.speeds.latest(T, T) is None
    run.sample(T+1, measured=.1)
    run.gate.receive('speed', message('speed', T+1, measured=.2), T+1, T+1)
    assert run.gate.speeds.latest(T+1, T+1) is None

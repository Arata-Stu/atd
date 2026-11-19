from types import SimpleNamespace as NS

import pytest

from atd_vehicle_interface.joy_drive import DriveGate, JOY_TIMEOUT_NS, drive_feedback


T = 50_000_000_000
MS = 1_000_000
RC = dict(host_selected=False, active_path=1, servo_us=1600, esc_us=1320)
STANDBY = dict(host_selected=True, active_path=0, servo_us=1500, esc_us=1500)
HOST = dict(STANDBY, active_path=2)


def command(stamp, steering=0., throttle=0., valid=True, source='joy', frame='atd_gamepad'):
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return NS(header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec)),
              valid=valid, source=source, steering=steering, throttle=throttle)


class Board:
    def __init__(self):
        self.writes = []

    def arm(self):
        self.writes.append('arm')
        return True

    def disarm(self):
        self.writes.append('disable')

    def drive(self, steering, throttle):
        self.writes.append((steering, throttle))
        return True


def active():
    board, gate = Board(), DriveGate()
    gate.step(board, STANDBY, T, T)
    gate.receive(command(T+MS), T+MS, T+MS)
    gate.step(board, STANDBY, T+2*MS, T+2*MS)
    gate.step(board, HOST, T+20*MS, T+20*MS)
    assert board.writes == ['arm', (0., 0.)]
    return board, gate


def test_feedback_distinguishes_observed_host_from_local_drive_authority():
    board, gate = active()
    board.serial = '1234567890abcdef'
    sample = dict(HOST, sequence=7, fault_bits=0, rx_mode_us=1811, selector=1, servo_us=1721, esc_us=1234)
    board.latest_report = lambda: (T, sample)
    reported = drive_feedback(board, gate, 'link', '走行中')
    assert reported == dict(stamp_ns=T, has_report=True, serial=board.serial, session_id='link',
                            joy_active=True, reason='走行中', sequence_number=7, rx_mode_us=1811,
                            selector=1, active_path=2, fault_bits=0, servo_us=1721, esc_us=1234)
    gate.stop(board)
    disarmed = drive_feedback(board, gate, 'link', '中立待ち')
    assert disarmed == dict(reported, joy_active=False, reason='中立待ち')
    # The latest S still reports HOST until the next firmware status packet.
    assert (disarmed['active_path'], disarmed['stamp_ns']) == (2, T)
    sample.update(RC, selector=0, rx_mode_us=1100, sequence=8)
    rc = drive_feedback(board, gate, 'link', 'プロポ')
    assert (rc['active_path'], rc['rx_mode_us'], rc['servo_us'], rc['esc_us'], rc['joy_active']) == (1,1100,1600,1320,False)
    sample.update(active_path=3, fault_bits=1, servo_us=1500, esc_us=1500)
    fault = drive_feedback(board, gate, 'link', '異常')
    assert (fault['has_report'], fault['fault_bits'], fault['joy_active']) == (True, 1, False)
    sample.update(rx_mode_us=1000000)
    assert not drive_feedback(board, gate, 'link', '')['has_report']
    offline = drive_feedback(None, gate, '', 'USB 切断')
    assert not offline['has_report'] and offline['reason'] == 'USB 切断' and offline['stamp_ns'] == 0


def test_rc_to_host_requires_later_neutral_and_new_board_feedback():
    board, gate = Board(), DriveGate()
    gate.receive(command(T), T, T)
    gate.step(board, RC, T, T)
    gate.step(board, STANDBY, T+10*MS, T+10*MS)
    gate.receive(command(T+20*MS, .4, .9), T+20*MS, T+20*MS)
    gate.step(board, STANDBY, T+22*MS, T+22*MS)
    assert not board.writes
    gate.receive(command(T+30*MS), T+30*MS, T+30*MS)
    gate.step(board, STANDBY, T+30*MS, T+30*MS)
    gate.receive(command(T+40*MS, -.35, .7), T+40*MS, T+40*MS)
    gate.step(board, STANDBY, T+40*MS, T+40*MS)
    assert board.writes == ['arm']
    gate.step(board, HOST, T+50*MS, T+50*MS)
    assert board.writes[-1] == (-.35, .7)
    gate.step(board, RC, T+60*MS, T+60*MS)
    gate.step(board, RC, T+90*MS, T+90*MS)
    gate.step(board, STANDBY, T+100*MS, T+100*MS)
    gate.receive(command(T+110*MS, -.35, .7), T+110*MS, T+110*MS)
    gate.step(board, STANDBY, T+110*MS, T+110*MS)
    assert board.writes == ['arm', (-.35,.7), 'disable']


def test_negative_neutral_and_negative_follow_operator_without_automatic_sequence():
    board, gate = active()
    for i, value in enumerate([.75, -.43, -.43, 0., -.61, .42], start=3):
        stamp = T+i*20*MS
        gate.receive(command(stamp, -.5, value), stamp, stamp)
        gate.step(board, HOST, stamp, stamp)
    assert board.writes[2:] == [(-.5, v) for v in [.75,-.43,-.43,0.,-.61,.42]]


@pytest.mark.parametrize('bad', [
    dict(valid=False), dict(source='planner'), dict(frame='map'), dict(steering=float('nan')),
    dict(throttle=float('inf')), dict(steering=-1.01), dict(throttle=1.01),
])
def test_invalid_even_between_ticks_disarms_before_any_further_movement(bad):
    board, gate = active()
    gate.receive(command(T+40*MS, **bad), T+40*MS, T+40*MS)
    gate.receive(command(T+45*MS, .5, .7), T+45*MS, T+45*MS)
    gate.step(board, HOST, T+50*MS, T+50*MS)
    assert board.writes == ['arm', (0.,0.), 'disable']
    gate.step(board, STANDBY, T+60*MS, T+60*MS)
    assert board.writes[-1] == 'disable'


def test_both_clocks_and_original_source_stamp_expire_even_with_republication():
    for wall_delta, steady_delta in [(0, JOY_TIMEOUT_NS), (JOY_TIMEOUT_NS, 0)]:
        board, gate = active()
        stamp = T+30*MS
        message = command(stamp, throttle=.8)
        gate.receive(message, stamp, stamp)
        gate.step(board, HOST, stamp, stamp)
        gate.receive(message, stamp+3*MS, stamp+4*MS)
        gate.step(board, HOST, stamp+wall_delta, stamp+steady_delta)
        assert board.writes[-2:] == [(0.,.8), 'disable']


def test_old_future_conflicting_and_out_of_order_input_cannot_refresh_motion():
    for stamp, throttle in [(0,0.), (T-500*MS,.6), (T+60*MS,.6), (T+MS,.6), (T,0.)]:
        board, gate = active()
        gate.receive(command(stamp, throttle=throttle), T+40*MS, T+40*MS)
        gate.step(board, HOST, T+50*MS, T+50*MS)
        assert board.writes[-1] == 'disable'


def test_late_new_sample_does_not_bridge_a_gap_while_executor_was_stalled():
    board, gate = active()
    stamp = T+400*MS
    gate.receive(command(stamp, throttle=.8), stamp, stamp)
    gate.step(board, HOST, stamp, stamp)
    assert board.writes[-1] == 'disable'


def test_ack_from_non_neutral_board_is_rejected():
    board, gate = Board(), DriveGate()
    gate.step(board, STANDBY, T, T)
    gate.receive(command(T+MS), T+MS, T+MS)
    gate.step(board, STANDBY, T+MS, T+MS)
    gate.step(board, dict(HOST, servo_us=1530), T+30*MS, T+30*MS)
    assert board.writes == ['arm', 'disable']


def test_ch3_change_in_a_new_status_before_usb_write_requires_another_neutral(monkeypatch):
    board, gate = active()
    monkeypatch.setattr(board, 'drive', lambda *_: False)
    gate.receive(command(T+40*MS, .2, .5), T+40*MS, T+40*MS)
    gate.step(board, HOST, T+40*MS, T+40*MS)
    gate.step(board, STANDBY, T+60*MS, T+60*MS)
    gate.receive(command(T+70*MS, .2, .5), T+70*MS, T+70*MS)
    gate.step(board, STANDBY, T+70*MS, T+70*MS)
    assert board.writes == ['arm', (0.,0.), 'disable']
    monkeypatch.setattr(board, 'arm', lambda: False)
    gate.receive(command(T+80*MS), T+80*MS, T+80*MS)
    gate.step(board, STANDBY, T+80*MS, T+80*MS)
    assert not gate.requested and gate.host_since is None

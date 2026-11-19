"""Joy の CDC0 入出力・経路を疑似 tty と実際の JPB1 フレームで検査する。"""
import binascii
import os
import pty
import select

import pytest

from atd_esc_trials import board


SERIAL = '0123456789ABCDEF'


class Link:
    def __init__(self, master, port):
        self.master = master
        self.port = port
        self.seq = 0

    def status(self, *, mode=1900, selector=1, path=0, steer=1500, throttle=1500, fault=0,
               rx_steer=1500, rx_throttle=1500):
        self.seq = (self.seq + 1) & 0xffffffff
        value = f'JPB1,S,{self.seq},{rx_steer},{rx_throttle},{mode},{steer},{throttle},6000,{selector},{path},{fault}'
        os.write(self.master, board._packet(value.encode()))

    def command(self):
        assert select.select([self.master], [], [], .1)[0]
        frame = os.read(self.master, 4096)
        payload, crc = frame.rstrip(b'\r\n').rsplit(b',', 1)
        assert int(crc, 16) == binascii.crc_hqx(payload, 0xffff)
        version, kind, _, *numbers = payload.decode().split(',')
        assert (version, kind) == ('JPB1', 'C')
        return tuple(map(int, numbers))  # steer, forward, reverse, brake, flags

    def silent(self):
        return not select.select([self.master], [], [], .002)[0]


@pytest.fixture
def link(monkeypatch):
    master, slave = pty.openpty()
    try:
        monkeypatch.setattr(board, '_identify', lambda fd: SERIAL)
        yield Link(master, os.ttyname(slave))
    finally:
        os.close(slave)
        os.close(master)


def opened(link):
    result = board.ManualBoard(link.port, SERIAL.lower())
    result.open()
    assert result.serial == SERIAL and link.command() == (0, 0, 0, 0, 1)
    return result


def arm(link, dev):
    link.status()
    assert dev.poll()['host_selected']
    assert dev.arm()
    assert link.command() == (0, 0, 0, 0, 3)
    link.status(path=2)
    assert dev.poll()['active_path'] == 2


def test_steering_signed_throttle_and_release(link):
    dev = opened(link)
    try:
        assert dev.poll() is None
        # A physical receiver and CH3 are necessary, including when Rev6 permits receiver-less HOST.
        for state in [dict(mode=0, rx_steer=0, rx_throttle=0), dict(mode=1500), dict(mode=1240, selector=0, path=1),
                      dict(fault=1), dict(rx_throttle=0), dict(selector=0)]:
            link.status(**state)
            assert dev.poll()['host_selected'] is False
            assert dev.arm() is False
            assert link.silent()
        arm(link, dev)
        for steer, throttle, expected in [(1., .8, (1000, 800, 0, 0, 3)), (0., -.9, (0, 0, 900, 0, 3)),
                                          (0., 0., (0, 0, 0, 0, 3)), (-.4, -.25, (-400, 0, 250, 0, 3))]:
            assert dev.drive(steer, throttle)
            assert link.command() == expected
        dev.disarm()
        assert link.command() == (0, 0, 0, 0, 1)
        # Previous S frame reported active; cannot reuse it for restart.
        with pytest.raises(RuntimeError):
            dev.arm()
        assert link.silent()
        arm(link, dev)
    finally:
        dev.close()
    assert link.command() == (0, 0, 0, 0, 1)


def test_require_actual_neutral_output_after_arm(link):
    dev = opened(link)
    link.status()
    dev.arm()
    assert link.command() == (0, 0, 0, 0, 3)
    link.status(path=2, steer=1650)
    with pytest.raises(RuntimeError, match='中立'):
        dev.drive(.7, .4)
    assert link.command() == (0, 0, 0, 0, 1)
    assert dev.fd is None


def test_status_snapshot_keeps_real_receipt_time_without_new_io(link, monkeypatch):
    times = iter([100, 200])
    monkeypatch.setattr(board, 'time_ns', lambda: next(times))
    dev = opened(link)
    try:
        assert dev.latest_report() is None
        link.status(path=1, mode=1200, selector=0, steer=1610, throttle=1240)
        dev.poll()
        stamp, status = dev.latest_report()
        assert (stamp, status['servo_us'], status['esc_us']) == (100, 1610, 1240)
        link.status(path=0)
        assert dev.latest_report() == (100, status)
        dev.poll()
        assert dev.latest_report()[0] == 200
        dev.poll()
        assert dev.latest_report()[0] == 200
        assert link.silent()
    finally:
        dev.close()
    assert dev.latest_report() is None


@pytest.mark.parametrize('fault', ['receiver_off', 'sequence', 'crc', 'backlog', 'expired', 'command_expired'])
def test_any_loss_invalidates_whole_session_including_buffered_host(link, monkeypatch, fault):
    clock = [50.]
    monkeypatch.setattr(board.time, 'monotonic', lambda: clock[0])
    dev = opened(link)
    arm(link, dev)
    if fault == 'receiver_off':
        link.status(path=2, mode=0, rx_steer=0, rx_throttle=0)
    elif fault == 'sequence':
        link.seq += 1
        link.status(path=2)
    elif fault == 'crc':
        os.write(link.master, b'JPB1,S,invalid,0000\r\n')
    elif fault == 'backlog':
        for _ in range(5):
            link.status(path=2)
    else:
        if fault == 'command_expired':
            clock[0] += .10
            link.status(path=2)
            dev.poll()
        clock[0] += .11 if fault == 'command_expired' else .2
        link.status(path=2)
    with pytest.raises(RuntimeError):
        dev.drive(.35, .68)
    assert link.command() == (0, 0, 0, 0, 1)
    with pytest.raises(RuntimeError):
        dev.drive(.35, .68)
    assert link.silent() and dev.fd is None


def test_rc_handoff_during_drive_preserves_port_and_requires_new_neutral_arm(link):
    dev = opened(link)
    try:
        arm(link, dev)
        # CH3 moved after the caller's poll(), and changed back before the next read().
        link.status(path=1, mode=1200, selector=0)
        link.status(path=0)
        assert dev.drive(.3, .4) is False
        assert link.command() == (0, 0, 0, 0, 1)
        assert dev.fd is not None
        dev.disarm()
        assert link.silent()
        arm(link, dev)
        link.status(path=1, mode=1100, selector=0)
        assert dev.poll()['host_selected'] is False
        assert link.command() == (0, 0, 0, 0, 1)
        assert not dev.arm() and link.silent()
        # A switch away in between poll and arm also emits no enabled frame.
        link.status(path=0)
        assert dev.poll()['host_selected']
        link.status(path=1, mode=1200, selector=0)
        assert not dev.arm() and link.silent()
    finally:
        dev.close()


def test_discovery_rechecks_opened_serial_and_refuses_ambiguity(link, monkeypatch):
    monkeypatch.setattr(board, '_devices', lambda *_: [(link.port, SERIAL), ('/unused', 'B' * 16)])
    with pytest.raises(RuntimeError, match='2 件'):
        board.ManualBoard().open()
    dev = board.ManualBoard(serial=SERIAL)
    dev.open()
    assert link.command() == (0, 0, 0, 0, 1)
    dev.close()
    monkeypatch.setattr(board, '_identify', lambda _: 'C' * 16)
    with pytest.raises(ValueError, match='固有 ID'):
        board.ManualBoard(serial=SERIAL).open()
    assert link.silent()


def test_esc_trial_cannot_open_port_owned_by_joy(link):
    dev = opened(link)
    try:
        arm(link, dev)
        with pytest.raises(OSError), board.Board(link.port):
            pytest.fail('Joy 走行中の CDC0 で試験を開始')
        assert link.silent()
        dev.drive(.15, .1)
        assert link.command() == (150, 100, 0, 0, 3)
    finally:
        dev.close()

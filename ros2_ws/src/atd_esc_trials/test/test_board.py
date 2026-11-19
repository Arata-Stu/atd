"""実機を動かさず Linux sysfs 属性と疑似 tty 上の Rev6 手順を検証する。"""
import binascii
import json
import os
import pty
import select
import threading
import time

import pytest

from atd_esc_trials import board
from atd_esc_trials.runner import run_reserved
from atd_esc_trials.store import TrialStore


def test_discovery_and_opened_device_identity(tmp_path):
    sysfs, dev = tmp_path / 'sys', tmp_path / 'dev'
    dev.mkdir()
    usb = tmp_path / 'usb'
    usb.mkdir()
    for name, value in dict(idVendor='cafe', idProduct='4002', product='JPBB Rev6 Pico', serial='1234567890ABCDEF').items():
        (usb / name).write_text(value)
    for i, interface in enumerate(('00', '02')):
        path = usb / interface
        path.mkdir()
        (path / 'bInterfaceNumber').write_text(interface)
        syspath = sysfs / f'ttyACM{i}'
        syspath.mkdir(parents=True)
        (syspath / 'device').symlink_to(path)
        (dev / syspath.name).symlink_to('/dev/null')
    assert board.discover_ports(sysfs, dev) == [str(dev / 'ttyACM0')]
    with open('/dev/null', 'rb') as stream:
        number = os.fstat(stream.fileno()).st_rdev
        node = sysfs / f'{os.major(number)}:{os.minor(number)}'
        node.mkdir()
        (node / 'device').symlink_to(usb / '00')
        assert board._identify(stream.fileno(), sysfs) == '1234567890ABCDEF'
    (usb / 'serial').write_text('wrong')
    assert not board.discover_ports(sysfs, dev)


class Pico:
    """S の連番は C の seq と独立。firmware同様、最初の C から状態を定期送信する。"""
    def __init__(self, master):
        self.master = master
        self.commands = []
        self.reported = []
        self.path, self.esc = 0, 1500
        self.rx = 1900
        self.fault = 0
        self.seq = 100
        self.suppress = False
        self.suppress_motion_report = False
        self.stop = threading.Event()

    def run(self):
        buf = b''
        next_status = time.monotonic()
        while not self.stop.is_set():
            if select.select([self.master], [], [], .008)[0]:
                buf += os.read(self.master, 4096)
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    payload, crc = line.rstrip(b'\r').rsplit(b',', 1)
                    assert int(crc, 16) == binascii.crc_hqx(payload, 65535)
                    protocol, kind, seq, steer, forward, reverse, brake, flags = payload.decode().split(',')
                    assert protocol == 'JPB1' and kind == 'C' and int(steer) == 0
                    forward, reverse, brake, flags = map(int, (forward, reverse, brake, flags))
                    self.commands.append((forward, reverse, brake, flags))
                    if flags == 1:
                        self.path, self.esc = 0, 1500
                    elif self.rx >= 1700 and (self.path == 2 or not any((forward, reverse, brake))):
                        self.path = 2
                        self.esc = 1500 + (brake or reverse) // 2 - forward // 2
            if (self.commands and not self.suppress and not (self.suppress_motion_report and self.esc != 1500)
                    and time.monotonic() >= next_status):
                next_status = time.monotonic() + .05  # Rev6_STATUS_PERIOD_MS
                self.seq = (self.seq + 1) & 0xffffffff
                status = (f'JPB1,S,{self.seq},1500,1500,{self.rx},1500,{self.esc},6000,'
                          f'{int(self.rx >= 1700)},{self.path if self.rx >= 1700 else 1},{self.fault}').encode()
                os.write(self.master, board._packet(status))
                self.reported.append((self.seq, self.path if self.rx >= 1700 else 1, self.esc))


@pytest.fixture
def link(monkeypatch):
    master, slave = pty.openpty()
    try:
        pico = Pico(master)
        thread = threading.Thread(target=pico.run, daemon=True)
        thread.start()
        monkeypatch.setattr(board, '_identify', lambda _: '1234567890abcdef')
        yield os.ttyname(slave), pico
    finally:
        pico.stop.set()
        thread.join(timeout=1)
        os.close(slave)
        os.close(master)


def test_neutral_arm_forward_brake_reverse_and_disable(link):
    port, pico = link
    with board.Board(port) as driver:
        assert driver.feedback()['active_path'] == 0
        driver.command(0., True)
        assert driver.feedback()['active_path'] == 2
        for value, reverse in ((.2, False), (-.3, False), (0., False), (-.15, True), (.1, False)):
            driver.command(value, True, reverse=reverse)
        driver.command(0., False)
        with pytest.raises(RuntimeError, match='失効'):
            driver.command(0., True)
    assert pico.commands == [(0, 0, 0, 1), (0, 0, 0, 3), (200, 0, 0, 3), (0, 0, 300, 3),
                             (0, 0, 0, 3), (0, 150, 0, 3), (100, 0, 0, 3), (0, 0, 0, 1)]


@pytest.mark.parametrize('failure', ['manual', 'receiver_off', 'fault', 'stale', 'sequence', 'motion_first'])
def test_fault_invalidates_and_releases(link, failure):
    port, pico = link
    with board.Board(port) as driver:
        if failure == 'motion_first':
            with pytest.raises(RuntimeError, match='最初'):
                driver.command(.1, True)
        else:
            driver.command(0., True)
            if failure in ('manual', 'receiver_off'):
                pico.rx = 1200 if failure == 'manual' else 0
            elif failure == 'fault':
                pico.fault = 1
            elif failure == 'sequence':
                pico.seq += 20
            else:
                pico.suppress = True
            with pytest.raises(RuntimeError):
                deadline = time.monotonic() + 1
                while time.monotonic() < deadline:
                    driver.feedback()
                    time.sleep(.01)
        with pytest.raises(RuntimeError, match='失効'):
            driver.command(0., True)
        deadline = time.monotonic() + 1
        while pico.commands[-1] != (0, 0, 0, 1) and time.monotonic() < deadline:
            time.sleep(.005)
        assert pico.commands[-1] == (0, 0, 0, 1)


def test_bad_status_crc_is_rejected():
    with pytest.raises(RuntimeError, match='CRC'):
        board._decode(b'JPB1,S,1,1500,1500,2000,1500,1500,6000,1,2,0,BAAD')


def test_ownership_error_sends_nothing(link, monkeypatch):
    port, pico = link
    def deny(fd, flags):
        raise BlockingIOError('tty already locked')
    monkeypatch.setattr(board.fcntl, 'flock', deny)
    with pytest.raises(BlockingIOError), board.Board(port):
        pytest.fail('ロック中のポートで試験を開始')
    assert not pico.commands


class ConfirmStopped:
    """停止確認だけは人の目視の代わりに固定のテスト証跡を返す。"""
    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def begin(self):
        pass

    def evidence(self):
        return 'テストの停止確認'


def test_reserved_driving_sequence_and_pwm_over_tty(link, tmp_path):
    port, pico = link
    client, runner = TrialStore(tmp_path), TrialStore(tmp_path)
    steps = [dict(action='forward', level=.2, duration_s=.16),
             dict(action='brake', level=.4, duration_s=.16),
             dict(action='wait_stopped', duration_s=.5),
             dict(action='neutral', duration_s=.16),
             dict(action='reverse', level=.1, duration_s=.16),
             dict(action='forward', level=.3, duration_s=.16)]
    trial = client.prepare(steps)['trial']
    assert trial['status'] == 'queued'
    result = run_reserved(runner, trial['id'], port, ConfirmStopped())
    assert client.status()['trial'] == result
    assert result['status'] == 'succeeded', result['message']
    assert result['stop_evidence'] == 'テストの停止確認'
    assert result['esc_state'] == 'unknown' and result['pwm_us'] is None
    distinct = []
    for _, path, pwm in pico.reported:
        if not distinct or distinct[-1] != (path, pwm):
            distinct.append((path, pwm))
    assert distinct == [(0, 1500), (2, 1500), (2, 1400), (2, 1700), (2, 1500),
                        (2, 1550), (2, 1350), (0, 1500)]
    events = [json.loads(line) for line in (tmp_path / (trial['id'] + '.jsonl')).read_text().splitlines()]
    assert {e['step_index'] for e in events if e['status'] == 'running'} >= set(range(len(steps)))
    assert {(e['step_index'], e['pwm_us']) for e in events if e.get('pwm_observed')} >= {
        (0, 1400), (1, 1700), (2, 1700), (3, 1500), (4, 1550), (5, 1350)}


def test_reserved_short_motion_without_new_pwm_evidence_fails(link, tmp_path):
    port, pico = link
    # 20ms の区間に S (50ms 周期) が到着しない場合を再現。
    pico.suppress_motion_report = True
    store = TrialStore(tmp_path)
    trial = store.prepare([dict(action='forward', level=.2, duration_s=.02),
                           dict(action='neutral', duration_s=.02)])['trial']
    result = run_reserved(store, trial['id'], port, ConfirmStopped())
    assert result['status'] == 'failed' and '1400' in result['message']
    assert result['step_index'] == 0 and result['pwm_us'] is None
    assert pico.commands[-1] == (0, 0, 0, 1)
    assert (200, 0, 0, 3) in pico.commands
    events = [json.loads(line) for line in (tmp_path / (trial['id'] + '.jsonl')).read_text().splitlines()]
    assert any(e.get('requested_pwm_us') == 1400 and e['pwm_observed'] is False for e in events)
    assert all(not e.get('pwm_observed') for e in events)


@pytest.mark.parametrize('interruption', ['cancel', 'manual', 'no_status'])
def test_reserved_motion_aborts_and_disables(link, tmp_path, interruption):
    port, pico = link
    client = TrialStore(tmp_path)
    trial = client.prepare([dict(action='forward', level=.1, duration_s=2)])['trial']
    observed = {}
    def intervene():
        deadline = time.monotonic() + 1.5
        while not any(path == 2 and pwm == 1450 for _, path, pwm in pico.reported):
            if time.monotonic() >= deadline:
                return
            time.sleep(.002)
        observed.update(client.status()['trial'])
        if interruption == 'cancel':
            client.cancel(trial['id'])
        elif interruption == 'manual':
            pico.rx = 1200
        else:
            pico.suppress = True
    operator = threading.Thread(target=intervene)
    operator.start()
    result = run_reserved(TrialStore(tmp_path), trial['id'], port, ConfirmStopped())
    operator.join(timeout=2)
    assert observed['status'] == 'running'
    assert result['status'] == ('cancelled' if interruption == 'cancel' else 'failed'), result['message']
    assert result['pwm_us'] is None and result['throttle'] is None
    deadline = time.monotonic() + .3
    while pico.commands[-1][-1] != 1 and time.monotonic() < deadline:
        time.sleep(.002)
    assert pico.commands[-1] == (0, 0, 0, 1)
    assert all(flags == 1 for _, _, _, flags in pico.commands[pico.commands.index((0, 0, 0, 1), 1):])

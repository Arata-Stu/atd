"""実際の制御計算と照合した JPB1 コマンドを疑似 tty から同梱 Rev6 の C 実装へ通す。"""
import ctypes
import json
import os
from pathlib import Path
import pty
import select
import subprocess
from types import SimpleNamespace

import pytest

from atd_control.core import pure_pursuit
from atd_control.speed import FeedforwardTable, SpeedController
from atd_esc_trials import board as transport
from atd_vehicle_interface import auto_board
from atd_vehicle_interface.auto_calibration import AutoCalibration
from atd_vehicle_interface.auto_drive import AutoDriveGate
from atd_vehicle_interface.auto_status import snapshot
from atd_web_bridge.auto_drive import AutoDriveWindow


@pytest.fixture
def circuit(tmp_path, monkeypatch):
    package = Path(__file__).resolve().parents[2] / 'atd_vehicle_interface'
    fw = package / 'firmware/pico_rev6'
    library = tmp_path / 'rev6.so'
    subprocess.run(['cc', '-std=c11', '-shared', '-fPIC', '-I', str(fw / 'include'),
                    str(package / 'test/joy_drive_rev6.c'),
                    *(str(fw / 'src' / name) for name in ('rev6_core.c', 'jpbb_protocol.c', 'rev6_transport.c')),
                    '-o', str(library)], check=True)
    pico = ctypes.CDLL(str(library))
    pico.fixture_status.restype = ctypes.c_char_p
    pico.fixture_init()
    pico.fixture_receiver(992, 992)
    master, slave = pty.openpty()
    tick_ms, sequence = 0, 0
    clock = SimpleNamespace(monotonic=lambda: 10. + tick_ms / 1000)

    def now_ns():
        return (10000 + tick_ms) * 1_000_000

    monkeypatch.setattr(transport, 'time', clock)
    monkeypatch.setattr(transport, 'time_ns', lambda: now_ns() - 2_000_000)
    monkeypatch.setattr(auto_board, 'time', clock)
    monkeypatch.setattr(auto_board, 'time_ns', now_ns)
    monkeypatch.setattr(transport, '_identify', lambda _: '1234567890ABCDEF')
    board = auto_board.AutoBoard(os.ttyname(slave), brake_level=.2, brake_hold_s=.2)
    commands = []
    pending = bytearray()

    def apply():
        while select.select([master], [], [], .001)[0]:
            pending.extend(os.read(master, 4096))
        while b'\n' in pending:
            line, _, rest = pending.partition(b'\n')
            pending[:] = rest
            frame = bytes(line).rstrip(b'\r')
            assert frame.startswith(b'JPB1,C,')
            pico.fixture_command(frame)
            commands.append(tuple(int(field) for field in frame.split(b',')[3:-1]))

    def advance(mode=1811, ms=50, usb=True):
        nonlocal tick_ms, sequence
        apply()
        tick_ms += ms
        pico.fixture_step(ms, mode, usb)
        sequence += 1
        packet = pico.fixture_status(sequence)
        os.write(master, packet)
        return board.poll()

    try:
        board.open()
        yield SimpleNamespace(board=board, advance=advance, commands=commands, apply=apply,
                              now_ns=now_ns, pico=pico)
    finally:
        board.close()
        os.close(master)
        os.close(slave)


def active_host(circuit):
    for _ in range(5):
        circuit.advance(172)
    for _ in range(5):
        sample = circuit.advance()
    assert sample['host_selected'] and sample['active_path'] == 0
    assert circuit.board.arm()
    sample = circuit.advance()
    assert (sample['active_path'], sample['esc_us']) == (2, 1500)


def test_stop_without_confirmed_forward_pwm_stays_neutral(circuit):
    active_host(circuit)
    circuit.board.drive(.1, .2)
    # 正指令を送った直後で、Pico の前進 PWM はまだ観測していない。
    circuit.board.stop_forward()
    sample = circuit.advance()
    assert (sample['active_path'], sample['esc_us']) == (0, 1500)
    assert not any(reverse or brake for _, _, reverse, brake, _ in circuit.commands)


@pytest.mark.parametrize('return_to_rc', [False, True])
def test_single_continuous_brake_and_physical_override(circuit, return_to_rc):
    active_host(circuit)
    circuit.board.drive(.12, .22)
    sample = circuit.advance()
    assert (sample['servo_us'], sample['esc_us']) == (1560, 1390)
    circuit.board.stop_forward()
    assert circuit.board.braking
    sample = circuit.advance()
    assert (sample['active_path'], sample['esc_us']) == (2, 1600)
    circuit.pico.fixture_receiver(172, 172)
    mode = 172 if return_to_rc else 1811
    for _ in range(7):
        sample = circuit.advance(mode)
    assert not circuit.board.braking
    expected = (1, 1000, 1000) if return_to_rc else (0, 1500, 1500)
    assert (sample['active_path'], sample['servo_us'], sample['esc_us']) == expected
    circuit.board.stop_forward()  # 2回目の負パルスを生じない。
    circuit.advance(mode)
    circuit.apply()
    brakes = [i for i, cmd in enumerate(circuit.commands) if cmd[3]]
    assert brakes and brakes == list(range(brakes[0], brakes[-1] + 1))
    assert all(reverse == 0 for _, _, reverse, _, _ in circuit.commands)


def test_pursuit_pid_to_pwm_tracking_loss_and_ch3_override(circuit):
    # 合成校正。実車設定には使用しない。
    table = FeedforwardTable('test-run', 'a' * 64, (.2, .5, 1.), (900, 825, 700), -1)
    cal = AutoCalibration.from_document(dict(
        schema='atd.auto_drive.v1', brake_level=.2, brake_hold_s=.2,
        steering=[dict(angle_deg=a, servo_us=p) for a, p in [(-25, 1250), (0, 1500), (25, 1750)]],
        forward_throttle=[dict(ch2_raw=700, esc_us=1350), dict(ch2_raw=992, esc_us=1500)]), table)
    gate = AutoDriveGate('track', 'custom', .5, cal)
    pid = SpeedController(table, max_target_speed_mps=.5)
    monitor = AutoDriveWindow()
    publisher = 'a1b2'
    epoch = 1
    loss_reason = '追跡喪失。新しい地図照合の確証が必要です'

    def observed():
        return monitor.current(publisher, circuit.now_ns(), circuit.now_ns())

    def observe(mode, velocity, tracking=True):
        sample = circuit.advance(mode)
        now = circuit.now_ns()
        sec, nanosec = divmod(now, 10**9)
        header = SimpleNamespace(frame_id='map', stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
        identity = dict(header=header, map_id='track', map_sha256='b'*64, hdmap_sha256='c'*64,
                        session_id='live', epoch=epoch)
        state = 'tracking' if tracking else 'lost'
        gate.receive('localization', SimpleNamespace(**identity, state=state, speed_mps=velocity,
                     reason='' if tracking else loss_reason), now, now)
        if tracking:
            lateral = pure_pursuit([(0., 0.), (2., .12)], x=0., y=0., yaw=0.,
                                   speed_mps=velocity, wheelbase_m=.25)
            speed = pid.update(now, 1.5, velocity)
            assert speed['target_speed_mps'] == .5
            shared = dict(**identity, line='custom', validity_s=.25)
            gate.receive('steering', SimpleNamespace(**shared, valid=lateral['valid'], reason='',
                         controller=lateral['controller'], has_steering=True, steering_rad=lateral['steering_rad']),
                         now, now)
            gate.receive('speed', SimpleNamespace(**shared, **speed, has_speeds=True,
                         calibration_run_id=table.run_id, proposal_sha256=table.proposal_sha256), now, now)
        reason = gate.step(circuit.board, sample, now, now)
        wire = snapshot(gate, circuit.board, 'a' * 32, reason, now, now)
        monitor.receive(json.loads(json.dumps(wire, allow_nan=False)), publisher, now, now)
        return sample

    for _ in range(6):
        observe(172, 0.)
    manual = observed()['current']
    assert manual['source'] == 'propo'
    assert manual['board']['esc_us'] == 1500
    for _ in range(10):
        observe(1811, 0.)
    for _ in range(4):
        sample = observe(1811, .25)
    assert gate.active and sample['active_path'] == 2
    assert 1500 < sample['servo_us'] < 1750 and 1350 < sample['esc_us'] < 1500
    driving = observed()['current']
    assert driving['state'] == 'driving'
    assert (driving['speed']['target_mps'], driving['speed']['estimated_mps']) == (.5, .25)
    for channel in ('servo_us', 'esc_us'):
        assert driving['board'][channel] == sample[channel]
    circuit.apply()
    steer, forward, reverse, brake, flags = circuit.commands[-1]
    assert (reverse, brake, flags) == (0, 0, 3)
    assert driving['command']['steering'] == steer / 1000
    assert driving['command']['throttle'] == forward / 1000
    assert driving['command']['servo_us'] == 1500 + int(steer / 2)
    assert driving['command']['esc_us'] == 1500 - forward // 2
    epoch += 1  # atd_localization は追跡喪失時に世代を進める。
    observe(1811, .25, tracking=False)
    braking = observed()
    assert braking['current']['state'] == 'braking'
    assert braking['current']['command']['kind'] == 'brake'
    assert braking['current']['command']['throttle'] < 0
    assert braking['last_stop']['reason'] == loss_reason
    stopped = [observe(1811, 0.) for _ in range(10)]  # 再捕捉しても HOST のまま再発進しない。
    assert any(s['active_path'] == 2 and s['esc_us'] == 1600 for s in stopped)
    assert stopped[-1]['active_path'] == 0 and stopped[-1]['esc_us'] == 1500 and not gate.active
    assert observed()['current']['state'] == 'latched_stop'
    assert observed()['last_stop'] == braking['last_stop']
    for _ in range(3):
        sample = observe(172, 0.)
    assert sample['active_path'] == 1
    assert observed()['current']['source'] == 'propo'
    for _ in range(10):
        observe(1811, 0.)
    assert gate.active
    circuit.pico.fixture_receiver(172, 172)
    # Jetson の次の ROS/USB tick が一度も入らない間に CH3 を戻す。
    circuit.apply()
    circuit.pico.fixture_step(60, 172, True)
    manual = transport._decode(circuit.pico.fixture_status(0).rstrip())
    assert (manual['active_path'], manual['servo_us'], manual['esc_us']) == (1, 1000, 1000)
    assert all(reverse == 0 for _, _, reverse, _, _ in circuit.commands)

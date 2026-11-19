"""手動走行の送信バイトを PTY に通し、同梱 Rev6 の C 実装で切替と出力値を検証する。"""
import ctypes
import os
from pathlib import Path
import pty
import select
import subprocess
from types import SimpleNamespace

from atd_esc_trials import board as transport
from atd_vehicle_interface.joy_drive import DriveGate, drive_feedback


def test_physical_ch3_selects_joy_or_receiver(tmp_path, monkeypatch):
    tests = Path(__file__).resolve().parent
    firmware = tests.parent / 'firmware/pico_rev6'
    library = tmp_path / 'rev6.so'
    subprocess.run(['cc', '-std=c11', '-shared', '-fPIC', '-I', str(firmware / 'include'),
                    str(tests / 'joy_drive_rev6.c'),
                    *(str(firmware / 'src' / name) for name in ('rev6_core.c', 'jpbb_protocol.c', 'rev6_transport.c')),
                    '-o', str(library)], check=True)
    pico = ctypes.CDLL(str(library))
    pico.fixture_status.restype = ctypes.c_char_p
    pico.fixture_init()

    # Substitute only USB descriptor identification and the hardware clock. The driver owns a real tty.
    elapsed_ms = 0
    monkeypatch.setattr(transport, '_identify', lambda _: '1234567890abcdef')
    monkeypatch.setattr(transport, 'time', SimpleNamespace(monotonic=lambda: 1 + elapsed_ms / 1000))
    master, slave = pty.openpty()
    board = transport.ManualBoard(port=os.ttyname(slave))
    gate = DriveGate()
    received = bytearray()
    sent = []
    reported = None
    report_stamp = 0
    seen_sources = set()

    def apply_usb():
        while select.select([master], [], [], .001)[0]:
            received.extend(os.read(master, 4096))
        while b'\n' in received:
            line, _, rest = received.partition(b'\n')
            received[:] = rest
            frame = bytes(line).rstrip(b'\r')
            # A pending HOST frame may be rejected when CH3 changes between two 20 Hz reports.
            assert frame.startswith(b'JPB1,C,')
            pico.fixture_command(frame)
            sent.append(frame)

    def tick(mode, steering=0., throttle=0., joy=True):
        nonlocal elapsed_ms, reported, report_stamp
        elapsed_ms += 10
        pico.fixture_step(10, mode, True)
        packet = pico.fixture_status(elapsed_ms // 50)
        new_report = elapsed_ms % 50 == 0
        if new_report:
            os.write(master, packet)
            reported = transport._decode(packet.rstrip())
        state = board.poll()
        clock_ns = (elapsed_ms + 1000) * 1_000_000
        if joy:
            sec, nanosec = divmod(clock_ns, 1_000_000_000)
            header = SimpleNamespace(frame_id='atd_gamepad', stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
            gate.receive(SimpleNamespace(header=header, source='joy', valid=True, steering=steering,
                                         throttle=throttle, enable_requested=False), clock_ns, clock_ns)
        reason = gate.step(board, state, clock_ns, clock_ns)
        feedback = drive_feedback(board, gate, 'test-session', reason)
        if reported is None:
            assert not feedback['has_report']
        else:
            assert feedback['has_report'] and feedback['sequence_number'] == reported['sequence']
            for field in ('rx_mode_us', 'selector', 'active_path', 'servo_us', 'esc_us', 'fault_bits'):
                assert feedback[field] == reported[field]
            if new_report:
                assert feedback['stamp_ns'] > report_stamp
                report_stamp = feedback['stamp_ns']
            else:
                assert feedback['stamp_ns'] == report_stamp
            if feedback['joy_active']:
                assert reported['selector'] == 1 and reported['active_path'] == 2 and gate.active
            seen_sources.add((feedback['active_path'], feedback['joy_active']))
        apply_usb()
        snapshot = transport._decode(packet.rstrip())
        return snapshot['active_path'], snapshot['servo_us'], snapshot['esc_us']

    try:
        board.open()
        apply_usb()
        for _ in range(20):  # No receiver, including the firmware's USB-only HOST allowance at boot
            tick(-1, .4, .6)
        assert all(frame.split(b',')[-2] == b'1' for frame in sent)
        for _ in range(20):
            state = tick(172, .4, .6)
        assert state == (1, 1000, 2000)
        for _ in range(20):  # A held trigger at the instant of handover must never arm.
            state = tick(1811, .4, .6)
        assert state == (0, 1500, 1500)
        assert all(frame.split(b',')[-2] == b'1' for frame in sent)
        for _ in range(10):
            state = tick(1811)
        assert state == (2, 1500, 1500)
        for steer, throttle, pwm in [(.4, .6, (1700, 1200)), (-.4, -.4, (1300, 1700)),
                                     (0., 0., (1500, 1500)), (0., -.4, (1500, 1700))]:
            for _ in range(3):
                state = tick(1811, steer, throttle)
            assert state == (2, *pwm)
        for _ in range(30):  # Joy stops while driving but the receiver still permits HOST.
            state = tick(1811, joy=False)
        assert state == (0, 1500, 1500)
        for _ in range(8):
            state = tick(1811, .4, .6)
        assert state == (0, 1500, 1500)
        for _ in range(10):
            state = tick(1811)
        assert state == (2, 1500, 1500)
        for _ in range(3):
            state = tick(1811, .4, .6)
        assert state == (2, 1700, 1200)
        for _ in range(10):
            state = tick(172, .4, .6)
        assert state == (1, 1000, 2000)
        previous = len(sent)
        for _ in range(8):
            tick(172, .4, .6)
        assert sent[previous:] == []
        assert {(1, False), (0, False), (2, True)} <= seen_sources
        board.close()
        pico.fixture_step(500, 172, False)  # Receiver continues to drive after Jetson/USB shutdown.
        status = transport._decode(pico.fixture_status(0).rstrip())
        assert (status['active_path'], status['servo_us'], status['esc_us']) == (1, 1000, 2000)
    finally:
        board.close()
        os.close(slave)
        os.close(master)

from types import SimpleNamespace

import pytest

from atd_esc_trials import board as transport
from atd_vehicle_interface import auto_board


def test_command_snapshot_matches_written_wire_and_ignores_failed_writes(monkeypatch):
    sent = []
    stamp = 12_000_000_000
    monkeypatch.setattr(auto_board, 'time_ns', lambda: stamp)
    monkeypatch.setattr(transport, 'time', SimpleNamespace(monotonic=lambda: 3.))

    def write(fd, frame):
        sent.append(frame)
        return len(frame)

    monkeypatch.setattr(transport.os, 'write', write)
    dev = auto_board.AutoBoard(brake_level=.1, brake_hold_s=.2)
    dev.fd = 7
    assert dev.latest_command(stamp, 3_000_000_000) is None
    dev._phase = 'arming'
    dev._write(0., True)
    assert dev.latest_command(stamp, 3_000_000_000) == dict(stamp_ns=stamp, sequence=0, kind='arm',
        steering=0., throttle=0., servo_us=1500, esc_us=1500)
    dev._phase = 'active'
    dev._write(.1754, True, steering=-.1534)
    assert sent[-1].split(b',')[3:-1] == [b'-153', b'175', b'0', b'0', b'3']
    # C の符号付き整数除算は負の奇数で 0 方向。
    assert dev.latest_command(stamp, 3_000_000_000) == dict(stamp_ns=stamp, sequence=1, kind='drive',
        steering=-.153, throttle=.175, servo_us=1424, esc_us=1413)
    dev._write(-.0714, True)
    assert sent[-1].split(b',')[3:-1] == [b'0', b'0', b'0', b'71', b'3']
    assert dev.latest_command(stamp, 3_000_000_000) == dict(stamp_ns=stamp, sequence=2, kind='brake',
        steering=0., throttle=-.071, servo_us=1500, esc_us=1535)
    dev.disarm()
    assert dev.latest_command(stamp, 3_000_000_000) == dict(stamp_ns=stamp, sequence=3, kind='disable',
        steering=0., throttle=0., servo_us=None, esc_us=None)
    assert dev.latest_command(stamp+150_000_000, 3_000_000_000) is None
    assert dev.latest_command(stamp, 3_150_000_000) is None
    monkeypatch.setattr(transport.os, 'write', lambda fd, data: len(data)-1)
    with pytest.raises(RuntimeError):
        dev._write(.6, True)
    assert dev.latest_command(stamp, 3_000_000_000) is None

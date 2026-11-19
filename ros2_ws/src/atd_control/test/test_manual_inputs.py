import pytest

from atd_control.manual_inputs import JOY_MAX_AGE_NS, PROPO_MAX_AGE_NS, JoyStream, joy_reading, match_manual, pico_reading


T = 1_000_000_000


def command(stamp=T, throttle=-.75, **changes):
    sec, nanosec = divmod(stamp, 10**9)
    return dict(header=dict(stamp=dict(sec=sec, nanosec=nanosec)), source='joy', valid=True,
                throttle=throttle, enable_requested=False) | changes


def pico(path):
    msg = dict(header=dict(frame_id='pico'), serial='rev6', instance_id='one', active_path=path, rc_valid=True,
               power_ready=True, sbus_fresh=True, fault_bits=0, sbus_flags=0, sbus_age_ms=5, channels=[990, 1375] + [990]*14)
    return (T, *pico_reading(msg))


@pytest.mark.parametrize('throttle', [-1., -.5, 0., .5, 1.])
def test_signed_input_is_independent_of_enable_and_physical_route(throttle):
    joy = (T, *joy_reading(command(throttle=throttle)))
    for path, source in [(0, 'standby'), (1, 'propo'), (2, 'host_joy'), (3, 'fault'), (99, 'unknown')]:
        sample = match_manual(T, pico(path), joy)
        assert sample['manual_source'] == source
        assert sample['joy_throttle'] == throttle and sample['joy_enable_requested'] is False
        assert sample['joy_age_s'] == 0. and not sample['joy_reason']
        assert sample['propo_raw'] == (1375 if path == 1 else None)
    auto = match_manual(T, pico(2), joy, joy_drives=False)
    assert auto['manual_source'] == 'host' and auto['joy_throttle'] == throttle


@pytest.mark.parametrize('changes', [dict(throttle=float('nan')), dict(throttle=-1.01), dict(throttle=True),
                                    dict(valid=False), dict(source='autonomy'), dict(enable_requested=None)])
def test_invalid_command_never_substitutes_zero(changes):
    sample = match_manual(T, pico(2), (T, *joy_reading(command(**changes))))
    assert sample['manual_source'] == 'host' and sample['joy_reason']
    assert sample['joy_throttle'] is sample['joy_enable_requested'] is sample['joy_age_s'] is None


def test_causal_and_independent_freshness_boundaries():
    joy = (T, *joy_reading(command(enable_requested=True)))
    assert match_manual(T - 1, pico(2), joy)['joy_throttle'] is None
    assert match_manual(T + PROPO_MAX_AGE_NS, pico(2), joy)['manual_source'] == 'host_joy'
    at_limit = match_manual(T + JOY_MAX_AGE_NS, pico(2), joy)
    assert at_limit['manual_source'] == 'unknown' and at_limit['joy_age_s'] == .25 and at_limit['joy_enable_requested']
    expired = match_manual(T + JOY_MAX_AGE_NS + 1, pico(1), joy)
    assert expired['joy_throttle'] is expired['propo_raw'] is None
    assert expired['joy_reason'] and expired['propo_reason']
    assert match_manual(T, pico(1))['manual_source'] == 'propo'
    assert match_manual(T, joy=joy)['manual_source'] == 'unknown'
    assert match_manual(T)['manual_source'] == 'unknown'


@pytest.mark.parametrize('invalid_stamp', [T, 0])
def test_teleop_repeats_and_invalidation_without_retroactive_loss(invalid_stamp):
    stream = JoyStream()
    first = stream.event(command(), T + 10)
    assert first == (T, T, -.75, False, '')
    assert stream.event(command(), T + 20) is None
    invalid = stream.event(command(invalid_stamp, valid=False), T + 30)
    assert invalid[0] == T + 30 and invalid[2:4] == (None, None) and invalid[4]
    assert stream.event(command(invalid_stamp, valid=False), T + 40) is None
    # An old held command or an out-of-order sample behind the barrier cannot reopen the stream.
    assert stream.event(command(), T + 50) is None
    assert stream.event(command(T + 35), T + 55) is None
    assert stream.event(command(T + 56, 0., enable_requested=True), T + 60) == (T + 56, T + 56, 0., True, '')


def test_conflicting_reused_and_future_stamps_do_not_prolong_a_sample():
    stream = JoyStream()
    assert stream.event(command(), T)[2] == -.75
    assert stream.event(command(throttle=1.), T + 1)[2] is None
    assert stream.event(command(), T + 2) is None
    assert stream.event(command(T + 1000), T + 3)[2] is None
    assert stream.event(command(T + 4), T + 5)[2] == -.75

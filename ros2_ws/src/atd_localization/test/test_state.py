from itertools import permutations
from math import sin, cos, pi

import pytest

from atd_localization.state import InitialHintRequest, RigidPose, TrackingState
from concurrent.futures import Future
from types import SimpleNamespace


T = 10**10
I = RigidPose.checked((0., 0., 0.), (0., 0., 0., 1.))
GOOD = {'localized_in_exist_map': 'Yes', 'vo_status': 'OK'}


def camera(state, stamp=T, wall=5.):
    state.clock(stamp)
    for side in ('left', 'right'):
        state.camera(side, stamp, wall, True)


def observe(state, stamp=T, wall=5., order=('vo', 'tf', 'diag')):
    # map<-odom rotated 90 deg, vehicle at +2 m in odom looking left.
    rot = (0., 0., sin(pi/4), cos(pi/4))
    for part in order:
        if part == 'tf':
            state.transform(stamp, wall, RigidPose.checked((1., 2., 0.), rot))
        elif part == 'vo':
            state.odometry(stamp, wall, RigidPose.checked((2., 0., 0.), rot), -.4, .12)
        else:
            state.diagnostic(stamp, wall, GOOD)


@pytest.mark.parametrize('order', list(permutations(('tf', 'vo', 'diag'))))
def test_exact_timestamp_join_composes_position_heading_and_preserves_reverse(order):
    state = TrackingState()
    camera(state)
    state.hint(T - 1)
    observe(state, order=order)
    status, _, observation = state.status(5.)
    assert status == 'tracking' and observation.stamp == T
    assert observation.pose.xyz == pytest.approx((1., 4., 0.))
    assert observation.pose.xyzw == pytest.approx((0., 0., 1., 0.))
    assert observation.speed == -.4 and observation.yaw_rate == .12


def test_no_identity_hint_or_approximate_timestamp_fallback():
    state = TrackingState()
    camera(state)
    observe(state)
    assert state.status(5.)[0] == 'waiting_hint'
    state.hint(T + 1)
    state.transform(T + 1, 5., I)
    state.diagnostic(T + 1, 5., GOOD)
    state.odometry(T + 2, 5., I, 0., 0.)
    assert state.status(5.)[2] is None
    state.odometry(T + 1, 5.1, I, 0., 0.)
    assert state.status(5.1)[0] == 'tracking'


def test_latched_yes_after_loss_is_not_a_new_saved_map_match():
    state = TrackingState()
    camera(state)
    state.hint(T - 1)
    observe(state)
    assert state.status(5.)[0] == 'tracking'
    state.diagnostic(T + 1, 5.1, dict(GOOD, vo_status='Lost'))
    assert state.status(5.1)[0] == 'lost' and state.epoch == 1
    state.hint(T)  # delayed hint predating loss
    state.hint(T + 2)
    observe(state, T + 3, 5.1)
    assert state.status(5.1)[2] is None
    state.diagnostic(T + 4, 5.2, dict(GOOD, localized_in_exist_map='No'))
    observe(state, T + 5, 5.3)
    assert state.status(5.3)[0] == 'tracking' and state.epoch == 1


def test_camera_mismatch_requires_restart_to_reload_cuvslam_intrinsics():
    state = TrackingState()
    camera(state)
    state.hint(T - 1)
    observe(state)
    assert state.status(5.)[0] == 'tracking'
    state.camera('left', T + 1, 5., False)
    assert state.status(5.)[0] == 'lost' and state.epoch == 1
    camera(state, T + 2)
    assert state.status(5.)[0] == 'lost'


def test_camera_drops_and_source_time_or_receipt_staleness():
    state = TrackingState()
    camera(state)
    state.hint(T - 1)
    observe(state)
    assert state.status(5.)[0] == 'tracking'
    assert state.status(6.1)[0] == 'stale'  # stopped /clock, sensor or playback
    camera(state, T + 2_000_000_000, 6.1)
    observe(state, T + 3, 6.1)
    assert state.status(6.1)[0] == 'stale'  # old sensor stamp just received


def test_clock_rewind_clears_registration_and_source_replacement_is_visible():
    state = TrackingState()
    camera(state)
    state.hint(T - 1)
    observe(state)
    assert state.status(5.)[0] == 'tracking'
    camera(state, T - 100)
    observe(state, T - 100)
    assert state.epoch == 1 and state.status(5.)[0] == 'waiting_hint'
    assert state.source('map→odom', b'first')
    assert not state.source('map→odom', b'second')
    assert state.status(5.)[0] == 'lost'


def test_invalid_pose_speed_or_future_stamp_cannot_produce_valid_observation():
    with pytest.raises(ValueError):
        RigidPose.checked((0, 0, 0), (0, 0, 0, 0))
    with pytest.raises(ValueError):
        RigidPose.checked((0, float('nan'), 0), (0, 0, 0, 1))
    state = TrackingState()
    camera(state)
    state.hint(T - 1)
    state.diagnostic(T, 5., GOOD)
    state.transform(T, 5., I)
    state.odometry(T, 5., I, float('nan'), 0.)
    assert state.status(5.)[2] is None
    observe(state, T + 2_000_000_000)
    assert state.status(5.)[2] is None


def test_missed_startup_hint_is_requested_once_when_ready_even_if_hint_was_seen_by_only_one_consumer():
    bootstrap = InitialHintRequest()
    started = []
    def request():
        future = Future()
        started.append(future)
        return future
    bootstrap.tick(0., stop=False, ready=lambda: False, request=request)
    assert not started
    for now in (1., 1.5, 2.):
        bootstrap.tick(now, stop=False, ready=lambda: True, request=request)
    assert len(started) == 1
    started[0].set_result(SimpleNamespace(success=True))
    for now in (3., 10., 100.):
        bootstrap.tick(now, stop=False, ready=lambda: True, request=request)
    assert bootstrap.done and not bootstrap.error and len(started) == 1


def test_startup_hint_retry_is_bounded_even_if_service_hangs_or_fails():
    bootstrap = InitialHintRequest()
    calls = []
    def request():
        future = Future()
        calls.append(future)
        if len(calls) == 2:
            future.set_result(SimpleNamespace(success=False, message='rejected'))
        if len(calls) == 3:
            raise RuntimeError('unavailable')
        return future
    for now in range(40):
        bootstrap.tick(now, stop=False, ready=lambda: True, request=request)
    assert len(calls) == 3 and calls[0].cancelled() and bootstrap.done and bootstrap.error


def test_no_bootstrap_after_success_or_tracking_loss():
    bootstrap = InitialHintRequest()
    for stopped in (True, False):
        bootstrap.tick(10., stop=stopped, ready=lambda: True, request=lambda: pytest.fail('unexpected VGL request'))


def test_initial_request_waits_for_fresh_cuvslam_diagnostics_and_both_camera_infos():
    state = TrackingState()
    camera(state)
    assert not state.initialized_stereo(5.)
    state.diagnostic(T, 5., dict(GOOD, localized_in_exist_map='No'))
    assert state.initialized_stereo(5.)
    assert not state.initialized_stereo(6.1)

from itertools import permutations
from math import sin, cos, pi

import pytest

from atd_localization.state import RigidPose, TrackingState


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


@pytest.mark.parametrize('fps', [30, 90])
@pytest.mark.parametrize('sparse_tf', [False, True])
def test_new_images_publish_once_as_soon_as_ready(fps, sparse_tf):
    state = TrackingState(allow_sparse_tf=sparse_tf)
    camera(state)
    state.hint(T - 1)
    stamps = [T + round(i * 1e9 / fps) for i in range(fps)]
    received = []
    for index, stamp in enumerate(stamps):
        wall = 5. + (stamp - T) / 1e9
        camera(state, stamp, wall)
        order = ('vo', 'tf', 'diag') if index % 2 == 0 else ('diag', 'tf', 'vo')
        for part in order:
            if part == 'tf' and sparse_tf and index != 0:
                continue
            observe(state, stamp, wall, (part,))
            result = state.next_status(wall)
            if result is not None and result[2] is not None:
                received.append(result[2].stamp)
        assert state.next_status(wall, heartbeat=True) is None
    assert received == stamps
    assert state.next_status(wall + 2., heartbeat=True)[0] == 'stale'
    assert state.next_status(wall + 2.) is None
    assert state.next_status(wall + 2., heartbeat=True)[0] == 'stale'


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


def test_sparse_map_corrections_are_causal_and_preserve_observation_time():
    state = TrackingState(allow_sparse_tf=True)
    camera(state)
    state.hint(T - 2)
    state.transform(T - 1, 5., I)  # Pre-registration TF cannot place the vehicle on the map.
    state.diagnostic(T, 5., GOOD)
    state.odometry(T, 5., I, 0., 0.)
    assert state.status(5.)[2] is None
    state.transform(T, 5., RigidPose.checked((3., 0., 0.), I.xyzw))
    assert state.status(5.)[2].pose.xyz == (3., 0., 0.)

    # No new correction for over a second. Neither VO nor the previous pose is
    # restamped while the matching tracking diagnostic is still missing.
    stamp = T + 2_000_000_000
    camera(state, stamp, 7.)
    state.odometry(stamp, 7., RigidPose.checked((1., 0., 0.), I.xyzw), .8, 0.)
    assert state.status(7.)[0] == 'stale' and state.epoch == 0
    state.transform(stamp + 10, 7., RigidPose.checked((9., 0., 0.), I.xyzw))
    state.diagnostic(stamp, 7., GOOD)
    status, _, sample = state.status(7.)
    assert status == 'tracking' and state.epoch == 0
    assert (sample.stamp, sample.pose.xyz, sample.speed) == (stamp, (4., 0., 0.), .8)

    # DDS can deliver a newer correction after the original pose was published.
    state.transform(stamp, 7., RigidPose.checked((4., 0., 0.), I.xyzw))
    for offset, expected_x in ((1, 4.), (10, 9.)):
        state.diagnostic(stamp + offset, 7., GOOD)
        state.odometry(stamp + offset, 7., I, .7, 0.)
        assert state.status(7.)[2].pose.xyz == (expected_x, 0., 0.)


def test_sparse_correction_is_discarded_on_loss_and_clock_rewind():
    state = TrackingState(allow_sparse_tf=True)
    camera(state)
    state.hint(T - 1)
    observe(state)
    assert state.status(5.)[0] == 'tracking'
    state.diagnostic(T + 1, 5., dict(GOOD, vo_status='Lost'))
    assert state.status(7.)[0] == 'lost' and state.epoch == 1
    state.diagnostic(T + 2, 5., dict(GOOD, localized_in_exist_map='No'))
    state.hint(T + 3)
    state.diagnostic(T + 4, 5., GOOD)
    state.odometry(T + 4, 5., I, 0., 0.)
    state.transform(T + 1, 5., I)  # An old queued correction cannot re-arm the recovered session.
    assert state.status(5.)[2] is None
    state.transform(T + 4, 5., I)
    assert state.status(5.)[0] == 'tracking'
    camera(state, T - 99)
    state.hint(T - 100)
    state.diagnostic(T - 99, 5., GOOD)
    state.odometry(T - 99, 5., I, 0., 0.)
    assert state.status(5.)[2] is None


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


@pytest.mark.parametrize('tf_first', [True, False])
def test_vgl_uses_only_matched_causal_corrections_for_current_vo(tf_first):
    state = TrackingState(map_correction_source='vgl')
    stamp = T + 230_000_000  # VGL returns its result while VO is already 230 ms ahead.
    camera(state, stamp)
    state.diagnostic(stamp, 5., {'vo_status': 'OK', 'localized_in_exist_map': 'No'})
    state.odometry(stamp, 5., RigidPose.checked((2., 0., 0.), I.xyzw), .9, .1)
    correction = RigidPose.checked((1., 2., 0.), (0., 0., sin(pi/4), cos(pi/4)))
    if tf_first:
        state.transform(T, 5., correction)
    else:
        state.hint(T)
    assert state.next_status(5.)[2] is None
    if tf_first:
        state.hint(T)
    else:
        state.transform(T, 5., correction)
    sample = state.next_status(5.)[2]
    assert sample.stamp == stamp and sample.pose.xyz == pytest.approx((1., 4., 0.))
    assert sample.speed == .9 and sample.yaw_rate == .1
    assert state.next_status(5., heartbeat=True) is None

    # A match without its TF, an uncorroborated TF, and a future measured
    # correction do not move intervening odometry into a different map pose.
    later = stamp + 3_000_000_000
    camera(state, later, 8.)
    state.transform(later - 10, 8., I)
    state.hint(later - 5)
    state.hint(later + 5)
    state.transform(later + 5, 8., I)
    state.odometry(later, 8., I, 0., 0.)
    assert state.next_status(8.)[2] is None  # The new VO diagnostic is still missing.
    state.diagnostic(later, 8., {'vo_status': 'OK'})
    assert state.next_status(8.)[2].pose == correction
    state.odometry(later + 5, 8., I, 0., 0.)
    state.diagnostic(later + 5, 8., {'vo_status': 'OK'})
    assert state.next_status(8.)[2].pose == I


def test_vgl_loss_rejects_delayed_old_match_and_clock_rewind_clears_anchor():
    state = TrackingState(map_correction_source='vgl')
    camera(state)
    state.transform(T, 5., I)
    state.hint(T)
    state.diagnostic(T, 5., {'vo_status': 'OK'})
    state.odometry(T, 5., I, 0., 0.)
    assert state.status(5.)[0] == 'tracking'
    state.diagnostic(T + 1, 5., {'vo_status': 'Lost'})
    state.hint(T + 2)
    state.diagnostic(T + 3, 5., {'vo_status': 'Lost'})
    state.transform(T + 2, 5., I)
    state.odometry(T + 4, 5., I, 0., 0.)
    state.diagnostic(T + 4, 5., {'vo_status': 'OK', 'localized_in_exist_map': 'Yes'})
    assert state.status(5.)[0] == 'lost' and state.epoch == 1
    state.hint(T + 4)
    assert state.status(5.)[2] is None
    state.transform(T + 4, 5., I)
    assert state.status(5.)[2].stamp == T + 4
    camera(state, T - 100)
    state.diagnostic(T - 100, 5., {'vo_status': 'OK'})
    state.odometry(T - 100, 5., I, 0., 0.)
    assert state.status(5.)[0] == 'waiting_hint' and state.epoch == 2

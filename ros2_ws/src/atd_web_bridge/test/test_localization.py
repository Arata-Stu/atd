from types import SimpleNamespace as NS

from atd_web_bridge.core import Cache
from atd_web_bridge.localization import live_observations, observation


def sample(state='tracking'):
    return NS(state=state, map_id='test', map_sha256='a'*64, hdmap_sha256='b'*64, session_id='launch',
              epoch=3, reason='test', header=NS(stamp=NS(sec=10, nanosec=0), frame_id='map'),
              pose=NS(position=NS(x=2., y=-1., z=0.), orientation=NS(w=1., x=0., y=0., z=0.)),
              speed_mps=-.2, yaw_rate_rps=.3)


def test_live_uses_single_coherent_source_and_expires_it():
    cache = Cache(2.)
    cache.put('pose', dict(x=999))
    assert live_observations(cache, 0, 10.)['pose']['x'] == 999
    cache.put('localization', observation(sample()))
    result = live_observations(cache, 1, 10.5)
    assert result['pose']['x'] == 2. and result['velocity']['linear_mps'] == -.2
    assert result['localization']['remaining_s'] == 1.5
    for n, clock in [(2, 10.), (0, 10.), (1, 12.5), (1, 2.)]:
        stale = live_observations(cache, n, clock)
        assert stale['localization']['state'] == 'stale' and stale['pose'] is None and stale['velocity'] is None
    cache.put('localization', observation(sample('lost')))
    assert live_observations(cache, 1, 10.)['pose'] is None


def test_invalid_pose_or_identity_has_no_readings():
    msg = sample()
    msg.pose.orientation.w = 0
    assert observation(msg)['state'] == 'stale'
    msg = sample()
    msg.hdmap_sha256 = ''
    assert observation(msg)['state'] == 'stale'


def test_live_plan_keeps_its_source_pose_and_invalidates_after_loss_or_expiry():
    from atd_web_bridge.planning import observation as planning_observation, live_plan
    cache = Cache(2.5)
    position = sample()
    position.header.stamp.nanosec = 100_000_000
    cache.put('localization', observation(position))
    loc = live_observations(cache, 1, 10.2)['localization']
    msg = sample()
    msg.pose.position.x = 1.5
    msg.line, msg.valid, msg.validity_s = 'custom', True, .5
    msg.trajectory, msg.trajectory_speeds_mps = [NS(x=1., y=-1.), NS(x=1., y=0.)], [1.4, 1.2]
    msg.progress_m, msg.lateral_error_m, msg.target_speed_mps, msg.estimated_speed_mps = 3.2, -.5, 1.4, -.2
    cache.put('planning', planning_observation(msg))
    live = live_plan(cache, 1, loc, 10.2)
    assert live['valid'] and live['vehicle_pose']['x'] == 1.5
    assert live['trajectory'][0] == [1., -1.] and live['estimated_speed_mps'] == -.2
    for n, current, now in [(0, loc, 10.2), (2, loc, 10.2), (1, dict(loc, state='lost'), 10.2),
                            (1, dict(loc, epoch=4), 10.2), (1, dict(loc, hdmap_sha256='e'*64), 10.2),
                            (1, loc, 10.51), (1, loc, 9.)]:
        result = live_plan(cache, n, current, now)
        assert not result['valid'] and result['trajectory'] == [] and result['target_speed_mps'] is None
    msg.target_speed_mps = float('nan')
    assert not planning_observation(msg)['valid']
    msg.valid, msg.reason = False, 'ラインが未生成'
    assert planning_observation(msg)['reason'] == 'ラインが未生成'


def test_control_target_has_its_own_pose_and_requires_a_matching_fresh_plan():
    from atd_web_bridge.control import live_control, observation as controller_observation
    cache = Cache(2.5)
    msg = sample()
    msg.line, msg.valid, msg.validity_s, msg.controller = 'centerline', True, .5, 'pure_pursuit'
    msg.target, msg.has_steering, msg.preview_limited = NS(x=3., y=-1.), False, False
    msg.steering_rad, msg.wheelbase_m = 0., 0.
    msg.min_lookahead_m, msg.lookahead_time_s, msg.max_lookahead_m = .35, .4, 1.5
    msg.requested_lookahead_m, msg.actual_lookahead_m, msg.curvature_per_m = 1., 1., 0.
    now = 10.1
    status = controller_observation(msg)
    cache.put('control', status)
    plan = dict(status, vehicle_pose=dict(x=2.1, y=-1., yaw=0.), stamp=10.1, remaining_s=.4)
    current = live_control(cache, 1, plan, now)
    assert current['valid'] and current['vehicle_pose']['x'] == 2. and current['target'] == [3., -1.]
    assert current['steering_rad'] is None and current['config']['wheelbase_m'] is None
    assert 0 < current['remaining_s'] <= .4
    for publishers, latest, clock in [
            (0, plan, now), (2, plan, now), (1, None, now), (1, dict(plan, valid=False), now),
            (1, dict(plan, epoch=9), now), (1, dict(plan, session_id='restart'), now),
            (1, dict(plan, hdmap_sha256='c'*64), now), (1, dict(plan, line='custom'), now),
            (1, plan, 10.51), (1, plan, 9.)]:
        expired = live_control(cache, publishers, latest, clock)
        assert not expired['valid'] and expired['target'] is None and expired['curvature_per_m'] is None
    msg.wheelbase_m, msg.has_steering = .25, True
    assert controller_observation(msg)['steering_rad'] == 0.
    msg.target.x = float('nan')
    assert not controller_observation(msg)['valid']
    msg.valid, msg.reason = False, '後退中'
    assert controller_observation(msg)['reason'] == '後退中'


def test_speed_comparison_keeps_real_channels_independent_and_expires_together():
    from atd_web_bridge.speed_control import SpeedWindow, live_speed, observation as speed_observation
    cache = Cache(2.5)
    msg = sample()
    msg.line, msg.validity_s, msg.valid, msg.has_speeds = 'raceline', .5, True, True
    msg.target_speed_mps, msg.estimated_speed_mps, msg.speed_error_mps = 1.5, 1.25, .25
    msg.calibration_run_id, msg.proposal_sha256 = 'calibration-run', 'c'*64
    msg.kp, msg.ki, msg.kd = 40., 8., 1.
    msg.feedforward_raw, msg.p_raw, msg.i_raw, msg.d_raw = 1200., 10., 1., -1.
    msg.calculated_raw, msg.saturated = 1210., False
    msg.has_propo, msg.propo_raw, msg.propo_age_s, msg.propo_reason = True, 1190, .016, ''
    msg.manual_source, msg.has_joy, msg.joy_throttle = 'propo', True, -.43
    msg.joy_age_s, msg.joy_enable_requested, msg.joy_reason = .081, False, ''
    status = speed_observation(msg)
    plan = dict(status, valid=True, remaining_s=.35)
    cache.put('speed_control', status)
    current = live_speed(cache, 1, plan, 10.1)
    assert (current['calculated_raw'], current['propo_raw'], current['remaining_s']) == (1210., 1190, .35)
    assert current['manual_source'] == 'propo' and current['joy_throttle'] == -.43
    assert current['joy_enable_requested'] is False
    msg.valid, msg.reason = False, '校正速度域外'
    cache.put('speed_control', speed_observation(msg))
    partial = live_speed(cache, 1, plan, 10.1)
    assert partial['propo_raw'] == 1190 and partial['target_speed_mps'] == 1.5 and partial['remaining_s'] > 0
    assert partial['calculated_raw'] is None and partial['reason'] == msg.reason
    assert partial['joy_throttle'] == -.43 and partial['manual_source'] == 'propo'
    for sources, latest, now in [(2, plan, 10.1), (1, dict(plan, epoch=4), 10.1), (1, plan, 10.6), (1, None, 10.1)]:
        unavailable = live_speed(cache, sources, latest, now)
        assert unavailable['calculated_raw'] is None and unavailable['propo_raw'] is None
        assert unavailable['joy_throttle'] is None and unavailable['manual_source'] == 'unknown'
    msg.valid, msg.has_propo, msg.propo_reason = True, False, 'HOST 経路'
    msg.manual_source = 'host_joy'
    independent = speed_observation(msg)
    assert independent['calculated_raw'] == 1210. and independent['propo_raw'] is None
    assert independent['propo_reason'] == msg.propo_reason
    assert independent['joy_throttle'] == -.43 and independent['manual_source'] == 'host_joy'

    window = SpeedWindow(cache)
    for i, throttle in enumerate((1190, 1310, 1190)):
        msg.header.stamp.nanosec = i * 10_000_000
        msg.propo_raw = throttle
        msg.has_propo = i < 2
        msg.manual_source = 'propo' if i < 2 else 'host_joy'
        msg.joy_throttle = -.43 if i < 2 else 0.
        window.receive(msg)
    history = window.current(1, plan, 10.12)['history']
    assert [p['propo_raw'] for p in history] == [1190, 1310, None]
    assert [p['joy_throttle'] for p in history] == [-.43, -.43, 0.]
    assert [p['manual_source'] for p in history] == ['propo', 'propo', 'host_joy']
    assert [p['time_s'] for p in history] == [10., 10.01, 10.02]
    msg.proposal_sha256 = 'e'*64
    window.receive(msg)
    assert len(window.current(1, plan, 10.12)['history']) == 1
    assert not window.current(1, plan, 11.)['history']
    for field, bad in [('joy_age_s', .251), ('joy_throttle', float('nan')), ('joy_throttle', 1.1)]:
        malformed = speed_observation(NS(**{**vars(msg), field: bad}))
        assert malformed['joy_throttle'] is None and malformed['manual_source'] == 'host'
        assert malformed['joy_enable_requested'] is None and malformed['calculated_raw'] == 1210.

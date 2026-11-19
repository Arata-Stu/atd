import math

import pytest
from shapely.geometry import Polygon

from atd_api.planning_replay import _core

_core('atd_planning')
_core('atd_control', 'simulation')
from atd_processing.benchmark import WallContact, compare_course, default_controllers, evaluate_course


def circle(radius):
    return [[radius * math.cos(i * math.tau / 120), radius * math.sin(i * math.tau / 120)] for i in range(120)]


def map_document(half_width=.40, speed=1.3):
    points = circle(3.)
    rows = []
    station = 0.
    for i, (x, y) in enumerate(points):
        if i:
            station += math.dist(points[i-1], points[i])
        rows.append([station, x, y, i * math.tau / len(points) + math.pi/2, 1/3, speed, 0.])
    return dict(id='circle', frame='map', closed=True, left=circle(3. - half_width),
                right=circle(3. + half_width), lines={'raceline': dict(points=points, profile=rows)})


def test_paired_trials_lap_metrics_and_detailed_replay_agree():
    doc = map_document()
    # Identical fixed and adaptive settings must yield identical raw trial results.
    config = dict(fixed_lookahead_m=.65, min_lookahead_m=.65, lookahead_time_s=0)
    data = evaluate_course(doc, config, laps=2)
    fixed, adaptive = data['results']
    assert fixed['runs'] == adaptive['runs']
    assert fixed['finish_rate'] == 1 and fixed['finishes'] == fixed['trials'] == 3
    assert fixed['collisions'] == 0 and 14 < fixed['mean_lap_time_s'] < 15.5
    assert fixed['mean_trial_rms_lateral_error_m'] == pytest.approx(sum(
        run['rms_lateral_error_m'] for run in fixed['runs']) / fixed['trials'])
    for run in fixed['runs']:
        assert len(run['lap_times_s']) == 2
        assert 0 <= run['elapsed_s'] - sum(run['lap_times_s']) <= data['model']['dt_s']
        assert run['lap_times_s'][0] > run['lap_times_s'][1]  # First lap starts from rest.
    trace = compare_course(doc, config, trial=2)
    assert trace['course']['left'][0]['x_m'] == doc['left'][0][0]
    assert trace['runs'][0]['lap_times_s'] == fixed['runs'][1]['lap_times_s']
    assert trace['runs'][0]['samples'][-1]['t_s'] == fixed['runs'][1]['elapsed_s']


def test_footprint_and_sweep_detect_contact_with_clear_endpoints():
    polygon = Polygon([(-1,-1), (2,-1), (2,1), (-1,1)], holes=[[(.48,-.04),(.50,-.04),(.50,.04),(.48,.04)]])
    wall = WallContact(polygon)
    assert not wall((0,0,0), (0,0,0))
    assert not WallContact(polygon)((1,0,0), (1,0,0))
    assert wall((0,0,0), (1,0,0))
    # Axle clears the upper wall, but the rotated front of the vehicle touches it.
    assert WallContact(polygon)((0,.85,math.pi/2), (0,.85,math.pi/2))
    assert WallContact(polygon)((0,.905,0), (0,.905,0))


def test_collisions_stay_in_denominator_and_do_not_create_false_laps():
    result = evaluate_course(map_document(.13), laps=1)
    for method in result['results']:
        assert method['finishes'] == 1 and method['collisions'] == 2
        assert method['finish_rate'] == pytest.approx(1/3)
        assert method['mean_lap_time_s'] == method['runs'][0]['lap_times_s'][0]
        for run in method['runs'][1:]:
            assert run['outcome'] == 'collision' and run['collision']['t_s'] == 0
            assert run['lap_times_s'] == []


def test_unreachable_speed_is_controller_failure_and_cancel_propagates():
    result = evaluate_course(map_document(speed=4), laps=1, trials=1)
    for method in result['results']:
        assert method['runs'][0]['outcome'] == 'controller_error'
        assert method['finish_rate'] == 0 and method['collisions'] == 0
        assert method['mean_lap_time_s'] is None

    def cancel():
        raise InterruptedError('cancelled')
    with pytest.raises(InterruptedError):
        evaluate_course(map_document(), check=cancel)


def test_custom_controller_starts_fresh_and_matches_its_reference_on_every_trial():
    builds = []
    defaults = default_controllers()

    def build_copy():
        calls = []
        builds.append(calls)
        fixed = defaults['fixed']()
        def steer(trajectory, **observation):
            calls.append(observation['speed_mps'])
            value = fixed(trajectory, **observation)
            # The benchmark also works with controllers that expose no Pure Pursuit target point.
            return {'valid': value['valid'], 'steering_rad': value['steering_rad']}
        return steer

    config = dict(fixed=defaults['fixed'], independent_copy=build_copy)
    data = evaluate_course(map_document(), controllers=config, laps=1, trials=3)
    fixed, copy = data['results']
    assert [r['mode'] for r in data['results']] == ['fixed', 'independent_copy']
    assert fixed['runs'] == copy['runs']
    assert len(builds) == 3 and all(len(calls) > 1 and calls[0] == 0 for calls in builds)


@pytest.mark.parametrize('options', [{'laps': 0}, {'trials': 4}, {'trials': False}])
def test_invalid_counts_are_rejected(options):
    with pytest.raises(ValueError):
        evaluate_course(map_document(), **options)

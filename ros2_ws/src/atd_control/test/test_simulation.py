import math
import unittest

from atd_control.simulation import _Vehicle, _drive_interval, compare


class SimulationTests(unittest.TestCase):
    def test_default_comparison_travels_two_laps_and_uses_measured_speed_for_lookahead(self):
        data = compare({})
        fixed, adaptive = data['runs']
        assert [fixed['mode'], adaptive['mode']] == ['fixed', 'adaptive']
        origin = fixed['samples'][0]
        for run in (fixed, adaptive):
            samples = run['samples']
            assert run['completed'] and not run['reason']
            assert run['metrics']['laps_completed'] == 2
            assert samples[-1]['progress_m'] >= 2 * data['course']['length_m']
            assert samples[0]['t_s'] == samples[0]['speed_mps'] == 0.
            assert all(samples[0][k] == origin[k] for k in ('x_m', 'y_m', 'yaw_rad'))
            assert all(a['t_s'] < b['t_s'] for a, b in zip(samples, samples[1:]))
            assert all(0 <= p['throttle'] <= 1 and abs(p['steering_deg']) <= 28 for p in samples)
            assert max(p['target_speed_mps'] for p in samples) - min(p['target_speed_mps'] for p in samples) > .5
        assert all(p['lookahead_m'] == .65 for p in fixed['samples'])
        assert all(math.isclose(p['lookahead_m'], min(1.5, .35 + .4 * p['speed_mps']))
                   for p in adaptive['samples'])
        assert math.dist([fixed['samples'][20][k] for k in ('x_m', 'y_m')],
                         [adaptive['samples'][20][k] for k in ('x_m', 'y_m')]) > .001

    def test_matching_lookahead_gives_identical_independent_runs(self):
        result = compare(dict(fixed_lookahead_m=.51, min_lookahead_m=.51, lookahead_time_s=0,
                              steering_delay_ms=53, throttle_delay_ms=38))
        fixed, adaptive = result['runs']
        assert fixed['completed'] and adaptive['completed']
        assert fixed['samples'] == adaptive['samples']
        assert fixed['metrics'] == adaptive['metrics']

    def test_axes_apply_fractional_transport_delays_before_actuator_response(self):
        vehicle = _Vehicle(0., 0., 0.)
        pending = []
        settings = dict(steering_delay_ms=25., throttle_delay_ms=12.5, steering_tau_s=.12, speed_tau_s=.5)
        # この小さい入力では加速度制限内に収まり、一次遅れの解析解と比較できる。
        _drive_interval(vehicle, pending, 0, .2, .08, settings)
        assert vehicle.steering == 0.
        assert math.isclose(vehicle.speed, .08 * 3.2 * (1 - math.exp(-.0075 / .5)))
        _drive_interval(vehicle, pending, 20_000_000, .2, .08, settings)
        assert math.isclose(vehicle.steering, .2 * (1 - math.exp(-.015 / .12)))
        assert math.isclose(vehicle.speed, .08 * 3.2 * (1 - math.exp(-.0275 / .5)))

    def test_incomplete_run_reports_reason_and_does_not_extend_trajectory(self):
        data = compare(dict(fixed_lookahead_m=.15, steering_delay_ms=1000))
        run = data['runs'][0]
        assert not run['completed'] and run['reason']
        assert run['metrics']['laps_completed'] == 0
        assert run['metrics']['steering_saturation_s'] > 0
        assert 0 <= run['metrics']['elapsed_s'] - run['samples'][-1]['t_s'] <= data['model']['dt_s'] + 1e-8
        assert all(math.isfinite(value) for p in run['samples'] for value in p.values())

    def test_invalid_configuration_is_rejected_before_any_run(self):
        for config in ({'fixed_lookahead_m': False}, {'lookahead_time_s': float('nan')},
                       {'max_lookahead_m': float('inf')}, {'max_lookahead_m': .2, 'min_lookahead_m': .4},
                       {'throttle_delay_ms': -1}, {'steering_delay_ms': '5'}, {'speed_tau_s': 0},
                       {'unknown': 1}, [], None):
            with self.subTest(config=config), self.assertRaises(ValueError):
                compare(config)

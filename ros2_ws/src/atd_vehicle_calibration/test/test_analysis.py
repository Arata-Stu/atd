"""Synthetic response/synchronization tests. No ROS/bag/hardware is exercised."""
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from atd_vehicle_calibration.analysis import analyze


def pico(stamp, **changes):
    result = dict(stamp=stamp, segment=0, instance_id='usb_session', serial='pico_serial',
                  steering_raw=300, throttle_raw=400, rc_valid=True, manual_valid=True,
                  active_path=1, power_ready=True, sbus_fresh=True, sbus_age_ms=0, sbus_flags=0)
    result.update(changes)
    return result


def vo(stamp, *, vx=1.0, ax=0.0, yaw=0.0, **changes):
    result = dict(stamp=stamp, segment=0, velocity_body=[vx, 0.0, 0.0], acceleration_body=[ax, 0.0, 0.0],
                  yaw_rate_rps=yaw, orientation=[0.0, 0.0, 0.0, 1.0])
    result.update(changes)
    return result


def inputs(times, **control):
    return dict(pico=[pico(stamp, **control) for stamp in times], imu=[], metadata={}, provenance={}, rejected=[])


def trajectory(times, **motion):
    return dict(samples=[vo(stamp, **motion) for stamp in times], rejected=[], provenance={})


class AnalysisTest(unittest.TestCase):
    def test_observed_raw_input_does_not_assume_center_or_forward_sign(self):
        rows, summary = analyze(inputs([1.0], steering_raw=40, throttle_raw=25), trajectory([1.0], vx=2, ax=.8, yaw=.4))
        row = rows[0]
        self.assertTrue(row['valid'])
        self.assertEqual(row['throttle_raw'], 25)
        self.assertEqual(row['longitudinal_regime'], 'accelerating')
        self.assertAlmostEqual(row['curvature_inv_m'], .2)
        self.assertIsNone(row['equivalent_steering_rad'])
        self.assertIsNone(row['imu_ax_at_sensor_mps2'])
        self.assertFalse(summary['method']['actuator_model_fitted'])
        self.assertFalse(summary['method']['input_sign_or_endpoints_assumed'])
        self.assertEqual(len(summary['responses']['throttle_acceleration']), 1)

    def test_control_is_causal_not_nearest_future_input(self):
        bag = dict(pico=[pico(1.0, throttle_raw=100), pico(1.04, throttle_raw=1000)], imu=[])
        rows, _ = analyze(bag, trajectory([1.02]))
        self.assertEqual(rows[0]['throttle_raw'], 100)
        self.assertAlmostEqual(rows[0]['pico_age_s'], .02)
        rows, _ = analyze(bag, trajectory([.99, 1.1]))
        self.assertEqual([row['reason'] for row in rows], ['pico_not_synchronized'] * 2)
        self.assertEqual(rows[0]['vo_vx_mps'], 1.0)

    def test_manual_invalid_excludes_response_but_preserves_independent_vo(self):
        bag = inputs([1.0, 1.05, 1.1])
        bag['pico'][1]['manual_valid'] = False
        rows, summary = analyze(bag, trajectory([1.0, 1.05, 1.1], ax=.5, yaw=.2))
        self.assertEqual([row['valid'] for row in rows], [True, False, True])
        self.assertTrue(rows[1]['motion_valid'])
        self.assertEqual(rows[1]['vo_ax_mps2'], .5)
        self.assertEqual(rows[1]['vo_yaw_rate_rps'], .2)
        self.assertEqual(rows[1]['reason'], 'manual_input_invalid')
        self.assertNotEqual(rows[0]['segment'], rows[2]['segment'])
        self.assertEqual(summary['coverage']['valid_samples'], 2)
        self.assertEqual(summary['coverage']['motion_valid_samples'], 3)
        self.assertEqual(sum(cell['samples'] for cell in summary['responses']['throttle_acceleration']), 2)

    def test_defensive_board_conditions_fail_closed(self):
        changes = [dict(rc_valid=False), dict(connected=False), dict(active_path=2), dict(sbus_flags=4),
                   dict(sbus_flags=8), dict(sbus_age_ms=51), dict(fault_bits=1), dict(power_ready=False), dict(sbus_fresh=False),
                   dict(steering_raw=-1), dict(throttle_raw=2048), dict(throttle_raw=True), dict(segment=None)]
        for change in changes:
            with self.subTest(change=change):
                rows, summary = analyze(inputs([1.0], **change), trajectory([1.0]))
                self.assertFalse(rows[0]['valid'])
                self.assertEqual(summary['responses']['throttle_acceleration'], [])

    def test_near_zero_division_is_unknown_and_reverse_keeps_sign(self):
        bag = inputs([1.0, 1.1])
        motion = dict(samples=[vo(1.0, vx=.01, yaw=.4), vo(1.1, vx=-2, ax=-.5, yaw=.4)])
        rows, _ = analyze(bag, motion, wheelbase_m=.25)
        self.assertIsNone(rows[0]['curvature_inv_m'])
        self.assertIsNone(rows[0]['equivalent_steering_rad'])
        self.assertAlmostEqual(rows[1]['curvature_inv_m'], -.2)
        self.assertAlmostEqual(rows[1]['equivalent_steering_rad'], math.atan(-.05))
        self.assertEqual(rows[1]['longitudinal_regime'], 'accelerating')
        self.assertEqual(rows[1]['turn_regime'], 'left')

    def test_clock_gap_and_independent_segment_ids_split_analysis(self):
        bag = inputs([1.0, 1.1, 1.5, 1.6], segment=91)
        motion = trajectory([1.0, 1.1, 1.5, 1.6], segment=5)
        rows, summary = analyze(bag, motion)
        self.assertTrue(all(row['valid'] for row in rows))
        self.assertEqual(rows[0]['segment'], rows[1]['segment'])
        self.assertNotEqual(rows[1]['segment'], rows[2]['segment'])
        self.assertEqual(rows[2]['segment'], rows[3]['segment'])
        self.assertEqual(len(summary['segments']), 2)

    def test_motion_clock_rollback_remains_invalid_until_prior_high_watermark(self):
        bag = inputs([1.0, 1.05, 1.1, 1.15, 1.2])
        rows, _ = analyze(bag, trajectory([1.1, 1.05, 1.08, 1.2]))
        self.assertEqual([row['valid'] for row in rows], [True, False, False, True])
        self.assertEqual(rows[1]['reason'], 'motion_clock_regression')
        self.assertFalse(rows[1]['motion_valid'])
        self.assertIsNone(rows[1]['vo_vx_mps'])

    def test_overlapping_pico_clock_epochs_are_ambiguous(self):
        bag = dict(pico=[pico(1.0), pico(1.1), pico(1.05, segment=1), pico(1.15, segment=1)], imu=[])
        rows, _ = analyze(bag, trajectory([1.075]))
        self.assertFalse(rows[0]['valid'])
        self.assertEqual(rows[0]['reason'], 'ambiguous_pico_clock_epoch')
        self.assertEqual(rows[0]['vo_vx_mps'], 1.0)

    def test_new_monotonic_session_does_not_hold_prior_session_into_it(self):
        bag = dict(pico=[pico(1.0), pico(1.02, segment=1, instance_id='new')], imu=[])
        rows, _ = analyze(bag, trajectory([1.0, 1.02]))
        self.assertTrue(all(row['valid'] for row in rows))
        self.assertNotEqual(rows[0]['segment'], rows[1]['segment'])
        self.assertEqual(rows[1]['pico_segment'], 1)

    def test_imu_uses_same_pico_segment_and_reports_time_offset(self):
        bag = inputs([1.0, 1.04], segment=7)
        bag['imu'] = [dict(stamp=1.02, segment=8, kind='gyro', vector_body=[0, 0, 20]),
                      dict(stamp=1.03, segment=7, kind='gyro', vector_body=[0, 0, .3]),
                      dict(stamp=1.01, segment=7, kind='accel', vector_body=[2, 0, 9.80665])]
        rows, summary = analyze(bag, trajectory([1.02], yaw=.2), gravity_world_mps2=[0, 0, -9.80665])
        self.assertAlmostEqual(rows[0]['imu_gyro_z_rps'], .3)
        self.assertAlmostEqual(rows[0]['gyro_minus_vo_yaw_rate_rps'], .1)
        self.assertAlmostEqual(rows[0]['imu_gyro_dt_s'], .01)
        self.assertAlmostEqual(rows[0]['imu_ax_at_sensor_mps2'], 2)
        self.assertEqual(summary['coverage']['gravity_corrected_samples'], 1)

    def test_gravity_requires_explicit_world_vector_and_respects_attitude(self):
        bag = inputs([1.0])
        bag['imu'] = [dict(stamp=1.0, segment=0, kind='accel', vector_body=[-9.80665, 0, 0])]
        motion = trajectory([1.0], orientation=[0, math.sqrt(.5), 0, math.sqrt(.5)])
        rows, _ = analyze(bag, motion)
        self.assertEqual(rows[0]['imu_accel_x_specific_mps2'], -9.80665)
        self.assertIsNone(rows[0]['imu_ax_at_sensor_mps2'])
        rows, _ = analyze(bag, motion, gravity_world_mps2=[0, 0, -9.80665])
        self.assertAlmostEqual(rows[0]['imu_ax_at_sensor_mps2'], 0, places=10)

    def test_missing_or_nonmonotonic_imu_is_unknown(self):
        bag = inputs([1.0, 1.1])
        bag['imu'] = [dict(stamp=1.1, segment=0, kind='gyro', vector_body=[0, 0, 1]),
                      dict(stamp=1.0, segment=0, kind='gyro', vector_body=[0, 0, 2])]
        rows, summary = analyze(bag, trajectory([1.0, 1.1]))
        self.assertTrue(all(row['imu_gyro_z_rps'] is None for row in rows))
        self.assertEqual(summary['coverage']['imu_preprocessing_rejected']['imu_clock_regression'], 1)
        self.assertTrue(all(row['valid'] for row in rows))

    def test_steady_input_plateaus_split_despite_same_motion_regime(self):
        times = [1 + index * .1 for index in range(12)]
        bag = inputs(times)
        for index, sample in enumerate(bag['pico']):
            sample['throttle_raw'] = 300 if index < 6 else 800
        rows, summary = analyze(bag, trajectory(times, vx=1.5), min_regime_duration_s=.4)
        self.assertEqual(len(summary['episodes']), 2)
        self.assertTrue(all(episode['sustained'] and episode['stable_raw'] for episode in summary['episodes']))
        cells = summary['responses']['throttle_steady_speed']
        self.assertEqual(len(cells), 2)
        self.assertEqual(sum(cell['samples'] for cell in cells), 12)
        self.assertTrue(all(row['sustained'] for row in rows))

    def test_short_plateaus_do_not_estimate_steady_speed(self):
        _, summary = analyze(inputs([1.0, 1.1]), trajectory([1.0, 1.1]), min_regime_duration_s=.5)
        self.assertEqual(summary['responses']['throttle_steady_speed'], [])
        self.assertEqual(summary['responses']['throttle_acceleration'][0]['samples'], 2)

    def test_derivative_window_cannot_cross_unknown_control_history(self):
        bag = inputs([1.0, 1.1, 1.2])
        motion = dict(samples=[vo(1.0, window_start=.9, window_end=1.1),
                               vo(1.1, window_start=1.0, window_end=1.2)])
        rows, _ = analyze(bag, motion)
        self.assertEqual(rows[0]['reason'], 'motion_window_crosses_control_segment')
        self.assertEqual(rows[0]['vo_vx_mps'], 1)
        self.assertTrue(rows[1]['valid'])

    def test_empty_unobserved_and_invalid_values_are_never_zero_filled(self):
        rows, summary = analyze(dict(pico=[], imu=[]), dict(samples=[]))
        self.assertEqual(rows, [])
        self.assertIsNone(summary['coverage']['speed_mps']['mean'])
        self.assertTrue(all(not table for table in summary['responses'].values()))
        self.assertIn('null/unknown', summary['binning']['unobserved_cells'])
        rows, _ = analyze(inputs([1.0, 1.1]), dict(samples=[vo(1.0, tracking_valid=False), vo(1.1, vx=math.nan)]))
        self.assertEqual(rows[0]['reason'], 'tracking_invalid')
        self.assertIsNone(rows[1]['vo_vx_mps'])
        json.dumps(rows, allow_nan=False)

    def test_endpoints_and_speed_bins_are_explicit_without_extrapolation(self):
        bag = dict(pico=[pico(1.0, throttle_raw=0, steering_raw=2047)], imu=[])
        rows, summary = analyze(bag, trajectory([1.0], vx=20, yaw=-1))
        cell = summary['responses']['throttle_acceleration'][0]
        self.assertEqual((cell['raw_lo'], cell['raw_hi']), (0, 63))
        self.assertEqual(cell['speed_lo_mps'], 8.0)
        self.assertIsNone(cell['speed_hi_mps'])
        self.assertEqual(summary['responses']['steering_response'][0]['raw_hi'], 2047)
        self.assertEqual(rows[0]['turn_regime'], 'right')
        for value in rows[0].values():
            self.assertIsInstance(value, (int, float, str, bool, type(None)))
        json.dumps(summary, allow_nan=False)

    def test_held_out_cell_mean_comparison_reports_units_and_uncovered(self):
        _, reference = analyze(inputs([1.0]), trajectory([1.0], ax=1))
        _, result = analyze(inputs([2.0]), trajectory([2.0], ax=2), reference_summary=reference)
        validation = result['validation']['tables']['throttle_acceleration']
        self.assertEqual(validation['total_samples'], 1)
        self.assertEqual(validation['covered_samples'], 1)
        self.assertEqual(validation['out_of_range_samples'], 0)
        self.assertEqual(validation['rmse'], 1)
        self.assertEqual(validation['rmse_unit'], 'm/s^2')
        _, result = analyze(inputs([2.0], throttle_raw=800), trajectory([2.0], ax=2), reference_summary=reference)
        validation = result['validation']['tables']['throttle_acceleration']
        self.assertEqual(validation['out_of_range_samples'], 1)
        self.assertIsNone(validation['rmse'])
        self.assertFalse(result['method']['actuator_model_fitted'])

    def test_reference_requires_matching_conditions(self):
        _, reference = analyze(inputs([1.0]), trajectory([1.0]))
        for changes in (dict(raw_bin_width=32), dict(max_sync_s=.04), dict(wheelbase_m=.25), dict(gravity_world_mps2=[0, 0, -9.8])):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                analyze(inputs([2.0]), trajectory([2.0]), reference_summary=reference, **changes)
        changed = deepcopy(reference)
        changed['source']['motion_provenance']['window_s'] = .5
        with self.assertRaises(ValueError):
            analyze(inputs([2.0]), trajectory([2.0]), reference_summary=changed)

    def test_invalid_configuration_is_rejected(self):
        for changes in (dict(wheelbase_m=0), dict(max_sync_s=math.nan), dict(max_gap_s=.01), dict(raw_bin_width=True),
                        dict(speed_bin_edges_mps=[0, 1, .5]), dict(speed_bin_edges_mps=[1, 2]),
                        dict(gravity_world_mps2=[0, 0, math.inf]), dict(raw_stability_tolerance=-1)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                analyze(inputs([1.0]), trajectory([1.0]), **changes)

    def test_camera_pose_motion_and_analysis_preserve_rear_axle_response(self):
        from atd_vehicle_calibration.inputs import rotate
        from atd_vehicle_calibration.motion import estimate_motion
        times = [1000.0 + index * .02 for index in range(101)]
        mount = dict(parent='base_link', child='camera_link', translation=[.3, .08, .1], rotation=[0, 0, 0, 1])
        poses = []
        for stamp in times:
            t = stamp - times[0]
            orientation = [0, 0, math.sin(t / 2), math.cos(t / 2)]
            center = [2 * math.sin(t), 2 * (1 - math.cos(t)), 0]
            position = [a + b for a, b in zip(center, rotate(mount['translation'], orientation))]
            poses.append(dict(stamp=stamp, position=position, orientation=orientation))
        motion = estimate_motion(poses, pose_frame='camera_link', static_transforms=[mount])
        bag = inputs(times, steering_raw=30, throttle_raw=100)
        bag['imu'] = [dict(stamp=stamp, segment=0, kind='gyro', vector_body=[0, 0, 1]) for stamp in times]
        rows, summary = analyze(bag, motion, wheelbase_m=.3)
        self.assertGreater(summary['coverage']['valid_samples'], 80)
        for row in rows:
            if row['valid']:
                self.assertAlmostEqual(row['vo_vx_mps'], 2, places=4)
                self.assertAlmostEqual(row['vo_ax_mps2'], 0, places=2)
                self.assertAlmostEqual(row['curvature_inv_m'], .5, places=4)
                self.assertAlmostEqual(row['equivalent_steering_rad'], math.atan(.15), places=4)
                self.assertAlmostEqual(row['gyro_minus_vo_yaw_rate_rps'], 0, places=4)
        self.assertEqual(summary['coverage']['turn_regimes'], {'left': summary['coverage']['valid_samples']})


if __name__ == '__main__':
    unittest.main()

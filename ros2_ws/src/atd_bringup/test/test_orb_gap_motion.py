import json
import math
import unittest

from atd_bringup.orb_gap_motion import evaluate_gaps


def sample(x, y=0., yaw=0.):
    return dict(est_x_m=x, est_y_m=y, est_yaw_rad=yaw, gt_x_m=x, gt_y_m=y, gt_yaw_rad=yaw)


class OrbGapMotionTest(unittest.TestCase):
    def setUp(self):
        self.stamps = [0, 100_000_000, 400_000_000]
        self.epochs = {0: 0, 100_000_000: 0, 400_000_000: 1}
        end = sample(2*math.sin(.15), 2*(1-math.cos(.15)))
        # Reacquired pose is deliberately wrong. None of the predictions may use it.
        end.update(est_x_m=42., est_yaw_rad=math.pi)
        self.samples = dict(zip(self.stamps, [sample(-.1), sample(0.), end]))
        self.imu = list(range(0, 410_000_000, 10_000_000))

    def test_measured_turn_and_time_boundaries(self):
        # 1 m/s. Two opposite turns of 0.15 s each, with yaw back at zero at the end.
        rates = [1. if stamp < 250_000_000 else -1. for stamp in self.imu]
        rates[-1] = 80.  # The sample arriving at the end has not driven motion yet.
        result = evaluate_gaps(self.stamps, self.epochs, self.samples, self.imu, rates)
        self.assertEqual((result['gap_count'], result['paired_count']), (1, 1))
        gap = result['longest_gap']
        self.assertEqual(gap['history_count'], 2)
        self.assertAlmostEqual(gap['forward_mps'], 1.)
        self.assertAlmostEqual(gap['errors']['gyro']['position_error_m'], 0.)
        self.assertAlmostEqual(gap['errors']['gyro']['abs_heading_error_deg'], 0.)
        self.assertGreater(gap['errors']['straight']['position_error_m'], .02)
        self.assertAlmostEqual(result['paired_metrics']['hold']['median_position_error_m'],
                               gap['errors']['hold']['position_error_m'])
        json.dumps(result, allow_nan=False)

    def test_gyro_gap_or_end_outside_support_excludes_all_methods_from_paired_metrics(self):
        for imu in (self.imu[:-1], self.imu[11:], self.imu[:20] + self.imu[22:]):
            with self.subTest(imu=imu):
                result = evaluate_gaps(self.stamps, self.epochs, self.samples, imu, [1.]*len(imu))
                self.assertEqual(result['paired_count'], 0)
                self.assertEqual(result['longest_gap']['status'], 'insufficient_gyro_coverage')
                self.assertIn('turn', result['longest_gap']['errors'])
                self.assertIsNone(result['paired_metrics']['hold']['median_position_error_m'])

    def test_history_stops_at_missing_sample_or_epoch_change(self):
        # An older pose in the same epoch cannot bridge an unknown intervening observation.
        stamps = [0, 50_000_000, 100_000_000, 400_000_000]
        for epochs, samples in (({**self.epochs, 50_000_000: 0}, self.samples),
                                ({**self.epochs, 50_000_000: 3}, {**self.samples, 50_000_000: sample(-.05)})):
            result = evaluate_gaps(stamps, epochs, samples, self.imu, [1.]*len(self.imu))
            gap = result['longest_gap']
            self.assertEqual((gap['history_count'], gap['status']), (1, 'insufficient_history'))
            self.assertEqual(set(gap['errors']), {'hold'})

    def test_sparse_ground_truth_does_not_create_pose_gaps(self):
        stamps = [0, 250_000_000, 400_000_000]
        result = evaluate_gaps(stamps, self.epochs, self.samples, [], [])
        self.assertEqual(result['gap_count'], 0)
        self.assertIsNone(result['longest_gap'])
        # A genuine gap without a ground-truth endpoint is retained and labelled.
        result = evaluate_gaps(self.stamps, self.epochs, {0: sample(0)}, [], [])
        self.assertEqual(result['longest_gap']['status'], 'missing_gt_sample')

    def test_multiple_gaps_only_use_common_supported_intervals(self):
        later = 800_000_000
        stamps = self.stamps + [later]
        result = evaluate_gaps(stamps, self.epochs, {**self.samples, later: sample(800.)}, self.imu,
                               [0.] * len(self.imu))
        self.assertEqual((result['gap_count'], result['paired_count']), (2, 1))
        self.assertEqual(result['longest_gap']['end_stamp_ns'], later)
        for method in result['paired_metrics'].values():
            self.assertEqual(method['count'], 1)

    def test_crossing_pi_preserves_signed_yaw_and_reverse_speed(self):
        # The vehicle reverses along a circular arc while yaw passes from +pi to -pi.
        yaw0 = math.pi - .05
        def arc(t):
            return sample(math.sin(yaw0) - math.sin(yaw0+t),
                          math.cos(yaw0+t) - math.cos(yaw0), math.remainder(yaw0+t, math.tau))
        samples = {s: arc(s*1e-9) for s in self.stamps}
        gap = evaluate_gaps(self.stamps, self.epochs, samples, self.imu, [1.]*len(self.imu))['longest_gap']
        self.assertAlmostEqual(gap['forward_mps'], -1.)
        self.assertAlmostEqual(gap['yaw_rate_radps'], 1.)
        self.assertAlmostEqual(gap['errors']['turn']['position_error_m'], 0.)

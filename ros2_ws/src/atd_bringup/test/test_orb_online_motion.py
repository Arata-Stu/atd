import math
import unittest

from atd_bringup.orb_gap_motion import evaluate_gaps
from atd_bringup.orb_online_motion import GyroGapMotion


MS = 1_000_000


class OrbOnlineMotionTest(unittest.TestCase):
    def setUp(self):
        self.motion = GyroGapMotion()

    def anchor(self):
        self.motion.observe(0, -.1, 0., 0., 1)
        self.motion.observe(100*MS, 0., 0., 0., 1)

    def test_online_motion_matches_offline_and_ignores_future_rates(self):
        self.anchor()
        stamps = list(range(95*MS, 410*MS, 10*MS))
        rates = [1. if t < 245*MS else -1. for t in stamps]
        rates[-1] = 100.
        samples = {}
        for stamp, x in ((0, -.1), (100*MS, 0.), (400*MS, 0.)):
            samples[stamp] = dict(est_x_m=x, est_y_m=0., est_yaw_rad=0., gt_x_m=0., gt_y_m=0., gt_yaw_rad=0.)
        offline = evaluate_gaps(list(samples), {0: 1, 100*MS: 1, 400*MS: 2}, samples, stamps, rates)
        for t, rate in zip(stamps, rates):
            self.motion.add_gyro(t, rate)
        # 実際に配信する画像時刻で積分した後、再取得した画像と同じ時刻で照合する。
        for t in (131, 195, 299, 350, 400):
            output, reason = self.motion.predict(t*MS)
            self.assertEqual(reason, 'predicted')
        metric = offline['longest_gap']['errors']['gyro']
        self.assertAlmostEqual(math.hypot(output.x, output.y), metric['position_error_m'])
        self.assertAlmostEqual(abs(math.degrees(output.yaw)), metric['abs_heading_error_deg'])
        self.assertAlmostEqual(output.speed_mps, 1.)
        self.assertAlmostEqual(output.age_s, .3)
        self.assertEqual(output.yaw_rate_radps, -1.)

    def test_hold_limit_and_missing_middle_sample(self):
        self.anchor()
        for t in (97, 105, 115, 150, 160):
            self.motion.add_gyro(t*MS, 0.)
        self.assertEqual(self.motion.predict(135*MS)[1], 'predicted')
        self.assertEqual(self.motion.predict(135*MS+1)[1], 'insufficient_gyro_coverage')
        self.assertEqual(self.motion.predict(160*MS)[1], 'insufficient_gyro_coverage')
        self.motion.add_gyro(135*MS, 0.)
        result, reason = self.motion.predict(160*MS)
        self.assertEqual(reason, 'predicted')
        self.assertAlmostEqual(result.x, .06)

    def test_gyro_and_frame_arrival_reordering_recomputes_from_measurements(self):
        self.anchor()
        self.motion.add_gyro(90*MS, 0.)
        self.assertEqual(self.motion.predict(110*MS)[0].yaw, 0.)
        self.motion.add_gyro(105*MS, 2.)  # 前回の終点より遅れて到着したサンプル
        exact, _ = self.motion.predict(105*MS)
        self.assertEqual((exact.yaw, exact.yaw_rate_radps), (0., 2.))
        self.assertAlmostEqual(self.motion.predict(113*MS)[0].yaw, .016)
        self.assertAlmostEqual(self.motion.predict(107*MS)[0].yaw, .004)
        self.assertFalse(self.motion.add_gyro(105*MS, -8.))
        self.assertFalse(self.motion.add_gyro(106*MS, math.nan))
        self.assertFalse(self.motion.observe(100*MS, 20., 20., 1., 1))
        self.assertAlmostEqual(self.motion.predict(113*MS)[0].yaw, .016)

    def test_clock_epoch_history_and_horizon_boundaries(self):
        self.motion = GyroGapMotion(max_gap_s=.125)
        self.assertEqual(self.motion.predict(MS)[1], 'no_observation')
        self.anchor()
        self.assertEqual(self.motion.predict(100*MS)[1], 'not_after_observation')
        self.assertEqual(self.motion.predict(225*MS+1)[1], 'horizon_exceeded')
        for t in range(100, 226, 5):
            self.motion.add_gyro(t*MS, 0.)
        self.assertAlmostEqual(self.motion.predict(225*MS)[0].x, .125)
        self.motion.observe(225*MS, 5., 0., 0., 2)
        self.assertEqual(self.motion.predict(230*MS)[1], 'insufficient_history')
        self.motion.observe(235*MS, 4.99, 0., 0., 2)
        output, _ = self.motion.predict(240*MS)
        self.assertAlmostEqual(output.speed_mps, -1.)
        self.assertAlmostEqual(output.x, 4.985)
        self.motion.observe(436*MS, 1., 0., 0., 2)
        self.assertEqual(self.motion.predict(446*MS)[1], 'insufficient_history')
        self.motion.reset()
        self.anchor()
        self.assertEqual(self.motion.predict(101*MS)[1], 'insufficient_gyro_coverage')

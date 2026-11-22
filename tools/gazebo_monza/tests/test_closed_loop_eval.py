"""Finish-line and body-boundary tests independent of ROS or rendering."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from circuits import load_lap
from closed_loop_eval import CourseEvaluation


class ClosedLoopScoringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.boundaries, cls.route, _ = load_lap('odaiba')

    def test_two_real_odaiba_laps_cross_all_sectors_and_timing_line(self):
        evaluation = CourseEvaluation(self.boundaries, self.route, 2)
        # Start at the timing line, with a trace interval deliberately not aligned to the finish.
        samples = self.route[:-1:2]
        samples = samples + samples + samples[:2]
        for i, (_, x, y, yaw) in enumerate(samples):
            result = evaluation.observe(10 + i * .1, x, y, yaw)
            if result:
                break
        self.assertEqual(result['status'], 'completed')
        self.assertEqual(result['laps_completed'], 2)
        self.assertAlmostEqual(sum(result['lap_times_s']), result['finish_sim_s'] - 10)
        self.assertAlmostEqual(result['distance_m'], 2 * self.route[-1][0], delta=.45)

    def test_reverse_lap_and_rocking_across_timing_line_do_not_count(self):
        evaluation = CourseEvaluation(self.boundaries, self.route, 1)
        samples = self.route[-1::-3] + self.route[:3] + [self.route[-2], self.route[1]] * 4
        for i, (_, x, y, yaw) in enumerate(samples):
            self.assertIsNone(evaluation.observe(i * .1, x, y, yaw))
        self.assertEqual(evaluation.lap_times, [])

    def test_front_corner_contact_and_skipping_across_an_island(self):
        for y, yaw in ((.63, 3.14159), (4., 3.14159)):
            with self.subTest(y=y):
                evaluation = CourseEvaluation(self.boundaries, self.route, 1)
                self.assertIsNone(evaluation.observe(1., 0., 0., yaw))
                self.assertEqual(evaluation.observe(1.2, 0., y, yaw)['status'], 'off_course')

    def test_nonmonotonic_or_invalid_gt_never_reports_finish(self):
        for invalid in ((1., 0.), (2., float('nan'))):
            evaluation = CourseEvaluation(self.boundaries, self.route, 1)
            self.assertIsNone(evaluation.observe(1., 0., 0., 3.14159))
            self.assertEqual(evaluation.observe(*invalid, 0., 3.14159)['status'], 'invalid_trace')

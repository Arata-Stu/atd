import json
import math
from pathlib import Path
import tempfile
import unittest

from atd_bringup.replay_gt_geometry import Alignment, PlanarPose, Trace


class TraceTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / 'rig_motion.jsonl'

    def test_exact_timestamps_wrap_and_no_extrapolation(self):
        # These simulation times differ by less than one float ULP at this epoch.
        self.path.write_text(
            '{"sim_time_s":1800000000.000000001,"x_m":10,"y_m":22,"yaw_rad":3.1}\n'
            '{"sim_time_s":1800000000.000000009,"x_m":12,"y_m":18,"yaw_rad":-3.1}\n')
        trace = Trace.from_file(self.path)
        begin, end = trace.stamps
        self.assertEqual((begin, end), (1800000000000000001, 1800000000000000009))
        self.assertEqual(trace.at(begin), PlanarPose(10, 22, 3.1))
        self.assertEqual(trace.at(end), PlanarPose(12, 18, -3.1))
        middle = trace.at(begin + 4)
        self.assertEqual((middle.x, middle.y), (11, 20))
        self.assertAlmostEqual(abs(middle.yaw), math.pi)
        self.assertIsNone(trace.at(begin - 1))
        self.assertIsNone(trace.at(end + 1))

    def test_invalid_records_include_line_number(self):
        sample = dict(sim_time_s=1.2, x_m=0, y_m=0, yaw_rad=0)
        for bad in (sample, dict(sample, sim_time_s=0.9), dict(sample, sim_time_s=True),
                    dict(sample, sim_time_s=-1), dict(sample, sim_time_s=float('nan')),
                    dict(sample, sim_time_s='2'), dict(sample, sim_time_s=2, x_m='2'),
                    dict(sample, sim_time_s=2, yaw_rad=float('inf')), {}, []):
            with self.subTest(bad=bad):
                self.path.write_text(json.dumps(sample) + '\n' + json.dumps(bad) + '\n')
                with self.assertRaisesRegex(ValueError, r'rig_motion.jsonl:2'):
                    Trace.from_file(self.path)
        self.path.write_text(json.dumps(sample) + '\n')
        with self.assertRaisesRegex(ValueError, '2 点'):
            Trace.from_file(self.path)


class AlignmentTests(unittest.TestCase):
    def test_nonzero_world_origin_and_full_map_rotation(self):
        # World heading +Y at the anchor. In map its forward is +X, its left is +Z.
        anchor = PlanarPose(12.0, -4.0, math.pi / 2)
        c = math.sqrt(0.5)
        transform = Alignment.from_anchor(anchor, (2.0, 3.0, 5.0), (2*c, 0, 0, 2*c))
        def check(pose, expected_position, expected_orientation):
            position, orientation = transform.apply(pose)
            for got, expected in zip(position, expected_position):
                self.assertAlmostEqual(got, expected)
            # q and -q give the same orientation.
            dot = sum(a*b for a, b in zip(orientation, expected_orientation))
            self.assertAlmostEqual(abs(dot), 1)
            self.assertAlmostEqual(math.hypot(*orientation), 1)

        check(anchor, (2, 3, 5), (c, 0, 0, c))
        # +3 m forward in rear-axle frame; heading unchanged.
        check(PlanarPose(12, -1, math.pi/2), (5, 3, 5), (c, 0, 0, c))
        # +2 m left in rear-axle frame; relative heading +90 degrees.
        check(PlanarPose(10, -4, math.pi), (2, 3, 7), (0.5, -0.5, 0.5, 0.5))

    def test_invalid_estimate_cannot_set_anchor(self):
        for position, orientation in [
            ((1, 2), (0, 0, 0, 1)), ((1, 2, math.inf), (0, 0, 0, 1)),
            ((1, 2, 3), (0, 0, 0, 0)), ((1, 2, 3), (0, math.nan, 0, 0)),
        ]:
            with self.subTest(position=position, orientation=orientation):
                with self.assertRaises(ValueError):
                    Alignment.from_anchor(PlanarPose(0, 0, 0), position, orientation)

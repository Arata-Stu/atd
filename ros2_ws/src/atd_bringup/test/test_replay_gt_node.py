from types import SimpleNamespace as NS
import unittest

from atd_bringup.replay_gt_geometry import PlanarPose, Trace
from atd_bringup.replay_gt_node import GroundTruthPath


def odometry(ns, x, frame='map', child='base_link', qw=1.0):
    sec, nanosec = divmod(ns, 1_000_000_000)
    return NS(header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec)),
              child_frame_id=child,
              pose=NS(pose=NS(position=NS(x=x, y=0, z=0),
                              orientation=NS(x=0, y=0, z=0, w=qw))))


class GroundTruthPathTests(unittest.TestCase):
    def setUp(self):
        trace = Trace((10, 20, 30), tuple(PlanarPose(x, 0, 0) for x in (1, 2, 3)))
        self.path = GroundTruthPath(trace, lambda t, p, q: (t, p[0]))

    def test_anchors_on_first_valid_common_map_pose(self):
        self.path.set_clock(17)
        self.assertFalse(self.path.observe_odometry(odometry(9, 0)))
        self.assertFalse(self.path.observe_odometry(odometry(15, 0, frame='odom')))
        self.assertFalse(self.path.observe_odometry(odometry(15, 0, child='camera_link')))
        with self.assertRaises(ValueError):
            self.path.observe_odometry(odometry(15, 0, qw=0))
        self.assertIsNone(self.path.update())
        self.assertTrue(self.path.observe_odometry(odometry(15, 40)))
        self.assertFalse(self.path.observe_odometry(odometry(16, 99)))
        self.assertEqual(self.path.update(), (17, [(15, 40), (17, 40.2)]))

    def test_clock_continues_without_poses_across_pause_and_end(self):
        self.assertTrue(self.path.observe_odometry(odometry(15, 40)))
        self.assertIsNone(self.path.update())
        self.path.set_clock(12)
        self.assertIsNone(self.path.update())
        self.path.set_clock(15)
        self.assertEqual(self.path.update(), (15, [(15, 40)]))
        self.path.set_clock(18)
        self.assertEqual(self.path.update(), (18, [(15, 40), (18, 40.3)]))
        self.path.set_clock(20)
        self.assertEqual(self.path.update(), (20, [(15, 40), (20, 40.5)]))
        self.assertFalse(self.path.set_clock(20))
        self.assertIsNone(self.path.update())
        self.path.set_clock(35)
        self.assertEqual(self.path.update(), (30, [(15, 40), (20, 40.5), (30, 41.5)]))
        self.path.set_clock(50)
        self.assertIsNone(self.path.update())

    def test_backward_clock_clears_history_and_reanchors(self):
        self.path.set_clock(25)
        self.path.observe_odometry(odometry(18, 30))
        self.assertEqual(self.path.update()[0], 25)
        self.assertTrue(self.path.set_clock(10))
        self.assertIsNone(self.path.update())
        self.assertIsNone(self.path.take_error())
        self.path.observe_odometry(odometry(26, 80))  # seek 前の遅着
        self.assertTrue(self.path.observe_odometry(odometry(10, 50)))
        self.assertEqual(self.path.update(), (10, [(10, 50)]))
        self.path.set_clock(20)
        self.assertEqual(self.path.update(), (20, [(10, 50), (20, 51)]))

    def test_position_error_uses_delayed_image_stamp_even_as_clock_advances(self):
        self.path.set_clock(29)
        self.path.observe_odometry(odometry(10, 40))
        self.path.observe_odometry(odometry(19, 37))
        self.path.observe_odometry(odometry(22, 38))
        self.path.observe_odometry(odometry(20, 90))  # 順序の逆転した推定値
        self.assertEqual(self.path.update()[0], 29)
        self.assertEqual(self.path.take_error(), (22, (38, 0, 0), (41.2, 0, 0)))
        self.path.set_clock(30)
        self.assertIsNone(self.path.take_error())
        self.path.observe_odometry(odometry(28, 39))
        self.assertEqual(self.path.take_error(), (28, (39, 0, 0), (41.8, 0, 0)))

    def test_invalid_and_out_of_trace_estimates_dont_replace_error(self):
        self.path.set_clock(29)
        self.path.observe_odometry(odometry(10, 40))
        self.path.observe_odometry(odometry(25, 38))
        self.path.observe_odometry(odometry(31, 60))
        self.path.observe_odometry(odometry(26, 60, frame='odom'))
        with self.assertRaises(ValueError):
            self.path.observe_odometry(odometry(27, float('nan')))
        self.assertEqual(self.path.take_error(), (25, (38, 0, 0), (41.5, 0, 0)))

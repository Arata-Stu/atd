import math
from types import SimpleNamespace as Msg
import unittest

from atd_bringup.gazebo_closed_loop_paths import ClosedLoopPaths


def message(stamp, x, y, yaw, frame):
    sec, ns = divmod(stamp, 10**9)
    return Msg(header=Msg(frame_id=frame, stamp=Msg(sec=sec, nanosec=ns)),
               pose=Msg(position=Msg(x=x, y=y, z=0.),
                        orientation=Msg(x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2))))


def estimate(stamp, x, y, *, state='tracking', epoch=1, hdmap='h'):
    msg = message(stamp, x, y, math.pi, 'map')
    msg.session_id, msg.epoch, msg.state = 'session', epoch, state
    msg.map_id, msg.map_sha256, msg.hdmap_sha256 = 'course', 'm', hdmap
    return msg


def world(stamp, y):
    return message(stamp, 8., y, math.pi / 2, 'gazebo_world')


class ClosedLoopPathTests(unittest.TestCase):
    def setUp(self):
        self.paths = ClosedLoopPaths(('course', 'm', 'h'), lambda stamp, p, q: (stamp, p, q))

    def test_interpolated_anchor_in_either_arrival_order_without_future_extrapolation(self):
        for arrives_first in ('gt', 'estimate'):
            paths = ClosedLoopPaths(('course', 'm', 'h'), lambda stamp, p, q: (stamp, p, q))
            paths.world_pose(world(0, 5.))
            msg = estimate(25_000_000, 10., 20.)
            if arrives_first == 'estimate':
                paths.localization(msg)
                self.assertIsNone(paths.alignment)
            paths.world_pose(world(50_000_000, 5.1))
            if arrives_first == 'gt':
                paths.localization(msg)
            self.assertAlmostEqual(paths.alignment.world_origin.y, 5.05)
            paths.world_pose(world(100_000_000, 5.2))
            ns, (x, y, z), q = paths.gt.current
            self.assertEqual(ns, 100_000_000)
            self.assertAlmostEqual(x, 9.85)
            self.assertAlmostEqual(y, 20.)
            self.assertAlmostEqual(abs(q[2]), 1.)
            self.assertEqual(paths.gt.points[0][0], 25_000_000)
            self.assertEqual(paths.estimate.current[0], 25_000_000)

    def test_wrong_map_tracking_gap_and_new_epoch_preserve_world_alignment(self):
        paths = self.paths
        paths.world_pose(world(1, 5.))
        paths.localization(estimate(1, 0., 0., hdmap='another-version'))
        paths.localization(estimate(1, 0., 0., state='localizing'))
        self.assertIsNone(paths.alignment)
        self.assertIsNone(paths.estimate.current)
        paths.localization(estimate(1, 10., 20.))
        paths.localization(estimate(2, 11., 20.))
        first_alignment = paths.alignment
        paths.localization(estimate(2, 99., 999.))
        self.assertEqual(len(paths.estimate.points), 2)
        paths.localization(estimate(3, 0., 0., state='lost'))
        self.assertIsNone(paths.estimate.current)
        self.assertEqual(len(paths.estimate.points), 2)
        paths.world_pose(world(4, 6.05))
        paths.localization(estimate(5, 15., 22., epoch=2))
        self.assertEqual([sample[0] for sample in paths.estimate.points], [5])
        self.assertIs(first_alignment, paths.alignment)
        self.assertAlmostEqual(paths.gt.current[1][0], 8.95)

    def test_no_guess_before_first_gt_or_across_long_gap(self):
        paths = self.paths
        paths.localization(estimate(1, 10., 20.))
        paths.world_pose(world(50_000_000, 5.))
        paths.localization(estimate(52_000_000, 10., 20.))
        paths.world_pose(world(1_000_000_000, 10.))
        self.assertIsNone(paths.alignment)
        paths.localization(estimate(1_000_000_000, 10., 20.))
        self.assertIsNotNone(paths.alignment)
        paths.world_pose(world(1_100_000_000, 10.001))
        paths.world_pose(world(1_050_000_000, 99.))
        self.assertEqual(len(paths.gt.points), 1)
        self.assertEqual(paths.gt.current[0], 1_100_000_000)

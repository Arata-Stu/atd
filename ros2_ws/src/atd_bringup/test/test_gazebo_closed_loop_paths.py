import importlib.util
import math
from pathlib import Path
import runpy
from types import SimpleNamespace as Msg
import unittest

from atd_bringup.gazebo_closed_loop_paths import ClosedLoopPaths, lookahead_remaining_ns


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

    def test_headless_keeps_current_gt_without_collecting_rviz_histories(self):
        paths = ClosedLoopPaths(('course', 'm', 'h'), lambda stamp, p, q: (stamp, p, q), history_size=0)
        paths.world_pose(world(1, 5.))
        self.assertIsNone(paths.gt.current)
        paths.localization(estimate(1, 10., 20.))
        paths.localization(estimate(2, 0., 0., state='lost'))
        paths.world_pose(world(3, 6.))
        self.assertEqual(paths.gt.current[0], 3)
        self.assertAlmostEqual(paths.gt.current[1][0], 9.)
        self.assertFalse(paths.gt.points or paths.estimate.points)


class LookaheadTests(unittest.TestCase):
    stamp = 2_000_000_000
    identity = ('course', 'm', 'h')
    segment = ('session', 1)

    def sample(self):
        msg = estimate(self.stamp, 4., -1.)
        msg.target = Msg(x=3.7, y=-1.4, z=0.)
        msg.line, msg.controller, msg.valid = 'raceline', 'pure_pursuit', True
        msg.validity_s = .25
        return msg

    def test_expires_by_source_stamp_on_simulation_clock(self):
        msg = self.sample()

        def remaining(now):
            return lookahead_remaining_ns(msg, now, self.identity, 'raceline', self.segment)

        self.assertEqual(remaining(self.stamp), 250_000_000)
        self.assertEqual(lookahead_remaining_ns(msg, self.stamp, self.identity, 'raceline', self.segment,
                                                selection_stamp_ns=self.stamp), 0)
        self.assertEqual(remaining(self.stamp + 240_000_000), 10_000_000)
        self.assertEqual(remaining(self.stamp + 250_000_000), 0)
        self.assertEqual(remaining(self.stamp - 1_000_000_000), 0)
        msg.validity_s = .04
        self.assertEqual(remaining(self.stamp + 50_000_000), 0)

    def test_wrong_source_lost_tracking_and_invalid_geometry_hide_target(self):
        for change in (
            {'valid': False}, {'controller': 'other'}, {'line': 'centerline'}, {'map_id': 'another'},
            {'map_sha256': 'new'}, {'hdmap_sha256': 'new'}, {'session_id': 'earlier'}, {'epoch': 0},
            {'validity_s': 0.}, {'validity_s': math.nan}, {'target': Msg(x=0., y=math.inf, z=0.)},
        ):
            with self.subTest(change=change):
                msg = self.sample()
                for name, value in change.items():
                    setattr(msg, name, value)
                self.assertEqual(lookahead_remaining_ns(msg, self.stamp, self.identity, 'raceline', self.segment), 0)
        msg = self.sample()
        self.assertEqual(lookahead_remaining_ns(msg, self.stamp, self.identity, 'raceline', None), 0)
        msg.header.frame_id = 'gazebo_world'
        self.assertEqual(lookahead_remaining_ns(msg, self.stamp, self.identity, 'raceline', self.segment), 0)

    @unittest.skipUnless(importlib.util.find_spec('rclpy'), 'ROS 2 の生成メッセージが必要')
    def test_rviz_receives_actual_target_and_origin_and_can_delete_both(self):
        from atd_interfaces.msg import ControlStatus
        from visualization_msgs.msg import Marker

        executable = Path(__file__).resolve().parents[1] / 'scripts/gazebo_closed_loop_paths'
        markers = runpy.run_path(str(executable))['lookahead_markers']
        msg = ControlStatus()
        msg.header.frame_id, msg.header.stamp.sec = 'map', 2
        msg.pose.position.x, msg.pose.position.y = 4., -1.
        msg.target.x, msg.target.y = 3.7, -1.4
        shown = markers(msg, 90_000_000).markers
        self.assertEqual(len(shown), 2)
        self.assertTrue(all(m.action == Marker.ADD and m.header == msg.header for m in shown))
        sphere, = (m for m in shown if m.type == Marker.SPHERE)
        segment, = (m for m in shown if m.type == Marker.LINE_STRIP)
        self.assertEqual((sphere.pose.position.x, sphere.pose.position.y), (3.7, -1.4))
        self.assertEqual([(p.x, p.y) for p in segment.points], [(4., -1.), (3.7, -1.4)])
        for marker in shown:
            self.assertEqual((marker.lifetime.sec, marker.lifetime.nanosec), (0, 90_000_000))
        hidden = markers(None, 0).markers
        self.assertTrue(all(m.action == Marker.DELETE for m in hidden))
        self.assertEqual({(m.ns, m.id) for m in shown}, {(m.ns, m.id) for m in hidden})

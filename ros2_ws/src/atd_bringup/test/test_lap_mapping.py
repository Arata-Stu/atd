import json
from pathlib import Path
import runpy
import sys
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from atd_bringup.lap_mapping import SECOND_NS, Tracking, map_databases


class LapMappingTests(unittest.TestCase):
    def test_checks_freshness_coverage_and_motion(self):
        track = Tracking()
        for t in range(1, 7):
            track.add_status(t * SECOND_NS, 2 if t == 1 else 1)
            track.add_odometry(t * SECOND_NS, (float(t), 0., 0.))
        self.assertTrue(track.ready(6 * SECOND_NS))
        self.assertFalse(track.ready(8 * SECOND_NS))
        report = track.assess(3 * SECOND_NS, 5 * SECOND_NS)
        self.assertEqual(report['vo_distance_m'], 2.)
        self.assertEqual(report['coverage'], 'continuous')
        self.assertEqual(report['loss_samples'], 0)
        with self.assertRaisesRegex(ValueError, '発着時'):
            track.assess(SECOND_NS, 5 * SECOND_NS)
        with self.assertRaisesRegex(ValueError, '両端'):
            track.assess(3 * SECOND_NS, 7 * SECOND_NS)
        track.add_status(8 * SECOND_NS, 1)
        track.add_odometry(8 * SECOND_NS, (7., 0., 0.))
        interrupted = track.assess(5 * SECOND_NS, 8 * SECOND_NS)
        self.assertEqual(interrupted['coverage'], 'interrupted')
        self.assertEqual(interrupted['odometry_max_gap_sim_s'], 2.)
        self.assertEqual(interrupted['first_gap_lap_s'], 1.)
        self.assertIsNone(interrupted['first_loss_lap_s'])
        with self.assertRaisesRegex(ValueError, '発着時刻付近'):
            track.assess(5 * SECOND_NS, 7 * SECOND_NS + 1)
        with self.assertRaisesRegex(ValueError, '逆行'):
            track.add_status(3 * SECOND_NS, 1)

    def test_recovers_before_the_finish_and_reports_the_first_lost_frame(self):
        track = Tracking()
        for t, state in enumerate((1, 1, 2, 2, 1, 1, 1), 1):
            track.add_status(t * SECOND_NS, state)
            if state == 1:
                track.add_odometry(t * SECOND_NS, (float(t), 0., 0.))
        report = track.assess(SECOND_NS, 7 * SECOND_NS)
        self.assertEqual(report['coverage'], 'interrupted')
        self.assertEqual(report['first_loss_lap_s'], 2.)
        self.assertEqual(report['loss_samples'], 2)
        self.assertEqual(report['first_interruption_lap_s'], 1.)
        with self.assertRaisesRegex(ValueError, '発着時'):
            track.assess(SECOND_NS, 4 * SECOND_NS)

    def test_publisher_stuck_at_one_pose_cannot_complete_a_lap(self):
        track = Tracking()
        for t in (1, 2, 3):
            track.add_odometry(t * SECOND_NS, (0, 0, 0))
            track.add_status(t * SECOND_NS, 1)
        with self.assertRaisesRegex(ValueError, '移動'):
            track.assess(SECOND_NS, 3 * SECOND_NS)

    def test_lock_file_empty_database_and_symlink_do_not_prove_a_saved_map(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'lock.mdb').write_bytes(b'lock')
            with self.assertRaises(ValueError):
                map_databases(root)
            (root / 'data.mdb').write_bytes(b'map')
            self.assertEqual(map_databases(root), {'data.mdb': 3})
            (root / 'linked.mdb').symlink_to(root / 'data.mdb')
            with self.assertRaises(ValueError):
                map_databases(root)
            (root / 'linked.mdb').unlink()
            (root / 'empty.mdb').touch()
            with self.assertRaises(ValueError):
                map_databases(root)

    def test_supervisor_waits_for_finish_stamp_before_saving_and_checks_the_files(self):
        def fake_module(name, **members):
            module = ModuleType(name)
            module.__dict__.update(members)
            return module

        request_type = type('FilePath', (), {'Request': type('Request', (), {})})
        ros = fake_module('rclpy', ok=lambda: True)
        modules = {
            'rclpy': ros, 'rclpy.node': fake_module('node', Node=object),
            'rclpy.utilities': fake_module('utilities', remove_ros_args=lambda args: args),
            'nav_msgs.msg': fake_module('msg', Odometry=object),
            'isaac_ros_visual_slam_interfaces.msg': fake_module('msg', VisualSlamStatus=object),
            'isaac_ros_visual_slam_interfaces.srv': fake_module('srv', FilePath=request_type),
        }
        script = Path(__file__).resolve().parents[1] / 'scripts/gazebo_lap_map'
        with patch.dict(sys.modules, modules):
            run = runpy.run_path(str(script))['run']

        for outcome in ('saved', 'regained', 'rejected', 'no_files', 'cancelled', 'nonempty',
                        'second_odometry', 'second_status_midlap'):
            with self.subTest(outcome=outcome), TemporaryDirectory() as tmp:
                folder = Path(tmp)
                target = folder / 'nvidia/cuvslam_map'
                subscriptions = {}
                frame = 0
                called = []
                run.__globals__['monotonic'] = lambda: float(frame)

                def count_publishers(topic):
                    if outcome == 'second_odometry' and frame >= 1 and topic.endswith('/odometry'):
                        return 2
                    if outcome == 'second_status_midlap' and frame >= 2 and topic.endswith('/status'):
                        return 2
                    return 1

                def spin_once(node, timeout_sec):
                    nonlocal frame
                    frame += 1
                    self.assertLess(frame, 5, 'supervisor did not finish')
                    header = SimpleNamespace(stamp=SimpleNamespace(sec=frame, nanosec=0), frame_id='odom')
                    status = 2 if outcome == 'regained' and frame == 2 else 1
                    subscriptions['/visual_slam/status'](SimpleNamespace(header=header, vo_state=status))
                    position = SimpleNamespace(x=float(frame), y=0., z=0.)
                    pose = SimpleNamespace(pose=SimpleNamespace(position=position))
                    subscriptions['/visual_slam/tracking/odometry'](SimpleNamespace(
                        header=header, child_frame_id='base_link', pose=pose))
                    if frame == 2 and outcome != 'second_status_midlap':
                        self.assertTrue((folder / 'mapping.ready').is_file())
                        (folder / 'lap.complete.json').write_text(json.dumps(
                            {'start_sim_ns': SECOND_NS, 'finish_sim_ns': 4 * SECOND_NS}))
                        if outcome == 'cancelled':
                            (folder / 'mapping.cancel').touch()

                def save(request):
                    self.assertEqual(frame, 4)
                    self.assertEqual(request.file_path, str(target))
                    called.append(request)
                    if outcome in ('saved', 'regained'):
                        (target / 'data.mdb').write_bytes(b'map')
                    return SimpleNamespace(done=lambda: True,
                                           result=lambda: SimpleNamespace(success=outcome != 'rejected'))

                ros.spin_once = spin_once
                service = SimpleNamespace(service_is_ready=lambda: True, call_async=save)
                node = SimpleNamespace(
                    create_subscription=lambda typ, topic, cb, depth: subscriptions.update({topic: cb}),
                    create_client=lambda typ, topic: service,
                    count_publishers=count_publishers,
                    get_logger=lambda: SimpleNamespace(info=lambda msg: None),
                    get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=frame * SECOND_NS)))
                if outcome == 'nonempty':
                    target.mkdir(parents=True)
                    (target / 'existing.mdb').write_bytes(b'preserve')
                if outcome in ('saved', 'regained'):
                    result = run(node, folder, target)
                    self.assertEqual(result['databases'], {'data.mdb': 3})
                    self.assertEqual(result['tracking']['status_samples'], 4)
                    self.assertEqual(result['tracking']['coverage'],
                                     'interrupted' if outcome == 'regained' else 'continuous')
                elif outcome in ('second_odometry', 'second_status_midlap'):
                    with self.assertRaisesRegex(RuntimeError, '発行元が複数.*atd bringup gazebo'):
                        run(node, folder, target)
                    self.assertEqual((folder / 'mapping.ready').is_file(), outcome == 'second_status_midlap')
                else:
                    with self.assertRaises((RuntimeError, ValueError)):
                        run(node, folder, target)
                self.assertEqual(len(called), int(outcome in ('saved', 'regained', 'rejected', 'no_files')))

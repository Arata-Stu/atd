"""Bag と同じ試行の post-update trace だけを選べることを ROS なしで確認する。"""

import json
from pathlib import Path
import tempfile
import unittest

from atd_bringup.replay_gt_source import find_ground_truth, store_ground_truth
from atd_bringup.replay_gt_course import load_course, recorded_course


class ReplayGTSourceTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.results = self.root / 'results'
        self.run = '20261120T090823Z-bag-shm-Ab12c3'

    def make_bag(self, name):
        bag = self.root / name
        bag.mkdir()
        (bag / 'metadata.yaml').write_text('test metadata')
        (bag / 'test.mcap').write_bytes(b'1234')
        return bag

    def legacy(self, circuit):
        bag = self.make_bag(f'gazebo-{circuit}-{self.run}')
        runs = self.results / 'runs'
        directory = (runs / 'gazebo_monza' if circuit == 'monza' else runs / 'gazebo' / circuit) / self.run
        trace = directory / 'scene/rig_motion.jsonl'
        trace.parent.mkdir(parents=True)
        trace.write_text('{"sim_time_s": 1, "x_m": 2, "y_m": 3, "yaw_rad": 0}\n')
        (directory / 'run.json').write_text(json.dumps({
            'circuit': circuit, 'mode': 'bag', 'status': 'complete', 'scene': 'scene/scene.json',
            'bag_path': f'/data/record/rosbags/{bag.name}', 'bag_audit': {'file_bytes': {'test.mcap': 4}},
        }))
        return bag, trace

    def test_portable_bag_can_be_renamed_and_moved_without_run(self):
        bag = self.make_bag('original')
        trace = self.root / 'observed.jsonl'
        trace.write_text('{"sim_time_s": 1, "x_m": 2, "y_m": 3, "yaw_rad": 0}\n')
        store_ground_truth(bag, trace, {'test.mcap': 4})
        trace.unlink()
        bag = bag.rename(self.root / 'backup')
        embedded = find_ground_truth(bag, self.results)
        self.assertEqual(embedded, bag / 'atd_ground_truth/rig_motion.jsonl')
        self.assertTrue(embedded.is_file())

    def test_damaged_sidecar_or_bag_is_reported_even_when_legacy_exists(self):
        bag, trace = self.legacy('monza')
        store_ground_truth(bag, trace, {'test.mcap': 4})
        for path in [bag / 'metadata.yaml', bag / 'atd_ground_truth/rig_motion.jsonl',
                     bag / 'test.mcap', bag / 'atd_ground_truth/manifest.json']:
            with self.subTest(damaged=path):
                original = path.read_bytes()
                path.write_bytes(b'modified')
                with self.assertRaises(ValueError):
                    find_ground_truth(bag, self.results)
                path.write_bytes(original)

    def test_existing_bags_find_the_matching_run_and_verify_mcap(self):
        for circuit in ['monza', 'silverstone']:
            with self.subTest(circuit=circuit):
                bag, trace = self.legacy(circuit)
                self.assertEqual(find_ground_truth(bag, self.results), trace)
                (bag / 'test.mcap').write_bytes(b'wrong')
                with self.assertRaisesRegex(ValueError, 'MCAP'):
                    find_ground_truth(bag, self.results)

    def test_completed_report_must_point_back_to_selected_bag(self):
        bag, trace = self.legacy('monza')
        report = trace.parents[1] / 'run.json'
        value = json.loads(report.read_text())
        for key, other in [('circuit', 'spa'), ('mode', 'video'), ('status', 'stopped'),
                           ('bag_path', '/data/record/rosbags/other')]:
            with self.subTest(key=key):
                report.write_text(json.dumps({**value, key: other}))
                with self.assertRaisesRegex(ValueError, '一致しません'):
                    find_ground_truth(bag, self.results)
        report.write_text(json.dumps(value))
        trace.unlink()
        with self.assertRaisesRegex(ValueError, '走行軌跡'):
            find_ground_truth(bag, self.results)

    def test_no_ground_truth_for_physical_bag_or_missing_legacy_run(self):
        for name in ['manual-drive', f'gazebo-monza-{self.run}']:
            self.assertIsNone(find_ground_truth(self.make_bag(name), self.results))

    def test_legacy_two_lap_report_and_new_portable_course_metadata(self):
        bag, trace = self.legacy('monza')
        report_file = trace.parents[1] / 'run.json'
        report = json.loads(report_file.read_text())
        report.update(speed_mps=5, lap_duration_s=115.84, laps_requested=2)
        report['bag_audit']['lap_start_sim_ns'] = 10**9
        report_file.write_text(json.dumps(report))
        expected = dict(circuit='monza', lap_length_m=579.2, lap_duration_s=115.84, laps=2, start_ns=10**9)
        self.assertEqual(load_course(bag, trace, self.results), expected)
        store_ground_truth(bag, trace, {'test.mcap': 4})
        embedded = find_ground_truth(bag, self.results)
        self.assertEqual(load_course(bag, embedded, self.results), expected)
        trace.write_text('different Gazebo run')
        self.assertIsNone(load_course(bag, embedded, self.results)['laps'])

        report['lap_length_m'] = 579.2017
        store_ground_truth(bag, trace, {'test.mcap': 4}, course=recorded_course(report))
        moved = bag.rename(self.root / 'saved-drive')
        result = load_course(moved, find_ground_truth(moved, self.results), self.results)
        self.assertEqual(result, {**expected, 'lap_length_m': 579.2017})

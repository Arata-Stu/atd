import fcntl
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from atd_bringup.replay_gt_geometry import Alignment, PlanarPose, Trace
from atd_bringup.replay_gt_node import GroundTruthPath
from atd_bringup.replay_result import CourseProgress, ReplayResult, ReplayRewound, compare


class ReplayResultTests(unittest.TestCase):
    def setUp(self):
        root = TemporaryDirectory()
        self.addCleanup(root.cleanup)
        self.root = Path(root.name)
        self.trace = Trace(tuple(i * 500_000_000 for i in range(5)),
                           tuple(PlanarPose(x, 8., 0.) for x in (1., 3., 1., 3., 1.)))
        self.course = dict(circuit='monza', laps=2, lap_length_m=4., lap_duration_s=1., start_ns=0)

    def run_writer(self, **options):
        return ReplayResult(self.root, '/data/record/rosbags/bag', .5, self.trace, self.course, **options)

    def test_saves_map_size_for_each_comparison_run(self):
        for options, expected in (({}, 0), (dict(slam_max_map_size=300), 300)):
            with self.run_writer(**options) as run:
                run.mark_eof()
            self.assertEqual(json.loads((run.directory / 'summary.json').read_text())['slam_max_map_size'], expected)

    def test_signed_errors_in_world_plane_after_tilted_map_anchor_and_wrap(self):
        yaw = math.radians(179)
        truth = PlanarPose(5., -3., yaw)
        # The map has an arbitrary origin and pitch relative to the Gazebo plane.
        align = Alignment.from_anchor(PlanarPose(-4., -5., .6), (9., 2., 4.),
                                      (0., math.sin(.39), 0., math.cos(.39)))
        est = PlanarPose(truth.x + 2*math.cos(yaw) + .4*math.sin(yaw),
                         truth.y + 2*math.sin(yaw) - .4*math.cos(yaw), math.radians(-179))
        position, orientation = align.apply(est)
        row = compare(align, truth, position, orientation)
        for key, expected in (('est_x_m', est.x), ('est_y_m', est.y), ('along_m', 2.),
                              ('lateral_m', -.4), ('heading_deg', 2.)):
            self.assertAlmostEqual(row[key], expected, places=10)

    def test_closure_cloud_uses_source_time_and_reports_only_observed_landmarks(self):
        with self.run_writer() as run:
            for stamp, count in [(-1, 100), (50_000_000, 0), (60_000_000, 5), (60_000_000, 5),
                                 (80_000_000, 9), (1_500_000_000, 2), (2_000_000_001, 100)]:
                run.record_closure_cloud(stamp, count)
            run.checkpoint(force=True)
            summary = json.loads((run.directory / 'summary.json').read_text())['loop_closure_cloud']
            self.assertEqual((summary['updates'], summary['nonempty_updates']), (4, 3))
            first, second = summary['first_nonempty_by_lap']
            self.assertEqual((first['lap'], first['point_count'], first['stamp_ns']), (1, 5, '60000000'))
            self.assertEqual((second['lap'], second['station_m'], second['point_count']), (2, 2., 2))
            self.assertEqual(summary['last_nonempty'], second)
            self.assertIn('閉路は未確認', run.closure_report())

    def test_station_from_measured_distance_through_lap_boundary(self):
        route = CourseProgress(self.trace, self.course)
        for stamp, expected in [(0, (1, 0., 0.)), (750_000_000, (1, 3., 3.)),
                                (1_000_000_000, (2, 0., 4.)), (2_000_000_000, (2, 4., 8.))]:
            point = route.point(stamp, self.trace.at(stamp))
            self.assertEqual((point['lap'], point['station_m'], point['distance_m']), expected)
        unknown = CourseProgress(self.trace, {**self.course, 'start_ns': 1_000_000_000})
        self.assertIsNone(unknown.point(1_000_000_000, self.trace.poses[2])['lap'])

    def test_commit_visibility_gaps_late_poses_and_writer_lock(self):
        with self.run_writer() as run:
            d = run.directory
            def summary():
                return json.loads((d / 'summary.json').read_text())
            with (d / 'samples.jsonl').open() as reader, self.assertRaises(BlockingIOError):
                fcntl.flock(reader, fcntl.LOCK_SH | fcntl.LOCK_NB)
            self.assertEqual(len(json.loads((d / 'reference.json').read_text())), 5)
            self.assertEqual(summary()['sample_count'], 0)
            origin = self.trace.poses[0]
            align = Alignment.from_anchor(origin, (0., 0., 0.), (0., 0., 0., 1.))
            for stamp in (0, 100_000_000):
                pose = self.trace.at(stamp)
                run.record(stamp, pose, align, *align.apply(pose))
            with patch('atd_bringup.replay_result.time.monotonic', return_value=run.next_checkpoint + 1.):
                run.checkpoint()
            self.assertEqual(summary()['sample_count'], 2)
            run.mark_eof()
            self.assertEqual(summary()['status'], 'complete')
            for stamp in (1_600_000_000, 1_600_000_000, 1_500_000_000):
                pose = self.trace.at(stamp)
                run.record(stamp, pose, align, *align.apply(pose))
        self.assertEqual(summary()['sample_count'], 3)
        rows = [json.loads(line) for line in (d / 'samples.jsonl').read_text().splitlines()]
        self.assertEqual([r['segment'] for r in rows], [0, 0, 1])
        self.assertEqual(rows[-1]['stamp_ns'], '1600000000')
        self.assertEqual(rows[-1]['lap'], 2)
        with (d / 'samples.jsonl').open() as reader:
            fcntl.flock(reader, fcntl.LOCK_SH | fcntl.LOCK_NB)

    def test_interrupted_failed_and_repeated_attempts_preserve_artifacts(self):
        ids = []
        for error, status in ((KeyboardInterrupt(), 'partial'), (ValueError('input'), 'failed'),
                              (ReplayRewound('時計が戻りました'), 'partial')):
            with self.assertRaises(type(error)), self.run_writer() as result:
                ids.append(result.directory)
                raise error
            summary = json.loads((result.directory / 'summary.json').read_text())
            self.assertEqual(summary['status'], status)
            self.assertEqual(summary['sample_count'], 0)
            self.assertTrue(summary['reason'])
        self.assertEqual(len(set(ids)), 3)

    def test_rewind_after_first_sample_stops_persisting_previous_epoch(self):
        with self.run_writer() as result:
            path = GroundTruthPath(self.trace, lambda *pose: pose, result)
            path.set_clock(7)
            align = Alignment.from_anchor(self.trace.poses[0], (0., 0., 0.), (0., 0., 0., 1.))
            result.record(0, self.trace.poses[0], align, *align.apply(self.trace.poses[0]))
            with self.assertRaises(ReplayRewound):
                path.set_clock(6)

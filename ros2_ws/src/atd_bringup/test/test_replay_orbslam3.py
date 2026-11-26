"""保存地図の誤一致を初期座標合わせで消さず、画像時刻・資産指定を確認する。"""
from contextlib import ExitStack, redirect_stderr
from io import StringIO
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace as NS

from atd_localization.replay_inputs import stereo_window

from atd_bringup import replay_orbslam3, replay_vslam
from atd_bringup.replay_gt_geometry import Alignment, PlanarPose, Trace
from atd_bringup.replay_gt_node import GroundTruthPath
from atd_bringup.replay_orb_inputs import noise_settings, resolve_imu_noise
from atd_bringup.replay_orbslam3 import orb_executable, saved_alignment
from atd_bringup.replay_result import ReplayResult


class OrbReplayTests(unittest.TestCase):
    def test_final_csvs_are_readable_while_rviz_remains_open(self):
        subscriptions = {}

        def subscribe(kind, topic, callback, qos):
            subscriptions[topic] = callback
            return topic

        node = NS(create_subscription=subscribe, destroy_subscription=subscriptions.pop)
        modules = {name: Mock() for name in ('rclpy', 'rclpy.qos', 'atd_interfaces.msg', 'nav_msgs.msg',
                                              'std_msgs.msg', 'std_srvs.srv')}
        with TemporaryDirectory() as temp, ExitStack() as mocks:
            root = Path(temp)
            bag = root / 'second-bag'
            bag.mkdir()
            (bag / 'metadata.yaml').write_text('test bag')
            options = NS(map='track', save_map=None, tracking='vo', rate=1., start_offset=0.)
            meta = dict(map_id='track', map_sha256='hash', hdmap_sha256='hd', tracking='vo',
                        source_metadata_sha256='other', atlas_path=root / 'atlas.osa')
            inputs = NS(camera_fps=90., first_left_ns=1, last_left_ns=1, pairs=1)
            mocks.enter_context(patch.dict('sys.modules', modules))
            mocks.enter_context(patch.dict('os.environ', ATD_REPLAY_RECORD_ROOT=str(root / 'record')))
            mocks.enter_context(patch.object(replay_orbslam3, 'inputs_from_bag', return_value=inputs))
            mocks.enter_context(patch('atd_localization.orb_assets.orb_map_metadata', return_value=meta))
            mocks.enter_context(patch('atd_localization.orb_replay.recorded_calibration',
                                      return_value=({}, (640, 480))))
            mocks.enter_context(patch('atd_localization.orb_replay.OrbBootstrap'))
            with ExitStack() as viewer:
                orb = replay_orbslam3.OrbReplay(viewer, root, node, bag, options, None, {}, 'orb')
                stamp = NS(sec=0, nanosec=1)
                header = NS(stamp=stamp, frame_id='map')
                pose = NS(position=NS(x=1, y=0, z=0), orientation=NS(x=0, y=0, z=0, w=1))
                subscriptions['/orbslam3/processed'](header)
                odom = NS(header=header, child_frame_id='base_link', pose=NS(pose=pose))
                subscriptions[replay_orbslam3.SLAM_ODOMETRY](odom)
                subscriptions['/localization/status'](NS(header=header, state='tracking', epoch=0, reason='ok',
                                                         map_id='track', map_sha256='hash'))
                with patch.object(replay_orbslam3, 'time', NS(monotonic=iter((0, 1, 2)).__next__)):
                    orb.finish(Mock(), Mock(), None)

                self.assertFalse(subscriptions)  # Late status heartbeats cannot append after EOF.
                summary = json.loads((orb.directory / 'summary.json').read_text())
                self.assertEqual((summary['status'], summary['trajectory_samples']), ('complete', 1))
                self.assertEqual((orb.directory / 'trajectory.csv').read_text().splitlines()[1:], ['1,1,0,0,0,0,0,1'])
                self.assertEqual((orb.directory / 'status.csv').read_text().splitlines()[1:], ['1,tracking,0,ok'])

    def test_node_is_required_before_bag_scan_or_child_processes(self):
        with TemporaryDirectory() as folder:
            index = NS(get_package_prefix=lambda package: folder, PackageNotFoundError=KeyError)
            with patch.dict('sys.modules', {'ament_index_python.packages': index}), \
                    patch.object(replay_vslam, 'validate_bag') as scan, \
                    patch.object(replay_vslam.subprocess, 'Popen') as spawn:
                with self.assertRaisesRegex(RuntimeError, 'atd-build --packages-up-to atd_orbslam3'):
                    replay_vslam.run('course', .5, orb_options=NS())
                scan.assert_not_called()
                spawn.assert_not_called()
                node = Path(folder) / 'lib/atd_orbslam3/orbslam3_node'
                node.parent.mkdir(parents=True)
                node.touch(mode=0o644)
                with self.assertRaisesRegex(RuntimeError, 'orbslam3_node'):
                    orb_executable()
                node.chmod(0o755)
                self.assertEqual(orb_executable(), str(node))

    def test_fixed_origin_survives_midlap_start_and_reports_wrong_relocalization(self):
        trace = Trace((10, 20), (PlanarPose(10, 0, 0), PlanarPose(20, 0, 0)))
        fixed = Alignment.from_anchor(PlanarPose(2, 0, 0), (0, 0, 0), (0, 0, 0, 1))
        with TemporaryDirectory() as folder, ReplayResult(folder, 'second-bag', .5, trace, {}) as result:
            result.summary['alignment_source'] = 'saved_map'
            path = GroundTruthPath(trace, lambda t, p, q: p, result, fixed_alignment=fixed)
            msg = NS(header=NS(stamp=NS(sec=0, nanosec=15), frame_id='map'), child_frame_id='base_link',
                     pose=NS(pose=NS(position=NS(x=17., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))))
            self.assertTrue(path.observe_odometry(msg))
            # At time 15 the world x is 15 and GT map x=13. The reported map x=17 is wrong by 4 m.
            self.assertEqual(result.summary['initial_error'], dict(along_m=4., lateral_m=0., heading_deg=0.))
            self.assertEqual(result.summary['alignment']['world_origin_xy_yaw'], (2, 0, 0))

    def test_only_mapping_run_of_matching_circuit_can_supply_fixed_transform(self):
        with TemporaryDirectory() as folder:
            atlas = Path(folder) / 'atlas.osa'
            self.assertIsNone(saved_alignment(atlas, 'odaiba'))
            data = dict(circuit='odaiba', world_origin_xy_yaw=[2, 1, 0], map_origin_xyz=[0, 0, 0],
                        map_from_world_xyzw=[0, 0, 0, 1])
            path = atlas.with_name('gazebo_alignment.json')
            path.write_text(json.dumps(data))
            self.assertIsNone(saved_alignment(atlas, 'monza'))
            self.assertEqual(saved_alignment(atlas, 'odaiba').world_origin.x, 2.)
            data['map_from_world_xyzw'][3] = 0
            path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                saved_alignment(atlas, 'odaiba')

    def test_stamp_uses_left_camera_and_drops_unpaired_tail(self):
        window = stereo_window([10, 20, 30, 40], [11, 29, 49], 1, left_stamp=True)
        self.assertEqual((window['first_stamp_ns'], window['last_stamp_ns'], window['stereo_pairs']), (10, 30, 2))
        with self.assertRaisesRegex(ValueError, 'ステレオ観測'):
            stereo_window([10, 20], [12, 22], 1, left_stamp=True)

    def test_no_ambiguous_or_unsupported_modes(self):
        with redirect_stderr(StringIO()), patch.object(replay_vslam, 'select_bag') as select:
            for args in (['--map', 'a'], ['--start-offset', '1'], ['--start-offset', 'nan', '--map', 'a'],
                         ['--tracking', 'vo', '--imu-calibration', 's.yaml'], ['--map', 'a', '--save-map', 'b'],
                         ['--map', 'a', '--tracking', 'vio', '--imu-calibration', 's.yaml']):
                backend = [] if args == ['--map', 'a'] else ['--backend', 'orbslam3']
                with self.subTest(args=args), self.assertRaises(SystemExit):
                    replay_vslam.main(['bag', *backend, *args])
            select.assert_not_called()
        with patch.object(replay_vslam, 'select_bag', return_value='bag'), patch.object(replay_vslam, 'run') as run:
            self.assertEqual(replay_vslam.main(['bag', '--backend', 'orbslam3', '--map', 'track',
                                                '--start-offset', '17.5', '--headless']), 0)
            options = run.call_args.kwargs['orb_options']
            self.assertEqual((options.map, options.start_offset, options.tracking), ('track', 17.5, 'vo'))
            self.assertEqual(replay_vslam.main(['bag', '--backend', 'orbslam3', '--tracking', 'vio', '--headless']), 0)
            options = run.call_args.kwargs['orb_options']
            self.assertEqual((options.tracking, options.map, options.imu_calibration), ('vio', None, None))

    def test_imu_noise_requires_all_four_calibrated_positive_values(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'imu.yaml'
            path.write_text('gyro_noise_density: 0.01\n')
            with self.assertRaises(ValueError):
                noise_settings(path)
            values = dict(gyro_noise_density=.1, accel_noise_density=.2, gyro_random_walk=.003, accel_random_walk=.04)
            path.write_text(json.dumps(values))
            self.assertEqual(noise_settings(path), values)

    def test_gazebo_reference_is_scoped_to_verified_recordings_and_explicit_override_wins(self):
        share = Path(__file__).resolve().parents[1]
        index = Mock()
        index.get_package_share_directory.return_value = str(share)
        with patch.dict('sys.modules', {'ament_index_python.packages': index}):
            name, values = resolve_imu_noise(None, gazebo=True)
            self.assertEqual(name, 'gazebo_euroc_reference')
            self.assertEqual(values, dict(gyro_noise_density=1.7e-4, accel_noise_density=2.e-3,
                                          gyro_random_walk=1.9393e-5, accel_random_walk=3.e-3))
        with self.assertRaisesRegex(ValueError, '--imu-calibration'):
            resolve_imu_noise(None, gazebo=False)
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'measured.yaml'
            override = {key: value * 2 for key, value in values.items()}
            path.write_text(json.dumps(override))
            for simulation in (True, False):
                self.assertEqual(resolve_imu_noise(path, gazebo=simulation), (str(path.resolve()), override))


if __name__ == '__main__':
    unittest.main()

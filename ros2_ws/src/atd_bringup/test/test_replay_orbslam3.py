"""保存地図の誤一致を初期座標合わせで消さず、画像時刻・資産指定を確認する。"""
from contextlib import redirect_stderr
from io import StringIO
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS

from atd_localization.replay_inputs import stereo_window

from atd_bringup import replay_vslam
from atd_bringup.replay_gt_geometry import Alignment, PlanarPose, Trace
from atd_bringup.replay_gt_node import GroundTruthPath
from atd_bringup.replay_orb_inputs import noise_settings
from atd_bringup.replay_orbslam3 import saved_alignment
from atd_bringup.replay_result import ReplayResult


class OrbReplayTests(unittest.TestCase):
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
                         ['--tracking', 'vio'], ['--map', 'a', '--save-map', 'b'],
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

    def test_imu_noise_requires_all_four_calibrated_positive_values(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'imu.yaml'
            path.write_text('gyro_noise_density: 0.01\n')
            with self.assertRaises(ValueError):
                noise_settings(path)
            values = dict(gyro_noise_density=.1, accel_noise_density=.2, gyro_random_walk=.003, accel_random_walk=.04)
            path.write_text(json.dumps(values))
            self.assertEqual(noise_settings(path), values)


if __name__ == '__main__':
    unittest.main()

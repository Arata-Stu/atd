from contextlib import ExitStack, nullcontext, redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

import yaml

from atd_bringup import replay_vslam as replay
from atd_bringup.replay_inputs import ImuProfile, SLAM_ODOMETRY


class ReplaySessionTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.launch, self.rviz, self.player = [Mock() for _ in range(3)]
        for child in (self.launch, self.rviz, self.player):
            child.poll.side_effect = lambda c=child: c.returncode
            child.returncode = None
        self.spawn = self.stack.enter_context(patch.object(replay.subprocess, 'Popen',
            side_effect=[self.launch, self.rviz, self.player]))
        self.stop = self.stack.enter_context(patch.object(replay, 'stop'))
        self.ready = self.stack.enter_context(patch('atd_bringup.replay_readiness.wait_and_resume'))
        self.tick = self.stack.enter_context(patch.object(replay.time, 'sleep'))
        self.output = self.stack.enter_context(redirect_stdout(StringIO()))
        self.config = Path(__file__).resolve().parents[1] / 'config/bag_vslam.rviz'

    def session(self, ground_truth=None, result=None, **options):
        replay.run_session(Mock(), 'two words/run', 0.5, self.config, 'qos.yaml', ground_truth, result, **options)

    def test_eof_keeps_viewer_open_until_closed(self):
        def resume(node, *, player_name, check_running, ground_truth, vio, rviz_name):
            self.assertFalse(vio)
            self.assertFalse(ground_truth)
            self.assertEqual(rviz_name, '/rviz2')
            check_running()
            cmd = self.spawn.call_args.args[0]
            self.assertEqual(cmd[cmd.index('--remap') + 1], '__node:=' + player_name.lstrip('/'))
            self.player.returncode = 0
        self.ready.side_effect = resume
        self.tick.side_effect = lambda _: setattr(self.rviz, 'returncode', 0)
        self.session()
        self.assertIn('末尾に到達', self.output.getvalue())
        launch, rviz, player = [call.args[0] for call in self.spawn.call_args_list]
        self.assertEqual(launch[-2:], ['bag_vslam.launch.py', 'slam_max_map_size:=0'])
        self.assertIn(str(self.config), rviz)
        self.assertIn('use_sim_time:=true', rviz)
        self.assertIn('two words/run', player)
        for arg in ('--start-paused', '--disable-keyboard-controls', '--clock'):
            self.assertIn(arg, player)
        self.assertEqual(player[player.index('--rate') + 1], '0.5')
        self.assertEqual(player[player.index('--qos-profile-overrides-path') + 1], 'qos.yaml')
        self.assertEqual(player[player.index('--topics') + 1:], list(replay.STEREO_TOPICS))
        self.assertEqual([c.args[0] for c in self.stop.call_args_list],
                         [self.player, self.rviz, self.launch])
        displays = yaml.safe_load(self.config.read_text())['Visualization Manager']['Displays']
        self.assertEqual({d['Topic']['Value'] for d in displays if d['Enabled'] and 'Topic' in d},
                         {SLAM_ODOMETRY, '/visual_slam/vis/landmarks_cloud',
                          '/visual_slam/vis/loop_closure_cloud', '/atd/replay/ground_truth_path',
                          '/atd/replay/position_error',
                          '/camera/camera/infra1/image_rect_raw', '/camera/camera/infra2/image_rect_raw'})
        eyes = [d['Topic'] for d in displays if d['Class'] == 'rviz_default_plugins/Image']
        self.assertEqual(len(eyes), 2)
        for topic in eyes:
            self.assertEqual((topic['Reliability Policy'], topic['Durability Policy']), ('Best Effort', 'Volatile'))
        gt, = [d for d in displays if d['Class'] == 'rviz_default_plugins/Path']
        self.assertEqual(gt['Topic']['Value'], '/atd/replay/ground_truth_path')
        self.assertEqual((gt['Topic']['Reliability Policy'], gt['Topic']['Durability Policy']),
                         ('Reliable', 'Transient Local'))
        odom, = [d for d in displays if d['Class'] == 'rviz_default_plugins/Odometry']
        self.assertEqual(odom['Topic']['Value'], SLAM_ODOMETRY)
        self.assertEqual(odom['Topic']['Reliability Policy'], 'Best Effort')
        self.assertEqual((odom['Keep'], odom['Position Tolerance'], odom['Angle Tolerance']), (20, 0., 0.))
        self.assertEqual(odom['Shape']['Value'], 'Arrow')
        self.assertNotEqual(gt['Color'], odom['Shape']['Color'])
        self.assertFalse(odom['Covariance']['Value'])
        error_topic = next(d['Topic'] for d in displays if d['Class'] == 'rviz_default_plugins/Marker')
        self.assertEqual((error_topic['Value'], error_topic['Reliability Policy'], error_topic['Durability Policy']),
                         ('/atd/replay/position_error', 'Reliable', 'Transient Local'))

    def test_gazebo_replay_starts_overlay_before_resume_and_keeps_it_after_eof(self):
        trace, result, spin = Mock(), Mock(), Mock()
        observer = Mock(spin_once=spin)
        overlay = self.stack.enter_context(patch.object(replay, 'ground_truth_viewer', return_value=nullcontext(observer)))

        def resume(node, *, player_name, check_running, ground_truth, vio, rviz_name):
            check_running()
            self.assertTrue(ground_truth)
            overlay.assert_called_once_with(trace, result, visualize=True)
            self.player.returncode = 0

        def close(**_):
            result.mark_eof.assert_called_once()
            self.assertIn('末尾に到達', self.output.getvalue())
            self.stop.assert_not_called()
            self.rviz.returncode = 0

        self.ready.side_effect = resume
        spin.side_effect = close
        with patch.object(replay.time, 'monotonic', side_effect=[1., 1.2, 2.]):
            self.session(trace, result)
        self.assertEqual(spin.call_count, 2)
        observer.inspect_loop_graph.assert_called_once_with()
        self.assertEqual([c.args[0] for c in self.stop.call_args_list],
                         [self.player, self.rviz, self.launch])

    def test_headless_reaches_eof_and_collects_delayed_odometry_without_rviz(self):
        self.config = None
        self.spawn.side_effect = [self.launch, self.player]
        self.ready.side_effect = lambda *args, **kwargs: setattr(self.player, 'returncode', 0)
        clock = [0.]
        trace, result = Mock(), Mock(last_stamp=None)

        def spin(timeout_sec):
            result.mark_eof.assert_not_called()
            clock[0] += timeout_sec
            if clock[0] >= 3.:
                result.last_stamp = 200
            elif clock[0] >= 1.5:
                result.last_stamp = 100

        observer = Mock(spin_once=spin)
        with patch.object(replay.time, 'monotonic', side_effect=lambda: clock[0]), patch.object(
                replay, 'ground_truth_viewer', return_value=nullcontext(observer)) as overlay:
            self.session(trace, result)
        self.assertEqual(result.last_stamp, 200)
        self.assertTrue(self.ready.call_args.kwargs['ground_truth'])
        self.assertIsNone(self.ready.call_args.kwargs['rviz_name'])
        overlay.assert_called_once_with(trace, result, visualize=False)
        self.assertEqual([call.args[0][0] for call in self.spawn.call_args_list], ['ros2', 'ros2'])
        self.assertEqual([call.args[0] for call in self.stop.call_args_list], [self.player, self.launch])
        result.mark_eof.assert_called_once_with()
        observer.inspect_loop_graph.assert_called_once_with()

    def test_headless_empty_comparison_is_a_failure(self):
        self.config = None
        self.spawn.side_effect = [self.launch, self.player]
        self.player.returncode = 0
        result = Mock(last_stamp=None)
        with patch.object(replay.time, 'monotonic', side_effect=[0., 0., 2.]), patch.object(
                replay, 'ground_truth_viewer', return_value=nullcontext(Mock())):
            with self.assertRaisesRegex(RuntimeError, '0 件'):
                self.session(Mock(), result)
        result.mark_eof.assert_not_called()
        self.assertEqual(self.stop.call_count, 2)

    def test_bag_without_ground_truth_only_subscribes_to_its_estimate_and_sensors(self):
        with TemporaryDirectory() as directory:
            path = replay.estimated_only_config(self.config, directory)
            displays = yaml.safe_load(path.read_text())['Visualization Manager']['Displays']
        topics = {d['Topic']['Value'] for d in displays if 'Topic' in d}
        self.assertIn(SLAM_ODOMETRY, topics)
        self.assertNotIn('/atd/replay/ground_truth_path', topics)
        self.assertNotIn('/atd/replay/position_error', topics)

    def test_closing_viewer_during_startup_cleans_up_paused_player(self):
        self.rviz.returncode = 0
        self.ready.side_effect = lambda node, **kwargs: kwargs['check_running']()
        self.session(slam_max_map_size=300)
        self.assertIn('slam_max_map_size:=300', self.spawn.call_args_list[0].args[0])
        self.assertNotIn('bag を再生しています', self.output.getvalue())
        self.assertEqual(self.stop.call_count, 3)

    def test_cli_accepts_reproducible_limits_and_rejects_out_of_range_values_before_selecting_bag(self):
        with patch.object(replay, 'select_bag', return_value='bag') as select, patch.object(replay, 'run') as run:
            for options, limit in (([], 0), (['--slam-max-map-size', '300'], 300)):
                self.assertEqual(replay.main(['bag', *options]), 0)
                run.assert_called_with('bag', .5, slam_max_map_size=limit, tracking='vo', headless=False)
            self.assertEqual(replay.main(['bag', '--tracking', 'vio', '--headless']), 0)
            run.assert_called_with('bag', .5, slam_max_map_size=0, tracking='vio', headless=True)
            select.reset_mock()
            for invalid in ('-1', '2147483648', '3.5'):
                with self.subTest(invalid=invalid), self.assertRaises(SystemExit), redirect_stderr(StringIO()):
                    replay.main(['bag', '--slam-max-map-size', invalid])
            select.assert_not_called()

    def test_vio_uses_unified_imu_and_its_measured_rate(self):
        self.ready.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            self.session(imu=ImuProfile('camera_gyro_optical_frame', 400.0))
        launch, _, player = [call.args[0] for call in self.spawn.call_args_list]
        self.assertIn('imu_frequency:=400.0', launch)
        self.assertIn('tracking_mode:=1', launch)
        self.assertIn('imu_frame:=camera_gyro_optical_frame', launch)
        self.assertEqual(player[player.index('--topics') + 1:], [*replay.STEREO_TOPICS, replay.IMU_TOPIC])
        self.assertTrue(self.ready.call_args.kwargs['vio'])

    def test_gyro_bridge_sends_gyro_to_replay_without_switching_orb_to_vio(self):
        orb = Mock(command=['orb', '--ros-args'], params={'mode': 'localization'},
                   seed_topics={'/tf_static'}, gyro_topic='/camera/camera/gyro/sample')
        orb.options.start_offset = 0
        self.ready.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            self.session(orb=orb)
        launch, _, player = [call.args[0] for call in self.spawn.call_args_list]
        self.assertEqual(launch, orb.command)
        self.assertEqual(player[player.index('--topics')+1:],
                         [*(t for t in replay.STEREO_TOPICS if t not in orb.seed_topics), orb.gyro_topic])
        self.assertFalse(self.ready.call_args.kwargs['vio'])
        with TemporaryDirectory() as directory:
            preset = yaml.safe_load(replay.replay_config(self.config, directory, set(), bridge=True).read_text())
        arrows = {d['Topic']['Value']: d['Shape']['Color'] for d in preset['Visualization Manager']['Displays']
                  if d['Class'].endswith('/Odometry')}
        self.assertNotEqual(arrows['/orbslam3/continuous_odometry'], arrows[SLAM_ODOMETRY])

    def test_playback_failure_after_resume_is_reported(self):
        self.player.returncode = 2
        with self.assertRaisesRegex(RuntimeError, 'rosbag2.*exit 2'):
            self.session()
        self.assertEqual(self.stop.call_count, 3)

    def test_mapper_exits_during_startup(self):
        self.launch.returncode = 1
        self.ready.side_effect = lambda node, **kwargs: kwargs['check_running']()
        with self.assertRaisesRegex(RuntimeError, 'cuVSLAM'):
            self.session()
        self.assertEqual(self.stop.call_count, 3)

    def test_partial_start_reaps_the_children_that_started(self):
        self.spawn.side_effect = [self.launch, FileNotFoundError('rviz2')]
        with self.assertRaises(FileNotFoundError):
            self.session()
        self.stop.assert_called_once_with(self.launch)

    def test_interrupt_during_startup_reaps_all_children(self):
        self.ready.side_effect = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            self.session()
        self.assertEqual(self.stop.call_count, 3)

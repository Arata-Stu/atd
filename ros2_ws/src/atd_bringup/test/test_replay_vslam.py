from contextlib import ExitStack, nullcontext, redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

import yaml

from atd_bringup import replay_vslam as replay


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
        def resume(node, *, player_name, check_running, ground_truth):
            self.assertFalse(ground_truth)
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
                         {'/visual_slam/tracking/slam_path', '/visual_slam/vis/landmarks_cloud',
                          '/visual_slam/vis/loop_closure_cloud', '/atd/replay/ground_truth_path',
                          '/atd/replay/position_error',
                          '/camera/camera/infra1/image_rect_raw', '/camera/camera/infra2/image_rect_raw'})
        eyes = [d['Topic'] for d in displays if d['Class'] == 'rviz_default_plugins/Image']
        self.assertEqual(len(eyes), 2)
        for topic in eyes:
            self.assertEqual((topic['Reliability Policy'], topic['Durability Policy']), ('Best Effort', 'Volatile'))
        paths = {d['Topic']['Value']: d for d in displays if d['Class'] == 'rviz_default_plugins/Path'}
        gt = paths['/atd/replay/ground_truth_path']
        self.assertEqual((gt['Topic']['Reliability Policy'], gt['Topic']['Durability Policy']),
                         ('Reliable', 'Transient Local'))
        self.assertNotEqual(gt['Color'], paths['/visual_slam/tracking/slam_path']['Color'])
        error_topic = next(d['Topic'] for d in displays if d['Class'] == 'rviz_default_plugins/Marker')
        self.assertEqual((error_topic['Value'], error_topic['Reliability Policy'], error_topic['Durability Policy']),
                         ('/atd/replay/position_error', 'Reliable', 'Transient Local'))

    def test_gazebo_replay_starts_overlay_before_resume_and_keeps_it_after_eof(self):
        trace, result, spin = Mock(), Mock(), Mock()
        overlay = self.stack.enter_context(patch.object(replay, 'ground_truth_viewer', return_value=nullcontext(spin)))

        def resume(node, *, player_name, check_running, ground_truth):
            check_running()
            self.assertTrue(ground_truth)
            overlay.assert_called_once_with(trace, result)
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
        self.assertEqual([c.args[0] for c in self.stop.call_args_list],
                         [self.player, self.rviz, self.launch])

    def test_bag_without_ground_truth_only_subscribes_to_its_estimate_and_sensors(self):
        with TemporaryDirectory() as directory:
            path = replay.estimated_only_config(self.config, directory)
            displays = yaml.safe_load(path.read_text())['Visualization Manager']['Displays']
        topics = {d['Topic']['Value'] for d in displays if 'Topic' in d}
        self.assertIn('/visual_slam/tracking/slam_path', topics)
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
                run.assert_called_with('bag', .5, slam_max_map_size=limit)
            select.reset_mock()
            for invalid in ('-1', '2147483648', '3.5'):
                with self.subTest(invalid=invalid), self.assertRaises(SystemExit), redirect_stderr(StringIO()):
                    replay.main(['bag', '--slam-max-map-size', invalid])
            select.assert_not_called()

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

from contextlib import ExitStack, redirect_stdout
from io import StringIO
from pathlib import Path
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

    def session(self):
        replay.run_session(Mock(), 'two words/run', 0.5, self.config, 'qos.yaml')

    def test_eof_keeps_viewer_open_until_closed(self):
        def resume(node, *, player_name, check_running):
            check_running()
            cmd = self.spawn.call_args.args[0]
            self.assertEqual(cmd[cmd.index('--remap') + 1], '__node:=' + player_name.lstrip('/'))
            self.player.returncode = 0
        self.ready.side_effect = resume
        self.tick.side_effect = lambda _: setattr(self.rviz, 'returncode', 0)
        self.session()
        self.assertIn('末尾に到達', self.output.getvalue())
        launch, rviz, player = [call.args[0] for call in self.spawn.call_args_list]
        self.assertEqual(launch[-1], 'bag_vslam.launch.py')
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
                          '/visual_slam/vis/loop_closure_cloud',
                          '/camera/camera/infra1/image_rect_raw', '/camera/camera/infra2/image_rect_raw'})
        eyes = [d['Topic'] for d in displays if d['Class'] == 'rviz_default_plugins/Image']
        self.assertEqual(len(eyes), 2)
        for topic in eyes:
            self.assertEqual((topic['Reliability Policy'], topic['Durability Policy']), ('Best Effort', 'Volatile'))

    def test_closing_viewer_during_startup_cleans_up_paused_player(self):
        self.rviz.returncode = 0
        self.ready.side_effect = lambda node, *, player_name, check_running: check_running()
        self.session()
        self.assertNotIn('bag を再生しています', self.output.getvalue())
        self.assertEqual(self.stop.call_count, 3)

    def test_playback_failure_after_resume_is_reported(self):
        self.player.returncode = 2
        with self.assertRaisesRegex(RuntimeError, 'rosbag2.*exit 2'):
            self.session()
        self.assertEqual(self.stop.call_count, 3)

    def test_mapper_exits_during_startup(self):
        self.launch.returncode = 1
        self.ready.side_effect = lambda node, *, player_name, check_running: check_running()
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

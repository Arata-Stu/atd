"""保存地図の選択と、2コンテナの起動・終了契約をローカル child で検査する。"""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
import closed_loop_job as job


class ClosedLoopJobTests(unittest.TestCase):
    def test_missing_id_and_missing_hdmap_explain_how_to_select_a_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / 'maps/gazebo'
            cmd = ['bash', str(job.PROJECT / 'scripts/gazebo.sh'), 'closed-loop', '--course', 'odaiba',
                   '--map', 'gazebo', '--line', 'raceline', '--speed', '1.5', '--laps', '2', '--rviz']
            for exists, expected in [(False, str(folder)), (True, 'hdmap/map.json')]:
                with self.subTest(folder_exists=exists):
                    if exists:
                        folder.mkdir(parents=True)
                    result = subprocess.run(cmd, env={**os.environ, 'ATD_RESULT_DIR': tmp},
                                            capture_output=True, text=True)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn('地図 ID「gazebo」', result.stderr)
                    self.assertIn(expected, result.stderr)
                    self.assertIn('--map を省略', result.stderr)
            self.assertFalse((Path(tmp) / 'runs').exists())

    def test_picker_filters_by_requested_line_and_resolution(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            template = json.loads((job.PROJECT / 'samples/odaiba/hdmap/map.json').read_text())
            for name, line, shape in [('race', 'raceline', [480, 640]), ('center', 'centerline', [480, 640]),
                                      ('other_resolution', 'raceline', [720, 1280])]:
                folder = root / 'maps' / name
                (folder / 'hdmap').mkdir(parents=True)
                doc = dict(template, id=name, source={'kind': 'nvidia_offline'}, lines={line: template['lines'][line]})
                (folder / 'hdmap/map.json').write_text(json.dumps(doc))
                (folder / 'vgl.json').write_text(json.dumps(dict(status='complete', input_shape=[1, 3, *shape])))
            args = SimpleNamespace(results=root, map=None, line='raceline', resolution='640x480', dry_run=False)
            with patch.object(job, 'choose', return_value='race') as pick:
                self.assertEqual(job.select(args), ('race', 'raceline', '640x480'))
                self.assertEqual(pick.call_args.args[1], ['race'])
            args.map = 'center'
            with self.assertRaisesRegex(ValueError, '速度付き raceline がありません'), patch.object(job, 'choose') as pick:
                job.select(args)
            pick.assert_not_called()

    def test_worker_requires_installed_executable_in_selected_overlay(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            packages = SimpleNamespace(get_package_prefix=lambda name: tmp, PackageNotFoundError=KeyError)
            argv = ['closed_loop_job.py', 'worker', '--output', tmp, '--', 'ros2', 'launch']
            with patch.dict(sys.modules, {'ament_index_python.packages': packages}), \
                    patch.object(sys, 'argv', argv), patch.object(job, 'worker', return_value=0) as launch:
                with self.assertRaisesRegex(RuntimeError, 'atd build --packages-up-to atd_bringup'):
                    job.main()
                binary = folder / 'lib/atd_bringup/gazebo_closed_loop_log'
                binary.parent.mkdir(parents=True)
                binary.touch()
                binary.chmod(0o755)
                self.assertEqual(job.main(), 0)
                launch.assert_called_once_with(folder, ['ros2', 'launch'], cancel_name='closed_loop.cancel')

    def test_host_selects_map_resolution_and_routes_speed_to_controller(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = json.loads((job.PROJECT / 'samples/odaiba/hdmap/map.json').read_text())
            source.update(id='odaiba_1p5', source={'kind': 'nvidia_offline'})
            folder = root / 'maps' / source['id']
            (folder / 'hdmap').mkdir(parents=True)
            (folder / 'hdmap/map.json').write_text(json.dumps(source))
            (folder / 'vgl.json').write_text(json.dumps(dict(status='complete', input_shape=[1, 3, 720, 1280])))
            cmd = ['bash', str(job.PROJECT / 'scripts/gazebo.sh'), 'closed-loop', '--dry-run',
                   '--map', source['id'], '--line', 'centerline', '--laps', '3', '--speed', '1.25']
            env = {**os.environ, 'ATD_RESULT_DIR': tmp}
            res = subprocess.run(cmd, check=True, text=True, capture_output=True, env=env)
            gazebo, stack = [shlex.split(s) for s in res.stdout.splitlines() if s.startswith('docker ')]
            for key, expected in [('--mode', 'closed-loop'), ('--laps', '3'), ('--resolution', '1280x720'),
                                  ('--camera-hz', '30')]:
                self.assertEqual(gazebo[gazebo.index(key)+1], expected)
            self.assertNotIn('--speed', gazebo)
            self.assertIn('max_speed_mps:=1.25', stack)
            self.assertEqual(gazebo[gazebo.index('--full-throttle-speed') + 1], '3.2')
            self.assertIn('full_throttle_speed_mps:=3.2', stack)
            self.assertIn('line:=centerline', stack)
            self.assertIn('rviz:=false', stack)
            self.assertIn('map_id:=odaiba_1p5', stack)
            self.assertIn('gazebo_closed_loop.launch.py', stack)
            self.assertIn('run_dir:=' + stack[stack.index('--output') + 1], stack)
            self.assertNotIn('autonomous.launch.py', stack)
            profile = 'FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/atd/docker/middleware_profiles/fastdds.xml'
            self.assertIn(profile, gazebo)
            self.assertIn(profile, stack)
            for speed, override, expected in [('10', [], '10.0'), ('5', ['--full-throttle-speed', '10'], '10.0')]:
                with self.subTest(speed=speed, override=override):
                    preview = subprocess.check_output([*cmd, '--speed', speed, *override], env=env, text=True)
                    gazebo, stack = [shlex.split(s) for s in preview.splitlines() if s.startswith('docker ')]
                    self.assertEqual(gazebo[gazebo.index('--full-throttle-speed') + 1], expected)
                    self.assertNotIn('--speed', gazebo)
                    self.assertIn('max_speed_mps:=' + str(float(speed)), stack)
                    self.assertIn('full_throttle_speed_mps:=' + expected, stack)
            for option, setting in [('--rviz', 'true'), ('--no-rviz', 'false')]:
                with self.subTest(option=option):
                    preview = subprocess.check_output([*cmd, option], env=env, text=True)
                    gazebo, stack = [shlex.split(s) for s in preview.splitlines() if s.startswith('docker ')]
                    self.assertIn(f'rviz:={setting}', stack)
                    self.assertNotIn(option, gazebo)
                    self.assertNotEqual(subprocess.run([*cmd[:2], 'bag', '--dry-run', option],
                                                       env=env, capture_output=True).returncode, 0)
            for extra in [('--resolution', '640x480'), ('--duration', '5'), ('--line', 'custom'),
                          ('--map', '../bad'), ('--engine-dir', '../../bad'), ('--speed', '.1'),
                          ('--full-throttle-speed', '1'), ('--full-throttle-speed', 'nan')]:
                with self.subTest(extra=extra):
                    self.assertNotEqual(subprocess.run([*cmd, *extra], env=env, capture_output=True).returncode, 0)
            self.assertFalse((root / 'runs').exists())

    def test_host_recognizes_saved_endurance_result_after_interrupt(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            argv = ['closed_loop_job.py', 'host', '--output', tmp, '--results', tmp,
                    '--circuit', 'odaiba', '--domain', '74', '--container', 'fixture',
                    '--profile', '/test.xml', '--map', 'test', '--line', 'centerline',
                    '--speed', '1.5', '--web', '--tracking', 'vio', '--imu', '200',
                    '--map-correction-source', 'vgl', '--vgl-frequency', '0', '--', 'gazebo', '--laps', '0']

            def interrupt(_args, command, ros):
                self.assertEqual(command[command.index('--imu') + 1], '200')
                self.assertIn('tracking:=vio', ros)
                self.assertIn('imu_frequency:=200', ros)
                self.assertIn('map_correction_source:=vgl', ros)
                self.assertIn('vgl_frequency:=0.0', ros)
                (out / 'closed_loop.result.json').write_text(json.dumps(dict(status='stopped', laps_completed=3)))
                raise KeyboardInterrupt

            with patch.object(sys, 'argv', argv), patch.object(job, 'check_isaac_mounts'), \
                    patch.object(job.signal, 'signal'), patch.object(job, 'supervise', side_effect=interrupt):
                self.assertEqual(job.main(), 0)
            session = json.loads((out / 'closed_loop.host.json').read_text())
            self.assertEqual(session['status'], 'stopped')
            self.assertTrue(session['web'])
            self.assertEqual((session['tracking'], session['imu_hz']), ('vio', 200))
            self.assertEqual((session['map_correction_source'], session['vgl_frequency_hz']), ('vgl', 0.0))

    def test_result_or_early_exit_cancels_both_children(self):
        for outcome in ('completed', 'off_course', 'early', 'interrupted'):
            with self.subTest(outcome=outcome), tempfile.TemporaryDirectory() as tmp:
                out = Path(tmp)
                prelude = """
import json, sys, time
from pathlib import Path
p = Path(sys.argv[1])
def wait(name):
    deadline = time.monotonic() + 6
    while not (p/name).exists():
        if time.monotonic() >= deadline: raise TimeoutError(name)
        time.sleep(.01)
"""
                sim = prelude + """
assert not any(p.iterdir())
(p/'gazebo.ready').touch()
wait('test.started')
if sys.argv[2] in ('completed', 'off_course'):
    (p/'closed_loop.result.json').write_text(json.dumps({'status': sys.argv[2]}))
wait('closed_loop.cancel')
if sys.argv[2] in ('early', 'interrupted'):
    sys.path.insert(0, sys.argv[3])
    from closed_loop_trial import run_trial
    report = {'imu_hz': 0}
    def start(children, stereo, course, check_stop, imu_hz):
        check_stop()
    run_trial({}, p, {'course': 'odaiba', 'trajectory': {'laps': 2}}, report, start, None, None)
(p/'test.sim.exited').touch()
"""
                ros = prelude + """
assert (p/'gazebo.ready').exists()
(p/'test.started').touch()
if sys.argv[2] == 'early':
    print('[ERROR] [launch]: 候補が複数あります')
    print('[INFO] launch stopped')
else:
    wait('closed_loop.cancel')
(p/'test.ros.exited').touch()
sys.exit(1 if sys.argv[2] == 'early' else 0)
"""
                args = SimpleNamespace(output=out, container='fixture')
                processes = [[sys.executable, '-c', code, tmp, outcome, str(TOOLS)] for code in (sim, ros)]
                if outcome == 'early':
                    with self.assertRaisesRegex(RuntimeError, '候補が複数'):
                        job.supervise(args, *processes)
                    result = json.loads((out / 'closed_loop.result.json').read_text())
                    self.assertEqual(result['status'], 'error')
                    self.assertIn('Isaac ROS=1', result['reason'])
                    self.assertIn('候補が複数', result['reason'])
                elif outcome == 'interrupted':
                    def interrupt_after_start(delay):
                        if (out / 'test.started').is_file():
                            raise KeyboardInterrupt
                        time.sleep(delay)

                    clock = SimpleNamespace(monotonic=time.monotonic, sleep=interrupt_after_start)
                    with patch.object(job, 'time', clock), self.assertRaises(KeyboardInterrupt):
                        job.supervise(args, *processes)
                    result = json.loads((out / 'closed_loop.result.json').read_text())
                    self.assertEqual(result['status'], 'cancelled')
                    self.assertIn('ユーザー', result['reason'])
                else:
                    self.assertEqual(job.supervise(args, *processes), 0 if outcome == 'completed' else 1)
                self.assertTrue((out / 'test.sim.exited').is_file())
                self.assertTrue((out / 'test.ros.exited').is_file())

    def test_startup_deadline_counts_only_unpaused_wall_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            (out / 'gazebo.ready').touch()
            (out / 'scene').mkdir()
            pause = out / 'scene/closed_loop.paused'
            events = iter([(10., True), (1010., True), (1011., False), (1300., False), (1301., False)])
            wall = 0.

            def next_event(_seconds):
                nonlocal wall
                wall, paused = next(events)
                if paused:
                    pause.touch()
                else:
                    pause.unlink(missing_ok=True)

            clock = SimpleNamespace(monotonic=lambda: wall, sleep=next_event)
            running = Mock(**{'poll.return_value': None})
            with patch.object(job, 'time', clock), patch.object(job.subprocess, 'Popen', return_value=running), \
                    patch.object(job, 'finish_children'), self.assertRaisesRegex(RuntimeError, '300実時間秒'):
                job.supervise(SimpleNamespace(output=out, container='fixture'), ['gz'], ['ros'])
            self.assertEqual(wall, 1301.)

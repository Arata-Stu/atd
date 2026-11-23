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
from unittest.mock import patch

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
import closed_loop_job as job


class ClosedLoopJobTests(unittest.TestCase):
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
            self.assertIn('--ipc=host', gazebo)
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
                    '--speed', '1.5', '--web', '--', 'gazebo', '--laps', '0']

            def interrupt(_args, _command, _ros):
                (out / 'closed_loop.result.json').write_text(json.dumps(dict(status='stopped', laps_completed=3)))
                raise KeyboardInterrupt

            with patch.object(sys, 'argv', argv), patch.object(job, 'check_isaac_mounts'), \
                    patch.object(job.signal, 'signal'), patch.object(job, 'supervise', side_effect=interrupt):
                self.assertEqual(job.main(), 0)
            session = json.loads((out / 'closed_loop.host.json').read_text())
            self.assertEqual(session['status'], 'stopped')
            self.assertTrue(session['web'])

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

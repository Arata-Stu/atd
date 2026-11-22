"""保存地図の選択と、2コンテナの起動・終了契約をローカル child で検査する。"""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
import closed_loop_job as job


class ClosedLoopJobTests(unittest.TestCase):
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
            self.assertIn('line:=centerline', stack)
            self.assertIn('map_id:=odaiba_1p5', stack)
            self.assertIn('gazebo_closed_loop.launch.py', stack)
            self.assertNotIn('autonomous.launch.py', stack)
            self.assertIn('--ipc=host', gazebo)
            for extra in [('--resolution', '640x480'), ('--duration', '5'), ('--line', 'custom'),
                          ('--map', '../bad'), ('--engine-dir', '../../bad'), ('--speed', '5')]:
                with self.subTest(extra=extra):
                    self.assertNotEqual(subprocess.run([*cmd, *extra], env=env, capture_output=True).returncode, 0)
            self.assertFalse((root / 'runs').exists())

    def test_result_or_early_exit_cancels_both_children(self):
        for outcome in ('completed', 'off_course', 'early'):
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
if sys.argv[2] != 'early':
    (p/'closed_loop.result.json').write_text(json.dumps({'status': sys.argv[2]}))
wait('closed_loop.cancel')
(p/'test.sim.exited').touch()
"""
                ros = prelude + """
assert (p/'gazebo.ready').exists()
(p/'test.started').touch()
if sys.argv[2] != 'early': wait('closed_loop.cancel')
(p/'test.ros.exited').touch()
"""
                args = SimpleNamespace(output=out, container='fixture')
                processes = [[sys.executable, '-c', code, tmp, outcome] for code in (sim, ros)]
                if outcome == 'early':
                    with self.assertRaisesRegex(RuntimeError, '終了'):
                        job.supervise(args, *processes)
                else:
                    self.assertEqual(job.supervise(args, *processes), 0 if outcome == 'completed' else 1)
                self.assertTrue((out / 'test.sim.exited').is_file())
                self.assertTrue((out / 'test.ros.exited').is_file())

"""用途別 uv 環境の同期先・Python 設定・実行境界を一時ディレクトリで確認する。"""

import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / 'docker/scripts/atd-python.sh'
FAKE_UV = r'''
import json
import os
from pathlib import Path
import sys
import venv

args = sys.argv[1:]
with open(os.environ['ATD_TEST_LOG'], 'a') as stream:
    stream.write(json.dumps({'args': args, 'env': {key: os.environ.get(key) for key in (
        'UV_PROJECT_ENVIRONMENT', 'UV_CACHE_DIR', 'VIRTUAL_ENV', 'PYTHONPATH', 'PYTHONHOME')}}) + '\n')
if args[0] == 'venv':
    output = Path(args[-1])
    venv.EnvBuilder(with_pip=False, system_site_packages='--system-site-packages' in args).create(output)
    if os.environ.get('ATD_TEST_FAKE_VENV_PYTHON'):
        python = output / 'bin/python'
        python.unlink()
        python.write_text('#!' + sys.executable + '\n' + """
import json
import os
import sys
if sys.argv[1:2] == ['-I']:
    os.execv(sys.executable, [sys.executable, *sys.argv[1:]])
with open(os.environ['ATD_TEST_BUILD_LOG'], 'w') as stream:
    json.dump({'args': sys.argv[1:], 'cwd': os.getcwd(), 'venv': os.environ.get('VIRTUAL_ENV')}, stream)
""")
        python.chmod(0o755)
elif args[0] == 'sync':
    sys.exit(int(os.environ.get('ATD_TEST_SYNC_EXIT', '0')))
else:
    sys.exit('unexpected uv operation')
'''


class PythonEnvironmentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='atd-python-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.workspace = self.root / 'ws'
        self.workspace.mkdir()
        self.projects = self.root / 'projects'
        for profile in ('ros', 'train', 'vgl-export'):
            project = self.projects / profile
            project.mkdir(parents=True)
            (project / 'pyproject.toml').write_text(
                f'[project]\nname = "atd-{profile}"\nversion = "0.0.0"\nrequires-python = ">=3.9"\ndependencies = []\n')
            (project / 'uv.lock').write_text('version = 1\n')
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        uv = self.bin / 'uv'
        uv.write_text(f'#!{sys.executable}\n' + FAKE_UV)
        uv.chmod(0o755)
        (self.bin / 'uname').write_text('#!/bin/sh\nif [ "$1" = -s ]; then printf "Linux\\n"; else printf "%s\\n" "${ATD_TEST_ARCH:-x86_64}"; fi\n')
        (self.bin / 'uname').chmod(0o755)
        (self.bin / 'id').write_text('#!/bin/sh\nprintf "%s\\n" "${ATD_TEST_UID:-1000}"\n')
        (self.bin / 'id').chmod(0o755)
        self.log = self.root / 'uv-calls.jsonl'
        self.build_log = self.root / 'build.json'
        self.ros_lib = self.root / 'ros-python'
        self.ros_lib.mkdir()
        (self.ros_lib / 'atd_fake_ros_module.py').write_text('available = True\n')
        self.setup = self.root / 'ros-setup.bash'
        self.setup.write_text(
            'export PYTHONPATH=' + shlex.quote(str(self.ros_lib)) + '\n'
            'export RMW_IMPLEMENTATION=old-ros-value\n'
            'export ROS_DISCOVERY_SERVER=old-ros-server\n')
        self.env = {
            'PATH': os.pathsep.join((str(self.bin), str(Path(sys.executable).parent), os.defpath)),
            'HOME': str(self.root),
            'ISAAC_ROS_WS': str(self.workspace),
            'ATD_PYTHON_PROJECTS_DIR': str(self.projects),
            'ATD_SYSTEM_PYTHON': sys.executable,
            'ATD_ROS_SETUP': str(self.setup),
            'ATD_TEST_LOG': str(self.log),
            'ATD_TEST_BUILD_LOG': str(self.build_log),
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        }

    def invoke(self, *args, env=None):
        return subprocess.run(['/bin/bash', str(SCRIPT), *args],
                              env={**self.env, **(env or {})}, text=True,
                              capture_output=True, timeout=15)

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def venv(self, profile):
        return self.workspace / '.atd-python/x86_64' / profile

    def test_profiles_use_separate_venvs_cache_and_system_site_policy(self):
        for profile in ('ros', 'train'):
            result = self.invoke(profile, 'sync')
            self.assertEqual(result.returncode, 0, result.stderr)
        creations = [call for call in self.calls() if call['args'][0] == 'venv']
        self.assertEqual(len(creations), 2)
        for profile, creation in zip(('ros', 'train'), creations):
            self.assertEqual(creation['args'][-1], str(self.venv(profile)))
            self.assertEqual('--system-site-packages' in creation['args'], profile == 'ros')
            self.assertIn('--no-managed-python', creation['args'])
            self.assertIn('--no-python-downloads', creation['args'])
        syncs = [call for call in self.calls() if call['args'][0] == 'sync']
        self.assertEqual(len(syncs), 2)
        for profile, call in zip(('ros', 'train'), syncs):
            self.assertIn('--locked', call['args'])
            self.assertEqual(call['env']['UV_PROJECT_ENVIRONMENT'], str(self.venv(profile)))
            self.assertEqual(call['env']['UV_CACHE_DIR'], str(self.workspace / '.atd-python/cache'))
        self.assertTrue((self.workspace / '.atd-python/COLCON_IGNORE').is_file())

    def test_existing_venv_is_reused_but_policy_mismatch_is_rejected(self):
        result = self.invoke('train', 'sync')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.log.unlink()
        result = self.invoke('train', 'sync')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([call['args'][0] for call in self.calls()], ['sync'])
        self.log.unlink()
        config = self.venv('train') / 'pyvenv.cfg'
        config.write_text(config.read_text().replace('include-system-site-packages = false', 'include-system-site-packages = true'))
        content = config.read_bytes()
        result = self.invoke('train', 'sync')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('system-site-packages', result.stderr)
        self.assertEqual(self.calls(), [])
        self.assertEqual(config.read_bytes(), content)

    def test_check_is_read_only_and_does_not_sync_or_source_ros_setup(self):
        result = self.invoke('ros', 'check')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.workspace / '.atd-python').exists())
        self.assertEqual(self.calls(), [])
        self.assertEqual(self.invoke('ros', 'sync').returncode, 0)
        self.log.unlink()
        sentinel = self.root / 'setup-was-sourced'
        self.setup.write_text('touch ' + shlex.quote(str(sentinel)) + '\n')
        before = {path: path.stat().st_mtime_ns for path in self.workspace.rglob('*')}
        result = self.invoke('ros', 'check')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(sentinel.exists())
        self.assertEqual(self.calls(), [])
        self.assertEqual({path: path.stat().st_mtime_ns for path in self.workspace.rglob('*')}, before)

    def test_training_process_does_not_inherit_ros_or_active_python_environment(self):
        result = self.invoke('train', 'run', '--', 'python', '-c',
                             'import importlib.util,json,os,sys; print(json.dumps({"prefix":sys.prefix,"ros":importlib.util.find_spec("atd_fake_ros_module") is not None,"path":os.environ.get("PYTHONPATH"),"home":os.environ.get("PYTHONHOME"),"venv":os.environ.get("VIRTUAL_ENV")}))',
                             env={'PYTHONPATH': str(self.ros_lib), 'VIRTUAL_ENV': '/old/ros-venv'})
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout.splitlines()[-1])
        self.assertEqual(data['prefix'], str(self.venv('train')))
        self.assertFalse(data['ros'])
        self.assertIsNone(data['path'])
        self.assertIsNone(data['home'])
        self.assertEqual(data['venv'], str(self.venv('train')))
        for call in self.calls():
            self.assertIsNone(call['env']['VIRTUAL_ENV'])
            self.assertIsNone(call['env']['PYTHONPATH'])

    def test_vgl_export_profile_is_isolated_and_rejected_on_jetson_before_uv(self):
        result = self.invoke('vgl-export', 'run', '--', 'python', '-c',
                             'import json,os,sys; print(json.dumps([sys.prefix,os.environ.get("PYTHONPATH")]))',
                             env={'PYTHONPATH': str(self.ros_lib)})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout.splitlines()[-1]), [str(self.venv('vgl-export')), None])
        creation = next(call for call in self.calls() if call['args'][0] == 'venv')
        self.assertNotIn('--system-site-packages', creation['args'])
        self.log.unlink()
        result = self.invoke('vgl-export', 'sync', env={'ATD_TEST_ARCH': 'aarch64'})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Notebook', result.stderr)
        self.assertEqual(self.calls(), [])

    def test_ros_setup_and_overlay_are_available_while_dds_values_are_preserved(self):
        overlay = self.workspace / 'install/setup.bash'
        overlay.parent.mkdir()
        overlay.write_text('export ATD_TEST_OVERLAY=loaded\nexport RMW_IMPLEMENTATION=old-overlay\n')
        result = self.invoke('ros', 'run', '--', 'python', '-c',
                             'import atd_fake_ros_module,json,os; print(json.dumps({key:os.environ.get(key) for key in ["RMW_IMPLEMENTATION","ROS_DISCOVERY_SERVER","ATD_TEST_OVERLAY"]}))')
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout.splitlines()[-1])
        self.assertEqual(data, {'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'ROS_DISCOVERY_SERVER': None, 'ATD_TEST_OVERLAY': 'loaded'})

    def test_parent_venv_bin_is_removed_from_training_path(self):
        old = self.root / 'old-ros-env'
        (old / 'bin').mkdir(parents=True)
        inherited = os.pathsep.join((str(old / 'bin'), self.env['PATH']))
        result = self.invoke('train', 'run', '--', 'python', '-c',
                             'import json,os; print(json.dumps(os.environ["PATH"].split(":")))',
                             env={'VIRTUAL_ENV': str(old), 'PATH': inherited})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn(str(old / 'bin'), json.loads(result.stdout.splitlines()[-1]))

    def test_run_preserves_argument_boundaries(self):
        args = ['space argument', '$(not-executed)', ';literal', '*glob']
        result = self.invoke('train', 'run', '--', 'python', '-c',
                             'import json,sys; print(json.dumps(sys.argv[1:]))', *args)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout.splitlines()[-1]), args)

    def test_missing_lock_and_invalid_action_do_not_create_environment(self):
        (self.projects / 'ros/uv.lock').unlink()
        result = self.invoke('ros', 'sync')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('uv.lock', result.stderr)
        self.assertFalse((self.workspace / '.atd-python').exists())
        self.assertEqual(self.calls(), [])
        for args in ((), ('other', 'sync'), ('train', 'build'), ('train', 'run'), ('ros', 'sync', 'unexpected')):
            with self.subTest(args=args):
                self.assertNotEqual(self.invoke(*args).returncode, 0)

    def test_sync_failure_stops_requested_command(self):
        sentinel = self.root / 'unwanted-command'
        result = self.invoke('train', 'run', '--', 'touch', str(sentinel), env={'ATD_TEST_SYNC_EXIT': '17'})
        self.assertEqual(result.returncode, 17)
        self.assertFalse(sentinel.exists())

    def test_root_cannot_create_user_environment(self):
        result = self.invoke('ros', 'sync', env={'ATD_TEST_UID': '0'})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('root', result.stderr)
        self.assertFalse((self.workspace / '.atd-python').exists())
        self.assertEqual(self.calls(), [])

    def test_build_uses_ros_interpreter_and_preserves_user_cmake_arguments(self):
        result = self.invoke('ros', 'build', '--', '--packages-select', 'atd_example',
                             '--cmake-args', '-DCMAKE_BUILD_TYPE=Release', env={'ATD_TEST_FAKE_VENV_PYTHON': '1'})
        self.assertEqual(result.returncode, 0, result.stderr)
        build = json.loads(self.build_log.read_text())
        self.assertEqual(build['cwd'], str(self.workspace))
        self.assertEqual(build['args'], ['/usr/bin/colcon', 'build', '--base-paths', str(self.workspace / 'src'),
                                         '--packages-select', 'atd_example',
                                         '--cmake-args', '-DPython3_EXECUTABLE=' + str(self.venv('ros') / 'bin/python'),
                                         '-DCMAKE_BUILD_TYPE=Release'])
        self.assertEqual(build['venv'], str(self.venv('ros')))

    def test_build_handles_inline_cmake_args_and_rejects_typed_python_override(self):
        result = self.invoke('ros', 'build', '--cmake-args=-DCMAKE_BUILD_TYPE=Release',
                             env={'ATD_TEST_FAKE_VENV_PYTHON': '1'})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(self.build_log.read_text())['args'], [
            '/usr/bin/colcon', 'build', '--base-paths', str(self.workspace / 'src'), '--cmake-args',
            '-DPython3_EXECUTABLE=' + str(self.venv('ros') / 'bin/python'), '-DCMAKE_BUILD_TYPE=Release'])
        self.build_log.unlink()
        for args in (('--cmake-args', '-DPython3_EXECUTABLE:FILEPATH=/usr/bin/python3'),
                     ('--cmake-args=-DPython3_EXECUTABLE:FILEPATH=/usr/bin/python3',)):
            with self.subTest(args=args):
                result = self.invoke('ros', 'build', *args)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('CMake 上書き', result.stderr)
                self.assertFalse(self.build_log.exists())

    def test_build_preserves_explicit_package_search_path(self):
        source = self.root / 'selected-sources'
        result = self.invoke('ros', 'build', '--base-paths', str(source),
                             env={'ATD_TEST_FAKE_VENV_PYTHON': '1'})
        self.assertEqual(result.returncode, 0, result.stderr)
        arguments = json.loads(self.build_log.read_text())['args']
        self.assertEqual(arguments.count('--base-paths'), 1)
        self.assertEqual(arguments[arguments.index('--base-paths') + 1], str(source))


if __name__ == '__main__':
    unittest.main()

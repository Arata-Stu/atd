"""UI/入力入口の配置、引数保持、readonly frontend cache を偽コマンドで検証。"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

PROJECT = Path(__file__).resolve().parents[1]


class EntryTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='atd-web-entry-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.scripts = self.root / 'scripts'
        self.scripts.mkdir()
        for name in ('webui.sh', 'input.sh'):
            shutil.copyfile(PROJECT / 'scripts' / name, self.scripts / name)
        (self.scripts / 'detect_host.sh').write_text('detect_atd_host() { printf "%s\\n" "${ATD_TEST_HOST:-pc}"; }\n')
        (self.scripts / 'python.sh').write_text('printf "%s\\0" "$@" > "$ATD_TEST_ARGS"\nexit "${ATD_TEST_EXIT:-0}"\n')
        self.output = self.root / 'args'
        self.env = {**os.environ, 'ATD_TEST_ARGS': str(self.output)}

    def invoke(self, name, *args, **env):
        return subprocess.run(['/bin/bash', str(self.scripts / name), *args],
                              env={**self.env, **env}, text=True, capture_output=True, timeout=10)

    def args(self):
        return self.output.read_bytes().decode().split('\0')[:-1]

    def test_input_is_jetson_only_and_dry_run_delegates_without_launch(self):
        result = self.invoke('input.sh')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output.exists())
        result = self.invoke('input.sh', '--dry-run', '--bridge-only', 'config:=/path with spaces/config.yaml')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args(), ['--host', 'jetson', '--dry-run', 'ros', 'run', '--',
            'ros2', 'launch', 'atd_web_bridge', 'bridge.launch.py', 'config:=/path with spaces/config.yaml'])

    def test_input_jetson_keeps_launch_arguments_and_exit_code(self):
        literal = 'profile:=$(touch should_not_run)'
        result = self.invoke('input.sh', literal, ATD_TEST_HOST='jetson', ATD_TEST_EXIT='8')
        self.assertEqual(result.returncode, 8)
        self.assertEqual(self.args()[-2:], ['input.launch.py', literal])

    def test_web_is_pc_only_and_uses_web_environment(self):
        result = self.invoke('webui.sh', ATD_TEST_HOST='jetson')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output.exists())
        result = self.invoke('webui.sh', 'serve', '--dry-run', '--port', '8877')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args(), ['--host', 'pc', '--dry-run', 'web', 'run', '--',
            '/usr/local/bin/atd-web', 'serve', '--port', '8877'])


FAKE_TOOL = r'''
import json, os, pathlib, shutil, sys
tool = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ['ATD_TEST_CALLS'], 'a') as stream:
    stream.write(json.dumps({'tool':tool,'args':args,'cwd':os.getcwd(),
        'pythonpath':os.environ.get('PYTHONPATH'), 'api_target':os.environ.get('ATD_API_TARGET'),
        'web_environment':os.environ.get('ATD_TEST_WEB_ENVIRONMENT')})+'\n')
if tool == 'uname':
    print('Linux' if args == ['-s'] else os.environ.get('ATD_TEST_ARCH','x86_64'))
elif tool == 'rsync':
    source, target = map(pathlib.Path,args[-2:])
    for item in source.iterdir():
        if item.name not in ('node_modules','dist','.source-hash'):
            shutil.copyfile(item,target/item.name)
elif tool == 'pnpm':
    if args == ['build']:
        pathlib.Path('dist').mkdir(exist_ok=True)
        pathlib.Path('dist/index.html').write_text('built')
    if args and args[0] == 'dev':
        sys.exit(int(os.environ.get('ATD_TEST_DEV_EXIT','0')))
elif tool == 'atd-python':
    assert args[:3] == ['web','run','--']
    if os.environ.get('ATD_TEST_WEB_SYNC_EXIT'):
        sys.exit(int(os.environ['ATD_TEST_WEB_SYNC_EXIT']))
    environment = dict(os.environ, ATD_TEST_WEB_ENVIRONMENT='active')
    # Real atd-python clears a parent's ROS PYTHONPATH before entering the web venv.
    environment.pop('PYTHONPATH', None)
    os.execvpe(args[3], args[3:], environment)
elif tool == 'python':
    assert os.environ.get('ATD_TEST_WEB_ENVIRONMENT') == 'active', 'system Python must not start the API'
    assert args[:3] == ['-m','uvicorn','atd_api.main:create_app']
    assert args[args.index('--host')+1] == '127.0.0.1'
    sys.exit(int(os.environ.get('ATD_TEST_SERVER_EXIT','0')))
'''


class ContainerWebTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='atd-web-container-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.project = self.root / 'project with spaces'
        self.source = self.project / 'apps/web/frontend'
        self.source.mkdir(parents=True)
        (self.source / 'package.json').write_text('{}')
        (self.source / 'pnpm-lock.yaml').write_text('lockfileVersion: 9')
        self.workspace = self.root / 'workspace with spaces'
        self.workspace.mkdir()
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.calls_file = self.root / 'calls'
        for tool in ('uname', 'uv', 'node', 'pnpm', 'rsync', 'python', 'atd-python'):
            path = self.bin / tool
            path.write_text(f'#!{sys.executable}\n'+FAKE_TOOL)
            path.chmod(0o755)
        (self.bin / 'python3').symlink_to(sys.executable)
        self.env = {**os.environ, 'PATH': str(self.bin)+os.pathsep+os.defpath,
                    'ATD_PROJECT_DIR': str(self.project), 'ISAAC_ROS_WS': str(self.workspace),
                    'ATD_TEST_CALLS': str(self.calls_file)}

    def invoke(self, *args, **env):
        return subprocess.run(['/bin/bash', str(PROJECT/'docker/scripts/atd-web.sh'), *args],
                              env={**self.env, **env}, text=True, capture_output=True, timeout=15)

    def calls(self, name=None):
        values = [json.loads(s) for s in self.calls_file.read_text().splitlines()] if self.calls_file.exists() else []
        return [v for v in values if name is None or v['tool'] == name]

    def test_serve_builds_only_in_cache_and_reuses_unchanged_bundle(self):
        result = self.invoke('serve', '--port', '8888')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse((self.source/'dist').exists())
        self.assertTrue((self.workspace/'.atd-web/frontend/dist/index.html').exists())
        self.assertEqual([v['args'] for v in self.calls('pnpm')], [['install','--frozen-lockfile'],['build']])
        command = self.calls('python')[0]
        self.assertIn(str(self.project/'apps/web/backend/src'), command['pythonpath'])
        self.assertEqual(command['args'][-1], '8888')
        self.calls_file.unlink()
        result = self.invoke('serve', ATD_TEST_SERVER_EXIT='6')
        self.assertEqual(result.returncode, 6)
        self.assertEqual(self.calls('pnpm'), [])

    def test_direct_container_entry_activates_web_environment_before_api(self):
        result = self.invoke('serve', PYTHONPATH='/opt/ros/lyrical/lib/python3.12/site-packages')
        self.assertEqual(result.returncode, 0, result.stderr)
        activation, = self.calls('atd-python')
        self.assertEqual(activation['args'][:4], ['web', 'run', '--', 'env'])
        api, = self.calls('python')
        self.assertEqual(api['web_environment'], 'active')
        self.assertEqual(api['pythonpath'], os.pathsep.join((
            str(self.project/'apps/web/backend/src'), str(self.project/'apps/web/processing'))))
        self.assertNotIn('/opt/ros/', api['pythonpath'])

    def test_web_environment_failure_stops_before_api_start(self):
        result = self.invoke('serve', ATD_TEST_WEB_SYNC_EXIT='9')
        self.assertEqual(result.returncode, 9, result.stderr)
        self.assertEqual(self.calls('python'), [])

    def test_invalid_port_and_jetson_are_rejected_before_build(self):
        for args, env in ((['serve','--port','80'], {}), (['serve'], {'ATD_TEST_ARCH':'aarch64'})):
            with self.subTest(args=args, env=env):
                self.calls_file.unlink(missing_ok=True)
                result = self.invoke(*args, **env)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.calls('pnpm'), [])
                self.assertEqual(self.calls('python'), [])

    def test_dev_points_vite_at_selected_loopback_api(self):
        result = self.invoke('dev', '--port', '8870', ATD_TEST_DEV_EXIT='7')
        self.assertEqual(result.returncode, 7, result.stderr)
        dev = [v for v in self.calls('pnpm') if v['args'][0] == 'dev'][0]
        self.assertEqual(dev['api_target'], 'http://127.0.0.1:8870')
        self.assertIn('--strictPort', dev['args'])


if __name__ == '__main__':
    unittest.main()

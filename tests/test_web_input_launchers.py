"""bringup/Web補助入口の引数・構成選択、readonly frontend cache を偽コマンドで検証。"""
import json
import os
from pathlib import Path
import pty
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
        (self.scripts / 'lib').mkdir()
        (self.scripts / 'tools').mkdir()
        shutil.copyfile(PROJECT / 'scripts/bringup.sh', self.scripts / 'bringup.sh')
        shutil.copyfile(PROJECT / 'scripts/lib/bringup.sh', self.scripts / 'lib/bringup.sh')
        shutil.copyfile(PROJECT / 'scripts/tools/webui.sh', self.scripts / 'tools/webui.sh')
        (self.scripts / 'lib/detect_host.sh').write_text('detect_atd_host() { printf "%s\\n" "${ATD_TEST_HOST:-pc}"; }\n')
        (self.scripts / 'tools/python.sh').write_text('printf "%s\\0" "$@" > "$ATD_TEST_ARGS"\nexit "${ATD_TEST_EXIT:-0}"\n')
        self.output = self.root / 'args'
        self.env = {**os.environ, 'ATD_TEST_ARGS': str(self.output)}

    def invoke(self, name, *args, **env):
        return subprocess.run(['/bin/bash', str(self.scripts / name), *args],
                              env={**self.env, **env}, text=True, capture_output=True, timeout=10)

    def args(self):
        return self.output.read_bytes().decode().split('\0')[:-1]

    def test_bringup_is_jetson_only_and_dry_run_delegates_without_launch(self):
        result = self.invoke('bringup.sh', 'input')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output.exists())
        result = self.invoke('bringup.sh', '--dry-run', 'bridge', 'config:=/path with spaces/config.yaml')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args(), ['--host', 'jetson', '--dry-run', 'ros', 'run', '--',
            'ros2', 'launch', 'atd_web_bridge', 'bridge.launch.py', 'config:=/path with spaces/config.yaml'])

    def test_bringup_keeps_literal_launch_arguments_and_exit_code(self):
        marker = self.root / 'must-not-exist'
        literal = f'profile:=$(touch {marker})'
        result = self.invoke('bringup.sh', 'input', literal, ATD_TEST_HOST='jetson', ATD_TEST_EXIT='8')
        self.assertEqual(result.returncode, 8)
        self.assertEqual(self.args()[-2:], ['input.launch.py', literal])
        self.assertFalse(marker.exists())

    def test_bare_dry_run_and_zero_parameter_profiles_support_bash_three(self):
        for arguments, launch in ((('--dry-run',), 'input.launch.py'),
                                  (('--dry-run', 'input'), 'input.launch.py'),
                                  (('--dry-run', 'bridge'), 'bridge.launch.py')):
            with self.subTest(arguments=arguments):
                result = self.invoke('bringup.sh', *arguments)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(self.args()[-1], launch)

    def test_help_list_and_invalid_profiles_do_not_launch(self):
        for option in ('--help', '--list'):
            result = self.invoke('bringup.sh', option)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('input', result.stdout)
            self.assertIn('bridge', result.stdout)
            self.assertFalse(self.output.exists())
        for arguments in ((), ('pico_serial:=pico-1',), ('autonomy',), ('input', 'bridge'), ('--unknown',)):
            with self.subTest(arguments=arguments):
                result = self.invoke('bringup.sh', *arguments, ATD_TEST_HOST='jetson')
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.output.exists())

    def test_launch_options_after_separator_keep_order(self):
        result = self.invoke('bringup.sh', '--dry-run', 'bridge', '--', '--show-args')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args()[-2:], ['bridge.launch.py', '--show-args'])

    def test_profile_picker_uses_only_registered_names(self):
        fake_bin = self.root / 'bin'
        fake_bin.mkdir()
        picker = fake_bin / 'fzf'
        picker.write_text('#!/bin/sh\ncat >/dev/null\nprintf "%s\\n" "$ATD_TEST_PICK"\n')
        picker.chmod(0o755)
        helper = self.scripts / 'lib/bringup.sh'
        for selection, succeeds in (('bridge\tobservations', True), ('../../other', False),
                                    ('$(touch forbidden)', False)):
            with self.subTest(selection=selection):
                # メニュー出力も実行コードとして解釈せず、登録済み構成だけを解決する。
                result = subprocess.run(['/bin/bash', '-c',
                    'source "$1"; selected="$(atd_bringup_choose)"; atd_bringup_lookup "$selected"',
                    'picker-test', str(helper)], text=True, capture_output=True, timeout=10,
                    env={**self.env, 'PATH': str(fake_bin) + os.pathsep + os.defpath,
                         'ATD_TEST_PICK': selection})
                self.assertEqual(result.returncode == 0, succeeds, result.stderr)
                if succeeds:
                    self.assertEqual(result.stdout.strip(), 'jetson\tatd_web_bridge\tbridge.launch.py\tplain')

    def test_interactive_entry_chooses_bridge_and_cancel_does_not_launch(self):
        fake_bin = self.root / 'menu-bin'
        fake_bin.mkdir()
        picker = fake_bin / 'fzf'
        picker.write_text('#!/bin/sh\ncat >/dev/null\n'
                          '[ "$ATD_TEST_PICK" != cancel ] || exit 130\n'
                          'printf "%s\\n" "$ATD_TEST_PICK"\n')
        picker.chmod(0o755)
        for selected in ('bridge\tmonitor', 'quit\tquit', 'cancel'):
            with self.subTest(selected=selected):
                self.output.unlink(missing_ok=True)
                master, slave = pty.openpty()
                try:
                    process = subprocess.Popen(['/bin/bash', str(self.scripts / 'bringup.sh')],
                        stdin=slave, stdout=slave, stderr=subprocess.PIPE,
                        env={**self.env, 'PATH': str(fake_bin) + os.pathsep + os.defpath,
                             'ATD_TEST_HOST': 'jetson', 'ATD_TEST_PICK': selected})
                    try:
                        _, error = process.communicate(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.communicate()
                        raise
                    self.assertEqual(process.returncode, 0, error.decode())
                finally:
                    os.close(master)
                    os.close(slave)
                if selected.startswith('bridge'):
                    self.assertEqual(self.args()[-1], 'bridge.launch.py')
                else:
                    self.assertFalse(self.output.exists())

    def test_number_picker_selects_bridge_or_quits_without_fzf(self):
        helper = self.scripts / 'lib/bringup.sh'
        commands = self.root / 'number-menu-bin'
        commands.mkdir()
        awk = shutil.which('awk')
        self.assertIsNotNone(awk)
        (commands / 'awk').symlink_to(awk)
        for response, expected in (('2\n', 'bridge'), ('3\n', 'd455'), ('5\n', 'quit'), ('\n', ''), ('90\n', '')):
            with self.subTest(response=response):
                result = subprocess.run(['/bin/bash', '-c', 'source "$1"; atd_bringup_choose',
                    'picker-test', str(helper)], input=response, text=True, capture_output=True,
                    timeout=10, env={**self.env, 'PATH': str(commands)})
                self.assertEqual(result.stdout.strip(), expected)
                self.assertEqual(result.returncode == 0, bool(expected))

    def test_camera_profiles_are_direct_in_noninteractive_execution(self):
        for profile in ('d455', 'manual'):
            with self.subTest(profile=profile):
                result = self.invoke('bringup.sh', profile, 'camera_serial:=123456', ATD_TEST_HOST='jetson')
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(self.args()[-5:], ['ros2', 'launch', 'atd_bringup', profile + '.launch.py',
                                                   'camera_serial:=123456'])

    def test_camera_dry_run_has_no_tui_and_marks_capabilities_unqueried(self):
        result = self.invoke('bringup.sh', '--dry-run', 'd455')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('未照会', result.stdout)
        self.assertIn('200Hz', result.stdout)
        self.assertEqual(self.args()[-4:], ['ros2', 'launch', 'atd_bringup', 'd455.launch.py'])

    def test_explicit_configure_is_camera_only_and_requires_tty_unless_dry_run(self):
        for arguments in (('input', '--configure'), ('d455', '--configure')):
            with self.subTest(arguments=arguments):
                result = self.invoke('bringup.sh', *arguments, ATD_TEST_HOST='jetson')
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.output.exists())
        result = self.invoke('bringup.sh', '--dry-run', '--configure', 'manual')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args()[-7:], ['ros2', 'run', 'atd_sensors', 'd455_setup',
                                          '--bringup-profile', 'manual', '--'])

    def test_camera_interactive_entry_opens_tui_and_arguments_allow_direct_launch(self):
        for arguments, expected in ((('d455',), 'd455_setup'),
                                     (('manual', 'enable_depth:=false'), 'manual.launch.py'),
                                     (('manual', '--configure', 'enable_depth:=false'), 'd455_setup')):
            with self.subTest(arguments=arguments):
                master, slave = pty.openpty()
                try:
                    result = subprocess.run(['/bin/bash', str(self.scripts / 'bringup.sh'), *arguments],
                        stdin=slave, stdout=slave, stderr=subprocess.PIPE,
                        env={**self.env, 'ATD_TEST_HOST': 'jetson'}, timeout=10)
                    self.assertEqual(result.returncode, 0, result.stderr.decode())
                finally:
                    os.close(master)
                    os.close(slave)
                self.assertIn(expected, self.args())

    def test_web_is_pc_only_and_uses_web_environment(self):
        result = self.invoke('tools/webui.sh', ATD_TEST_HOST='jetson')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output.exists())
        result = self.invoke('tools/webui.sh', 'serve', '--dry-run', '--port', '8877')
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
        'web_environment':os.environ.get('ATD_TEST_WEB_ENVIRONMENT'),
        'raceline_python':os.environ.get('ATD_RACELINE_PYTHON')})+'\n')
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
    if args == ['raceline','sync']:
        failure = int(os.environ.get('ATD_TEST_RACELINE_SYNC_EXIT','0'))
        if failure:
            sys.exit(failure)
        helper = pathlib.Path(os.environ['ISAAC_ROS_WS']) / '.atd-python/x86_64/raceline/bin/python'
        helper.parent.mkdir(parents=True,exist_ok=True)
        helper.write_text('#!/bin/sh\nexit 0\n')
        helper.chmod(0o755)
        sys.exit(0)
    if args in (['web','check'],['raceline','check']):
        sys.exit(0)
    assert args[:3] == ['web','run','--']
    if os.environ.get('ATD_TEST_WEB_SYNC_EXIT'):
        sys.exit(int(os.environ['ATD_TEST_WEB_SYNC_EXIT']))
    environment = dict(os.environ, ATD_TEST_WEB_ENVIRONMENT='active')
    # Real atd-python clears a parent's ROS PYTHONPATH before entering the web venv.
    environment.pop('PYTHONPATH', None)
    os.execvpe(args[3], args[3:], environment)
elif tool == 'python':
    assert os.environ.get('ATD_TEST_WEB_ENVIRONMENT') == 'active', 'system Python must not start the API'
    helper = pathlib.Path(os.environ['ATD_RACELINE_PYTHON'])
    assert helper.is_file() and os.access(helper,os.X_OK), 'dedicated helper executable must reach the API'
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
        self.env.pop('ATD_RACELINE_PYTHON', None)

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
        activation, = [call for call in self.calls('atd-python') if call['args'][:2] == ['web','run']]
        self.assertEqual(self.calls('atd-python')[0]['args'], ['raceline','sync'])
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

    def test_default_helper_sync_creates_dedicated_executable_and_exports_it(self):
        result = self.invoke('serve')
        self.assertEqual(result.returncode, 0, result.stderr)
        helper = self.workspace / '.atd-python/x86_64/raceline/bin/python'
        self.assertTrue(helper.is_file())
        self.assertTrue(os.access(helper, os.X_OK))
        self.assertEqual([call['args'][:2] for call in self.calls('atd-python')],
                         [['raceline', 'sync'], ['web', 'run']])
        api, = self.calls('python')
        self.assertEqual(api['raceline_python'], str(helper))
        self.assertNotEqual(api['raceline_python'], str(self.bin / 'python'))

    def test_helper_sync_failure_stops_api_without_falling_back(self):
        result = self.invoke('serve', ATD_TEST_RACELINE_SYNC_EXIT='13')
        self.assertEqual(result.returncode, 13, result.stderr)
        self.assertEqual([call['args'] for call in self.calls('atd-python')], [['raceline', 'sync']])
        self.assertEqual(self.calls('python'), [])
        self.assertFalse((self.workspace / '.atd-python/x86_64/raceline/bin/python').exists())

    def test_helper_override_skips_sync_and_is_preserved_in_api_environment(self):
        helper = self.root / 'custom helper with spaces'
        helper.write_text('#!/bin/sh\nexit 0\n')
        helper.chmod(0o755)
        result = self.invoke('serve', ATD_RACELINE_PYTHON=str(helper), ATD_TEST_RACELINE_SYNC_EXIT='13')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([call['args'][:2] for call in self.calls('atd-python')], [['web', 'run']])
        self.assertEqual(self.calls('python')[0]['raceline_python'], str(helper))
        self.assertFalse((self.workspace / '.atd-python').exists())

    def test_missing_or_nonexecutable_helper_override_does_not_start_api(self):
        missing = self.root / 'missing helper'
        not_executable = self.root / 'not executable'
        not_executable.write_text('helper')
        not_executable.chmod(0o644)
        for helper in (missing, not_executable, self.bin):
            with self.subTest(helper=helper):
                self.calls_file.unlink(missing_ok=True)
                result = self.invoke('serve', ATD_RACELINE_PYTHON=str(helper))
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('helper', result.stderr)
                self.assertEqual(self.calls('atd-python'), [])
                self.assertEqual(self.calls('python'), [])

    def test_build_only_does_not_sync_numerical_helper_or_start_api(self):
        result = self.invoke('build', ATD_TEST_RACELINE_SYNC_EXIT='13')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue((self.workspace / '.atd-web/frontend/dist/index.html').exists())
        self.assertEqual(self.calls('atd-python'), [])
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

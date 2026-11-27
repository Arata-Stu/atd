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
        shutil.copy2(PROJECT / 'scripts/webui.sh', self.scripts / 'webui.sh')
        shutil.copyfile(PROJECT / 'scripts/tools/webui.sh', self.scripts / 'tools/webui.sh')
        (self.scripts / 'lib/detect_host.sh').write_text('detect_atd_host() { printf "%s\\n" "${ATD_TEST_HOST:-pc}"; }\n')
        (self.scripts / 'lib/container.sh').write_text('atd_container_exec() { shift; printf "%s\\0" "$@" > "$ATD_TEST_ARGS"; }\n')
        (self.scripts / 'tools/python.sh').write_text('printf "%s\\0" "$@" > "$ATD_TEST_ARGS"\nexit "${ATD_TEST_EXIT:-0}"\n')
        self.output = self.root / 'args'
        self.env = {**os.environ, 'ATD_TEST_ARGS': str(self.output)}

    def invoke(self, name, *args, **env):
        return subprocess.run(['/bin/bash', str(self.scripts / name), *args],
                              env={**self.env, **env}, text=True, capture_output=True, timeout=10)

    def args(self):
        return self.output.read_bytes().decode().split('\0')[:-1]

    def test_vehicle_profile_is_jetson_only_and_dry_run_delegates_without_launch(self):
        result = self.invoke('bringup.sh', 'input')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output.exists())
        result = self.invoke('bringup.sh', '--dry-run', 'bridge', 'config:=/path with spaces/config.yaml')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args(), ['--host', 'jetson', '--dry-run', 'ros', 'run', '--',
            'ros2', 'launch', 'atd_web_bridge', 'bridge.launch.py', 'config:=/path with spaces/config.yaml'])

    def test_simulation_uses_pc_container_domain_and_explicit_clock(self):
        for domain, extra, profile in (('74', [], '/workspaces/atd/docker/gazebo_monza/fastdds.xml'),
                                       ('91', ['use_sim_time:=true', '--dds', 'udp'], '/etc/atd/fastdds.xml')):
            with self.subTest(domain=domain):
                env = {} if domain == '74' else {'ATD_GAZEBO_ROS_DOMAIN_ID': domain}
                result = self.invoke('bringup.sh', '--dry-run', 'gazebo', *extra, **env)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(self.args(), ['--host', 'pc', '--dry-run', 'ros', 'run', '--',
                    'env', 'ROS_DOMAIN_ID=' + domain,
                    'FASTDDS_DEFAULT_PROFILES_FILE=' + profile,
                    'ros2', 'launch', 'atd_bringup', 'gazebo.launch.py', 'use_sim_time:=true'])

    def test_rejects_mixed_vehicle_simulation_or_bad_domain(self):
        for args, env in (
                (('gazebo',), {'ATD_TEST_HOST': 'jetson'}),
                (('--dry-run', 'gazebo', 'use_sim_time:=false'), {}),
                (('--dry-run', 'gazebo', 'joy_drive:=true'), {}),
                (('--dry-run', 'gazebo', '--dds', 'wrong'), {}),
                (('--dry-run', 'input', '--dds', 'shm'), {}),
                (('--dry-run', 'gazebo'), {'ATD_GAZEBO_ROS_DOMAIN_ID': '233'}),
                (('--dry-run', 'localization', 'map_id:=M', 'use_sim_time:=true'), {})):
            with self.subTest(args=args, env=env):
                self.assertNotEqual(self.invoke('bringup.sh', *args, **env).returncode, 0)
                self.assertFalse(self.output.exists())

    def test_esc_trial_preserves_tty_entry_and_board_selection(self):
        result = self.invoke('bringup.sh', '--dry-run', 'esc-trial', 'port:=/dev/serial/by-id/board-if00')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args()[-5:], ['ros2', 'run', 'atd_bringup', 'esc_trial_session',
                                           'port:=/dev/serial/by-id/board-if00'])
        self.assertNotEqual(self.invoke('bringup.sh', 'esc-trial', ATD_TEST_HOST='jetson').returncode, 0)

    def test_joy_drive_delegates_to_camera_free_launch_with_control_port(self):
        result = self.invoke('bringup.sh', 'joy-drive')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output.exists())
        port = 'joy_drive_port:=/dev/serial/by-id/board-if00'
        result = self.invoke('bringup.sh', '--dry-run', 'joy-drive', port, 'pico_serial:=012345')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args(), ['--host', 'jetson', '--dry-run', 'ros', 'run', '--',
            'ros2', 'launch', 'atd_bringup', 'joy_drive.launch.py', port, 'pico_serial:=012345'])

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
                    'source "$1"; selected="$(atd_bringup_choose jetson)"; atd_bringup_lookup "$selected"',
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
        for response, expected in (('2\n', 'bridge'), ('3\n', 'd455'), ('5\n', 'localization'),
                                   ('6\n', 'esc-trial'), ('7\n', 'joy-drive'), ('8\n', 'autonomous'), ('9\n', 'quit'),
                                   ('\n', ''), ('90\n', '')):
            with self.subTest(response=response):
                result = subprocess.run(['/bin/bash', '-c', 'source "$1"; atd_bringup_choose jetson',
                    'picker-test', str(helper)], input=response, text=True, capture_output=True,
                    timeout=10, env={**self.env, 'PATH': str(commands)})
                self.assertEqual(result.stdout.strip(), expected)
                self.assertEqual(result.returncode == 0, bool(expected))

        result = subprocess.run(['/bin/bash', '-c', 'source "$1"; atd_bringup_choose pc',
            'picker-test', str(helper)], input='1\n', text=True, capture_output=True,
            timeout=10, env={**self.env, 'PATH': str(commands)})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), 'gazebo')
        self.assertNotIn('(jetson)', result.stderr)

    def test_camera_profiles_are_direct_in_noninteractive_execution(self):
        for profile in ('d455', 'manual'):
            with self.subTest(profile=profile):
                result = self.invoke('bringup.sh', profile, 'camera_serial:=123456', ATD_TEST_HOST='jetson')
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(self.args()[-5:], ['ros2', 'launch', 'atd_bringup', profile + '.launch.py',
                                                   'camera_serial:=123456'])

    def test_localization_direct_or_interactive_map_choice(self):
        for args in (('localization',), ('localization', 'map_id:='), ('localization', 'map_id:=A', 'map_id:=B')):
            result = self.invoke('bringup.sh', *args, ATD_TEST_HOST='jetson')
            self.assertNotEqual(result.returncode, 0, result.stdout)
        result = self.invoke('bringup.sh', '--dry-run', 'localization')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args()[-3:], ['run', 'atd_bringup', 'choose_localization_map'])
        result = self.invoke('bringup.sh', 'localization', 'map_id:=track', 'pico_serial:=PICO', ATD_TEST_HOST='jetson')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args()[-4:], ['atd_bringup', 'localization.launch.py', 'map_id:=track', 'pico_serial:=PICO'])

    def test_localization_passes_selected_line_to_jetson(self):
        for kind in ('raceline', 'centerline', 'custom'):
            result = self.invoke('bringup.sh', '--dry-run', 'localization', 'map_id:=track', 'line:=' + kind)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(self.args()[-2:], ['map_id:=track', 'line:=' + kind])
        for line in ('line:=', 'line:=other'):
            self.assertNotEqual(self.invoke('bringup.sh', '--dry-run', 'localization', line).returncode, 0)

    def test_autonomous_uses_dedicated_launch_or_map_picker(self):
        self.assertNotEqual(self.invoke('bringup.sh', 'autonomous', 'map_id:=track').returncode, 0)
        result = self.invoke('bringup.sh', '--dry-run', 'autonomous')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args()[-4:], ['run', 'atd_bringup', 'choose_localization_map', '--autonomous'])
        for kind in ('raceline', 'centerline', 'custom'):
            args = ['map_id:=track', 'line:=' + kind, 'max_speed_mps:=0.3']
            result = self.invoke('bringup.sh', 'autonomous', *args, ATD_TEST_HOST='jetson')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(self.args()[-5:], ['atd_bringup', 'autonomous.launch.py', *args])
        result = self.invoke('bringup.sh', '--dry-run', 'autonomous', 'map_id:=track', 'joy_drive:=true')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotEqual(self.invoke('bringup.sh', '--dry-run', 'localization', '--', '--autonomous').returncode, 0)

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

    def test_web_is_pc_only_and_delegates_without_python_sync(self):
        result = self.invoke('webui.sh', ATD_TEST_HOST='jetson')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output.exists())
        for entry in ('webui.sh', 'tools/webui.sh'):
            with self.subTest(entry=entry):
                result = self.invoke(entry, 'serve', '--dry-run', '--port', '8877')
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(self.args(), ['pc', 'true', '/usr/local/bin/atd-web', 'serve', '--port', '8877'])
        result = self.invoke('webui.sh', 'prepare')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args(), ['pc', 'false', '/usr/local/bin/atd-web', 'prepare'])


FAKE_TOOL = r'''
import json, os, pathlib, shutil, sys
tool = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
if tool == 'python' and args[:2] == ['-I', '-c']:
    os.execv(sys.executable, [sys.executable, *args])
with open(os.environ['ATD_TEST_CALLS'], 'a') as stream:
    stream.write(json.dumps({'tool':tool,'args':args,'cwd':os.getcwd(),
        'pythonpath':os.environ.get('PYTHONPATH'), 'api_target':os.environ.get('ATD_API_TARGET'),
        'venv':os.environ.get('VIRTUAL_ENV'), 'path':os.environ.get('PATH'),
        'executable':sys.argv[0],
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
        if os.environ.get('ATD_TEST_BUILD_EXIT'):
            sys.exit(int(os.environ['ATD_TEST_BUILD_EXIT']))
        pathlib.Path('dist').mkdir(exist_ok=True)
        pathlib.Path('dist/index.html').write_text('built')
    if args and args[0] == 'dev':
        sys.exit(int(os.environ.get('ATD_TEST_DEV_EXIT','0')))
elif tool == 'atd-python':
    assert args in (['web','sync'], ['raceline','sync'])
    profile = args[0]
    failure = int(os.environ.get(f'ATD_TEST_{profile.upper()}_SYNC_EXIT','0'))
    if failure:
        sys.exit(failure)
    venv = pathlib.Path(os.environ['ISAAC_ROS_WS']) / '.atd-python/x86_64' / profile
    (venv / 'bin').mkdir(parents=True, exist_ok=True)
    (venv / 'pyvenv.cfg').write_text(f'include-system-site-packages = false\nversion = {sys.version.split()[0]}\n')
    shutil.copy2(pathlib.Path(sys.argv[0]).with_name('python'), venv / 'bin/python')
elif tool == 'python':
    assert pathlib.Path(sys.argv[0]) == pathlib.Path(os.environ['VIRTUAL_ENV'])/'bin/python'
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
        self.profiles = self.project / 'docker/python'
        for profile in ('web', 'raceline'):
            (self.profiles/profile).mkdir(parents=True)
            (self.profiles/profile/'uv.lock').write_text('locked')
            (self.profiles/profile/'pyproject.toml').write_text('[project]')
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
                    'ATD_SYSTEM_PYTHON': sys.executable, 'ATD_PYTHON_PROJECTS_DIR': str(self.profiles),
                    'ATD_TEST_CALLS': str(self.calls_file)}
        self.env.pop('ATD_RACELINE_PYTHON', None)

    def invoke(self, *args, **env):
        return subprocess.run(['/bin/bash', str(PROJECT/'docker/scripts/atd-web.sh'), *args],
                              env={**self.env, **env}, text=True, capture_output=True, timeout=15)

    def calls(self, name=None):
        values = [json.loads(s) for s in self.calls_file.read_text().splitlines()] if self.calls_file.exists() else []
        return [v for v in values if name is None or v['tool'] == name]

    def prepare(self, **env):
        result = self.invoke('prepare', **env)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls('python'), [])
        self.calls_file.unlink()

    def test_prepare_builds_in_workspace_and_default_serves_with_no_sync_or_build(self):
        result = self.invoke('prepare')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls('python'), [])
        self.assertFalse((self.source/'dist').exists())
        self.assertTrue((self.workspace/'.atd-web/frontend/dist/index.html').exists())
        self.assertEqual([v['args'] for v in self.calls('pnpm')], [['install','--frozen-lockfile'],['build']])
        self.assertEqual([v['args'] for v in self.calls('atd-python')], [['web','sync'],['raceline','sync']])
        self.calls_file.unlink()
        other_venv = self.root/'parent'
        env = dict(PYTHONPATH='/opt/ros/lyrical/lib/python3.12/site-packages',
                   VIRTUAL_ENV=str(other_venv), PATH=f"{other_venv}/bin:{self.env['PATH']}")
        result = self.invoke('--port', '8888', ATD_TEST_SERVER_EXIT='6', **env)
        self.assertEqual(result.returncode, 6, result.stderr)
        api, = self.calls('python')
        self.assertEqual(api['args'][-1], '8888')
        self.assertEqual(api['venv'], str(self.workspace/'.atd-python/x86_64/web'))
        self.assertEqual(api['executable'], str(self.workspace/'.atd-python/x86_64/web/bin/python'))
        self.assertEqual(api['raceline_python'], str(self.workspace/'.atd-python/x86_64/raceline/bin/python'))
        self.assertEqual(api['pythonpath'], os.pathsep.join((
            str(self.project/'apps/web/backend/src'), str(self.project/'apps/web/processing'))))
        self.assertNotIn(str(other_venv/'bin'), api['path'].split(os.pathsep))
        self.assertEqual({v['tool'] for v in self.calls()}, {'uname', 'python'})

    def test_unprepared_or_stale_source_lock_or_bundle_never_triggers_implicit_install(self):
        def check():
            self.calls_file.unlink(missing_ok=True)
            result = self.invoke('serve')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('atd-web prepare', result.stderr)
            self.assertFalse({v['tool'] for v in self.calls()} - {'uname'})

        check()
        self.prepare()
        for path in (self.source/'package.json', self.source/'pnpm-lock.yaml', self.profiles/'web/uv.lock',
                     self.profiles/'raceline/pyproject.toml', self.workspace/'.atd-web/frontend/dist/index.html'):
            with self.subTest(path=path):
                saved = path.read_text()
                if path.name == 'index.html':
                    path.unlink()
                else:
                    path.write_text(saved+' changed')
                check()
                path.write_text(saved)

    def test_failed_reprepare_cannot_leave_ready_status(self):
        for flag in ('ATD_TEST_BUILD_EXIT', 'ATD_TEST_WEB_SYNC_EXIT', 'ATD_TEST_RACELINE_SYNC_EXIT'):
            with self.subTest(flag=flag):
                self.prepare()
                result = self.invoke('prepare', **{flag: '13'})
                self.assertEqual(result.returncode, 13, result.stderr)
                self.assertEqual(self.calls('python'), [])
                self.calls_file.unlink()
                result = self.invoke('serve')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('atd-web prepare', result.stderr)
                self.assertEqual(self.calls('python'), [])

    def test_helper_override_skips_sync_and_is_preserved_in_api_environment(self):
        helper = self.root/'custom helper with spaces'
        helper.write_text('#!/bin/sh\nexit 0\n')
        helper.chmod(0o755)
        self.prepare(ATD_RACELINE_PYTHON=str(helper), ATD_TEST_RACELINE_SYNC_EXIT='13')
        self.assertFalse((self.workspace/'.atd-python/x86_64/raceline').exists())
        result = self.invoke('serve', ATD_RACELINE_PYTHON=str(helper))
        self.assertEqual(result.returncode, 0, result.stderr)
        api, = self.calls('python')
        self.assertEqual(api['raceline_python'], str(helper))

    def test_missing_or_nonexecutable_helper_override_does_not_start_api(self):
        self.prepare()
        missing = self.root/'missing helper'
        not_executable = self.root/'not executable'
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

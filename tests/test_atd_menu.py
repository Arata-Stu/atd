"""日常操作メニューを、実機や Docker を起動せずに確認する。"""
import json
import os
from pathlib import Path
import pty
import shlex
import shutil
import subprocess
import tempfile
import unittest

PROJECT = Path(__file__).resolve().parents[1]


class AtdMenuTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.scripts = self.root / 'scripts'
        self.scripts.mkdir()
        self.atd = self.scripts / 'atd'
        shutil.copy2(PROJECT / 'scripts/atd', self.atd)
        shutil.copytree(PROJECT / 'scripts/lib', self.scripts / 'lib')
        (self.scripts / 'lib/detect_host.sh').write_text(
            'detect_atd_host() { echo "$MENU_TEST_HOST"; }\n'
            'atd_in_container() { [[ "${MENU_TEST_CONTAINER:-0}" = 1 ]]; }\n')

        # PATH に登録するのはメニューの表示に必要なコマンドだけ。
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        for tool in ('bash', 'cat', 'dirname', 'readlink', 'sed'):
            (self.bin / tool).symlink_to(shutil.which(tool))
        uname = self.bin / 'uname'
        uname.write_text('#!/bin/sh\nif [ "$1" = -s ]; then echo "${MENU_TEST_OS:-Linux}"; '
                         'elif [ "$MENU_TEST_HOST" = jetson ]; then echo aarch64; else echo x86_64; fi\n')
        uname.chmod(0o755)
        self.calls = self.root / 'calls'
        self.ssh_args = self.root / 'ssh-args'
        self.host_calls = self.root / 'host-calls'
        self.env = {'PATH': str(self.bin), 'HOME': str(self.root), 'TERM': 'xterm',
                    'MENU_TEST_HOST': 'pc', 'MENU_TEST_CALLS': str(self.calls),
                    'MENU_TEST_SSH_ARGS': str(self.ssh_args), 'MENU_TEST_HOST_CALLS': str(self.host_calls)}
        ssh = self.bin / 'ssh'
        # コンテナ内で試験する場合も、模擬 SSH 先はホストのファイル配置を使う。
        ssh.write_text('#!/bin/bash\nprintf "%s\\0" "$@" > "$MENU_TEST_SSH_ARGS"\n'
                       'if [[ -n "${MENU_TEST_SSH_RC:-}" ]]; then exit "$MENU_TEST_SSH_RC"; fi\n'
                       'remote="${!#}"\n'
                       'remote="${remote//\\/.dockerenv/fixture.dockerenv}"\n'
                       'remote="${remote//\\/run\\/.containerenv/fixture.containerenv}"\n'
                       'exec /bin/sh -c "$remote"\n')
        ssh.chmod(0o755)
        remote = self.root / '.local/bin/atd'
        remote.parent.mkdir(parents=True)
        remote.write_text('#!/bin/sh\nprintf "%s\\0" "$@" > "$MENU_TEST_HOST_CALLS"\n'
                          'exit "${MENU_TEST_REMOTE_RC:-0}"\n')
        remote.chmod(0o755)
        for filename in ('build', 'dev', 'setup', 'webui', 'gazebo', 'replay', 'bluetooth', 'hotspot',
                         'bringup', 'vgl', 'jetson_display_mode', 'jetson_max_performance'):
            wrapper = self.scripts / (filename + '.sh')
            list_stub = 'if [[ "$1" = list ]]; then printf "odaiba\\n"; exit; fi\n' if filename == 'gazebo' else ''
            wrapper.write_text('#!/bin/bash\n' + list_stub +
                               'printf "%s\\0" "${0##*/}" "$@" >> "$MENU_TEST_CALLS"\n'
                               'exit "${MENU_TEST_RC:-0}"\n')
            wrapper.chmod(0o755)

    def invoke(self, *args, **env):
        return subprocess.run([str(self.atd), *args], env={**self.env, **env}, text=True,
                              cwd=self.root, stdin=subprocess.DEVNULL, capture_output=True, timeout=5)

    def assert_call(self, *args):
        self.assertEqual(self.calls.read_bytes().split(b'\0'), [arg.encode() for arg in args] + [b''])
        self.calls.unlink()

    def menu(self, *args, picks=None, keys=b'', **env):
        if picks is not None:
            fzf = self.bin / 'fzf'
            fzf.write_text('''#!/bin/bash
step=0
if [[ -f "$HOME/step" ]]; then read -r step < "$HOME/step"; fi
step=$((step + 1))
echo "$step" > "$HOME/step"
wanted="$(sed -n "${step}p" "$HOME/picks")"
while IFS= read -r row; do
    if [[ "${row%%$'\\t'*}" = "$wanted" ]]; then printf '%s\\n' "$row"; exit 0; fi
done
exit 130
''')
            fzf.chmod(0o755)
            (self.root / 'step').write_text('0\n')
            (self.root / 'picks').write_text('\n'.join(picks) + '\n')
        master, slave = pty.openpty()
        try:
            if keys:
                os.write(master, keys)
            # 子のシェルが PTY からの入力待ちでも stderr の EOF を待たない。
            with tempfile.TemporaryFile() as errors:
                result = subprocess.run([str(self.atd), *args], cwd=self.root, stdin=slave, stdout=slave,
                                        stderr=errors, env={**self.env, **env}, timeout=5)
                errors.seek(0)
                self.assertEqual(result.returncode, 0, errors.read().decode())
        finally:
            os.close(slave)
            os.close(master)

    def test_direct_commands_preserve_arguments_and_child_status(self):
        marker = self.root / 'command-was-evaluated'
        # PATH を分離していても、もし再評価すればリダイレクトでファイルができる。
        literal = f'-DVALUE=$(: > "{marker}")'
        for host, command, script, args in (
            ('pc', 'build', 'build.sh', ['--packages-select', 'pkg with spaces', literal, '']),
            ('pc', 'web', 'webui.sh', ['prepare']),
            ('jetson', 'bluetooth', 'bluetooth.sh', ['status']),
            ('jetson', 'hotspot', 'hotspot.sh', ['on', '--band', '5']),
            ('jetson', 'hotspot', 'hotspot.sh', ['off']),
        ):
            with self.subTest(command=command, args=args):
                result = self.invoke(command, *args, MENU_TEST_HOST=host, MENU_TEST_RC='17')
                self.assertEqual(result.returncode, 17, result.stderr)
                self.assert_call(script, *args)
                self.assertFalse(marker.exists())

    def test_queries_and_noninteractive_invocation_do_not_execute(self):
        for option in ('--help', '--list'):
            result = self.invoke(option)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(result.stdout.strip())
        for args in ((), ('not-a-command',), ('../scripts/build.sh',)):
            self.assertNotEqual(self.invoke(*args).returncode, 0)
        self.assertFalse(self.calls.exists())

    def listed(self, host, container=False):
        result = self.invoke('--list', MENU_TEST_HOST=host, MENU_TEST_CONTAINER=str(int(container)))
        self.assertEqual(result.returncode, 0, result.stderr)
        return dict(line.split('\t', 1) for line in result.stdout.splitlines() if line.strip())

    def test_list_matches_host_and_container(self):
        for host in ('pc', 'jetson'):
            for container in (False, True):
                with self.subTest(host=host, container=container):
                    menu = self.listed(host, container)
                    self.assertIn('build', menu)
                    self.assertIn('bringup', menu)
                    self.assertEqual('web' in menu, host == 'pc')
                    self.assertEqual('gazebo' in menu, host == 'pc' and not container)
                    self.assertEqual('replay' in menu, host == 'pc')
                    jetson_commands = ['bluetooth', 'hotspot', 'display', 'performance']
                    if host == 'jetson':
                        for command in jetson_commands:
                            self.assertIn(command, menu)
                            if container:
                                self.assertIn('SSH', menu[command])
                                self.assertIn('ホスト', menu[command])
                    else:
                        self.assertTrue(set(jetson_commands).isdisjoint(menu))
                        result = self.invoke('hotspot', 'on', MENU_TEST_HOST=host,
                                             MENU_TEST_CONTAINER=str(int(container)))
                        self.assertNotEqual(result.returncode, 0)
                    for command in ('dev', 'vgl', 'setup', 'docker', 'cache'):
                        self.assertEqual(command in menu, not container)
        self.assertFalse(self.calls.exists())
        self.assertFalse(self.ssh_args.exists())

    def test_notebook_can_launch_simulation_and_ros_bringup(self):
        self.menu(picks=['gazebo', 'serve', 'odaiba', '424x240', '30', '400', 'quit'], keys=b'1.25\n\n')
        self.assert_call('gazebo.sh', 'serve', '--route', 'lap', '--circuit', 'odaiba', '--speed', '1.25',
                         '--resolution', '424x240', '--camera-hz', '30', '--imu', '400')
        for mode in ['bag', 'map', 'video', 'serve', 'run']:
            self.menu('gazebo', picks=[mode, 'odaiba', '848x480', '60', '200'], keys=b'1.75\n3\n')
            self.assert_call('gazebo.sh', mode, '--route', 'lap', '--circuit', 'odaiba',
                             '--speed', '1.75', '--laps', '3', '--resolution', '848x480', '--camera-hz', '60', '--imu', '200')
        self.menu('gazebo', picks=['bag', 'odaiba', '1280x720', '30', 'off'], keys=b'3\n2\n')
        self.assert_call('gazebo.sh', 'bag', '--route', 'lap', '--circuit', 'odaiba', '--speed', '3',
                         '--laps', '2', '--resolution', '1280x720', '--camera-hz', '30', '--imu', 'off')
        self.menu('gazebo', picks=['video', 'odaiba', '640x480', '90', 'off'], keys=b'\n\n')
        self.assert_call('gazebo.sh', 'video', '--route', 'lap', '--circuit', 'odaiba',
                         '--resolution', '640x480', '--camera-hz', '90', '--imu', 'off')
        self.menu('gazebo', picks=['diagnose', 'odaiba', '1280x720', '30', 'off'], keys=b'\n')
        self.assert_call('gazebo.sh', 'diagnose', '--route', 'lap', '--circuit', 'odaiba',
                         '--resolution', '1280x720', '--camera-hz', '30', '--imu', 'off')
        self.menu('gazebo', picks=['map', 'odaiba'], keys=b'1.25\nq\n')
        self.assertFalse(self.calls.exists())
        result = self.invoke('gazebo', 'bag', '--course', 'odaiba', '--speed', '.9', '--laps', '4')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_call('gazebo.sh', 'bag', '--course', 'odaiba', '--speed', '.9', '--laps', '4')
        for container in ('0', '1'):
            result = self.invoke('bringup', 'gazebo', MENU_TEST_CONTAINER=container)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assert_call('bringup.sh', 'gazebo')

    def test_closed_loop_menu_selects_a_saved_map_and_line_and_previews_both_containers(self):
        fixture = self.root / 'maps' / 'odaiba_saved'
        (fixture / 'hdmap').mkdir(parents=True)
        sample = json.loads((PROJECT / 'samples/odaiba/hdmap/map.json').read_text())
        sample.update(id=fixture.name, source={'kind': 'nvidia_offline'})
        (fixture / 'hdmap/map.json').write_text(json.dumps(sample))
        (fixture / 'vgl.json').write_text(json.dumps({'status': 'complete', 'input_shape': [1, 3, 720, 1280]}))
        for tool in ('python3', 'id'):
            (self.bin / tool).symlink_to(shutil.which(tool))
        # The real menu invokes our real host launcher in dry-run mode, with no Docker executable on PATH.
        script = shlex.quote(str(PROJECT / 'scripts/gazebo.sh'))
        (self.scripts / 'gazebo.sh').write_text(
            f'#!/bin/bash\nif [[ "$1" = list ]]; then exec bash {script} list; fi\n'
            f'exec bash {script} "$@" --dry-run > "$MENU_TEST_CALLS"\n')
        for display, enabled, laps, tracking, imu in [('off', 'false', 2, 'vo', 'off'), ('rviz', 'true', 2, 'vio', '400'),
                                                      ('web', 'false', 0, 'vio', '200'), ('both', 'true', 0, 'vo', 'off')]:
            choices = ['closed-loop', 'odaiba', '30', tracking]
            if tracking == 'vio':
                choices.append(imu)
            self.menu('gazebo', picks=[*choices, display, fixture.name, 'centerline'],
                      keys=f'1.25\n{laps}\n'.encode(), ATD_RESULT_DIR=str(self.root))
            result = self.calls.read_text()
            self.assertIn('map_id:=odaiba_saved line:=centerline max_speed_mps:=1.25', result)
            self.assertIn(f'rviz:={enabled}', result)
            self.assertIn(f'tracking:={tracking}', result)
            self.assertEqual('web:=true' in result, display in ('web', 'both'))
            gazebo = shlex.split(next(line for line in result.splitlines() if line.startswith('docker run ')))
            for key, expected in (('--mode', 'closed-loop'), ('--resolution', '1280x720'), ('--camera-hz', '30'),
                                  ('--imu', imu), ('--laps', str(laps))):
                self.assertEqual(gazebo[gazebo.index(key) + 1], expected)
        self.assertNotIn('bringup\t', self.invoke('--list', MENU_TEST_OS='Darwin').stdout)

    def test_replay_menu_and_notebook_launcher(self):
        self.menu('replay', keys=b'2\n')
        self.assert_call('replay.sh', '--tracking', 'vio')
        self.menu(picks=['replay', 'back', 'replay', 'vio', 'quit'], MENU_TEST_CONTAINER='1')
        self.assert_call('replay.sh', '--tracking', 'vio')
        self.menu('replay', picks=['vo'])
        self.assert_call('replay.sh', '--tracking', 'vo')
        self.menu('replay', picks=['orbslam3'])
        self.assert_call('replay.sh', '--backend', 'orbslam3')
        self.menu('replay', picks=['cancel'])
        self.assertFalse(self.calls.exists())
        self.menu('replay', '--tracking', 'vio', picks=[])
        self.assert_call('replay.sh', '--tracking', 'vio')
        result = self.invoke('replay')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_call('replay.sh')
        self.assertNotIn('replay\t', self.invoke('--list', MENU_TEST_OS='Darwin').stdout)

        shutil.copy2(PROJECT / 'scripts/replay.sh', self.scripts / 'replay.sh')
        bag = '/data/record/rosbags/my session/lap 01'
        options = [bag, '--rate', '0.5', '--tracking', 'vio']
        command = ['env', 'ROS_DOMAIN_ID=75',
                   'FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/atd/docker/gazebo_monza/fastdds.xml',
                   'ros2', 'run', 'atd_bringup', 'replay_vslam', *options]
        result = self.invoke('replay', '--dry-run', *options, ROS_DOMAIN_ID='74')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = shlex.split(result.stdout.split('command: ', 1)[1])
        self.assertIn('atd_pc_dev', args)
        self.assertEqual(args[args.index('/usr/local/bin/atd-shell') + 1:], command)

        result = self.invoke('replay', '--imu-gaps', 'fixed-run', '--dry-run')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = shlex.split(result.stdout.split('command: ', 1)[1])
        self.assertEqual(args[-5:], ['ros2', 'run', 'atd_bringup', 'evaluate_orb_gyro', 'fixed-run'])

        result = self.invoke('replay', '--compare', 'cu-id', 'orb-id', '--dry-run')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(shlex.split(result.stdout.split('command: ', 1)[1])[-6:],
                         ['ros2', 'run', 'atd_bringup', 'compare_replays', 'cu-id', 'orb-id'])

        # コンテナ内の起動時に ATD shell で overlay を読み直すところまで確認する。
        shell = self.root / 'docker/scripts/atd-shell.sh'
        shell.parent.mkdir(parents=True)
        shell.write_text('#!/bin/bash\nprintf "%s\\0" "$@" > "$MENU_TEST_CALLS"\nexit 17\n')
        shell.chmod(0o755)
        for domain in ('0', '232'):
            command[1] = 'ROS_DOMAIN_ID=' + domain
            result = self.invoke('replay', *options, MENU_TEST_CONTAINER='1', ATD_REPLAY_ROS_DOMAIN_ID=domain)
            self.assertEqual(result.returncode, 17, result.stderr)
            self.assert_call(*command)
        for domain in ('233', '-1', '1+1'):
            result = self.invoke('replay', '--dry-run', ATD_REPLAY_ROS_DOMAIN_ID=domain)
            self.assertEqual(result.returncode, 2)
            self.assertIn('ATD_REPLAY_ROS_DOMAIN_ID', result.stderr)
        self.assertFalse(self.calls.exists())

    def test_jetson_docker_delegates_selected_band_to_host(self):
        self.menu(picks=['hotspot', '5', 'quit'], MENU_TEST_HOST='jetson', MENU_TEST_CONTAINER='1')
        arguments = self.ssh_args.read_bytes().decode().split('\0')[:-1]
        self.assertEqual(arguments[-2], '127.0.0.1')
        self.assertEqual(arguments[arguments.index('-l') + 1], 'atd')
        self.assertEqual(arguments[arguments.index('-p') + 1], '22')
        self.assertIn('-t', arguments)
        self.assertEqual(self.host_calls.read_bytes().decode().split('\0'), ['hotspot', 'on', '--band', '5', ''])
        self.assertFalse(self.calls.exists())

    def test_jetson_docker_preserves_arguments_through_remote_shell(self):
        marker = self.root / 'evaluated'
        profile = f"team's AP; : > '{marker}'"
        result = self.invoke('hotspot', 'on', '--profile', profile, MENU_TEST_HOST='jetson',
                             MENU_TEST_CONTAINER='1', ATD_JETSON_SSH_USER='operator', MENU_TEST_REMOTE_RC='17')
        self.assertEqual(result.returncode, 17, result.stderr)
        arguments = self.ssh_args.read_bytes().decode().split('\0')[:-1]
        self.assertEqual(arguments[-2], '127.0.0.1')
        self.assertEqual(arguments[arguments.index('-l') + 1], 'operator')
        self.assertIn('-T', arguments)
        self.assertIn('BatchMode=yes', arguments)
        self.assertEqual(self.host_calls.read_bytes().decode().split('\0'), ['hotspot', 'on', '--profile', profile, ''])
        self.assertFalse(marker.exists())
        self.assertFalse(self.calls.exists())

    def test_jetson_docker_ssh_failure_returns_error_and_host_command(self):
        result = self.invoke('bluetooth', 'status', MENU_TEST_HOST='jetson', MENU_TEST_CONTAINER='1',
                             MENU_TEST_SSH_RC='255')
        self.assertEqual(result.returncode, 255)
        self.assertIn('atd bluetooth status', result.stderr + result.stdout)
        self.assertTrue(self.ssh_args.exists())
        self.assertFalse(self.host_calls.exists())
        self.assertFalse(self.calls.exists())

    def test_jetson_docker_rejects_another_container_as_ssh_target(self):
        (self.root / 'fixture.dockerenv').touch()
        result = self.invoke('performance', '--check', MENU_TEST_HOST='jetson', MENU_TEST_CONTAINER='1')
        self.assertNotEqual(result.returncode, 0)
        self.assertTrue(self.ssh_args.exists())
        self.assertFalse(self.host_calls.exists())
        self.assertFalse(self.calls.exists())

    def test_fzf_selects_band_and_back_returns_to_main_menu(self):
        self.menu(picks=['bluetooth', 'back', 'hotspot', '5', 'quit'], MENU_TEST_HOST='jetson')
        self.assert_call('hotspot.sh', 'on', '--band', '5')

    def test_build_menu_selects_a_workspace_package(self):
        package = self.root / 'ros2_ws/src/atd_teleop'
        package.mkdir(parents=True)
        (package / 'package.xml').touch()
        self.menu(picks=['build', 'atd_teleop', 'quit'])
        self.assert_call('build.sh', '--packages-up-to', 'atd_teleop')

    def test_fzf_cancel_and_quit_do_not_dispatch(self):
        for picks in (['cancel'], ['quit'], ['hotspot', 'cancel', 'quit']):
            with self.subTest(picks=picks):
                self.menu(picks=picks, MENU_TEST_HOST='jetson')
                self.assertFalse(self.calls.exists())

    def test_numbered_menu_selects_bluetooth_status_without_fzf(self):
        options = list(self.listed('jetson'))
        choice = options.index('bluetooth') + 1
        self.menu(keys=f'{choice}\n2\n{len(options) + 1}\n'.encode(), MENU_TEST_HOST='jetson')
        self.assert_call('bluetooth.sh', 'status')

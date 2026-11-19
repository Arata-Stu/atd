"""日常操作メニューを、実機や Docker を起動せずに確認する。"""
import os
from pathlib import Path
import pty
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
        uname.write_text('#!/bin/sh\nif [ "$MENU_TEST_HOST" = jetson ]; then echo aarch64; else echo x86_64; fi\n')
        uname.chmod(0o755)
        self.calls = self.root / 'calls'
        self.env = {'PATH': str(self.bin), 'HOME': str(self.root), 'TERM': 'xterm',
                    'MENU_TEST_HOST': 'pc', 'MENU_TEST_CALLS': str(self.calls)}
        for filename in ('build', 'dev', 'setup', 'webui', 'bluetooth', 'hotspot',
                         'bringup', 'vgl', 'jetson_display_mode', 'jetson_max_performance'):
            wrapper = self.scripts / (filename + '.sh')
            wrapper.write_text('#!/bin/bash\nprintf "%s\\0" "${0##*/}" "$@" >> "$MENU_TEST_CALLS"\n'
                               'exit "${MENU_TEST_RC:-0}"\n')
            wrapper.chmod(0o755)

    def invoke(self, *args, **env):
        return subprocess.run([str(self.atd), *args], env={**self.env, **env}, text=True,
                              cwd=self.root, stdin=subprocess.DEVNULL, capture_output=True, timeout=5)

    def assert_call(self, *args):
        self.assertEqual(self.calls.read_bytes().split(b'\0'), [arg.encode() for arg in args] + [b''])
        self.calls.unlink()

    def menu(self, *, picks=None, keys=b'', **env):
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
                result = subprocess.run([str(self.atd)], cwd=self.root, stdin=slave, stdout=slave,
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
        return [line.split()[0] for line in result.stdout.splitlines() if line.strip()]

    def test_list_matches_host_and_container(self):
        for host in ('pc', 'jetson'):
            for container in (False, True):
                with self.subTest(host=host, container=container):
                    menu = self.listed(host, container)
                    self.assertIn('build', menu)
                    self.assertEqual('web' in menu, host == 'pc')
                    self.assertEqual('dev' in menu, not container)
                    self.assertEqual('hotspot' in menu, host == 'jetson' and not container)
                    self.assertEqual('bluetooth' in menu, host == 'jetson' and not container)
                    if 'hotspot' not in menu:
                        result = self.invoke('hotspot', 'on', MENU_TEST_HOST=host,
                                             MENU_TEST_CONTAINER=str(int(container)))
                        self.assertNotEqual(result.returncode, 0)
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
        options = self.listed('jetson')
        choice = options.index('bluetooth') + 1
        self.menu(keys=f'{choice}\n2\n{len(options) + 1}\n'.encode(), MENU_TEST_HOST='jetson')
        self.assert_call('bluetooth.sh', 'status')

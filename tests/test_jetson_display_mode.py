"""GUI/CUI の起動既定値と即時移行を模擬 systemctl で検証する。"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]


class DisplayModeTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        self.script = root / 'jetson_display_mode.sh'
        shutil.copy2(PROJECT / 'scripts/jetson_display_mode.sh', self.script)
        (root / 'lib').mkdir()
        (root / 'lib/jetson_host.sh').write_text(
            'atd_require_jetson_host() { test "$TEST_HOST" = jetson; }\n'
            'atd_run_as_root() { TEST_PRIVILEGE=root "$@"; }\n')
        ctl = root / 'systemctl'
        ctl.write_text(
            '#!/bin/sh\n'
            'printf "%s %s\\n" "${TEST_PRIVILEGE:-user}" "$*" >> "$TEST_LOG"\n'
            'if [ "$1" = "$TEST_FAIL_ON" ]; then exit 17; fi\n'
            'if [ "$1" = get-default ]; then echo "$TEST_TARGET"; fi\n')
        ctl.chmod(0o755)
        self.log = root / 'calls'
        self.env = {**os.environ, 'TEST_LOG': str(self.log), 'TEST_HOST': 'jetson',
                    'PATH': str(root) + os.pathsep + os.defpath,
                    'TEST_TARGET': 'graphical.target', 'TEST_FAIL_ON': ''}

    def invoke(self, *args, **env):
        self.log.write_text('')
        return subprocess.run([str(self.script), *args], env={**self.env, **env},
                              capture_output=True, text=True, timeout=10)

    def test_status_reads_default_target_as_user(self):
        for target, label in [('graphical.target', 'GUI'), ('multi-user.target', 'CUI'),
                              ('rescue.target', 'その他')]:
            with self.subTest(target=target):
                result = self.invoke('status', TEST_TARGET=target)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(f'{label} ({target})', result.stdout)
                self.assertEqual(self.log.read_text().splitlines(), ['user get-default'])

    def test_immediate_switch_requires_explicit_now(self):
        for mode, target in [('gui', 'graphical.target'), ('cui', 'multi-user.target')]:
            for args in [(), ('--now',)]:
                with self.subTest(mode=mode, args=args):
                    result = self.invoke(mode, *args)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    expected = [f'root set-default {target}']
                    if args:
                        expected.append(f'root isolate {target}')
                    self.assertEqual(self.log.read_text().splitlines(), expected)

    def test_help_and_invalid_invocations_do_not_touch_systemctl(self):
        self.assertEqual(self.invoke('--help', TEST_HOST='pc').returncode, 0)
        self.assertEqual(self.log.read_text(), '')
        for args in [(), ('status', '--now'), ('--help', '--now'), ('typo',),
                     ('cui', 'now'), ('gui', '--now', '--now')]:
            with self.subTest(args=args):
                self.assertNotEqual(self.invoke(*args).returncode, 0)
                self.assertEqual(self.log.read_text(), '')
        self.assertNotEqual(self.invoke('cui', '--now', TEST_HOST='pc').returncode, 0)
        self.assertEqual(self.log.read_text(), '')

    def test_systemctl_failures_propagate(self):
        result = self.invoke('cui', '--now', TEST_FAIL_ON='set-default')
        self.assertEqual(result.returncode, 17)
        self.assertEqual(self.log.read_text().splitlines(), ['root set-default multi-user.target'])
        self.assertEqual(self.invoke('gui', '--now', TEST_FAIL_ON='isolate').returncode, 17)
        self.assertEqual(self.invoke('status', TEST_FAIL_ON='get-default').returncode, 17)


if __name__ == '__main__':
    unittest.main()

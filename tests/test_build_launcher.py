"""Build 専用入口の引数境界を検証する。Docker / colcon は実行しない。"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]


class BuildLauncherTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='atd-build-entry-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        scripts = self.root / 'scripts'
        scripts.mkdir()
        self.script = scripts / 'build.sh'
        shutil.copyfile(PROJECT / 'scripts/build.sh', self.script)
        (scripts / 'python.sh').write_text(
            'printf "%s\\0" "$@" > "$ATD_TEST_ARGS"\nexit "${ATD_TEST_EXIT:-0}"\n')
        self.output = self.root / 'arguments'
        self.env = {**os.environ, 'ATD_TEST_ARGS': str(self.output)}

    def invoke(self, *args, exit_code=0):
        return subprocess.run(['/bin/bash', str(self.script), *args],
                              env={**self.env, 'ATD_TEST_EXIT': str(exit_code)},
                              capture_output=True, text=True, timeout=10)

    def args(self):
        return self.output.read_bytes().decode().split('\0')[:-1]

    def test_default_build_uses_ros_without_an_empty_argument(self):
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args(), ['ros', 'build'])

    def test_colcon_options_need_no_extra_separator(self):
        result = self.invoke('--packages-up-to', 'atd_web_bridge', '--symlink-install')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args(), ['ros', 'build', '--packages-up-to', 'atd_web_bridge', '--symlink-install'])

    def test_host_and_dry_run_precede_ros_action(self):
        result = self.invoke('--host', 'jetson', '--dry-run', '--packages-select', 'atd_teleop')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args(), ['--host', 'jetson', '--dry-run', 'ros', 'build',
                                      '--packages-select', 'atd_teleop'])

    def test_literal_arguments_and_exit_status_are_preserved(self):
        sentinel = self.root / 'must-not-exist'
        literal = '$(touch ' + str(sentinel) + ');*'
        result = self.invoke('--cmake-args', '-DCONFIG=path with spaces', literal, exit_code=9)
        self.assertEqual(result.returncode, 9, result.stderr)
        self.assertFalse(sentinel.exists())
        self.assertEqual(self.args(), ['ros', 'build', '--cmake-args', '-DCONFIG=path with spaces', literal])

    def test_explicit_separator_only_ends_wrapper_options(self):
        result = self.invoke('--dry-run', '--', '--help')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.args(), ['--dry-run', 'ros', 'build', '--help'])

    def test_help_does_not_invoke_python_launcher(self):
        result = self.invoke('--help')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--packages-up-to atd_web_bridge', result.stdout)
        self.assertFalse(self.output.exists())

    def test_missing_host_value_does_not_invoke_python_launcher(self):
        for arguments in (('--host',), ('--host', ''), ('--host', '--dry-run')):
            with self.subTest(arguments=arguments):
                result = self.invoke(*arguments)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('--host の値が必要', result.stderr)
                self.assertFalse(self.output.exists())


if __name__ == '__main__':
    unittest.main()

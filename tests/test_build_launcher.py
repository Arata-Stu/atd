"""専用 build 入口が Python 管理を経由せず、一致するコンテナへ接続する。"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from test_python_launcher import FAKE_DOCKER


PROJECT = Path(__file__).resolve().parents[1]
HELPER = '/usr/local/bin/atd-build'


class BuildLauncherTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='atd-build-entry-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.project = self.root / 'project'
        scripts = self.project / 'scripts'
        scripts.mkdir(parents=True)
        self.script = scripts / 'build.sh'
        shutil.copyfile(PROJECT / 'scripts/build.sh', self.script)
        shutil.copytree(PROJECT / 'scripts/lib', scripts / 'lib')
        shutil.copytree(PROJECT / 'docker', self.project / 'docker',
                        ignore=shutil.ignore_patterns('.venv', '__pycache__'))
        (scripts / 'detect_host.sh').write_text(
            'detect_atd_host() { printf "%s\\n" "${ATD_TEST_HOST:-pc}"; }\n')
        (scripts / 'python.sh').write_text(
            'printf called > "$ATD_TEST_PYTHON_SENTINEL"\nexit 93\n')
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        (self.bin / 'docker').write_text(f'#!{sys.executable}\n' + FAKE_DOCKER)
        (self.bin / 'docker').chmod(0o755)
        self.calls_path = self.root / 'calls.jsonl'
        self.sentinel = self.root / 'python-launcher-called'
        self.env = {
            'PATH': os.pathsep.join((str(self.bin), str(Path(sys.executable).parent), os.defpath)),
            'HOME': str(self.root), 'ATD_TEST_CALLS': str(self.calls_path),
            'ATD_TEST_PROJECT': str(self.project), 'ATD_TEST_HELPER': HELPER,
            'ATD_TEST_PYTHON_SENTINEL': str(self.sentinel),
        }
        result = subprocess.run(
            ['/bin/bash', '-c', 'source "$1"; atd_runtime_revision "$2"',
             'build-entry-test', str(scripts / 'lib/runtime.sh'), str(self.project)],
            env=self.env, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.env['ATD_TEST_REVISION'] = result.stdout.strip()

    def invoke(self, *args, env=None, script=None):
        return subprocess.run(['/bin/bash', str(script or self.script), *args],
                              env={**self.env, **(env or {})},
                              capture_output=True, text=True, timeout=10)

    def calls(self):
        import json
        return [json.loads(line) for line in self.calls_path.read_text().splitlines()] if self.calls_path.exists() else []

    def commands(self):
        return [entry['args'] for entry in self.calls() if HELPER in entry['args']]

    def test_default_build_calls_dedicated_helper_without_python_launcher(self):
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.commands(), [[
            'exec', '-i', '-u', 'admin', '--workdir', '/workspaces/isaac_ros-dev',
            'atd_pc_dev', '/usr/local/bin/atd-shell', HELPER]])
        self.assertFalse(self.sentinel.exists())

    def test_colcon_options_need_no_profile_action_or_separator(self):
        result = self.invoke('--packages-up-to', 'atd_web_bridge', '--symlink-install')
        self.assertEqual(result.returncode, 0, result.stderr)
        command, = self.commands()
        self.assertEqual(command[-4:], [HELPER, '--packages-up-to', 'atd_web_bridge', '--symlink-install'])
        self.assertFalse(self.sentinel.exists())

    def test_host_override_and_dry_run_do_not_call_docker(self):
        result = self.invoke('--host', 'notebook', '--dry-run', '--packages-select', 'atd_teleop',
                             env={'ATD_TEST_HOST': 'jetson'})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('atd_pc_dev', result.stdout)
        self.assertIn(HELPER, result.stdout)
        self.assertNotIn('atd-python', result.stdout)
        self.assertEqual(self.calls(), [])
        self.assertFalse(self.sentinel.exists())

    def test_autodetection_and_explicit_jetson_select_target(self):
        result = self.invoke(env={'ATD_TEST_HOST': 'jetson'})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('atd_jetson_dev', self.commands()[-1])
        result = self.invoke('--host', 'jetson')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('atd_jetson_dev', self.commands()[-1])

    def test_literal_arguments_and_exit_status_are_preserved(self):
        marker = self.root / 'must-not-exist'
        literal = '$(touch ' + str(marker) + ');*'
        result = self.invoke('--cmake-args', '-DCONFIG=path with spaces', literal,
                             env={'ATD_TEST_COMMAND_EXIT': '9'})
        self.assertEqual(result.returncode, 9, result.stderr)
        self.assertFalse(marker.exists())
        self.assertEqual(self.commands()[-1][-3:], ['--cmake-args', '-DCONFIG=path with spaces', literal])

    def test_help_and_bad_host_do_not_access_docker(self):
        result = self.invoke('--help')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--packages-up-to atd_web_bridge', result.stdout)
        for arguments in (('--host',), ('--host', ''), ('--host', '--dry-run'), ('--host', 'server')):
            with self.subTest(arguments=arguments):
                result = self.invoke(*arguments)
                self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls(), [])

    def test_stale_missing_stopped_or_other_checkout_does_not_build(self):
        for state in ('missing', 'stopped', 'legacy', 'stale', 'different_checkout'):
            with self.subTest(state=state):
                self.calls_path.unlink(missing_ok=True)
                result = self.invoke(env={'ATD_TEST_CONTAINER': state})
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.commands(), [])
        self.assertFalse(self.sentinel.exists())

    def test_container_entry_calls_atd_build_directly_and_dry_run_is_read_only(self):
        marker = self.root / 'container-marker'
        marker.touch()
        helper = self.bin / 'atd-build'
        output = self.root / 'build-arguments'
        helper.write_text('#!/bin/bash\nprintf "%s\\0" "$@" > "$ATD_TEST_BUILD_ARGS"\n')
        helper.chmod(0o755)
        local_script = self.project / 'scripts/container-build.sh'
        local_script.write_text(self.script.read_text().replace('/.dockerenv', str(marker)).replace(
            HELPER, str(helper)))
        result = self.invoke('--packages-select', 'atd_teleop', script=local_script,
                             env={'ATD_TEST_BUILD_ARGS': str(output)})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(output.read_bytes().split(b'\0')[:-1], [b'--packages-select', b'atd_teleop'])
        before = output.read_bytes()
        result = self.invoke('--dry-run', '--packages-up-to', 'atd_web_bridge', script=local_script)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(output.read_bytes(), before)
        self.assertEqual(self.calls(), [])
        self.assertFalse(self.sentinel.exists())

    def test_container_missing_build_helper_fails_without_host_fallback(self):
        marker = self.root / 'container-marker'
        marker.touch()
        local_script = self.project / 'scripts/container-build.sh'
        local_script.write_text(self.script.read_text().replace('/.dockerenv', str(marker)).replace(
            HELPER, str(self.bin / 'missing-build')))
        result = self.invoke(script=local_script)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('再構築', result.stderr)
        self.assertEqual(self.calls(), [])


if __name__ == '__main__':
    unittest.main()

"""Python 操作入口が稼働中の一致するコンテナへだけ転送することを検証する。"""

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
FAKE_DOCKER = r'''
import json
import os
import sys

args = sys.argv[1:]
with open(os.environ['ATD_TEST_CALLS'], 'a') as stream:
    stream.write(json.dumps({'tool': 'docker', 'args': args}) + '\n')
if args[:2] == ['container', 'inspect']:
    state = os.environ.get('ATD_TEST_CONTAINER', 'running')
    if state == 'missing':
        sys.exit(1)
    environment = {
        'ATD_RUNTIME_API': '1',
        'ATD_RUNTIME_REV': os.environ['ATD_TEST_REVISION'],
    }
    if state == 'stale':
        environment['ATD_RUNTIME_REV'] = 'old-source'
    if state == 'legacy':
        environment.clear()
    source = os.environ['ATD_TEST_PROJECT']
    if state == 'different_checkout':
        source += '-other'
    print(json.dumps([{
        'State': {'Running': state != 'stopped'},
        'Config': {'Env': [key + '=' + value for key, value in environment.items()]},
        'Mounts': [{'Destination': '/workspaces/atd', 'Source': source, 'Type': 'bind', 'RW': False}],
    }]))
elif args[:1] == ['exec']:
    if args[-2:] == ['cat', '/etc/atd/runtime.ready']:
        ready = os.environ.get('ATD_TEST_READY', 'matching')
        if ready == 'missing':
            sys.exit(1)
        print(os.environ['ATD_TEST_REVISION'] if ready == 'matching' else 'old-ready')
    else:
        assert '/usr/local/bin/atd-shell' in args
        assert '/usr/local/bin/atd-python' in args
        sys.exit(int(os.environ.get('ATD_TEST_COMMAND_EXIT', '0')))
else:
    sys.exit('Docker mutation is forbidden: ' + repr(args))
'''


class PythonLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='atd-python-launcher-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.project = self.root / 'project'
        (self.project / 'scripts').mkdir(parents=True)
        self.script = self.project / 'scripts/python.sh'
        shutil.copyfile(PROJECT / 'scripts/python.sh', self.script)
        shutil.copytree(PROJECT / 'scripts/lib', self.project / 'scripts/lib')
        shutil.copytree(PROJECT / 'docker', self.project / 'docker', ignore=shutil.ignore_patterns('.venv', '__pycache__'))
        (self.project / 'ros2_ws/src').mkdir(parents=True)
        (self.project / 'ros2_ws/src/README.md').write_text('ROS package source fixture\n')
        (self.project / 'scripts/detect_host.sh').write_text(
            'detect_atd_host() { printf "%s\\n" "${ATD_TEST_HOST:-pc}"; }\n')
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.calls_path = self.root / 'calls.jsonl'
        docker = self.bin / 'docker'
        docker.write_text(f'#!{sys.executable}\n' + FAKE_DOCKER)
        docker.chmod(0o755)
        self.env = {
            'PATH': os.pathsep.join((str(self.bin), str(Path(sys.executable).parent), os.defpath)),
            'HOME': str(self.root),
            'ATD_TEST_CALLS': str(self.calls_path),
            'ATD_TEST_PROJECT': str(self.project),
        }
        result = subprocess.run(
            ['/bin/bash', '-c', 'source "$1"; atd_runtime_revision "$2"',
             'launcher-test', str(self.project / 'scripts/lib/runtime.sh'), str(self.project)],
            env=self.env, capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.env['ATD_TEST_REVISION'] = result.stdout.strip()

    def invoke(self, *args, env=None, script=None):
        return subprocess.run(
            ['/bin/bash', str(script or self.script), *args],
            env={**self.env, **(env or {})}, capture_output=True, text=True, timeout=10,
        )

    def calls(self):
        return [json.loads(line) for line in self.calls_path.read_text().splitlines()] if self.calls_path.exists() else []

    def commands(self):
        return [entry['args'] for entry in self.calls() if '/usr/local/bin/atd-python' in entry['args']]

    def test_arguments_are_forwarded_without_shell_evaluation_and_exit_code_is_preserved(self):
        sentinel = self.root / 'must-not-exist'
        argument = '$(touch ' + str(sentinel) + ')'
        result = self.invoke('--host', 'notebook', 'train', 'run', '--', 'python', 'script with spaces.py', argument,
                             env={'ATD_TEST_COMMAND_EXIT': '7'})
        self.assertEqual(result.returncode, 7, result.stderr)
        self.assertFalse(sentinel.exists())
        self.assertEqual(self.commands(), [[
            'exec', '-i', '-u', 'admin', '--workdir', '/workspaces/isaac_ros-dev', 'atd_pc_dev',
            '/usr/local/bin/atd-shell', '/usr/local/bin/atd-python', 'train', 'run', '--', 'python',
            'script with spaces.py', argument,
        ]])

    def test_raceline_profile_is_forwarded_to_notebook_with_literal_arguments(self):
        literal = '$(touch must-not-run);*'
        result = self.invoke('--host', 'notebook', 'raceline', 'run', '--', 'python', 'path with spaces.py', literal,
                             env={'ATD_TEST_HOST': 'jetson'})
        self.assertEqual(result.returncode, 0, result.stderr)
        command, = self.commands()
        self.assertIn('atd_pc_dev', command)
        self.assertNotIn('atd_jetson_dev', command)
        self.assertEqual(command[-6:], ['raceline', 'run', '--', 'python', 'path with spaces.py', literal])

    def test_no_arguments_reaches_container_picker_without_an_empty_argument(self):
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.commands()[0][-1], '/usr/local/bin/atd-python')

    def test_host_autodetection_and_explicit_override_select_container(self):
        result = self.invoke('ros', 'check', env={'ATD_TEST_HOST': 'jetson'})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('atd_jetson_dev', self.commands()[-1])
        result = self.invoke('--host', 'notebook', 'ros', 'check', env={'ATD_TEST_HOST': 'jetson'})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('atd_pc_dev', self.commands()[-1])

    def test_missing_stopped_legacy_stale_and_other_checkout_do_not_execute(self):
        for state in ('missing', 'stopped', 'legacy', 'stale', 'different_checkout'):
            with self.subTest(state=state):
                self.calls_path.unlink(missing_ok=True)
                result = self.invoke('ros', 'check', env={'ATD_TEST_CONTAINER': state})
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.commands(), [])
                self.assertFalse(any(entry['args'][:1] == ['run'] for entry in self.calls()))

    def test_missing_or_outdated_startup_marker_does_not_execute(self):
        for marker in ('missing', 'old'):
            with self.subTest(marker=marker):
                self.calls_path.unlink(missing_ok=True)
                result = self.invoke('ros', 'check', env={'ATD_TEST_READY': marker})
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('初期設定', result.stderr)
                self.assertEqual(self.commands(), [])

    def test_dry_run_does_not_call_docker_or_read_user_configuration(self):
        sentinel = self.root / 'unexpected-config-read'
        config = self.root / 'workspace.env'
        config.write_text('touch ' + str(sentinel) + '\n')
        result = self.invoke('--host', 'jetson', '--dry-run', 'train', 'run', '--', 'python', 'hello.py',
                             env={'ATD_WORKSPACE_CONFIG': str(config), 'ATD_STORAGE_CONFIG': str(config)})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('atd_jetson_dev', result.stdout)
        self.assertIn('hello.py', result.stdout)
        self.assertEqual(self.calls(), [])
        self.assertFalse(sentinel.exists())

    def test_interactive_terminal_adds_docker_tty_flag(self):
        master, slave = pty.openpty()
        try:
            result = subprocess.run(
                ['/bin/bash', str(self.script), 'ros', 'check'], env=self.env,
                stdin=slave, stdout=slave, stderr=slave, timeout=10,
            )
        finally:
            os.close(slave)
            os.close(master)
        self.assertEqual(result.returncode, 0)
        self.assertEqual(self.commands()[0][:3], ['exec', '-i', '-t'])

    def test_container_entry_executes_helper_directly_and_dry_run_stays_read_only(self):
        marker = self.root / 'container-marker'
        marker.touch()
        helper = self.bin / 'atd-python'
        helper.write_text(f'#!{sys.executable}\n' + '''
import json
import os
import sys
with open(os.environ['ATD_TEST_CALLS'], 'a') as stream:
    stream.write(json.dumps({'tool': 'atd-python', 'args': sys.argv[1:]}) + '\\n')
''')
        helper.chmod(0o755)
        local_script = self.root / 'container-python.sh'
        local_script.write_text(self.script.read_text().replace('/.dockerenv', str(marker)).replace(
            '/usr/local/bin/atd-python', str(helper)))
        result = self.invoke('train', 'check', script=local_script)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls(), [{'tool': 'atd-python', 'args': ['train', 'check']}])
        before = self.calls_path.read_bytes()
        result = self.invoke('--dry-run', 'ros', 'sync', script=local_script)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls_path.read_bytes(), before)

    def test_bad_host_and_unknown_option_fail_without_docker(self):
        for args in (('--host',), ('--host', 'server'), ('--unknown',)):
            with self.subTest(args=args):
                result = self.invoke(*args)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.calls(), [])


if __name__ == '__main__':
    unittest.main()

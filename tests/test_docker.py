"""Docker 診断は読取り専用で実行し、成功・失敗の結果をホスト別に保存する。"""

import json
import os
from pathlib import Path
import pty
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest


PROJECT = Path(__file__).resolve().parents[1]
FAKE = r'''
import json
import os
import sys
tool, args = sys.argv[1], sys.argv[2:]
with open(os.environ['ATD_TEST_CALLS'], 'a') as stream:
    stream.write(json.dumps({'tool': tool, 'args': args}) + '\n')
if tool == 'uname':
    print('Linux' if args == ['-s'] else os.environ.get('ATD_TEST_ARCH', 'x86_64'))
elif tool == 'git':
    assert args == ['lfs', 'version']
    sys.exit(int(os.environ.get('ATD_TEST_GIT_FAIL', '0')))
elif tool == 'isaac-ros':
    assert args == ['status', '--output', 'json']
    if os.environ.get('ATD_TEST_CLI_FAIL'):
        sys.exit(2)
    print(json.dumps({'mode': os.environ.get('ATD_TEST_MODE', 'docker')}))
elif tool == 'docker':
    if args[:1] == ['info']:
        if os.environ.get('ATD_TEST_DOCKER_FAIL'):
            sys.exit(2)
        print(json.dumps({} if os.environ.get('ATD_TEST_NO_NVIDIA') else {'nvidia': {}}))
    elif args[:2] == ['container', 'inspect']:
        state = os.environ.get('ATD_TEST_CONTAINER', 'missing')
        if state == 'missing':
            sys.exit(1)
        environment = {
            'ATD_RUNTIME_API': '1',
            'ATD_RUNTIME_REV': os.environ['ATD_TEST_RUNTIME_REV'],
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
            'FASTDDS_DEFAULT_PROFILES_FILE': '/etc/atd/fastdds.xml',
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SYSTEM_DEFAULT',
            'ROS_STATIC_PEERS': '',
            'RMW_FASTRTPS_USE_QOS_FROM_XML': '0',
        }
        mounts = [{'Destination': destination, 'Source': os.environ[key], 'RW': writable, 'Type': 'bind'}
                  for destination, key, writable in [
                      ('/workspaces/isaac_ros-dev', 'ATD_ROS_WS', True),
                      ('/workspaces/atd', 'ATD_TEST_PROJECT', True),
                      ('/data/record', 'ATD_TEST_RECORD', True),
                      ('/data/results', 'ATD_TEST_RESULT', True)]]
        native = os.path.realpath(os.path.join(os.environ['ATD_ROS_WS'], 'src')) == os.path.realpath(os.environ['ATD_TEST_ROS_SOURCE'])
        if not native or os.environ.get('ATD_TEST_STALE_NESTED_SOURCE'):
            mounts.append({'Destination': '/workspaces/isaac_ros-dev/src/atd',
                           'Source': os.environ['ATD_TEST_ROS_SOURCE'], 'RW': True, 'Type': 'bind'})
        if os.environ.get('ATD_TEST_BAD_MOUNT'):
            mounts[0]['Source'] += '-old'
        if os.environ.get('ATD_TEST_READONLY_CHECKOUT'):
            mounts[1]['RW'] = False
        if os.environ.get('ATD_TEST_BAD_SOURCE_MOUNT'):
            mounts[-1]['Source'] += '-old'
        if os.environ.get('ATD_TEST_BAD_DDS'):
            environment['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        print(json.dumps([{'State': {'Running': state != 'stopped'},
                           'Mounts': mounts,
                           'Config': {'Env': [key + '=' + value for key, value in environment.items()]}}]))
    elif args[:1] == ['exec']:
        assert args[-2:] == ['cat', '/etc/atd/runtime.ready']
        print(os.environ.get('ATD_TEST_READY', os.environ['ATD_TEST_RUNTIME_REV']))
    else:
        sys.exit('Docker mutation forbidden: ' + repr(args))
elif tool == 'dev':
    sys.exit(int(os.environ.get('ATD_TEST_DEV_EXIT', '0')))
else:
    sys.exit('unexpected command')
'''


class DockerStatusTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='atd-docker-status-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.project = self.root / 'project'
        (self.project / 'scripts/lib').mkdir(parents=True)
        (self.project / 'scripts/tools').mkdir()
        (self.project / 'config').mkdir()
        (self.project / 'ros2_ws/src').mkdir(parents=True)
        (self.project / 'ros2_ws/src/README.md').write_text('ROS package source fixture\n')
        shutil.copytree(PROJECT / 'docker', self.project / 'docker',
                        ignore=shutil.ignore_patterns('.venv', '__pycache__'))
        self.script = self.project / 'scripts/tools/docker.sh'
        shutil.copyfile(PROJECT / 'scripts/tools/docker.sh', self.script)
        shutil.copyfile(PROJECT / 'scripts/lib/detect_host.sh', self.project / 'scripts/lib/detect_host.sh')
        for relative in ('scripts/lib/workspace.sh', 'scripts/lib/runtime.sh', 'scripts/lib/isaac_cli.sh'):
            shutil.copyfile(PROJECT / relative, self.project / relative)
        self.workspace = self.root / 'workspace'
        (self.workspace / 'src/atd').mkdir(parents=True)
        for template, destination in (
            ('docker/isaac_ros/build_image_layers.yaml', self.root / 'scripts/.build_image_layers.yaml'),
            ('docker/isaac_ros/common-config.sh', self.workspace / 'scripts/.isaac_ros_common-config'),
            ('docker/isaac_ros/config.yaml', self.workspace / '.isaac-ros-cli/config.yaml'),
        ):
            source = self.project / template
            source.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(PROJECT / template, source)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
        self.record = self.root / 'record with spaces'
        self.result = self.root / 'result with spaces'
        self.record.mkdir()
        self.result.mkdir()
        self.storage = self.project / 'config/storage.env'
        self.storage.write_text('\n'.join(
            f'ATD_{host}_{kind}_DIR={shlex.quote(str(value))}'
            for host in ('PC', 'JETSON')
            for kind, value in (('RECORD', self.record), ('RESULT', self.result))
        ) + '\n')
        self.hostroot = self.root / 'host'
        (self.hostroot / 'etc').mkdir(parents=True)
        (self.hostroot / 'etc/os-release').write_text('ID=ubuntu\nVERSION_ID=24.04\n')
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.calls_file = self.root / 'calls.jsonl'
        for tool in ('docker', 'git', 'isaac-ros', 'uname'):
            executable = self.bin / tool
            executable.write_text(f'#!{sys.executable}\n' + FAKE.replace(
                'tool, args = sys.argv[1], sys.argv[2:]', f'tool, args = {tool!r}, sys.argv[1:]'))
            executable.chmod(0o755)
        (self.bin / 'python3').symlink_to(sys.executable)
        dev = self.project / 'scripts/dev.sh'
        dev.write_text(f'#!{sys.executable}\n' + FAKE.replace(
            'tool, args = sys.argv[1], sys.argv[2:]', "tool, args = 'dev', sys.argv[1:]"))
        # tools/docker.sh invokes dev.sh with bash, just like the normal entrypoint.
        recorder = self.root / 'dev.py'
        recorder.write_text(dev.read_text())
        dev.write_text('#!/bin/bash\nexec "$ATD_TEST_PYTHON" "$ATD_TEST_DEV" "$@"\n')
        self.env = {
            'PATH': os.pathsep.join((str(self.bin), '/usr/bin', '/bin')),
            'HOME': str(self.root),
            'ATD_SETUP_ROOT': str(self.hostroot),
            'ATD_TEST_CALLS': str(self.calls_file),
            'ATD_ROS_WS': str(self.workspace),
            'ATD_TEST_PYTHON': sys.executable,
            'ATD_TEST_DEV': str(recorder),
            'ATD_TEST_PROJECT': str(self.project),
            'ATD_TEST_ROS_SOURCE': str(self.project / 'ros2_ws/src'),
            'ATD_TEST_RECORD': str(self.record),
            'ATD_TEST_RESULT': str(self.result),
        }
        self.env['ATD_TEST_RUNTIME_REV'] = subprocess.check_output(
            ['/bin/bash', '-c', 'source "$1"; atd_runtime_revision "$2"', 'runtime',
             str(self.project / 'scripts/lib/runtime.sh'), str(self.project)],
            env=self.env, text=True).strip()

    def invoke(self, *args, env=None):
        return subprocess.run(['/bin/bash', str(self.script), *args],
                              env={**self.env, **(env or {})}, text=True,
                              capture_output=True, timeout=15)

    def reports(self, host='pc'):
        return sorted((self.project / 'logs/docker' / host).glob('*.log'))

    def calls(self):
        return [json.loads(line) for line in self.calls_file.read_text().splitlines()] if self.calls_file.exists() else []

    def test_success_prints_summary_and_saves_private_report(self):
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertIn('[OK  ]', result.stdout)
        self.assertIn('[SKIP]', result.stdout)
        self.assertIn('FAIL=0', result.stdout)
        self.assertEqual(len(self.reports()), 1)
        report = self.reports()[0]
        self.assertIn('setup: OK=', report.read_text())
        self.assertEqual(report.stat().st_mode & 0o777, 0o600)
        self.assertFalse(any(call['args'][:1] in (['run'], ['build'], ['stop'], ['rm']) for call in self.calls()))

    def test_failures_do_not_stop_remaining_checks_and_are_saved(self):
        self.storage.unlink()
        result = self.invoke(env={'ATD_TEST_DOCKER_FAIL': '1', 'ATD_TEST_MODE': 'native'})
        self.assertEqual(result.returncode, 1)
        report = self.reports()[0].read_text()
        self.assertIn('Docker-daemon', report)
        self.assertIn('Isaac-CLI', report)
        self.assertIn('DDS-source', report)
        self.assertIn('[FAIL]', report)

    def test_non_docker_cli_fails(self):
        result = self.invoke(env={'ATD_TEST_MODE': 'native'})
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] Isaac-CLI', result.stdout)

    def test_build_with_kart_launcher_reports_recovery_and_skips_dev(self):
        launcher = self.hostroot / 'usr/lib/isaac-ros-cli/run_dev.py'
        launcher.parent.mkdir(parents=True)
        launcher.write_text('raise ValueError("ISAAC_ROS_WS must be <kart project>/ros2_ws beside packages.repos")\n')
        result = self.invoke('--build')
        self.assertEqual(result.returncode, 1)
        report = self.reports()[0].read_text()
        self.assertIn('[FAIL] Isaac-CLI', report)
        self.assertIn('docs/setup/pc.md#kart-cli', report)
        self.assertFalse(any(call['tool'] == 'dev' for call in self.calls()))

    def test_missing_nvidia_runtime_fails_without_gpu_probe(self):
        result = self.invoke(env={'ATD_TEST_NO_NVIDIA': '1'})
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] NVIDIA-runtime', result.stdout)
        self.assertFalse(any(call['args'][:1] == ['run'] for call in self.calls()))

    def test_bad_dds_fails(self):
        dds = self.project / 'docker/middleware_profiles/fastdds.xml'
        dds.write_text(dds.read_text().replace('<useBuiltinTransports>false', '<useBuiltinTransports>true'))
        result = self.invoke()
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] DDS-source', result.stdout)

    def test_workspace_metadata_conflict_is_reported_without_overwrite(self):
        metadata = self.workspace / '.isaac-ros-cli/config.yaml'
        metadata.write_text('different: project\n')
        result = self.invoke()
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] workspace-CLI', result.stdout)
        self.assertEqual(metadata.read_text(), 'different: project\n')

    def test_running_container_requires_ready_marker(self):
        result = self.invoke(env={'ATD_TEST_CONTAINER': 'running', 'ATD_TEST_READY': 'old'})
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] container-startup', result.stdout)
        result = self.invoke(env={'ATD_TEST_CONTAINER': 'running'})
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertIn('[OK  ] container-startup', result.stdout)

    def test_running_container_source_revision_must_match(self):
        dependency = self.project / 'docker/dependencies.apt'
        dependency.write_text(dependency.read_text() + '\n# revised source\n')
        result = self.invoke(env={'ATD_TEST_CONTAINER': 'running'})
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] container-startup', result.stdout)

    def test_running_container_rejects_old_workspace_mount(self):
        result = self.invoke(env={'ATD_TEST_CONTAINER': 'running', 'ATD_TEST_BAD_MOUNT': '1'})
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] container-config', result.stdout)
        self.assertIn('[OK  ] container-startup', result.stdout)

    def test_running_container_rejects_previous_readonly_checkout(self):
        result = self.invoke(env={'ATD_TEST_CONTAINER': 'running', 'ATD_TEST_READONLY_CHECKOUT': '1'})
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] container-config', result.stdout)
        self.assertIn('/workspaces/atd', result.stdout)
        self.assertIn('再作成', result.stdout)

    def test_running_container_rejects_dds_environment_override(self):
        result = self.invoke(env={'ATD_TEST_CONTAINER': 'running', 'ATD_TEST_BAD_DDS': '1'})
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] container-config', result.stdout)
        self.assertIn('[OK  ] container-startup', result.stdout)

    def test_running_container_rejects_old_ros_source_mount(self):
        result = self.invoke(env={'ATD_TEST_CONTAINER': 'running', 'ATD_TEST_BAD_SOURCE_MOUNT': '1'})
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] container-config', result.stdout)
        self.assertIn('[OK  ] container-startup', result.stdout)

    def prepare_native_workspace_metadata(self):
        workspace = self.project / 'ros2_ws'
        for source, destination in (
            ('build_image_layers.yaml', self.project / 'scripts/.build_image_layers.yaml'),
            ('common-config.sh', workspace / 'scripts/.isaac_ros_common-config'),
            ('config.yaml', workspace / '.isaac-ros-cli/config.yaml'),
        ):
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(self.project / 'docker/isaac_ros' / source, destination)
        return workspace

    def test_native_workspace_accepts_running_container_without_nested_source_mount(self):
        workspace = self.prepare_native_workspace_metadata()
        result = self.invoke(env={'ATD_ROS_WS': str(workspace), 'ATD_TEST_CONTAINER': 'running'})
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertIn('[OK  ] container-config', result.stdout)
        self.assertFalse((workspace / 'src/atd').exists())

    def test_native_workspace_rejects_stale_nested_source_mount(self):
        workspace = self.prepare_native_workspace_metadata()
        result = self.invoke(env={'ATD_ROS_WS': str(workspace), 'ATD_TEST_CONTAINER': 'running',
                                  'ATD_TEST_STALE_NESTED_SOURCE': '1'})
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] container-config', result.stdout)

    def test_environment_only_storage_is_supported(self):
        self.storage.unlink()
        result = self.invoke(env={'ATD_RECORD_DIR': str(self.record), 'ATD_RESULT_DIR': str(self.result)})
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_resolved_storage_path_is_validated(self):
        target = self.root / 'bad$path'
        target.mkdir()
        alias = self.root / 'safe-link'
        alias.symlink_to(target, target_is_directory=True)
        result = self.invoke(env={'ATD_RECORD_DIR': str(alias)})
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] record', result.stdout)

    def test_resolved_workspace_uses_canonical_metadata_parent(self):
        aliases = self.root / 'aliases'
        aliases.mkdir()
        alias = aliases / 'workspace'
        alias.symlink_to(self.workspace, target_is_directory=True)
        result = self.invoke(env={'ATD_ROS_WS': str(alias)})
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

    def test_resolved_workspace_path_is_validated(self):
        spaced = self.root / 'workspace with spaces'
        spaced.mkdir()
        alias = self.root / 'safe-alias'
        alias.symlink_to(spaced, target_is_directory=True)
        result = self.invoke(env={'ATD_ROS_WS': str(alias)})
        self.assertEqual(result.returncode, 1)
        self.assertIn('[FAIL] workspace', result.stdout)

    def test_latest_reopens_saved_result_without_new_checks(self):
        self.invoke()
        old_calls = self.calls()
        old_reports = self.reports()
        result = self.invoke('--latest')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('最新の結果:', result.stdout)
        self.assertIn('setup: OK=', result.stdout)
        self.assertEqual(self.calls(), old_calls)
        self.assertEqual(self.reports(), old_reports)

    def test_interactive_logs_use_fzf_and_only_open_listed_files(self):
        self.invoke()
        old_calls = self.calls()
        picker = self.bin / 'fzf'
        picker.write_text(f'#!{sys.executable}\n' + '''
import os
import sys
from pathlib import Path
Path(os.environ['ATD_TEST_PICKER_MARKER']).write_text('called')
lines = sys.stdin.read().splitlines()
print('/etc/passwd' if os.environ.get('ATD_TEST_BAD_PICK') else lines[0])
''')
        picker.chmod(0o755)
        marker = self.root / 'picker.called'
        for invalid in (False, True):
            master, slave = pty.openpty()
            def drain():
                try:
                    while os.read(master, 65536):
                        pass
                except OSError:
                    pass
            reader = threading.Thread(target=drain, daemon=True)
            reader.start()
            try:
                process = subprocess.Popen(
                    ['/bin/bash', str(self.script), '--logs'],
                    env={**self.env, 'ATD_TEST_PICKER_MARKER': str(marker),
                         **({'ATD_TEST_BAD_PICK': '1'} if invalid else {})},
                    stdin=slave, stdout=slave, stderr=slave)
                returncode = process.wait(timeout=15)
            finally:
                os.close(slave)
                reader.join(timeout=2)
                os.close(master)
            self.assertEqual(returncode, 1 if invalid else 0)
            self.assertTrue(marker.exists())
        self.assertEqual(self.calls(), old_calls)

    def test_host_logs_are_kept_separately_and_notebook_alias_is_pc(self):
        result = self.invoke('--host', 'notebook')
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        (self.hostroot / 'etc/nv_tegra_release').write_text('# R38')
        result = self.invoke('--host', 'jetson', env={'ATD_TEST_ARCH': 'aarch64'})
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertEqual(len(self.reports()), 1)
        self.assertEqual(len(self.reports('jetson')), 1)

    def test_dry_run_does_not_run_checks_source_config_or_write(self):
        sentinel = self.root / 'unwanted'
        self.storage.write_text('touch ' + shlex.quote(str(sentinel)) + '\n')
        result = self.invoke('--build', '--dry-run')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(sentinel.exists())
        self.assertEqual(self.calls(), [])
        self.assertFalse((self.project / 'logs').exists())

    def test_start_blocks_on_failed_preflight(self):
        result = self.invoke('--start', env={'ATD_TEST_MODE': 'native'})
        self.assertEqual(result.returncode, 1)
        self.assertFalse(any(call['tool'] == 'dev' for call in self.calls()))

    def test_build_delegates_arguments_and_preserves_exit_status(self):
        result = self.invoke('--build', '--host', 'notebook', env={'ATD_TEST_DEV_EXIT': '7'})
        self.assertEqual(result.returncode, 7, result.stderr + result.stdout)
        dev_calls = [call for call in self.calls() if call['tool'] == 'dev']
        self.assertEqual(dev_calls, [{'tool': 'dev', 'args': ['--host', 'pc', '--build']}])
        self.assertIn('終了値: 7', self.reports()[0].read_text())

    def test_missing_log_and_invalid_option_fail(self):
        self.assertEqual(self.invoke('--latest').returncode, 1)
        self.assertEqual(self.invoke('--action', 'delete').returncode, 1)
        self.assertEqual(self.invoke('--host').returncode, 1)


if __name__ == '__main__':
    unittest.main()

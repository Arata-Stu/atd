"""host TUI の Docker guard・引数・SSH 設定保存を偽コマンドで検証する。"""
import json
import os
from pathlib import Path
import pty
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from tools.vgl.bundle import make_manifest

FAKE = r'''
import json
import os
import sys
tool, args = sys.argv[1], sys.argv[2:]
with open(os.environ['ATD_TEST_CALLS'], 'a') as stream:
    stream.write(json.dumps({'tool': tool, 'args': args}) + '\n')
if tool == 'docker':
    if args[:2] == ['container', 'inspect']:
        state = os.environ.get('ATD_TEST_CONTAINER', 'running')
        if state == 'missing':
            sys.exit(1)
        project = os.environ['ATD_TEST_PROJECT']
        results = os.environ['ATD_TEST_RESULTS']
        if state == 'wrong_results':
            results += '-old'
        print(json.dumps([{'State': {'Running': state != 'stopped'},
            'Config': {'Env': ['ATD_RUNTIME_API=1', 'ATD_RUNTIME_REV=' + ('old' if state == 'old' else 'revision')]},
            'Mounts': [{'Destination': '/workspaces/atd', 'Source': project, 'RW': state != 'readonly_checkout', 'Type': 'bind'},
                       {'Destination': '/data/results', 'Source': results, 'RW': True, 'Type': 'bind'}]}]))
    elif args[-2:] == ['cat', '/etc/atd/runtime.ready']:
        print(os.environ.get('ATD_TEST_READY', 'revision'))
    elif '/usr/local/bin/atd-vgl' in args:
        print('fake backend result')
        sys.exit(int(os.environ.get('ATD_TEST_BACKEND_EXIT', '0')))
    else:
        sys.exit('Docker mutation forbidden')
elif tool in ('ssh', 'scp'):
    if os.environ.get('ATD_TEST_SSH_FAIL'):
        sys.exit(8)
    print('fake ' + tool)
else:
    sys.exit('unexpected command')
'''


class VglLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='atd-vgl-launcher-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.project = self.root / 'project'
        (self.project / 'scripts/lib').mkdir(parents=True)
        self.script = self.project / 'scripts/vgl.sh'
        shutil.copyfile(PROJECT / 'scripts/vgl.sh', self.script)
        (self.project / 'scripts/lib/detect_host.sh').write_text('detect_atd_host() { printf "pc\\n"; }\n')
        (self.project / 'scripts/lib/runtime.sh').write_text('atd_runtime_revision() { printf "revision\\n"; }\n')
        for name in ('transfer.py', 'bundle.py'):
            path = self.project / 'tools/vgl' / name
            path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(PROJECT / 'tools/vgl' / name, path)
        self.results = self.root / 'results with spaces'
        self.results.mkdir()
        self.config = self.project / 'config/vgl.env'
        self.config.parent.mkdir()
        self.storage = self.project / 'config/storage.env'
        self.calls_file = self.root / 'calls.jsonl'
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        for tool in ('docker', 'ssh', 'scp'):
            path = self.bin / tool
            path.write_text(f'#!{sys.executable}\n' + FAKE.replace(
                'tool, args = sys.argv[1], sys.argv[2:]', f'tool, args = {tool!r}, sys.argv[1:]'))
            path.chmod(0o755)
        (self.bin / 'python3').symlink_to(sys.executable)
        self.env = {'PATH': os.pathsep.join((str(self.bin), '/usr/bin', '/bin')),
                    'HOME': str(self.root), 'ATD_RESULT_DIR': str(self.results),
                    'ATD_TEST_PROJECT': str(self.project), 'ATD_TEST_RESULTS': str(self.results),
                    'ATD_TEST_CALLS': str(self.calls_file)}

    def invoke(self, *args, env=None):
        return subprocess.run(['/bin/bash', str(self.script), *args], env={**self.env, **(env or {})},
                              text=True, capture_output=True, timeout=15)

    def invoke_tty(self, *args, env=None, input=b''):
        master, slave = pty.openpty()
        output = bytearray()
        def drain():
            try:
                while True:
                    block = os.read(master, 65536)
                    if not block:
                        return
                    output.extend(block)
            except OSError:
                return
        reader = threading.Thread(target=drain, daemon=True)
        reader.start()
        try:
            process = subprocess.Popen(['/bin/bash', str(self.script), *args],
                                       env={**self.env, **(env or {})}, stdin=slave, stdout=slave, stderr=slave)
            if input:
                os.write(master, input)
            code = process.wait(timeout=15)
        finally:
            os.close(slave)
            reader.join(timeout=2)
            os.close(master)
        return code, output.decode(errors='replace')

    def calls(self, tool=None):
        entries = [json.loads(line) for line in self.calls_file.read_text().splitlines()] if self.calls_file.exists() else []
        return [entry for entry in entries if tool is None or entry['tool'] == tool]

    def backend_calls(self):
        return [entry['args'] for entry in self.calls('docker') if '/usr/local/bin/atd-vgl' in entry['args']]

    def remote_config(self):
        self.config.write_text("ATD_VGL_SSH_TARGET=jetson\nATD_VGL_SSH_PORT=2222\n"
                               "ATD_VGL_REMOTE_PROJECT='/home/jetson/ATD project'\n"
                               "ATD_VGL_REMOTE_RESULT_DIR='/media/SSD/remote results'\n")

    def bundle(self):
        path = self.results / 'models/vgl/asset1/exported/exp1'
        for relative in ('models/aliked_lightglue/aliked.onnx', 'models/aliked_lightglue/lightglue.onnx',
                         'configs/isaac/keypoint_creation_config.pb.txt', 'configs/isaac/matching_task_worker_config.pb.txt'):
            target = path / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text('fixture')
        make_manifest(path, 'asset1', 'exp1', {'family': 'aliked_lightglue', 'mode': 'aliked',
                                            'width': 848, 'height': 480, 'top_k': 2048}, {})

    def fzf(self, action='first', sequence=None):
        path = self.bin / 'fzf'
        path.write_text(f'#!{sys.executable}\n' + f'''
import sys
from pathlib import Path
lines = sys.stdin.read().splitlines()
action = {action!r}
sequence = {sequence!r}
if sequence is not None:
    state = Path({str(self.root / 'fzf-state')!r})
    index = int(state.read_text()) if state.exists() else 0
    action = sequence[index] if index < len(sequence) else 'quit'
    state.write_text(str(index + 1))
if action == 'cancel':
    sys.exit(130)
print(lines[0] if action == 'first' else next(line for line in lines if line.startswith(action + ' ')))
''')
        path.chmod(0o755)

    def test_doctor_uses_ready_current_container_and_saves_log(self):
        result = self.invoke('doctor')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        call, = self.backend_calls()
        self.assertEqual(call[-3:], ['/usr/local/bin/atd-shell', '/usr/local/bin/atd-vgl', 'doctor'])
        self.assertIn('atd_pc_dev', call)
        reports = list((self.project / 'logs/vgl/pc').glob('*.log'))
        self.assertEqual(len(reports), 1)
        self.assertEqual(reports[0].stat().st_mode & 0o777, 0o600)

    def test_export_arguments_and_defaults(self):
        result = self.invoke('--host', 'notebook', 'export', '--asset-id', 'asset1', '--export-id', 'exp1')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        call, = self.backend_calls()
        self.assertEqual(call[call.index('export'):], ['export', '--asset-id', 'asset1', '--export-id', 'exp1',
                                                     '--mode', 'aliked', '--width', '848', '--height', '480'])

    def test_package_uses_existing_onnx_shape_without_forced_dimensions(self):
        result = self.invoke('export', '--asset-id', 'asset1', '--export-id', 'exp1', '--mode', 'package',
                             '--source-dir', '/data/results/native model')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        call, = self.backend_calls()
        self.assertNotIn('--width', call)
        self.assertNotIn('--height', call)
        self.assertEqual(call[-2:], ['--source-dir', '/data/results/native model'])

    def test_fzf_selects_resolution_profile_for_export(self):
        self.fzf()
        code, output = self.invoke_tty('export', '--asset-id', 'asset1', '--export-id', 'exp1', '--mode', 'aliked')
        self.assertEqual(code, 0, output)
        call, = self.backend_calls()
        self.assertIn('--width', call)
        self.assertEqual(call[call.index('--width') + 1], '848')

    def test_jetson_rejects_export_and_builds_locally(self):
        result = self.invoke('--host', 'jetson', 'export', '--asset-id', 'asset1', '--export-id', 'exp1')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls(), [])
        result = self.invoke('--host', 'jetson', 'build', '--asset-id', 'asset1', '--export-id', 'exp1', '--build-id', 'b1')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        call, = self.backend_calls()
        self.assertIn('atd_jetson_dev', call)
        self.assertEqual(call[-2:], ['--target-id', 'jetson_orin_nano'])
        self.assertEqual(self.calls('ssh'), [])

    def test_pc_build_delegates_to_remote_without_local_docker(self):
        self.remote_config()
        result = self.invoke('build', '--asset-id', 'asset1', '--export-id', 'exp1', '--build-id', 'b1')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.calls('docker'), [])
        call, = self.calls('ssh')
        self.assertEqual(call['args'][:4], ['-p', '2222', '--', 'jetson'])
        self.assertIn("'/home/jetson/ATD project/scripts/vgl.sh'", call['args'][-1])
        self.assertIn('--host jetson build', call['args'][-1])

    def test_transfer_is_host_only_with_three_copy_stages(self):
        self.remote_config()
        self.bundle()
        result = self.invoke('transfer', '--asset-id', 'asset1', '--export-id', 'exp1')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual([entry['tool'] for entry in self.calls()], ['ssh', 'scp', 'ssh'])

    def test_ssh_failure_is_logged_and_nonzero(self):
        self.remote_config()
        result = self.invoke('build', '--asset-id', 'asset1', '--export-id', 'exp1', env={'ATD_TEST_SSH_FAIL': '1'})
        self.assertEqual(result.returncode, 1)
        report, = (self.project / 'logs/vgl/pc').glob('*.log')
        self.assertIn('終了値: 1', report.read_text())

    def test_container_guards_block_stale_or_wrong_results(self):
        for state in ('missing', 'stopped', 'old', 'wrong_results', 'readonly_checkout'):
            with self.subTest(state=state):
                result = self.invoke('doctor', env={'ATD_TEST_CONTAINER': state})
                self.assertNotEqual(result.returncode, 0)
        result = self.invoke('doctor', env={'ATD_TEST_READY': 'old'})
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.backend_calls(), [])

    def test_backend_failure_is_logged_and_propagated(self):
        result = self.invoke('doctor', env={'ATD_TEST_BACKEND_EXIT': '6'})
        self.assertEqual(result.returncode, 6)
        report, = (self.project / 'logs/vgl/pc').glob('*.log')
        self.assertIn('終了値: 6', report.read_text())

    def test_dry_run_does_not_source_configs_or_run_tools(self):
        marker = self.root / 'unwanted'
        self.storage.write_text('touch ' + str(marker) + '\n')
        self.config.write_text('touch ' + str(marker) + '\n')
        result = self.invoke('--dry-run', 'transfer')
        self.assertEqual(result.returncode, 0)
        self.assertFalse(marker.exists())
        self.assertEqual(self.calls(), [])
        self.assertFalse((self.project / 'logs').exists())

    def test_logs_work_without_docker_or_storage(self):
        self.invoke('doctor')
        before = self.calls()
        result = self.invoke('logs', env={'ATD_RESULT_DIR': ''})
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('fake backend result', result.stdout)
        self.assertEqual(self.calls(), before)

    def test_fzf_menu_and_cancel(self):
        self.fzf(sequence=['doctor', 'quit'])
        code, output = self.invoke_tty()
        self.assertEqual(code, 0, output)
        self.assertEqual(len(self.backend_calls()), 1)
        self.calls_file.unlink()
        self.fzf('cancel')
        code, output = self.invoke_tty()
        self.assertEqual(code, 0, output)
        self.assertEqual(self.calls(), [])

    def test_menu_failure_returns_to_choices_and_runs_next_action(self):
        self.fzf(sequence=['doctor', 'list', 'quit'])
        code, output = self.invoke_tty(env={'ATD_TEST_BACKEND_EXIT': '6'})
        self.assertEqual(code, 0, output)
        calls = self.backend_calls()
        self.assertEqual([call[-1] for call in calls], ['doctor', 'list'])
        self.assertIn('終了値 6 で停止しました', output)
        self.assertEqual((self.root / 'fzf-state').read_text(), '3')

    def test_menu_forwards_given_ids_and_source_to_each_direct_action(self):
        self.fzf(sequence=['export', 'doctor', 'quit'])
        code, output = self.invoke_tty('--asset-id', 'asset1', '--export-id', 'exp1', '--mode', 'package',
                                       '--source-dir', '/data/results/source with spaces')
        self.assertEqual(code, 0, output)
        calls = self.backend_calls()
        self.assertEqual(len(calls), 2)
        for call in calls:
            self.assertIn('atd_pc_dev', call)
            self.assertEqual(call[call.index('--asset-id') + 1], 'asset1')
            self.assertEqual(call[call.index('--source-dir') + 1], '/data/results/source with spaces')

    def test_fzf_selects_existing_bundle(self):
        self.remote_config()
        self.bundle()
        self.fzf()
        code, output = self.invoke_tty('transfer')
        self.assertEqual(code, 0, output)
        self.assertEqual([entry['tool'] for entry in self.calls()], ['ssh', 'scp', 'ssh'])

    def test_first_transfer_prompts_and_saves_ssh_settings_privately(self):
        self.bundle()
        code, output = self.invoke_tty('transfer', '--asset-id', 'asset1', '--export-id', 'exp1',
                                       input=b'jetson\n/home/jetson/ATD project\n/media/SSD/remote results\n2222\n')
        self.assertEqual(code, 0, output)
        self.assertEqual(self.config.stat().st_mode & 0o777, 0o600)
        config = self.config.read_text()
        self.assertIn("ATD_VGL_REMOTE_PROJECT='/home/jetson/ATD project'", config)
        self.assertIn('ATD_VGL_SSH_PORT=2222', config)

    def test_invalid_ids_and_missing_ssh_settings_fail_before_tools(self):
        result = self.invoke('build', '--asset-id', '../escape', '--export-id', 'exp1')
        self.assertNotEqual(result.returncode, 0)
        result = self.invoke('build', '--asset-id', 'asset1', '--export-id', 'exp1')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls(), [])


if __name__ == '__main__':
    unittest.main()

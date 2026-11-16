"""SCP は偽物に置き換え、受信側 stdlib 検証と昇格を実行する。"""
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from tools.vgl.bundle import make_manifest
from tools.vgl import transfer


def bundle_at(path, body=b'onnx input', export='exp1'):
    for relative in ('models/aliked_lightglue/aliked.onnx', 'models/aliked_lightglue/lightglue.onnx',
                     'configs/isaac/keypoint_creation_config.pb.txt', 'configs/isaac/matching_task_worker_config.pb.txt'):
        file = path / relative
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_bytes(body)
    make_manifest(path, 'asset1', export,
                  {'family': 'aliked_lightglue', 'mode': 'aliked', 'width': 848, 'height': 480, 'top_k': 2048}, {})
    return path


class TransferTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='atd-vgl-transfer-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.bundle = bundle_at(self.root / 'local bundle')
        self.results = self.root / "remote results ' quoted"
        self.results.mkdir()
        self.calls = []
        self.real_run = subprocess.run
        self.tamper = False
        self.scp_fail = False

    def fake_run(self, command, **kwargs):
        self.calls.append(command)
        if command[0] == 'ssh':
            remote = shlex.split(command[-1])
            self.assertEqual(remote[:2], ['python3', '-'])
            return self.real_run([sys.executable, '-', *remote[2:]], **kwargs, capture_output=True)
        if command[0] == 'scp':
            if self.scp_fail:
                raise subprocess.CalledProcessError(4, command)
            destination = Path(command[-1].split(':', 1)[1])
            shutil.copytree(command[-2], destination)
            if self.tamper:
                (destination / 'models/aliked_lightglue/aliked.onnx').write_bytes(b'corrupted')
            return subprocess.CompletedProcess(command, 0)
        self.fail('unexpected command: ' + repr(command))

    def destination(self):
        return self.results / 'models/vgl/asset1/exported/exp1'

    def send(self):
        with patch.object(transfer.subprocess, 'run', side_effect=self.fake_run):
            transfer.send_bundle(self.bundle, 'user@jetson', str(self.results), 2222)

    def test_transfer_verifies_then_promotes_with_safe_quoted_paths(self):
        self.send()
        self.assertTrue((self.destination() / 'manifest.json').is_file())
        self.assertEqual([call[0] for call in self.calls], ['ssh', 'scp', 'ssh'])
        self.assertEqual(self.calls[0][:5], ['ssh', '-p', '2222', '--', 'user@jetson'])
        self.assertEqual(self.calls[1][:5], ['scp', '-P', '2222', '-r', '--'])
        # SFTP は path 内の引用符をそのまま扱い、shell quote を二重に埋め込まない。
        self.assertIn("remote results ' quoted", self.calls[1][-1])
        self.assertEqual(list(self.destination().parent.glob('.incoming-*')), [])

    def test_repeated_identical_bundle_is_noop(self):
        self.send()
        before = (self.destination() / 'manifest.json').read_bytes()
        self.send()
        self.assertEqual((self.destination() / 'manifest.json').read_bytes(), before)
        self.assertEqual(list(self.destination().parent.glob('.incoming-*')), [])

    def test_different_existing_export_is_never_overwritten(self):
        bundle_at(self.destination(), b'existing assets')
        before = (self.destination() / 'models/aliked_lightglue/aliked.onnx').read_bytes()
        with self.assertRaises(subprocess.CalledProcessError):
            self.send()
        self.assertEqual((self.destination() / 'models/aliked_lightglue/aliked.onnx').read_bytes(), before)
        self.assertEqual(len(list(self.destination().parent.glob('.incoming-*'))), 1)

    def test_corrupted_transfer_is_not_promoted_and_stage_is_preserved(self):
        self.tamper = True
        with self.assertRaises(subprocess.CalledProcessError):
            self.send()
        self.assertFalse(self.destination().exists())
        self.assertEqual(len(list(self.destination().parent.glob('.incoming-*'))), 1)

    def test_scp_failure_never_calls_commit(self):
        self.scp_fail = True
        with self.assertRaises(subprocess.CalledProcessError):
            self.send()
        self.assertEqual([call[0] for call in self.calls], ['ssh', 'scp'])
        self.assertFalse(self.destination().exists())

    def test_invalid_local_bundle_does_not_contact_ssh(self):
        (self.bundle / 'models/aliked_lightglue/aliked.onnx').write_bytes(b'invalid')
        with self.assertRaises(ValueError):
            self.send()
        self.assertEqual(self.calls, [])

    def test_receiver_rejects_path_traversal_and_unlisted_files(self):
        incoming = '.incoming-' + 'a' * 32
        parent = self.destination().parent
        parent.mkdir(parents=True)
        stage = parent / incoming
        shutil.copytree(self.bundle, stage)
        (stage / 'unlisted.txt').write_text('unlisted')
        command = [sys.executable, '-', 'commit', str(self.results), 'asset1', 'exp1', incoming]
        result = self.real_run(command, input=transfer.RECEIVER, text=True, capture_output=True)
        self.assertEqual(result.returncode, 1)
        self.assertFalse(self.destination().exists())
        (stage / 'unlisted.txt').unlink()
        manifest = json.loads((stage / 'manifest.json').read_text())
        manifest['files']['../escape'] = {'sha256': '0' * 64, 'size': 1}
        (stage / 'manifest.json').write_text(json.dumps(manifest))
        result = self.real_run(command, input=transfer.RECEIVER, text=True, capture_output=True)
        self.assertEqual(result.returncode, 1)
        self.assertFalse((parent / 'escape').exists())

    def test_remote_symlink_parent_is_refused(self):
        elsewhere = self.root / 'elsewhere'
        elsewhere.mkdir()
        (self.results / 'models').symlink_to(elsewhere, target_is_directory=True)
        with self.assertRaises(subprocess.CalledProcessError):
            self.send()
        self.assertFalse((elsewhere / 'vgl').exists())

    def test_remote_build_uses_one_shell_quoted_command(self):
        path = "/home/user/ATD ' $(touch injected)"
        with patch.object(transfer.subprocess, 'run') as run:
            transfer.remote_build('jetson', path, ['--asset-id', 'asset1', '--export-id', 'exp1'], 22)
        command = run.call_args.args[0]
        self.assertEqual(command[:5], ['ssh', '-p', '22', '--', 'jetson'])
        self.assertEqual(shlex.split(command[-1]), ['bash', path + '/scripts/vgl.sh', '--host', 'jetson',
                                                  'build', '--asset-id', 'asset1', '--export-id', 'exp1'])

    def test_rejects_unsafe_ssh_targets_paths_and_ports(self):
        for target in ('-oProxyCommand=x', '-user@host', 'jetson;id', 'user@host name', '$(id)', ''):
            with self.subTest(target=target), self.assertRaises(ValueError):
                transfer.validate_target(target)
        for path in ('relative', '/', '//', '/tmp/../etc', '/tmp/new\nline'):
            with self.subTest(path=path), self.assertRaises(ValueError):
                transfer.validate_remote_path(path)
        for port in ('bad', 0, 65536):
            with self.subTest(port=port), self.assertRaises(ValueError):
                transfer.validate_port(port)


if __name__ == '__main__':
    unittest.main()

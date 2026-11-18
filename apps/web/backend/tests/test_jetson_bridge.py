import io
import json
import subprocess
import unittest
from unittest.mock import patch

from atd_api.jetson_bridge import BridgeUnavailable, REMOTE, request


class JetsonAdapterTests(unittest.TestCase):
    connection = {'host': 'jetson.local', 'user': 'atd', 'port': 22, 'bridge_port': 8790}

    def test_strict_ssh_and_identity_checked_without_dds(self):
        def run(argv, **kwargs):
            body = json.loads(kwargs['input'])
            self.assertIn('StrictHostKeyChecking=yes', argv)
            self.assertIn('BatchMode=yes', argv)
            self.assertEqual(body['path'], '/live/status')
            return subprocess.CompletedProcess(argv, 0, json.dumps({
                'identity': {'schema': 'atd.web_bridge.v1', 'instance_id': 'test'},
                'request_id': body['nonce'], 'data': {'pose': None}}), '')
        with patch('atd_api.jetson_bridge.subprocess.run', side_effect=run):
            self.assertTrue(request(self.connection, '/live/status')['connected'])

    def test_wrong_identity_and_injection_host_refused(self):
        with patch('atd_api.jetson_bridge.subprocess.run', return_value=subprocess.CompletedProcess([], 0,
                json.dumps({'identity': {'schema': 'other'}, 'request_id': 'stale', 'data': {}}), '')):
            with self.assertRaises(BridgeUnavailable):
                request(self.connection, '/live/status')
        with patch('atd_api.jetson_bridge.subprocess.run') as run:
            with self.assertRaises(BridgeUnavailable):
                request({**self.connection, 'host': 'jetson;echo invalid'}, '/live/status')
            run.assert_not_called()

    def test_timeout_is_not_retried(self):
        with patch('atd_api.jetson_bridge.subprocess.run', side_effect=subprocess.TimeoutExpired('ssh', 14)) as run:
            with self.assertRaises(BridgeUnavailable):
                request(self.connection, '/input/reload', 'POST')
            self.assertEqual(run.call_count, 1)

    def test_remote_allows_role_configuration_but_never_driving_routes(self):
        for path, method in (('/teleop/status', 'GET'), ('/teleop/profile', 'GET'),
                             ('/teleop/profile', 'PUT'), ('/teleop/configure', 'POST'),
                             ('/teleop/reload', 'POST'), ('/propo/status', 'GET'),
                             ('/propo/profile', 'GET'), ('/propo/profile', 'PUT'),
                             ('/propo/configure', 'POST'), ('/propo/reload', 'POST')):
            with self.subTest(path=path, method=method), patch('urllib.request.build_opener') as opener:
                opener.return_value.open.return_value.__enter__.return_value.read.return_value = b'{}'
                payload = {'port': 8790, 'path': path, 'method': method, 'nonce': 'a' * 32, 'body': {}}
                with patch('sys.stdin', io.StringIO(json.dumps(payload))), patch('sys.stdout', io.StringIO()):
                    exec(compile(REMOTE, '<ssh_bridge>', 'exec'), {})
                target = opener.return_value.open.call_args.args[0]
                self.assertEqual(target.full_url, 'http://127.0.0.1:8790' + path)
                self.assertEqual(target.method, method)
        for path, method in (('/teleop/command', 'POST'), ('/teleop/status', 'PUT'), ('/teleop/reload', 'GET'),
                             ('/propo/command', 'POST'), ('/propo/bootsel', 'POST'),
                             ('/propo/status', 'PUT'), ('/propo/reload', 'GET')):
            with self.subTest(path=path, method=method), patch('urllib.request.build_opener') as opener:
                payload = {'port': 8790, 'path': path, 'method': method, 'nonce': 'a' * 32}
                with patch('sys.stdin', io.StringIO(json.dumps(payload))), self.assertRaisesRegex(ValueError, 'Unsupported'):
                    exec(compile(REMOTE, '<ssh_bridge>', 'exec'), {})
                opener.assert_not_called()

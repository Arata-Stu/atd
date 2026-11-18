"""Recording observation only: preserve source facts and source age, without ROS."""
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from atd_web_bridge.core import Cache


def fake_node_module():
    spec = importlib.util.spec_from_file_location('recording_fake_ros', Path(__file__).with_name('test_propo_routes.py'))
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    return helper.load_bridge_module()


class RecordingObservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = fake_node_module()

    def setUp(self):
        self.node = object.__new__(self.module.BridgeNode)
        self.node.cache = Cache()
        self.node.operation_lock = threading.RLock()
        self.node.identity = {'instance_id': 'recording-bridge'}
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=200_000_000_000))
        self.logger = Mock()
        self.node.get_logger = lambda: self.logger

    def receive(self, **changes):
        value = {'schema_version': 1, 'stamp': 199.9, 'instance_id': 'manager-1', 'state': 'recording',
                 'profile': 'manual', 'bag_path': '/data/record/rosbags/test',
                 'elapsed_s': 12., 'started_at': 187.9, 'message': '',
                 'last_request': None, 'last_result': None, 'readonly_future_flag': True}
        value.update(changes)
        self.node.recording_status(SimpleNamespace(data=yaml.safe_dump(value)))

    def status(self):
        return self.node.dispatch('/recording/status', 'GET', {})

    def test_missing_is_null_and_does_not_invent_recording_false(self):
        result = self.status()
        self.assertIsNone(result['status'])
        self.assertEqual(result['availability'], {'status': 'missing'})
        self.assertEqual(result['freshness_s'], 2.5)
        self.assertNotIn('recording', result)
        self.assertNotIn('ready', result)

    def test_original_stamp_and_source_values_preserved_when_stale(self):
        with patch('atd_web_bridge.core.time.monotonic', return_value=10):
            self.receive()
        with patch('atd_web_bridge.core.time.monotonic', return_value=10.4):
            fresh = self.status()
        self.assertAlmostEqual(fresh['status']['age_s'], .5)
        self.assertEqual(fresh['status']['stamp'], 199.9)
        self.assertEqual(fresh['availability']['status'], 'fresh')
        with patch('atd_web_bridge.core.time.monotonic', return_value=13):
            stale = self.status()
        self.assertTrue(stale['status']['stale'])
        self.assertEqual(stale['availability']['status'], 'stale')
        self.assertIs(stale['status']['readonly_future_flag'], True)
        self.assertEqual(stale['status']['state'], 'recording')
        self.assertEqual(stale['status']['elapsed_s'], 12.)
        self.assertIsNone(stale['status']['last_result'])

    def test_republished_frozen_source_stamp_cannot_refresh_recording_evidence(self):
        self.receive(stamp=190.)
        result = self.status()
        self.assertGreaterEqual(result['status']['age_s'], 10.)
        self.assertEqual(result['availability']['status'], 'stale')
        self.assertIs(result['status']['readonly_future_flag'], True)
        self.receive(stamp=199.9, state='idle', readonly_future_flag=False, bag_path='', elapsed_s=None, started_at=None)
        result = self.status()
        self.assertIs(result['status']['readonly_future_flag'], False)
        self.assertEqual(result['status']['bag_path'], '')
        self.assertIsNone(result['status']['elapsed_s'])
        self.assertIsNone(result['status']['started_at'])

    def test_schema_enum_and_instance_are_strict_and_unknown_readonly_fields_pass(self):
        for changes in ({'schema_version': True}, {'schema_version': 2}, {'state': True},
                        {'state': 'active'}, {'instance_id': ''}, {'instance_id': 1}):
            with self.subTest(changes=changes):
                self.node.cache.invalidate('recording_status')
                self.receive(**changes)
                self.assertIsNone(self.status()['status'])
        result_value = {'bag_path': '/old/bag', 'success': False, 'reason': 'disk_full',
                        'elapsed_s': 1.2, 'closed_at': 190., 'size_bytes': None, 'message_count': None}
        self.receive(state='error', last_result=result_value, topics=['/joy'], storage_id='mcap')
        value = self.status()['status']
        self.assertEqual(value['last_result'], result_value)
        self.assertEqual(value['topics'], ['/joy'])
        self.assertEqual(value['storage_id'], 'mcap')

    def test_invalid_and_future_stamp_are_never_fresh(self):
        for stamp in (None, True, -1., float('nan'), float('inf'), 10**400, 201.):
            with self.subTest(stamp=stamp):
                self.node.cache.invalidate('recording_status')
                self.receive(stamp=stamp)
                self.assertNotEqual(self.status()['availability']['status'], 'fresh')
        self.node.cache.invalidate('recording_status')
        self.node.recording_status(SimpleNamespace(data='['))
        self.assertIsNone(self.status()['status'])
        self.node.recording_status(SimpleNamespace(data='x' * 65537))
        self.assertIsNone(self.status()['status'])

    def test_configured_freshness_is_observable_and_cannot_be_renewed_by_reads(self):
        self.node.cache = Cache(.5)
        with patch('atd_web_bridge.core.time.monotonic', return_value=10):
            self.receive()
        with patch('atd_web_bridge.core.time.monotonic', return_value=10.3):
            self.assertEqual(self.status()['availability']['status'], 'fresh')
        with patch('atd_web_bridge.core.time.monotonic', return_value=10.6):
            self.assertEqual(self.status()['availability']['status'], 'stale')
            self.assertEqual(self.status()['freshness_s'], .5)

    def test_no_recording_control_http_operation(self):
        for path, method in (('/recording/status', 'POST'), ('/recording/status', 'PUT'),
                             ('/recording/start', 'POST'), ('/recording/stop', 'POST')):
            with self.subTest(path=path, method=method), self.assertRaisesRegex(ValueError, 'Unsupported'):
                self.node.dispatch(path, method, {})


if __name__ == '__main__':
    unittest.main()

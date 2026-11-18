"""Layout persistence and static-TF apply evidence, without ROS or hardware."""
import copy
import importlib.util
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from atd_web_bridge.core import Cache, VehicleProfileStore


def fake_node_module():
    spec = importlib.util.spec_from_file_location('vehicle_fake_ros', Path(__file__).with_name('test_propo_routes.py'))
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    return helper.load_bridge_module()


class VehicleRoutesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = fake_node_module()

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.node = object.__new__(self.module.BridgeNode)
        self.node.cache = Cache()
        self.node.identity = {'instance_id': 'vehicle-target'}
        self.node.operation_lock = threading.RLock()
        self.node.vehicle_profiles = VehicleProfileStore(Path(directory.name) / 'layout.yaml')
        self.profile = self.node.vehicle_profiles.read()['profile']
        self.profile['dimensions']['wheelbase'] = {'value_m': .257, 'source': 'manual'}

    def dispatch(self, path, method='GET', body=None):
        return self.node.dispatch(path, method, {'instance_id': 'vehicle-target', **(body or {})})

    def status(self, **changes):
        state = {'profile_valid': True, 'tf_published': True, 'profile_path': str(self.node.vehicle_profiles.path),
                 'active_profile': self.profile, 'source_age_s': .01, 'stamp': 100., 'tf_stamp': 90.,
                 'instance_id': 'static-node-1', 'published_frames': ['rear_axle', 'front_axle']}
        state.update(changes)
        self.node.cache.put('vehicle_layout_status', state)

    def save(self):
        value = self.dispatch('/vehicle/profile')
        return self.dispatch('/vehicle/profile', 'PUT', {'profile': self.profile, 'revision': value['revision']})

    def test_node_missing_can_save_but_never_claim_applied(self):
        initial = self.dispatch('/vehicle/profile')
        self.assertFalse(initial['saved'])
        self.assertFalse(initial['pending_restart'])
        self.assertEqual(initial['cad_reference']['axis_extent_m'], {'x': .12, 'y': .2105, 'z': .0192})
        self.assertNotIn('cad_reference', initial['profile'])
        self.assertIsNone(initial['profile']['dimensions']['body_length']['value_m'])
        self.assertIsNone(self.dispatch('/vehicle/status')['status'])
        saved = self.save()
        self.assertTrue(saved['saved'] and saved['pending_restart'])
        self.assertFalse(saved['applied'])
        self.assertIsNone(saved['active_profile'])

    def test_apply_requires_fresh_published_tf_matching_path_and_profile(self):
        self.save()
        for changes in ({'tf_published': False}, {'profile_valid': False},
                        {'profile_path': '/other/layout.yaml'}, {'source_age_s': 5},
                        {'source_age_s': None}, {'active_profile': {}},
                        {'active_profile': {**self.profile, 'version': 2}}):
            with self.subTest(changes=changes):
                self.status(**changes)
                result = self.dispatch('/vehicle/profile')
                self.assertFalse(result['applied'])
                self.assertTrue(result['pending_restart'])
        self.status()
        self.assertTrue(self.dispatch('/vehicle/profile')['applied'])
        self.assertTrue(self.dispatch('/vehicle/status')['ready'])
        self.profile['dimensions']['wheelbase']['value_m'] = .26
        changed = self.save()
        self.assertTrue(changed['pending_restart'])
        self.assertFalse(changed['applied'])
        self.assertEqual(changed['active_profile']['dimensions']['wheelbase']['value_m'], .257)

    def test_revision_identity_strict_validation_and_no_live_apply_routes(self):
        self.save()
        with self.assertRaisesRegex(ValueError, '変更'):
            self.dispatch('/vehicle/profile', 'PUT', {'profile': self.profile, 'revision': 'old'})
        with self.assertRaisesRegex(ValueError, '接続先'):
            self.node.dispatch('/vehicle/profile', 'PUT', {'profile': self.profile, 'revision': 'old'})
        before = self.node.vehicle_profiles.path.read_bytes()
        bad = copy.deepcopy(self.profile)
        bad['camera']['enabled'] = True
        with self.assertRaises(ValueError):
            self.dispatch('/vehicle/profile', 'PUT', {'profile': bad, 'revision': self.dispatch('/vehicle/profile')['revision']})
        self.assertEqual(self.node.vehicle_profiles.path.read_bytes(), before)
        for endpoint in ('apply', 'reload', 'restart', 'drive'):
            with self.subTest(endpoint=endpoint), self.assertRaisesRegex(ValueError, 'Unsupported'):
                self.dispatch('/vehicle/' + endpoint, 'POST')

    def test_source_age_and_stamp_are_preserved_across_cache_reads(self):
        with patch('atd_web_bridge.core.time.monotonic', return_value=10):
            self.status(source_age_s=1.5)
        with patch('atd_web_bridge.core.time.monotonic', return_value=11.2):
            result = self.dispatch('/vehicle/status')
        self.assertAlmostEqual(result['status']['age_s'], 2.7)
        self.assertEqual(result['status']['stamp'], 100.)
        self.assertEqual(result['status']['tf_stamp'], 90.)
        self.assertTrue(result['status']['stale'])
        self.assertFalse(result['ready'])

    def test_status_callback_does_not_renew_frozen_source_stamp(self):
        import yaml
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=200_000_000_000))
        self.node.get_logger = lambda: Mock()
        self.node.vehicle_status(SimpleNamespace(data=yaml.safe_dump({'profile_valid': True, 'tf_published': True, 'stamp': 190.})))
        result = self.dispatch('/vehicle/status')
        self.assertGreaterEqual(result['status']['age_s'], 10)
        self.assertFalse(result['ready'])

    def test_future_invalid_stamp_and_status_timeout_never_apply(self):
        import yaml
        self.save()
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=200_000_000_000))
        self.node.get_logger = lambda: Mock()
        for stamp in (201., float('nan'), 10**400):
            with self.subTest(stamp=stamp):
                self.node.cache.invalidate('vehicle_layout_status')
                self.node.vehicle_status(SimpleNamespace(data=yaml.safe_dump({
                    'profile_valid': True, 'tf_published': True, 'stamp': stamp,
                    'profile_path': str(self.node.vehicle_profiles.path), 'active_profile': self.profile})))
                self.assertFalse(self.dispatch('/vehicle/profile')['applied'])
                self.assertFalse(self.dispatch('/vehicle/status')['ready'])
        with patch('atd_web_bridge.core.time.monotonic', return_value=10):
            self.status()
        with patch('atd_web_bridge.core.time.monotonic', return_value=13):
            self.assertFalse(self.dispatch('/vehicle/profile')['applied'])
            self.assertTrue(self.dispatch('/vehicle/status')['status']['stale'])


if __name__ == '__main__':
    unittest.main()

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from atd_web_bridge.core import Cache, ProfileStore, TeleopProfileStore, prepare_parameter_value


class CacheTests(unittest.TestCase):
    def test_json_integer_can_update_ros_double_but_range_and_nan_are_rejected(self):
        descriptor = {'type': 3, 'floating_point_range': [{'from_value': 0., 'to_value': 2., 'step': .5}]}
        self.assertIs(type(prepare_parameter_value(1, descriptor)), float)
        with self.assertRaises(ValueError):
            prepare_parameter_value(.3, descriptor)
        with self.assertRaises(ValueError):
            prepare_parameter_value(float('nan'), descriptor)
        with self.assertRaises(ValueError):
            prepare_parameter_value([1., float('inf')], {'type': 8})

    def test_missing_stale_and_previous_device_are_not_ready(self):
        cache = Cache()
        self.assertFalse(cache.input_status()['ready'])
        cache.put('status', {'connected': True, 'ready': True, 'configuring': False, 'generation': 4})
        cache.put('raw', {'axes': [0], 'buttons': [0], 'frame': 'atd_joy_raw_3'})
        cache.put('normalized', {'axes': [0], 'buttons': [0]})
        self.assertFalse(cache.input_status()['device_connected'])
        cache.put('raw', {'axes': [0], 'buttons': [0], 'frame': 'atd_joy_raw_4'})
        self.assertTrue(cache.input_status()['ready'])
        with patch('atd_web_bridge.core.time.monotonic', return_value=10**12):
            self.assertFalse(cache.input_status()['ready'])
            self.assertEqual(cache.input_status()['availability']['status'], 'stale')

    def test_profile_requires_fresh_config_state_and_revision(self):
        with tempfile.TemporaryDirectory() as folder:
            store = ProfileStore(Path(folder) / 'profile.yaml')
            original = store.read()
            state = {'status': {'stale': False, 'configuring': True, 'profile_path': str(store.path)}}
            with self.assertRaises(ValueError):
                store.write(original['profile'], original['revision'], {})
            with self.assertRaises(ValueError):
                store.write(original['profile'], 'old_revision', state)
            result = store.write(original['profile'], original['revision'], state)
            self.assertTrue(result['saved'])
            self.assertFalse(result['applied'])
            state['status']['stale'] = True
            with self.assertRaises(ValueError):
                store.write(original['profile'], original['revision'], state)

    def test_teleop_cache_does_not_report_stale_node_as_ready(self):
        cache = Cache()
        self.assertEqual(cache.teleop_status()['availability']['status'], 'missing')
        cache.put('teleop_status', {'ready': True, 'configuring': False})
        self.assertTrue(cache.teleop_status()['ready'])
        with patch('atd_web_bridge.core.time.monotonic', return_value=10**12):
            result = cache.teleop_status()
        self.assertFalse(result['ready'])
        self.assertIsNone(result['configuring'])
        self.assertEqual(result['availability']['status'], 'stale')

    def test_teleop_saved_profile_is_not_assumed_active(self):
        with tempfile.TemporaryDirectory() as folder:
            store = TeleopProfileStore(Path(folder) / 'teleop.yaml')
            original = store.read()
            self.assertFalse(original['saved'])
            state = {'status': {'stale': False, 'configuring': True, 'profile_valid': False,
                                'profile_path': str(store.path), 'profile': original['profile']}}
            value = original['profile']
            value['axes']['steering']['axis'] = 'left_x'
            value['axes']['throttle']['axis'] = 'r2'
            result = store.write(value, original['revision'], state)
            self.assertTrue(result['saved'])
            self.assertFalse(store.read(state)['applied'])
            state['status'].update(profile_valid=True, profile=value)
            self.assertTrue(store.read(state)['applied'])
            state['status']['profile']['axes']['throttle']['axis'] = 'left_y'
            self.assertFalse(store.read(state)['applied'])

    def test_teleop_edits_require_current_config_mode_path_and_revision(self):
        with tempfile.TemporaryDirectory() as folder:
            store = TeleopProfileStore(Path(folder) / 'teleop.yaml')
            original = store.read()
            state = {'status': {'stale': False, 'configuring': True, 'profile_path': str(store.path)}}
            for changes in ({'stale': True}, {'configuring': False}, {'profile_path': '/different/profile.yaml'}):
                with self.subTest(changes=changes), self.assertRaises(ValueError):
                    store.write(original['profile'], original['revision'], {'status': {**state['status'], **changes}})
            with self.assertRaises(ValueError):
                store.write(original['profile'], 'outdated_revision', state)
            self.assertFalse(store.path.exists())

    def test_teleop_rejects_invalid_mapping_without_overwriting_file(self):
        with tempfile.TemporaryDirectory() as folder:
            store = TeleopProfileStore(Path(folder) / 'teleop.yaml')
            original = store.read()
            state = {'status': {'stale': False, 'configuring': True, 'profile_path': str(store.path)}}
            store.write(original['profile'], original['revision'], state)
            contents = store.path.read_bytes()
            value = store.read()['profile']
            value['axes']['steering']['scale'] = float('nan')
            with self.assertRaises(ValueError):
                store.write(value, store.read()['revision'], state)
            self.assertEqual(store.path.read_bytes(), contents)


if __name__ == '__main__':
    unittest.main()

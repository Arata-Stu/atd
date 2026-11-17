import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from atd_web_bridge.core import Cache, ProfileStore, prepare_parameter_value


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


if __name__ == '__main__':
    unittest.main()

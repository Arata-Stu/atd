from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from atd_sensors import probe
from atd_sensors.profile import (DEFAULTS, DEFAULT_PATH, STREAMS, driver_arguments, load,
                                 normalize, save, with_overrides)


def device():
    profiles = []
    for key, value in DEFAULTS['streams'].items():
        from atd_sensors.profile import FORMATS
        profiles.append({'stream': key, 'format': FORMATS[key], 'width': value.get('width', 0),
                         'height': value.get('height', 0), 'fps': value['fps']})
    return {'name': 'Intel RealSense D455', 'serial': '123456', 'is_d455': True,
            'usb_type': '3.2', 'profiles': profiles}


class ProfileTests(unittest.TestCase):
    def test_default_requests_are_not_capability_guesses(self):
        value = normalize({})
        self.assertEqual(value['streams']['infra1'], {'enabled': True, 'width': 640, 'height': 480, 'fps': 90})
        self.assertEqual(value['streams']['color']['fps'], 30)
        self.assertEqual(value['streams']['accel']['fps'], 200)
        self.assertEqual(value['streams']['gyro']['fps'], 400)
        self.assertFalse(value['streams']['depth']['enabled'])
        self.assertEqual(value['serial'], '')
        config = Path(__file__).resolve().parents[1] / 'config/d455.yaml'
        self.assertEqual(json.loads(config.read_text()), DEFAULTS)

    def test_atomic_save_roundtrip_and_invalid_save_preserves_file(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'with spaces/d455.yaml'
            value = normalize({'serial': '123456'})
            save(path, value)
            self.assertEqual(load(path), value)
            before = path.read_bytes()
            with self.assertRaises(ValueError):
                save(path, {'streams': {'accel': {'fps': 0}}})
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual([item.name for item in path.parent.iterdir()], ['d455.yaml'])

    def test_invalid_schema_types_unknown_streams_and_ranges_rejected(self):
        for value in ({'version': 2}, {'unknown': True}, {'serial': '$(touch foo)'},
                      {'streams': {'imu': {}}}, {'streams': {'color': {'fps': True}}},
                      {'streams': {'color': {'enabled': 'false'}}},
                      {'streams': {'depth': {'width': 0}}}, {'unite_imu_method': 3}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                normalize(value)

    def test_no_implicit_missing_custom_config_or_relative_save(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(FileNotFoundError):
                load(Path(temp) / 'missing.yaml')
        with self.assertRaises(ValueError):
            save('relative.yaml', {})

    def test_launch_overrides_change_enabled_and_exact_modes(self):
        value = with_overrides(normalize({}), {'enable_depth': 'true', 'enable_color': 'false',
            'infra_profile': '848x480x60', 'depth_profile': '640,480,30', 'accel_fps': '250'})
        self.assertEqual(value['streams']['infra1']['width'], 848)
        self.assertEqual(value['streams']['infra2']['fps'], 60)
        self.assertEqual(value['streams']['accel']['fps'], 250)
        self.assertTrue(value['streams']['depth']['enabled'])
        self.assertFalse(value['streams']['color']['enabled'])
        for overrides in ({'infra_profile': '0x0x0'}, {'enable_depth': '1'}, {'accel_fps': 'ANY'}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                with_overrides(normalize({}), overrides)

    def test_driver_shared_infra_constraint_and_single_side_mode(self):
        value = deepcopy(DEFAULTS)
        value['streams']['infra2']['fps'] = 60
        with self.assertRaisesRegex(ValueError, 'shared infra'):
            normalize(value)
        value['streams']['infra1']['enabled'] = False
        arguments = driver_arguments(value)
        self.assertEqual(arguments['depth_module.infra_profile'], '640,480,60')

    def test_driver_namespace_frames_formats_and_serial_are_explicit(self):
        arguments = driver_arguments(normalize({'serial': '123456'}))
        self.assertEqual((arguments['camera_namespace'], arguments['camera_name']), ('camera', 'camera'))
        self.assertEqual(arguments['serial_no'], '_123456')
        self.assertEqual(arguments['depth_module.infra1_format'], 'Y8')
        self.assertEqual(arguments['rgb_camera.color_format'], 'RGB8')
        self.assertEqual(arguments['publish_tf'], 'true')
        self.assertEqual(arguments['tf_publish_rate'], '0.0')
        self.assertEqual(arguments['unite_imu_method'], '0')

    def test_multiple_devices_require_serial_and_offline_modes_are_empty(self):
        with self.assertRaisesRegex(ValueError, '複数'):
            probe.select([device(), {**device(), 'serial': 'other'}])
        self.assertEqual(probe.select([device()], '123456')['serial'], '123456')
        self.assertEqual(probe.modes({'profiles': []}, 'accel'), [])
        with self.assertRaises(ValueError):
            probe.select([], 'missing')

    def test_unsupported_default_accel_never_silently_changes_or_calls_validation(self):
        available = device()
        for value in available['profiles']:
            if value['stream'] == 'accel':
                value['fps'] = 250
        settings = normalize({})
        with patch('atd_sensors.probe.call') as call, self.assertRaisesRegex(ValueError, 'accel: 200'):
            probe.validate(settings, available)
        call.assert_not_called()
        self.assertEqual(settings['streams']['accel']['fps'], 200)

    def test_individually_supported_profiles_still_require_sdk_can_resolve(self):
        with patch('atd_sensors.probe.call', return_value={'can_resolve': False, 'serial': '123456'}), \
                self.assertRaisesRegex(ValueError, 'cannot resolve'):
            probe.validate(normalize({}), device())
        with patch('atd_sensors.probe.call', return_value={'can_resolve': True, 'serial': '123456'}) as call:
            result = probe.validate(normalize({}), device())
        self.assertEqual(result['serial'], '123456')
        arguments = call.call_args.args[0]
        self.assertIn('infra1:640:480:90', arguments)
        self.assertIn('accel:200', arguments)
        self.assertFalse(any(item.startswith('depth:') for item in arguments))

    def test_imu_combination_is_explicit(self):
        with self.assertRaises(ValueError):
            normalize({'unite_imu_method': 2, 'streams': {'accel': {'enabled': False}}})
        self.assertEqual(with_overrides(DEFAULTS, {'unite_imu_method': '2'})['unite_imu_method'], 2)


if __name__ == '__main__':
    unittest.main()

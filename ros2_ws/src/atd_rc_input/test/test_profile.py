import tempfile
from pathlib import Path
import unittest

from atd_rc_tools import profile


class ProfileTests(unittest.TestCase):
    def test_defaults_are_unassigned_and_driving_channels_are_fixed(self):
        value = profile.default_profile()
        self.assertEqual(profile.validate(value), value)
        self.assertTrue(all(role['channel'] == 0 for role in value['roles'].values()))
        self.assertEqual(profile.FIXED_ROLES, dict(steering=1, throttle=2, authority=3))
        self.assertEqual(profile.CHANNELS, tuple(range(4, 17)))

    def test_invalid_channels_types_thresholds_and_duplicates(self):
        for key, invalid in [('channel', -1), ('channel', 1), ('channel', 2), ('channel', 3),
                             ('channel', 17), ('channel', True),
                             ('low_threshold', -1), ('low_threshold', 1300),
                             ('high_threshold', 2048), ('inverted', 1)]:
            with self.subTest(key=key, invalid=invalid):
                value = profile.default_profile()
                value['roles']['recording'][key] = invalid
                with self.assertRaises(ValueError):
                    profile.validate(value)
        value = profile.default_profile()
        value['roles']['recording']['channel'] = 4
        value['roles']['auxiliary_1']['channel'] = 4
        with self.assertRaises(ValueError):
            profile.validate(value)

    def test_schema_rejects_unknown_keys_and_wrong_version(self):
        for value in [[], {}, {**profile.default_profile(), 'unknown': 1},
                      {**profile.default_profile(), 'version': True},
                      {**profile.default_profile(), 'debounce_ms': 1001}]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                profile.validate(value)

    def test_atomic_roundtrip_and_invalid_save_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'input/propo.yaml'
            value = profile.default_profile()
            value['roles']['recording']['channel'] = 6
            profile.save(path, value)
            self.assertEqual(profile.load(path), value)
            before = path.read_bytes()
            value['roles']['recording']['channel'] = 2
            with self.assertRaises(ValueError):
                profile.save(path, value)
            self.assertEqual(path.read_bytes(), before)

    def test_existing_profile_can_remap_recording_to_ch10_and_ch16(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'propo.yaml'
            value = profile.default_profile()
            value['roles']['recording']['channel'] = 6
            profile.save(path, value)
            for channel in (10, 16):
                with self.subTest(channel=channel):
                    value = profile.load(path)
                    value['roles']['recording']['channel'] = channel
                    profile.save(path, value)
                    restored = profile.load(path)
                    self.assertEqual(restored, value)
                    self.assertEqual(restored['version'], 1)
                    self.assertEqual(restored['roles']['auxiliary_1']['channel'], 0)

    def test_duplicate_yaml_keys_and_string_bool_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'profile.yaml'
            profile.save(path, profile.default_profile())
            path.write_text(path.read_text() + '\nversion: 1\n')
            with self.assertRaises(ValueError):
                profile.load(path)


if __name__ == '__main__':
    unittest.main()

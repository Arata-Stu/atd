import copy
from pathlib import Path
import tempfile
import unittest

from atd_teleop_tools import profile


class ProfileTests(unittest.TestCase):
    def test_defaults_leave_all_roles_unassigned_and_validate_canonical_copy(self):
        original = profile.default_profile()
        result = profile.validate(original)
        self.assertEqual(result, original)
        self.assertIsNot(result, original)
        self.assertTrue(all(not value for value in result['buttons'].values()))
        self.assertTrue(all(not value['axis'] and not value['subtract_axis'] for value in result['axes'].values()))
        result['axes']['steering']['axis'] = 'left_x'
        self.assertEqual(original['axes']['steering']['axis'], '')

    def test_trigger_difference_negative_scale_and_atomic_persistence(self):
        value = profile.default_profile()
        value['axes']['steering'] = dict(axis='left_x', subtract_axis='', scale=-.6)
        value['axes']['throttle'] = dict(axis='r2', subtract_axis='l2', scale=.3)
        value['buttons'].update(enable='r1', record_start='cross', record_stop='circle')
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'nested' / 'teleop.yaml'
            profile.save(path, value)
            self.assertEqual(profile.load(path), value)
            invalid = copy.deepcopy(value)
            invalid['axes']['steering']['scale'] = float('nan')
            before = path.read_bytes()
            with self.assertRaises(ValueError):
                profile.save(path, invalid)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(list(path.parent.iterdir()), [path])

    def test_unknown_keys_names_duplicate_buttons_and_strict_types_are_rejected(self):
        cases = []
        for bad in (True, 1.0, '1', 2):
            value = profile.default_profile()
            value['version'] = bad
            cases.append(value)
        for bad in (True, '0.5', float('nan'), float('inf'), -1.01, 1.01, None):
            value = profile.default_profile()
            value['axes']['throttle']['scale'] = bad
            cases.append(value)
        for bad in ('unknown', False, None, 0):
            value = profile.default_profile()
            value['axes']['throttle']['axis'] = bad
            cases.append(value)
        value = profile.default_profile()
        value['buttons']['enable'] = value['buttons']['record_start'] = 'r1'
        cases.append(value)
        value = profile.default_profile()
        value['axes']['steering']['extra'] = True
        cases.append(value)
        value = profile.default_profile()
        del value['axes']['throttle']['subtract_axis']
        cases.append(value)
        for value in cases:
            with self.subTest(value=value), self.assertRaises(ValueError):
                profile.validate(value)

    def test_loader_rejects_duplicate_keys_and_oversized_files(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'teleop.yaml'
            path.write_text('version: 1\nversion: 1\naxes: {}\nbuttons: {}\n')
            with self.assertRaisesRegex(ValueError, '重複'):
                profile.load(path)
            path.write_text(' ' * 65537)
            with self.assertRaisesRegex(ValueError, '64 KiB'):
                profile.load(path)


if __name__ == '__main__':
    unittest.main()

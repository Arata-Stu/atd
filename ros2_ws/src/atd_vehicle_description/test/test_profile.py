import copy
import math
from pathlib import Path
import tempfile
import unittest

from atd_vehicle_layout import profile


class ProfileTests(unittest.TestCase):
    def test_defaults_preserve_unknown_and_disable_camera(self):
        value = profile.default_profile()
        self.assertEqual(profile.validate(value), value)
        self.assertTrue(all(item['value_m'] is None for item in value['dimensions'].values()))
        self.assertFalse(value['camera']['enabled'])
        self.assertTrue(all(v is None for v in value['camera']['translation_m'].values()))

    def test_manual_pose_and_cad_candidate_keep_distinct_sources_and_si_units(self):
        value = profile.default_profile()
        value['dimensions']['wheelbase'] = {'value_m': .257, 'source': 'manual'}
        value['dimensions']['body_width'] = {'value_m': .12, 'source': 'cad_candidate'}
        value['camera'] = {'enabled': True, 'translation_m': dict(x=.1, y=0, z=.2),
                           'rotation_rpy_rad': dict(roll=0, pitch=math.pi / 2, yaw=-math.pi), 'source': 'manual'}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'vehicle/layout.yaml'
            profile.save(path, value)
            self.assertEqual(profile.load(path), value)
            self.assertIsNone(profile.load(path)['dimensions']['body_height']['value_m'])
            self.assertEqual(profile.load(path)['camera']['rotation_rpy_rad']['pitch'], math.pi / 2)

    def test_invalid_types_ranges_source_and_incomplete_enabled_pose(self):
        default = profile.default_profile()
        for number in (-1, 0, 21, 10**400, True, '0.2', float('nan'), float('inf')):
            value = copy.deepcopy(default)
            value['dimensions']['wheelbase'] = {'value_m': number, 'source': 'manual'}
            with self.subTest(number=number), self.assertRaises(ValueError):
                profile.validate(value)
        for field in ({'value_m': None, 'source': 'manual'}, {'value_m': .2, 'source': 'unmeasured'}):
            value = copy.deepcopy(default)
            value['dimensions']['wheelbase'] = field
            with self.subTest(field=field), self.assertRaises(ValueError):
                profile.validate(value)
        value = copy.deepcopy(default)
        value['camera']['enabled'] = True
        with self.assertRaises(ValueError):
            profile.validate(value)
        for number in (math.pi + .001, True, float('nan')):
            value = copy.deepcopy(default)
            value['camera']['source'] = 'manual'
            value['camera']['rotation_rpy_rad']['yaw'] = number
            with self.subTest(angle=number), self.assertRaises(ValueError):
                profile.validate(value)

    def test_partial_disabled_pose_is_saved_and_zero_requires_explicit_source(self):
        value = profile.default_profile()
        value['camera']['translation_m']['x'] = 0
        value['camera']['source'] = 'manual'
        value['dimensions']['rear_overhang'] = {'value_m': 0, 'source': 'manual'}
        self.assertEqual(profile.validate(value)['camera']['translation_m']['x'], 0.)
        self.assertIsNone(profile.validate(value)['camera']['translation_m']['y'])

    def test_schema_duplicate_keys_and_atomic_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'layout.yaml'
            profile.save(path, profile.default_profile())
            original = path.read_bytes()
            bad = {**profile.default_profile(), 'base_frame': 'old_kart'}
            with self.assertRaises(ValueError):
                profile.save(path, bad)
            self.assertEqual(path.read_bytes(), original)
            path.write_text(path.read_text() + '\nversion: 1\n')
            with self.assertRaises(ValueError):
                profile.load(path)


if __name__ == '__main__':
    unittest.main()

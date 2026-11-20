"""The partial Raccordo Junior must meet the GP road through a clear opening."""

from math import cos, dist, sin
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from geography import load_track
from monza_trackside import REFERENCE
from scenery import Features, _crosses_ring, _in_ring, _inside, monza_features
from world import LAP_ALTERNATE_WIDTH_M, alternate_pavement


class JuniorMergeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = monza_features(True)
        (cls.props, cls.center), = cls.data.lines('junior_merge_centerline')
        cls.width = LAP_ALTERNATE_WIDTH_M['junior_merge_centerline']

    def test_connector_enters_gp_road_from_the_right_after_ascari(self):
        self.assertFalse(list(monza_features(False).lines('junior_merge_centerline')))
        self.assertEqual(self.props['osm_id'], 34404709)
        self.assertEqual(self.props['tags']['note'], 'draft')
        self.assertAlmostEqual(sum(dist(a, b) for a, b in zip(self.center, self.center[1:])),
                               22.44, delta=.05)
        edges, poses = load_track('lap')
        main_road = [edges['left'], edges['right']]
        self.assertFalse(_inside(*self.center[0], main_road))
        self.assertTrue(_inside(*self.center[-1], main_road))
        s, _, _, _ = min(poses, key=lambda pose: dist(pose[1:3], self.center[-1]))
        self.assertAlmostEqual(s, 406.24, delta=.25)
        ax, ay = self.center[-2]
        _, x, y, yaw = min(poses, key=lambda pose: dist(pose[1:3], (ax, ay)))
        self.assertLess((ay-y)*cos(yaw)-(ax-x)*sin(yaw), 0)
        self.assertTrue(all(z < 0 for face in alternate_pavement(self.center, self.width)
                            for _, _, z in face))

    def test_full_width_clears_gravel_beds_and_barriers(self):
        barriers = Features([REFERENCE], self.data.project)
        # Test five extra centimetres on either shoulder for clearance of rail profiles.
        faces = [[p[:2] for p in face]
                 for face in alternate_pavement(self.center, self.width + .10)]
        for kind in ('fence', 'guard_rail', 'wall'):
            for props, points in barriers.lines(kind):
                for face in faces:
                    self.assertFalse(any(_in_ring(*p, face) for p in points), props)
                    self.assertFalse(any(_crosses_ring(a, b, face)
                                         for a, b in zip(points, points[1:])), props)
        for props, rings in self.data.polygons('sand'):
            for face in faces:
                self.assertFalse(any(_inside(*p, rings) for p in face), props)
                for ring in rings:
                    self.assertFalse(any(_in_ring(*p, face) for p in ring), props)
                    self.assertFalse(any(_crosses_ring(a, b, ring)
                                         for a, b in zip(face, face[1:]+face[:1])), props)

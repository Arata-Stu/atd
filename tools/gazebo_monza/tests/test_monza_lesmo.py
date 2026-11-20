"""Placement of the gravel-bed protection and lights at both Lesmo bends."""

from math import cos, sin
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from geography import load_track
from monza_lesmo import GRAVEL_BACKS, MARSHAL_LIGHTS, details
from scenery import _inside, monza_features


class LesmoTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.edges, cls.poses = load_track('lap')
        data = monza_features(True)
        cls.gravel = [rings for props, rings in data.polygons('sand')
                      if props['osm_id'] in GRAVEL_BACKS]
        cls.meshes = {name: faces for name, _, faces in details(cls.edges, data)}

    def test_tyres_stand_behind_both_gravel_beds(self):
        self.assertEqual(len(self.gravel), 2)
        belts = self.meshes['lesmo_runoff_belts']
        tyre_faces = self.meshes['lesmo_tyre_shoulders']
        self.assertTrue(belts and tyre_faces)
        vertices = {p[:2] for face in belts + tyre_faces for p in face}
        for polygon in self.gravel:
            self.assertFalse(any(_inside(*xy, polygon) for xy in vertices))
            self.assertTrue(any(abs(x-px) < .12 and abs(y-py) < .12
                                for px, py in polygon[0] for x, y in vertices))
        road = [self.edges['left'], self.edges['right']]
        # Ground-contact vertices include the extremities of every belt segment.
        self.assertFalse(any(_inside(x, y, road) for f in belts for x, y, z in f if z < .01))

    def test_marshal_screens_face_the_approaching_driver_on_the_assigned_side(self):
        screens = self.meshes['lesmo_marshal_screens']
        self.assertEqual(len(screens), 3)
        self.assertEqual([light[0] for light in MARSHAL_LIGHTS], [8, 9, 10])
        for screen, station, left in zip(screens, (205, 248, 280), (True, True, False)):
            cx, cy = (sum(p[axis] for p in screen)/len(screen) for axis in (0, 1))
            s, x, y, yaw = min(self.poses, key=lambda p: (p[1]-cx)**2+(p[2]-cy)**2)
            self.assertAlmostEqual(s, station, delta=1.2)
            self.assertEqual((cy-y)*cos(yaw)-(cx-x)*sin(yaw) > 0, left)
            a, b, c = screen[:3]
            nx = (b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1])
            ny = (b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2])
            self.assertLess(nx*cos(yaw)+ny*sin(yaw), 0)
            self.assertFalse(any(_inside(cx, cy, polygon) for polygon in self.gravel))

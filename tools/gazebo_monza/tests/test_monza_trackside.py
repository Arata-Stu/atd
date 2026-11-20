"""Geometry-only checks for Monza's full-lap roadside structures."""

from math import cos, hypot, sin
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import geography
import monza_trackside
import scenery


class MonzaTracksideTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.edges, cls.poses = geography.load_track('lap')
        cls.scenery = scenery.monza_features(True)
        cls.items = scenery.Features([monza_trackside.REFERENCE], geography.project)
        cls.meshes = {name: faces for name, _, faces in monza_trackside.geometry(cls.edges, cls.scenery)}

    def test_low_barriers_clear_gp_pavement_gravel_and_chicane_bypass(self):
        road = [self.edges['left'], self.edges['right']]
        sand = [rings for _, rings in self.scenery.polygons('sand')]
        branches = [points for kind in ('bypass_centerline', 'motorcycle_centerline')
                    for _, points in self.scenery.lines(kind)]
        for kind in ('guard_rail', 'fence', 'wall'):
            for props, points in self.items.lines(kind):
                for p, q in zip(points, points[1:]):
                    self.assertFalse(scenery._inside(*p, road))
                    self.assertFalse(any(scenery._crosses_ring(p, q, ring) for ring in road))
                    self.assertFalse(any(scenery._inside(*p, rings) or
                                         scenery._crosses_ring(p, q, rings[0]) for rings in sand))
                    # Test each consecutive segment; an open bypass has no closing segment.
                    self.assertFalse(any(scenery._crosses_ring(p, q, [a, b])
                                         for branch in branches for a, b in zip(branch, branch[1:])))

    def test_mapped_bridges_span_both_edges_and_leave_vertical_clearance(self):
        bridges = list(self.items.lines('overpass'))
        self.assertEqual({props['osm_way'] for props, _ in bridges}, {34404729, 194458112, 194458113})
        for props, (a, b) in bridges:
            self.assertTrue(all(scenery._crosses_ring(a, b, edge) for edge in self.edges.values()))
        road = [self.edges['left'], self.edges['right']]
        for name in ('monza_bridge_steel', 'monza_bridge_roof', 'monza_bridge_deck'):
            for face in self.meshes[name]:
                if min(p[2] for p in face) < .46:
                    self.assertTrue(all(not scenery._inside(x, y, road) for x, y, _ in face))
                    for p, q in zip(face, face[1:] + face[:1]):
                        self.assertFalse(any(scenery._crosses_ring(p, q, edge) for edge in road))

    def test_rear_straight_has_nearby_rails_on_both_sides(self):
        for s in (420, 450):
            _, x, y, yaw = min(self.poses, key=lambda pose: abs(pose[0]-s))
            sides = set()
            for _, points in self.items.lines('guard_rail'):
                for a, b in zip(points, points[1:]):
                    dx, dy = b[0]-a[0], b[1]-a[1]
                    t = max(0, min(1, ((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy)))
                    rx, ry = a[0]+t*dx-x, a[1]+t*dy-y
                    if hypot(rx, ry) < 2.4:
                        sides.add(cos(yaw)*ry-sin(yaw)*rx > 0)
            self.assertEqual(sides, {False, True})

    def test_inside_fence_enters_both_infrared_views_at_parabolica(self):
        _, x, y, yaw = min(self.poses, key=lambda pose: abs(pose[0]-485))
        supports = self.meshes['monza_posts']
        for eye in (-.095/2, .095/2):
            visible_from_right = []
            for face in supports:
                for px, py, z in face:
                    if z < .25:
                        continue
                    forward = cos(yaw)*(px-x) + sin(yaw)*(py-y) - .18
                    lateral = -sin(yaw)*(px-x) + cos(yaw)*(py-y) - eye
                    if 1 < forward < 8 and -forward*.90 < lateral < -.4:
                        visible_from_right.append(forward)
                        break
            self.assertTrue(visible_from_right)

    def test_home_wall_keeps_tagged_height_and_material_batches_are_bounded(self):
        pit = [props for props, _ in self.items.lines('wall') if props.get('osm_way') == 243090150]
        self.assertTrue(pit)
        self.assertEqual({float(w['tags']['height']) for w in pit}, {1.3})
        self.assertTrue(self.meshes['monza_pit_wall_green'])
        self.assertAlmostEqual(max(z for face in self.meshes['monza_pit_wall_green']
                                   for _, _, z in face), .13)
        self.assertTrue(self.meshes['monza_historic_wall'])
        self.assertLessEqual(len(self.meshes), 12)
        self.assertLess(sum(len(faces) for faces in self.meshes.values()), 30000)
        self.assertFalse(list(monza_trackside.geometry(self.edges, self.scenery, lap=False)))

"""Geometry-only checks for Monza's full-lap roadside structures."""

from math import cos, dist, hypot, sin
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

    def test_radio_masts_are_rooted_on_mapped_sites_outside_the_road(self):
        sites = self.items.by_kind['radio_mast']
        self.assertEqual({s['properties']['osm_node'] for s in sites}, {3710856693, 5774539345})
        bases = {(x, y) for face in self.meshes['monza_radio_masts'] for x, y, z in face if z == 0}
        for site in sites:
            cx, cy = geography.project(*site['geometry']['coordinates'])
            self.assertTrue(any(hypot(x-cx, y-cy) < .05 for x, y in bases))
        road = [self.edges['left'], self.edges['right']]
        self.assertFalse(any(scenery._inside(x, y, road) for x, y in bases))

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

    def test_grandstand_catch_panels_clear_the_seats_and_project_left_in_stereo(self):
        faces = self.meshes['monza_catch_mesh']
        road = [self.edges['left'], self.edges['right']]
        stands = [rings for _, rings in self.scenery.polygons('grandstand')]
        for a, b, c, d in faces:
            self.assertTrue(all(not scenery._inside(x, y, road) for x, y, z in (a, b, c, d)))
            for rings in stands:
                self.assertFalse(scenery._inside(*a[:2], rings))
                self.assertFalse(scenery._crosses_ring(a[:2], b[:2], rings[0]))

        # Before the grid and at intermediate stations on the opening straight.
        for station in (560., 0., 8., 20.):
            _, x, y, yaw = min(self.poses, key=lambda p: abs(p[0]-station))
            for eye in (-.0475, .0475):
                in_view = False
                for face in faces:
                    for px, py, z in face:
                        depth = cos(yaw)*(px-x) + sin(yaw)*(py-y) - .18
                        left = cos(yaw)*(py-y) - sin(yaw)*(px-x) - eye
                        if 1 < depth < 8 and .4 < left < depth*.94 and z >= .3:
                            in_view = True
                self.assertTrue(in_view, (station, eye))

    def test_home_wall_keeps_tagged_height_and_material_batches_are_bounded(self):
        pit = [props for props, _ in self.items.lines('wall') if props.get('osm_way') == 243090150]
        self.assertTrue(pit)
        self.assertEqual({float(w['tags']['height']) for w in pit}, {1.3})
        self.assertTrue(self.meshes['monza_pit_wall_green'])
        self.assertAlmostEqual(max(z for face in self.meshes['monza_pit_wall_green']
                                   for _, _, z in face), .13)
        self.assertTrue(self.meshes['monza_historic_wall'])
        # The wall adds four batches for wire, frames, gate leaves and exit paint.
        self.assertLessEqual(len(self.meshes), 18)
        wall_faces = sum(len(faces) for name, faces in self.meshes.items() if name.startswith('pit_wall_'))
        self.assertLess(sum(map(len, self.meshes.values())) - wall_faces, 30000)
        self.assertLess(wall_faces, 1300)
        self.assertFalse(list(monza_trackside.geometry(self.edges, self.scenery, lap=False)))

    def test_main_straight_overlaps_are_on_track_facing_blades_at_tagged_height(self):
        overlaps = self.meshes['monza_lap_steel']
        for station, expected_heights in ((8, {'left': .12}), (30, {'left': .11, 'right': .11})):
            _, x, y, heading = min(self.poses, key=lambda p: abs(p[0] - station))
            seen = {side: [] for side in expected_heights}
            for face in overlaps:
                a, b, c = face[:3]
                dx, dy = a[0] - x, a[1] - y
                if abs(cos(heading) * dx + sin(heading) * dy) > 1.:
                    continue
                side = 'left' if cos(heading) * dy - sin(heading) * dx > 0 else 'right'
                # Cross product's horizontal projection must face the racing lane.
                nx = (b[1]-a[1])*(c[2]-a[2]) - (b[2]-a[2])*(c[1]-a[1])
                ny = (b[2]-a[2])*(c[0]-a[0]) - (b[0]-a[0])*(c[2]-a[2])
                self.assertGreater(nx * -dx + ny * -dy, 0.)
                seen[side].extend(z for _, _, z in face)
            for side, height in expected_heights.items():
                self.assertTrue(seen[side], (station, side))
                self.assertAlmostEqual(max(seen[side]), height * .98)

    def test_vehicle_refuge_has_painted_end_and_open_overlap(self):
        rails = [(p, pts) for p, pts in self.items.lines('guard_rail') if p.get('refuge')]
        (props, first), (_, second) = rails
        self.assertEqual(props['vehicle_exit_band_rc_m'], .20)
        _, x, y, heading = min(self.poses, key=lambda p: abs(p[0] - 40.5))
        end, restart = first[-1], second[0]
        self.assertGreater(cos(heading)*(end[1]-y) - sin(heading)*(end[0]-x), 1.)
        gap = restart[0]-end[0], restart[1]-end[1]
        self.assertAlmostEqual(-sin(heading)*gap[0] + cos(heading)*gap[1], .45, delta=.01)
        self.assertLess(cos(heading)*gap[0] + sin(heading)*gap[1], 0)
        for _, rings in self.scenery.polygons('grandstand'):
            for a, b in zip(second, second[1:]):
                self.assertFalse(scenery._inside(*a, rings) or scenery._inside(*b, rings))
                self.assertFalse(scenery._crosses_ring(a, b, rings[0]))
        faces = self.meshes['monza_refuge_orange']
        self.assertEqual(len(faces), 8)
        for face in faces:
            self.assertAlmostEqual(dist(face[0], face[1]), .20)

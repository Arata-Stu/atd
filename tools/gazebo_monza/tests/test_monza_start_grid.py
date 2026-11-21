"""Place home-straight paint and pit/stand detailing against the pinned OSM edges."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import geography
import monza_start_grid
import scenery


def centroid(face):
    return tuple(sum(p[i] for p in face) / len(face) for i in (0, 1))


class MonzaStartGridTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.edges, _ = geography.load_track()
        cls.straight = monza_start_grid.PitStraight(cls.edges)
        cls.batches = {name: faces for name, _, faces
                       in monza_start_grid.details(cls.edges, scenery.monza_features(True))}

    def test_pole_is_on_left_with_alternating_slots_behind_start(self):
        paint = self.batches['grid_white_paint']
        control, start = [self.straight.station(centroid(face)) for face in paint[:2]]
        self.assertAlmostEqual(start, 0, places=6)
        self.assertAlmostEqual(start - control, 30.9, places=6)
        bars = paint[2::3]
        self.assertEqual(len(bars), 22)
        for i, face in enumerate(bars):
            x, y = centroid(face)
            s = self.straight.station((x, y))
            center = self.straight.ground(s, 0, 0)
            lateral = (x-center[0])*self.straight.left[0] + (y-center[1])*self.straight.left[1]
            self.assertGreater(lateral * (-1)**i, .25)
            self.assertLess(s, 0)
            if i:
                self.assertAlmostEqual(self.straight.station(centroid(bars[i-1]))-s, .8)

    def test_paint_and_repairs_face_up_and_stay_inside_the_track(self):
        gp_road = [self.edges['left'], self.edges['right']]
        for name in ('grid_white_paint', 'grid_yellow_guides', 'grid_asphalt_repairs'):
            for face in self.batches[name]:
                a, b, c = face[:3]
                self.assertGreater((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]), 0)
                for x, y, z in face:
                    self.assertTrue(scenery._inside(x, y, gp_road))
                    self.assertTrue(0 < z < .001)

    def test_pits_on_right_stands_on_left_and_bounded_draws(self):
        for name, faces in self.batches.items():
            if name.startswith(('pit_straight', 'pit_stands')):
                for face in faces:
                    if name in ('pit_straight_lane', 'pit_straight_apron'):
                        self.assertTrue(all(p[2] < 0 for p in face))
                        continue
                    x, y = centroid(face)
                    lateral = sum((p-o)*v for p, o, v in zip((x, y), self.straight.origin,
                                                            self.straight.left))
                    self.assertGreater(lateral if name.startswith('pit_stands') else -lateral, 1.3)
        self.assertLessEqual(len(self.batches), 23)
        self.assertLess(sum(map(len, self.batches.values())), 2750)

    def test_pit_entry_island_and_bollard_end_at_the_mapped_wall(self):
        road = [self.edges['left'], self.edges['right']]
        island, = self.batches['pit_access_blue']
        start, end = min(map(self.straight.station, island)), max(map(self.straight.station, island))
        self.assertAlmostEqual(start, monza_start_grid.PIT_ENTRY_DASHES[1])
        self.assertAlmostEqual(end, self.straight.station(geography.project(*monza_start_grid.PIT_WALL_TIP)))
        self.assertTrue(all(not scenery._inside(x, y, road) for x,y,z in island))
        post = self.batches['pit_entry_bollard']
        self.assertTrue(all(abs(self.straight.station(p)-start) < .018 for face in post for p in face))
        self.assertTrue(.10 < max(p[2] for face in post for p in face) < .11)
        self.assertTrue(all(not scenery._inside(x, y, road) for face in post for x,y,z in face))

    def test_track_edge_switches_to_dashes_only_at_both_pit_accesses(self):
        right = self.edges['right']
        holes = []
        for a, b in zip(right, right[1:] + right[:1]):
            s0, s1 = self.straight.station(a), self.straight.station(b)
            spans = list(self.straight.right_paint_spans(a, b))
            covered_to = 0.
            for begin, end in [*spans, (1., 1.)]:
                if begin > covered_to + 1e-8:
                    holes.append(tuple(s0+f*(s1-s0) for f in (covered_to,begin)))
                covered_to = end
        for lo, hi in holes:
            self.assertTrue(any(a-1e-8 <= lo < hi <= b+1e-8 for a,b in
                                (monza_start_grid.PIT_ENTRY_DASHES, monza_start_grid.PIT_EXIT_DASHES)))
        for a,b in (monza_start_grid.PIT_ENTRY_DASHES, monza_start_grid.PIT_EXIT_DASHES):
            missing = sum(hi-lo for lo,hi in holes if a-1e-8 <= lo < hi <= b+1e-8)
            self.assertAlmostEqual(missing/(b-a), .5, delta=.02)

    def test_both_speed_limit_boards_face_oncoming_pit_traffic(self):
        rings = self.batches['pit_speed_limit_red']
        centers = []
        for board in (rings[:16], rings[16:]):
            centers.append(centroid([p for face in board for p in face]))
            for face in board:
                a,b,c = face[:3]
                nx = (b[1]-a[1])*(c[2]-a[2]) - (b[2]-a[2])*(c[1]-a[1])
                ny = (b[2]-a[2])*(c[0]-a[0]) - (b[0]-a[0])*(c[2]-a[2])
                self.assertLess(nx*self.straight.forward[0]+ny*self.straight.forward[1], 0)
                self.assertTrue(all(not scenery._inside(x,y,[self.edges['left'],self.edges['right']])
                                    for x,y,z in face))
        for center in centers:
            self.assertAlmostEqual(self.straight.station(center), monza_start_grid.PIT_SPEED_LIMIT_STATION,
                                   delta=.03)
        self.assertAlmostEqual(sum((a-b)*n for a,b,n in zip(*centers,self.straight.left)), .81, delta=.01)

    def test_podium_is_raised_over_pit_wall_and_footbridge_reaches_garages(self):
        concrete = self.batches['pit_podium_concrete']
        top = max(concrete, key=len)
        station = self.straight.station(centroid(top))
        self.assertTrue(-28 < station < -27)
        self.assertTrue(all(z > .55 for x, y, z in top))
        area2 = sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(top, top[1:] + top[:1]))
        self.assertGreater(area2, 1.5)  # Up-facing, approx. 11 m diameter at full scale.
        gp_road = [self.edges['left'], self.edges['right']]
        for face in concrete + self.batches['pit_podium_handrails']:
            self.assertTrue(all(not scenery._inside(x, y, gp_road) for x, y, z in face))
        # The two bridge corners at the building end straddle the mapped doorway.
        data = scenery.monza_features(True)
        gx, gy = data.project(*monza_start_grid.PODIUM_GARAGE_ENTRANCE)
        corners = {(x, y) for face in concrete for x, y, z in face
                   if (x-gx)**2 + (y-gy)**2 < .10**2 and z > .55}
        self.assertEqual(len(corners), 2)
        self.assertAlmostEqual(sum(x for x, y in corners)/2, gx)
        self.assertAlmostEqual(sum(y for x, y in corners)/2, gy)

    def test_start_lights_span_the_signal_station_and_face_the_grid(self):
        gp_road = [self.edges['left'], self.edges['right']]
        for name in ('start_gantry_steel', 'start_gantry_housings', 'start_gantry_unlit_lenses'):
            for face in self.batches[name]:
                self.assertLess(abs(self.straight.station(centroid(face))), .04)
                for x, y, z in face:
                    if scenery._inside(x, y, gp_road):
                        self.assertGreaterEqual(z, .50)
        lights = self.batches['start_gantry_unlit_lenses']
        self.assertEqual(len(lights), 10)
        center = centroid([p for f in lights for p in f])
        self.assertAlmostEqual(self.straight.station(center), -.026)
        self.assertAlmostEqual(sum((p-o)*n for p,o,n in zip(center, self.straight.origin,
                                                            self.straight.left)), 0, places=3)
        for face in lights:
            a, b, c = face[:3]
            nx = (b[1]-a[1])*(c[2]-a[2]) - (b[2]-a[2])*(c[1]-a[1])
            ny = (b[2]-a[2])*(c[0]-a[0]) - (b[0]-a[0])*(c[2]-a[2])
            self.assertLess(nx*self.straight.forward[0] + ny*self.straight.forward[1], 0)

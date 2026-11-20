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
        self.assertLessEqual(len(self.batches), 16)
        self.assertLess(sum(map(len, self.batches.values())), 2200)

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

"""Check the Monza pit wall against its two mapped runs and the generated starting grid."""
from math import dist
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import geography
import monza_pit_wall
import monza_start_grid
import monza_trackside
import scenery


class PitWallTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.edges, _ = geography.load_track('lap')
        cls.straight = monza_start_grid.PitStraight(cls.edges)
        cls.mesh = {name: faces for name, _, faces in monza_trackside.geometry(cls.edges, scenery.monza_features(True))}
        trackside = scenery.Features([monza_trackside.REFERENCE], geography.project)
        cls.wall = [line for props, line in trackside.lines('wall') if props.get('osm_way') == 243090150]

    def test_closed_leaves_replace_concrete_at_both_grid_slots(self):
        pieces = [part for line in self.wall for part in monza_pit_wall.sections(self.straight, line)]
        self.assertEqual(len(self.wall), 2)
        gates = [(a, b) for is_gate, a, b in pieces if is_gate]
        paint = self.mesh['pit_wall_exit_orange']
        self.assertEqual(len(gates), 2)
        self.assertEqual(len(paint), 2)
        deck = self.mesh['monza_pit_wall_green']
        def extent(face):
            return min(self.straight.station(p) for p in face), max(self.straight.station(p) for p in face)

        for (a, b), slot, stripe in zip(gates, (21, 6), paint):
            s = self.straight.station
            self.assertAlmostEqual((s(a) + s(b))/2, -.24 - .8*(slot - 1))
            self.assertAlmostEqual(dist(a, b), .45, places=4)
            self.assertAlmostEqual(dist(stripe[0], stripe[-1]), .20)
            self.assertTrue(s(a) < s(stripe[0]) < s(stripe[-1]) < s(b))
            self.assertFalse(any(hi > s(a)+.002 and lo < s(b)-.002 for lo, hi in map(extent, deck)))
            leaves = [face for face in self.mesh['pit_wall_gate_leaves'] if s(a) < s(face[0]) < s(b)]
            self.assertEqual(len(leaves), 12)
            self.assertTrue(all(.001 < p[2] <= .13 for face in leaves for p in face))

    def test_wire_has_open_hatches_and_stays_clear_of_podium_gap_and_track(self):
        net = self.mesh['pit_wall_board_mesh']
        def bounds(face):
            s = [self.straight.station(p) for p in face]
            z = [p[2] for p in face]
            return min(s), max(s), min(z), max(z)
        rects = [bounds(face) for face in net]
        # Each window has strips above and below it, with actual empty space
        # at mid-height independent of alpha filtering or viewing direction.
        openings = [r for r in rects if .15 < r[3] < .16]
        self.assertGreaterEqual(len(openings), 10)
        for lo, hi, _, _ in openings:
            center = (lo + hi)/2
            self.assertFalse(any(a < center < b and z0 < .20 < z1 for a, b, z0, z1 in rects))

        first_end = self.straight.station(self.wall[0][-1])
        second_start = self.straight.station(self.wall[1][0])
        road = [self.edges['left'], self.edges['right']]
        for name in ('pit_wall_board_mesh', 'pit_wall_board_frames', 'pit_wall_gate_leaves', 'pit_wall_exit_orange'):
            vertices = {p for face in self.mesh[name] for p in face}
            for x, y, _ in vertices:
                station = self.straight.station((x, y))
                self.assertFalse(first_end+.01 < station < second_start-.01)
                self.assertFalse(scenery._inside(x, y, road))

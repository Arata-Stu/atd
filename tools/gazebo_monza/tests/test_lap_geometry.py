"""Check the mapped GP camera paths and their road boundaries."""
from math import degrees, dist, remainder, tau
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from circuits import circuit_ids, load_lap, track_data
from geography import (SCALE, _point_at, _stations, ascari_entry_kerb, kerb_intervals,
                       load_track, project, reference_metadata)


class LapGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.edges, cls.poses = load_track("lap")

    def test_timing_line_direction_distance_and_seam(self):
        poses = self.poses
        start = reference_metadata("lap")["start_finish"]
        self.assertEqual((start["osm_node"], start["osm_relation"]), (1828499259, 284565))
        self.assertEqual(start["roles"], ["start", "finish"])
        self.assertEqual(poses[0][1:3], list(project(9.2811729, 45.6189632)))
        self.assertEqual(poses[-1][1:], poses[0][1:])
        self.assertAlmostEqual(poses[-1][0], 579.3, delta=.5)  # Official 5,793 m at 1:10.
        clockwise_area = sum(b[1]*a[2] - a[1]*b[2] for a, b in zip(poses, poses[1:])) / 2
        self.assertGreater(clockwise_area, 0)
        for a, b in zip(poses, poses[1:]):
            step = b[0] - a[0]
            self.assertAlmostEqual(step, dist(a[1:3], b[1:3]), places=9)
            self.assertGreater(step, 0)
            self.assertLess(step, .12)
            self.assertLess(abs(degrees(remainder(b[3] - a[3], tau))), 7)
        self.assertAlmostEqual(degrees(poses[0][3]), 85, delta=3)
        self.assertLess(min(degrees(p[3]) for p in poses if 61 < p[0] < 65), -3)
        self.assertGreater(max(degrees(p[3]) for p in poses if 68 < p[0] < 72), 100)

    def test_complete_path_stays_in_pavement_including_interpolated_segments(self):
        xy = [p[1:3] for p in self.poses]
        x0, y0 = xy[0]
        clearance = .12  # Greater than the length of any interpolated segment above.
        for side, ring in self.edges.items():
            crossings = 0
            for (ax, ay), (bx, by) in zip(ring, ring[1:] + ring[:1]):
                dx, dy = bx - ax, by - ay
                if (ay > y0) != (by > y0) and x0 < ax + (y0 - ay)*dx/dy:
                    crossings += 1
                xmin, xmax = min(ax, bx) - clearance, max(ax, bx) + clearance
                ymin, ymax = min(ay, by) - clearance, max(ay, by) + clearance
                for x, y in xy:
                    if xmin <= x <= xmax and ymin <= y <= ymax:
                        t = min(1, max(0, ((x-ax)*dx + (y-ay)*dy) / (dx*dx + dy*dy)))
                        self.assertGreater(dist((x, y), (ax + t*dx, ay + t*dy)), clearance)
            # Start inside the outer ring, outside the inner ring. With the gap
            # to every boundary larger than any sample step, no segment can exit.
            self.assertEqual(crossings % 2, int(side == "left"))

    def test_default_remains_short_and_unknown_routes_fail(self):
        _, default = load_track()
        self.assertEqual(default, load_track("chicane")[1])
        self.assertEqual(len(default), 481)
        self.assertAlmostEqual(default[-1][0], 23.96104847, places=6)
        self.assertNotIn("start_finish", reference_metadata())
        with self.assertRaises(ValueError):
            load_track("pit")

    def test_lesmo_and_ascari_kerbs_follow_the_correct_edges(self):
        # Sample lap chainage independently of the left/right boundary lengths.
        lap = kerb_intervals(self.edges, True)
        short = kerb_intervals(self.edges, False)
        for side, vertices in self.edges.items():
            arc = _stations(vertices)
            self.assertEqual(short[side], lap[side][:1])
            self.assertTrue(all(a[1] < b[0] for a, b in zip(lap[side], lap[side][1:])))
            expected = {
                225: "right", 236: "left",   # Lesmo 1 apex and track-out
                257: "right", 265: "left",   # Lesmo 2 apex and track-out
                365: "left", 375: "right", 384: "left", 391: "right",
                340: None, 420: None,        # straights on either side of Ascari
            }
            for s, kerb_side in expected.items():
                with self.subTest(station=s, side=side):
                    xy = min(self.poses, key=lambda p: abs(p[0]-s))[1:3]
                    i = min(range(len(vertices)), key=lambda j: dist(vertices[j], xy))
                    on_kerb = any(start <= arc[i] <= end for start, end in lap[side])
                    self.assertEqual(on_kerb, side == kerb_side)

    def test_recessed_entry_kerb_starts_at_the_t8_braking_board(self):
        right = self.edges['right']
        start, end = ascari_entry_kerb(right)
        entry = _point_at(right, _stations(right), start)
        nearest_pose = min(self.poses, key=lambda pose: dist(pose[1:3], entry))
        self.assertAlmostEqual(nearest_pose[0], 352.35, delta=.25)
        self.assertAlmostEqual(end-start, 100*SCALE)
        for raised_start, raised_end in kerb_intervals(self.edges, True)['right']:
            self.assertTrue(end < raised_start or start > raised_end)

    def test_offset_roads_keep_both_triangles_facing_up_through_every_corner(self):
        def signed_area(a, b, c):
            return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

        for circuit in circuit_ids():
            if circuit == "monza":
                continue  # Independent boundary rings, covered above.
            with self.subTest(circuit=circuit):
                edges, poses, _ = load_lap(circuit)
                data = track_data(circuit)
                course, = [f for f in data["features"] if f["properties"]["kind"] == "course"]
                width = course["properties"]["width_m"] * data.get("horizontal_scale", SCALE)
                left, right = edges["left"], edges["right"]
                for i, (a, b) in enumerate(zip(left, right)):
                    j = (i + 1) % len(left)
                    c, d = right[j], left[j]
                    self.assertGreater(signed_area(a, b, c), 0, f"outside fold at sample {i}")
                    self.assertGreater(signed_area(a, c, d), 0, f"inside fold at sample {i}")
                    self.assertAlmostEqual(dist(a, b), width)
                    self.assertAlmostEqual((a[0] + b[0]) / 2, poses[i][1])
                    self.assertAlmostEqual((a[1] + b[1]) / 2, poses[i][2])


if __name__ == "__main__":
    unittest.main()

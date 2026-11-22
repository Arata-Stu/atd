"""Validate the venue-local Odaiba lap and its RC road boundaries."""
import json
from math import degrees, dist, remainder, tau
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from circuits import circuit_ids, load_lap, track_data


class LapGeometryTests(unittest.TestCase):
    def test_bundled_hdmap_matches_gazebo_edges_timing_line_and_direction(self):
        root = Path(__file__).resolve().parents[3] / "samples/odaiba"
        document = json.loads((root / "hdmap/map.json").read_text())
        boundaries, poses, _ = load_lap("odaiba")
        for side in ("left", "right"):
            self.assertEqual(document[side], [list(point) for point in boundaries[side]])
        self.assertEqual(set(document["lines"]), {"centerline", "raceline"})
        for kind, line in document["lines"].items():
            with self.subTest(line=kind):
                exported = json.loads((root / "lines" / f"{kind}.json").read_text())
                self.assertEqual(exported.pop("map_id"), document["id"])
                self.assertLessEqual(exported.pop("revision"), document["revision"])
                self.assertEqual(line, exported)
                a, b = line["points"][:2]
                self.assertLess(abs(a[0] - poses[0][1]), .05)
                self.assertLess(b[0], a[0])

    def test_timing_line_direction_distance_and_seam(self):
        _, poses, project = load_lap("odaiba")
        self.assertEqual(circuit_ids(), ["odaiba"])
        self.assertEqual(project(5, 10), (1, 2))
        self.assertEqual(poses[0][1:3], [0., 0.])
        self.assertEqual(poses[-1][1:], poses[0][1:])
        self.assertAlmostEqual(abs(degrees(poses[0][3])), 180, delta=2)
        self.assertAlmostEqual(poses[-1][0], 80, delta=.5)
        for a, b in zip(poses, poses[1:]):
            step = b[0] - a[0]
            self.assertAlmostEqual(step, dist(a[1:3], b[1:3]), places=9)
            self.assertGreater(step, 0)
            self.assertLess(step, .15)
            self.assertLess(abs(degrees(remainder(b[3] - a[3], tau))), 15)

    def test_offset_roads_keep_both_triangles_facing_up_through_every_corner(self):
        def signed_area(a, b, c):
            return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

        for circuit in circuit_ids():
            with self.subTest(circuit=circuit):
                edges, poses, _ = load_lap(circuit)
                data = track_data(circuit)
                course, = [f for f in data["features"] if f["properties"]["kind"] == "course"]
                width = course["properties"]["width_m"] * data["horizontal_scale"]
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

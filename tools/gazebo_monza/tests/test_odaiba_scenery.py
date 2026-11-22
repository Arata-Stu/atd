"""Registration of pit access and sight lines from the low stereo rig."""
import json
from math import cos, dist, sin, tan
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import circuits
import odaiba_scenery
from world import BASELINE_M, FORWARD_M, HFOV


class OdaibaSceneryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        edges, cls.poses, project = circuits.load_lap("odaiba")
        cls.project = staticmethod(project)
        cls.faces = {name: faces for name, _, faces in odaiba_scenery.geometry(edges, project)}
        cls.plan = json.loads(odaiba_scenery.REFERENCE.read_bytes())

    def test_kart_barriers_leave_both_pit_mouths_open(self):
        boards = self.faces["odaiba_barrier_white"] + self.faces["odaiba_barrier_blue"]
        for gate in ("entry_m", "exit_m"):
            anchor = self.project(*self.plan["pit_lane"][gate])
            nearest = min(dist(v[:2], anchor) for face in boards for v in face)
            with self.subTest(gate=gate):
                self.assertGreater(nearest, .25)
                self.assertLess(nearest, .65)

    def stereo_vertices(self, feature, station, near, far, min_z=0.):
        _, x, y, yaw = min(self.poses, key=lambda pose: abs(pose[0]-station))
        common = set()
        for face in self.faces["odaiba_" + feature]:
            for px, py, pz in face:
                forward = (px-x)*cos(yaw) + (py-y)*sin(yaw) - FORWARD_M
                lateral = -(px-x)*sin(yaw) + (py-y)*cos(yaw)
                if (near < forward < far and pz >= min_z
                        and abs(lateral)+BASELINE_M/2 < forward*tan(HFOV/2)
                        and abs(pz-.12) < forward*tan(HFOV/2)*720/1280):
                    common.add((px, py, pz))
        return common

    def test_pit_and_terrace_present_in_both_frusta_on_return_leg(self):
        # Close range makes this sensitive to a misplaced footprint or a second
        # 1/5 reduction. This geometric check leaves final appearance to OGRE.
        for feature, station in (("pit_columns", 59), ("terrace_rails", 69)):
            with self.subTest(feature=feature):
                self.assertGreaterEqual(len(self.stereo_vertices(feature, station, .3, 10)), 8)

    def test_start_approach_and_exit_have_binocular_landmarks_above_barriers(self):
        # At station 4, both the nearby equipment and a more distant planted verge
        # should be in view. Past the apex, the frame faces away from the north gate.
        for feature, station, near, far in (("truss", 0, 5, 8), ("truss", 4, 2, 4),
                                             ("leaves", 4, 5, 10), ("island_rails", 5.5, .3, 5)):
            with self.subTest(feature=feature, station=station):
                self.assertGreaterEqual(len(self.stereo_vertices(feature, station, near, far, .13)), 8)

    def test_turn_furniture_keeps_clear_of_every_lane(self):
        # Full lap including the adjacent folded lanes; half-road width is 0.70 m.
        for feature in ("truss", "equipment", "island_rails", "leaves", "shaded_leaves", "tree_bark"):
            corners = {v[:2] for face in self.faces["odaiba_" + feature] for v in face}
            with self.subTest(feature=feature):
                self.assertGreater(min(dist(c, pose[1:3]) for c in corners for pose in self.poses), .75)


if __name__ == "__main__":
    unittest.main()

"""Cover the former treeline gaps while keeping gravel and spectator areas open."""

from math import atan2, cos, sin
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from geography import load_track, project
from scenery import (Features, MONZA_BACKGROUND, SCENERY_FILE, SCENERY_LAP_FILE,
                     _crosses_ring, _inside, monza_features, woodland)


def planes(data):
    return {tuple(face) for _, faces in woodland(True, data=data) for face in faces}


class ForestMarginsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.full = monza_features(True)
        old = Features([SCENERY_FILE, SCENERY_LAP_FILE, MONZA_BACKGROUND], project)
        cls.extra = planes(cls.full) - planes(old)

    def test_forests_span_both_sides_of_the_camera_at_the_sparse_apexes(self):
        # Count ground-plane positions in the horizontal view cone, before occlusion.
        centers = {((face[0][0]+face[1][0])/2, (face[0][1]+face[1][1])/2)
                   for face in self.extra}
        _, poses = load_track('lap')
        for station in (224, 252, 366, 374):
            _, x, y, yaw = min(poses, key=lambda p: abs(p[0]-station))
            sides = set()
            for px, py in centers:
                dx, dy = px-x, py-y
                angle = atan2(dy*cos(yaw)-dx*sin(yaw), dx*cos(yaw)+dy*sin(yaw))
                if dx**2+dy**2 < 20**2 and abs(angle) < .73:
                    sides.add(angle > 0)
            self.assertEqual(sides, {False, True}, station)
        heights = [max(p[2] for p in face) for face in self.extra]
        self.assertLess(min(heights), 1.)
        self.assertGreater(max(heights), 2.)

    def test_new_crowns_and_trunks_clear_gravel_and_stands(self):
        polygons = [rings for kind in ('sand', 'grandstand', 'building', 'pit_building')
                    for _, rings in self.full.polygons(kind)]
        for face in self.extra:
            a, b = face[0][:2], face[1][:2]
            for rings in polygons:
                self.assertFalse(_inside(*a, rings) or _inside(*b, rings))
                self.assertFalse(any(_crosses_ring(a, b, ring) for ring in rings))

    def test_back_straight_scrub_is_low_and_left_of_the_road(self):
        beds = list(self.full.polygons('scrub'))
        self.assertTrue(beds)
        self.assertTrue(all(p['osm_polygons'] == ['way/425588793'] for p, _ in beds))
        _, poses = load_track('lap')
        count = 0
        for face in self.extra:
            a, b = face[0], face[1]
            x, y = (a[0]+b[0])/2, (a[1]+b[1])/2
            if not any(_inside(x, y, rings) for _, rings in beds):
                continue
            station, px, py, yaw = min(poses, key=lambda p: (p[1]-x)**2+(p[2]-y)**2)
            self.assertGreater(station, 435.)
            self.assertGreater((y-py)*cos(yaw)-(x-px)*sin(yaw), 0)
            self.assertLessEqual(max(p[2] for p in face), .57)
            count += 1
        self.assertGreater(count, 0)

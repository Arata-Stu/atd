"""Coverage and clearance checks for the full GP circuit's static geometry."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import geography
import scenery


class LapSceneryTest(unittest.TestCase):
    def test_short_section_is_preserved_and_other_corners_have_mapped_landmarks(self):
        def stand_names(lap):
            return {props['tags'].get('name') for props, _ in scenery.polygons('grandstand', lap)}

        self.assertLess(stand_names(False), stand_names(True))
        self.assertTrue({'Tribuna Centrale', 'Tribuna Roggia', 'Tribuna Ascari', 'Tribuna Parabolica'}
                        <= stand_names(True))
        self.assertEqual({props['tags']['name'] for props, _ in scenery.polygons('pit_building', True)},
                         {'Palazzina Box', 'Hospitality Building'})
        self.assertFalse(list(scenery.polygons('pit_building', False)))
        self.assertFalse(any(name == 'stand_canopies' for name, _, _ in scenery.grandstand()))
        self.assertTrue(any(name == 'stand_canopies' for name, _, _ in scenery.grandstand(lap=True)))
        self.assertGreater(len(list(scenery.sand_triangles(True))), len(list(scenery.sand_triangles())))

    def test_lap_tree_planes_clear_gp_road_and_stay_inside_woodland(self):
        def planes(lap):
            return {(tuple(face[0][:2]), tuple(face[1][:2]))
                    for _, faces in scenery.woodland(lap) for face in faces}

        short, full = planes(False), planes(True)
        self.assertLess(short, full)
        self.assertLess(len(full) // 2, 3000)
        regions = [rings for _, rings in scenery.polygons('woodland', True)]
        edges, _ = geography.load_track()
        for a, b in full - short:
            self.assertTrue(any(scenery._inside(*a, region) and scenery._inside(*b, region)
                                and not any(scenery._crosses_ring(a, b, ring) for ring in region)
                                for region in regions))
            self.assertFalse(scenery._inside(*a, [edges['left'], edges['right']]))
            self.assertFalse(any(scenery._crosses_ring(a, b, edge) for edge in edges.values()))

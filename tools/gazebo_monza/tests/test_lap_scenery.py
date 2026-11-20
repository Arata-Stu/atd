"""Coverage and clearance checks for the full GP circuit's static geometry."""
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from xml.etree import ElementTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import geography
import scenery
import world


class LapSceneryTest(unittest.TestCase):
    def test_background_is_in_range_of_both_lap_cameras(self):
        for course, route, expected in [('monza', 'lap', 100), ('monza', 'chicane', 45),
                                         ('silverstone', 'lap', 45)]:
            with self.subTest(course=course, route=route), TemporaryDirectory() as tmp:
                directory = Path(tmp)
                metadata = world.build(directory, circuit=course, route=route)
                sdf = ElementTree.parse(directory / 'world.sdf')
                sensors = sdf.findall(".//sensor/camera/clip/far")
                self.assertEqual([float(clip.text) for clip in sensors], [expected, expected])
                self.assertEqual(metadata['camera']['far_clip_m'], expected)
                if course == 'monza' and route == 'lap':
                    self.assertTrue((directory / 'building_walls.obj').is_file())
                    self.assertIn('background_sha256', metadata['geography'])
                    for name in ('grid_white_paint', 'start_gantry_housings', 'monza_bridge_roof',
                                 'monza_guardrail', 'monza_pit_wall_green', 'monza_historic_wall',
                                 'pit_podium_concrete', 'pit_podium_handrails', 'monza_radio_masts'):
                        visual = sdf.find(f".//visual[@name='{name}']")
                        self.assertIsNotNone(visual, name)
                        self.assertTrue(Path(visual.findtext('geometry/mesh/uri')).is_file(), name)
                    self.assertIn('trackside_sha256', metadata['geography'])
                    net = sdf.find(".//visual[@name='monza_catch_mesh']")
                    self.assertEqual(net.findtext('material/double_sided'), 'true')
                    tile = Path(net.findtext('material/pbr/metal/albedo_map'))
                    self.assertEqual(tile.name, 'monza_catch_mesh.png')
                    self.assertEqual(tile.read_bytes()[24:26], bytes((8, 6)))  # PNG RGBA8
                    obj = Path(net.findtext('geometry/mesh/uri')).read_text().splitlines()
                    uv = [tuple(map(float, row.split()[1:])) for row in obj if row.startswith('vt ')]
                    # Upright panels repeat wire diamonds in both axes.
                    self.assertGreater(abs(uv[1][0] - uv[0][0]), 1.)
                    self.assertGreater(abs(uv[2][1] - uv[1][1]), 1.)
                else:
                    self.assertNotIn('background_sha256', metadata['geography'])
                    self.assertIsNone(sdf.find(".//model[@name='monza_trackside']"))

    def test_background_has_mapped_objects_beyond_old_clipping_plane(self):
        from math import cos, sin, tan

        background = scenery.Features([scenery.MONZA_BACKGROUND], geography.project)
        buildings = [rings[0] for _, rings in background.polygons('building')]
        _, poses = geography.load_track('lap')
        # A footprint vertex can project inside both stereo images beyond the previous 45 m
        # from the start, after Rettifilo and on the far side of the lap. Occlusion is a render test.
        for station in (0, 72, 290):
            _, x, y, yaw = min(poses, key=lambda p: abs(p[0] - station))
            def projects_ahead(point):
                dx, dy = point[0] - x, point[1] - y
                depth = dx*cos(yaw) + dy*sin(yaw) - world.FORWARD_M
                side = dy*cos(yaw) - dx*sin(yaw)
                return 45 < depth < world.MONZA_LAP_FAR_M and abs(side) + world.BASELINE_M/2 < depth*tan(world.HFOV/2)
            self.assertTrue(any(projects_ahead(p) for ring in buildings for p in ring), station)
        self.assertEqual({props['tree_spacing_rc_m'] for props, _ in background.polygons('woodland')},
                         {4.5, 7.0})

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

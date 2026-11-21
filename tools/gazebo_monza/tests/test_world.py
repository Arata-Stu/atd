"""Headless, dependency-free checks for the generated Gazebo inputs."""
from datetime import timedelta
from hashlib import sha256
import json
from math import cos, degrees, dist, isclose, pi, sin
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import geography
from monza_motion import MonzaMotion, interpolate, travel_fraction
from sensor_profile import IMU_MODES, RESOLUTIONS, imu_streams
import world


def in_ring(x, y, ring):
    inside = False
    for (ax, ay), (bx, by) in zip(ring, ring[1:]):
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def ground_faces(visual):
    rows = Path(visual.findtext("geometry/mesh/uri")).read_text().splitlines()
    vertices = [tuple(map(float, row.split()[1:3])) for row in rows if row.startswith("v ")]
    return [[vertices[int(ref.split("/")[0]) - 1] for ref in row.split()[1:]]
            for row in rows if row.startswith("f ")]


def on_surface(x, y, faces):
    for a, b, c in faces:
        sides = [(q[0] - p[0]) * (y - p[1]) - (q[1] - p[1]) * (x - p[0])
                 for p, q in ((a, b), (b, c), (c, a))]
        if min(sides) >= -1e-8 or max(sides) <= 1e-8:
            return True
    return False


class MonzaWorldTest(unittest.TestCase):
    def test_pinned_car_chicane_and_rc_reference_frame(self):
        meta = geography.reference_metadata()
        self.assertEqual(meta["sha256"], sha256(geography.TRACK_FILE.read_bytes()).hexdigest())
        self.assertEqual(meta["osm_ways"], {
            "straight": {"id": 19842206, "version": 56},
            "chicane": {"id": 179968242, "version": 9},
            "exit": {"id": 179968264, "version": 6},
            "left": {"id": 1569593286, "version": 1},
            "right": {"id": 1569604642, "version": 1},
        })
        lon, lat = 9.2818939, 45.6244492
        self.assertEqual(geography.project(lon, lat), (0, 0))
        self.assertAlmostEqual(geography.project(lon + .001, lat)[0], 7.799, places=2)
        self.assertAlmostEqual(geography.project(lon, lat + .001)[1], 11.114, places=2)
        self.assertEqual(meta["horizontal_scale"], .1)

    def test_monza_geometry_and_sensors(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "scene"
            scene = world.build(output, height=.135, pitch_deg=4)
            self.assertNotIn("trace_interval_s", scene["trajectory"])
            sdf = ET.parse(output / "world.sdf")
            rig = sdf.find(".//model[@name='d455_rig']")
            left, right = rig.findall("./link/sensor")
            self.assertEqual([s.findtext("frame_id") for s in (left, right)],
                             ["camera_infra1_optical_frame", "camera_infra2_optical_frame"])
            self.assertEqual(left.findtext("topic"), "/monza/left/image")
            self.assertEqual(right.findtext("camera/camera_info_topic"), "/monza/right/camera_info")
            lp, rp = [list(map(float, s.findtext("pose").split())) for s in (left, right)]
            self.assertAlmostEqual(lp[1]-rp[1], .095)
            self.assertEqual(lp[2], .135)
            self.assertGreater(lp[4], 0)
            self.assertEqual(lp[2:], rp[2:])
            for sensor in (left, right):
                self.assertEqual(sensor.findtext("update_rate"), "90")
                self.assertEqual([sensor.findtext(f"camera/image/{key}") for key in ("width", "height", "format")],
                                 ["640", "480", "L_INT8"])
            self.assertEqual(sdf.findtext(".//render_engine"), "ogre2")
            self.assertEqual(rig.findtext("plugin/scene_file"), str((output / "scene.json").resolve()))
            self.assertEqual(rig.findtext("plugin/module_name"), "monza_motion")
            for tag in ("uri", "albedo_map"):
                self.assertTrue(all(Path(e.text).is_file() for e in sdf.iter(tag)))
            poses = scene["trajectory"]["samples_s_x_y_yaw"]
            self.assertEqual(poses[0][0], 0)
            self.assertAlmostEqual(poses[-1][0], 23.96, places=1)
            self.assertEqual(scene["trajectory"]["end_s_m"], poses[-1][0])
            self.assertTrue(all(a[0] < b[0] for a, b in zip(poses, poses[1:])))
            self.assertLess(min(dist((x, y), (0, 0)) for s, x, y, yaw in poses if 11.9 < s < 12.1), .05)
            # Heading is measured CCW from east. The pit straight runs north,
            # T1 turns right beyond east, and T2 turns left towards north again.
            self.assertAlmostEqual(degrees(poses[0][3]), 85, delta=3)
            self.assertLess(min(degrees(p[3]) for p in poses if 13 < p[0] < 16), -15)
            self.assertAlmostEqual(degrees(poses[-1][3]), 105, delta=3)

    def test_stereo_resolutions_and_imu_clock(self):
        for resolution, (width, height) in RESOLUTIONS.items():
            with self.subTest(resolution=resolution), tempfile.TemporaryDirectory() as tmp, patch.object(world, "track"):
                scene = world.build(tmp, resolution=resolution, imu_hz=400)
                sdf = ET.parse(Path(tmp) / "world.sdf")
                self.assertEqual([(int(c.findtext("width")), int(c.findtext("height")))
                                  for c in sdf.findall(".//sensor/camera/image")], [(width, height)] * 2)
                camera = scene["camera"]
                self.assertEqual((camera["width"], camera["height"]), (width, height))
                self.assertAlmostEqual(camera["nominal_focal_px"] / width, 337.2096 / 640, places=6)
                self.assertLess(float(sdf.findtext(".//physics/max_step_size")), 1 / 400)
                self.assertEqual(sdf.findtext(".//world/gravity"), "0 0 -9.80665")
                self.assertEqual(len(scene["imu"]["streams"]), 3)
        self.assertEqual(imu_streams(IMU_MODES["off"]), {})
        for options in ({"resolution": "1280x800"}, {"imu_hz": -1}):
            with self.assertRaises(ValueError):
                world.build(Path("/unused"), **options)

    def test_asphalt_mesh_covers_the_line_and_leaves_runoff_on_both_sides(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            scene = world.build(output)
            sdf = ET.parse(output / "world.sdf")
            self.assertEqual(scene["environment"], "outdoor_circuit")
            self.assertEqual(scene["scenery"], "monza_prima_variante_osm")
            manifest = scene["geography"]
            for filename, checksum in (("file", "sha256"), ("scenery_file", "scenery_sha256")):
                self.assertEqual(manifest[checksum], sha256((world.HERE / manifest[filename]).read_bytes()).hexdigest())
            self.assertIsNotNone(sdf.find(".//light[@type='directional']"))
            self.assertEqual(sdf.findtext(".//model[@name='grandstand']/static"), "true")
            trees = sdf.findall(".//model[@name='woodland']/link/visual")
            self.assertGreaterEqual(len(trees), 2)
            tree_roots = []
            for tree in trees:
                material = tree.find("material")
                self.assertEqual(material.findtext("double_sided"), "true")
                rows = Path(tree.findtext("geometry/mesh/uri")).read_text().splitlines()
                vertices = [tuple(map(float, r.split()[1:])) for r in rows if r.startswith("v ")]
                for i in range(0, len(vertices), 4):
                    base = [(x, y) for x, y, z in vertices[i:i+4] if z == 0]
                    self.assertEqual(len(base), 2)
                    (ax, ay), (bx, by) = base
                    tree_roots.append(((ax + bx) / 2, (ay + by) / 2))
            visuals = {v.get("name"): v for v in sdf.findall(".//model[@name='circuit']/link/visual")}
            self.assertEqual(Path(visuals["grass"].findtext(".//albedo_map")).name, "grass.jpg")
            self.assertEqual(Path(visuals["floor"].findtext(".//albedo_map")).name, "floor.jpg")
            faces = ground_faces(visuals["floor"])
            bypass = ground_faces(visuals["bypass_centerline"])
            decks = ground_faces(sdf.find(".//visual[@name='stand_decks']"))

        for s, x, y, yaw in scene["trajectory"]["samples_s_x_y_yaw"]:
            self.assertTrue(on_surface(x, y, faces), f"カメラ台が舗装から外れた位置 {s:.3f} m")
            if s < 5:
                # Mapped width near the ride start is ~1 m at RC scale.
                for offset in (-.9, .9):
                    self.assertFalse(on_surface(x - offset * sin(yaw), y + offset * cos(yaw), faces))
        # An incoming driver can continue onto the separate escape road,
        # whose straight centre does not belong to the double-bend GP strip.
        x, y = geography.project(9.281957, 45.625164)
        self.assertFalse(on_surface(x, y, faces))
        self.assertTrue(on_surface(x, y, bypass))
        reference = json.loads((geography.TRACK_FILE.parent / "monza_scenery.geojson").read_text())
        forest = [[[geography.project(*p) for p in ring] for ring in f["geometry"]["coordinates"]]
                  for f in reference["features"] if f["properties"]["kind"] == "woodland"]
        self.assertTrue(tree_roots)
        for x, y in set(tree_roots):
            self.assertTrue(any(in_ring(x, y, rings[0]) and not any(in_ring(x, y, hole) for hole in rings[1:])
                                for rings in forest), f"樹木が地図の林域から外れています {(x, y)}")
        # Use top-face centroids to match all six structures to real footprints.
        # Coincident XY wall vertices give degenerate triangles in this projection.
        deck_centres = {(sum(x for x, _ in tri) / 3, sum(y for _, y in tri) / 3)
                        for tri in decks if len(set(tri)) == 3}
        for feature in reference["features"]:
            if feature["properties"]["kind"] == "grandstand":
                ring = [geography.project(*p) for p in feature["geometry"]["coordinates"][0]]
                self.assertTrue(any(in_ring(x, y, ring) for x, y in deck_centres), feature["id"])

    def test_endpoints_and_benchmark_return_trip(self):
        self.assertEqual([travel_fraction(t, 10, False) for t in (0, 5, 10, 25)], [0, .5, 1, 1])
        self.assertEqual([travel_fraction(t, 10, True) for t in (0, 5, 10, 15, 20)], [0, .5, 1, .5, 0])
        x, y, yaw = interpolate([[0, 0, 0, pi-.1], [2, 1, 3, -pi+.1]], [0, 2], 1)
        self.assertEqual((x, y), (.5, 1.5))
        self.assertTrue(isclose(yaw, pi))

    def test_closed_loop_crosses_the_start_line_forwards(self):
        for t, phase in [(0, 0), (9.9, .99), (10, 0), (10.1, .01), (24, .4)]:
            self.assertAlmostEqual(travel_fraction(t, 10, True, closed=True), phase)
        self.assertEqual(travel_fraction(11, 10, False, closed=True), 1)

    def test_speed_and_laps_use_the_actual_route_length(self):
        with tempfile.TemporaryDirectory() as tmp:
            scene = world.build(tmp, circuit="spa", speed_mps=3.5, laps=3, wait_for_capture=True,
                                record_ground_truth=True)
            route = scene["trajectory"]
            self.assertEqual(route["trace_interval_s"], .05)
            self.assertEqual(route["route"], "lap")
            self.assertEqual(route["laps"], 3)
            self.assertEqual(route["speed_mps"], 3.5)
            samples = route["samples_s_x_y_yaw"]
            length = sum(dist(a[1:3], b[1:3]) for a, b in zip(samples, samples[1:]))
            self.assertAlmostEqual(route["end_s_m"], length)
            self.assertAlmostEqual(route["travel_seconds"], length / 3.5)
            self.assertAlmostEqual(route["total_travel_seconds"], 3 * length / 3.5)
            self.assertEqual(samples[0][1:], samples[-1][1:])

    def test_invalid_motion_settings_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "scene"
            for options in ({"speed_mps": 0}, {"speed_mps": float("inf")},
                            {"speed_mps": 2, "travel_seconds": 10}, {"laps": -1},
                            {"laps": 1.5}, {"laps": 2, "repeat": True},
                            {"laps": 2, "route": "chicane"}):
                with self.subTest(options=options), self.assertRaises(ValueError):
                    world.build(output, **options)
            self.assertFalse(output.exists())

    def test_gated_run_crosses_intermediate_finish_and_stops_on_the_last_lap(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            motion = MonzaMotion()
            motion.closed, motion.repeat, motion.laps, motion.duration = True, False, 3, 4.
            motion.start_s = None
            motion.distances = [0, 2, 4]
            motion.samples = [[0, 0, 0, 0], [2, 2, 0, 0], [4, 0, 0, 0]]
            motion.capture_ready, motion.lap_start = root / "ready", root / "start"
            motion.trace, motion.capture_finish = root / "trace", root / "finished"
            motion.pose_type = lambda *pose: pose
            positions = []
            motion.model = SimpleNamespace(set_world_pose_cmd=lambda _, pose: positions.append(pose[0]))
            motion.link = SimpleNamespace(world_pose=lambda _: SimpleNamespace(
                x=lambda: positions[-1], y=lambda: 0, yaw=lambda: 0))
            for t in range(16):
                if t == 2:
                    motion.capture_ready.touch()
                tick = SimpleNamespace(sim_time=timedelta(seconds=t), paused=False)
                motion.pre_update(tick, None)
                motion.post_update(tick, None)
                self.assertEqual(motion.capture_finish.exists(), t >= 14)
            self.assertEqual(positions, [0, 1, 2, 1] * 3 + [0, 0])
            trace = [json.loads(line) for line in motion.trace.read_text().splitlines()]
            self.assertEqual(trace[-1]["sim_time_s"], 14.)

    def test_lap_video_waits_for_first_stereo_pair_before_moving(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            motion = MonzaMotion()
            motion.closed, motion.repeat, motion.duration, motion.start_s = True, False, 10, None
            motion.distances = [0, 4, 8]
            motion.samples = [[0, 0, 0, 0], [4, 4, 0, 0], [8, 0, 0, 0]]
            motion.capture_ready, motion.lap_start = root / "lap_capture.ready", root / "lap_start.json"
            poses = []
            motion.pose_type = lambda *pose: pose
            motion.model = SimpleNamespace(set_world_pose_cmd=lambda _, pose: poses.append(pose))
            for t in (2, 5, 8, 18):
                if t == 5:
                    motion.capture_ready.touch()
                motion.pre_update(SimpleNamespace(sim_time=timedelta(seconds=t), paused=False), None)
            self.assertEqual([pose[0] for pose in poses], [0, 2.4, 0])
            self.assertEqual(json.loads(motion.lap_start.read_text()), {"sim_time_s": 5})

    def test_trace_reads_physics_pose_at_half_second_intervals(self):
        with tempfile.TemporaryDirectory() as temporary:
            motion = MonzaMotion()
            motion.trace = Path(temporary) / "rig_motion.jsonl"
            motion.next_trace_s = 0
            motion.duration = 10
            measured = SimpleNamespace(x=lambda: 1.3, y=lambda: 2.1, yaw=lambda: -.4)
            motion.link = SimpleNamespace(world_pose=lambda ecm: measured)
            for t, pause in [(0, True), (.01, False), (.23, False), (.51, False)]:
                motion.post_update(SimpleNamespace(sim_time=timedelta(seconds=t), paused=pause), None)
            samples = [json.loads(line) for line in motion.trace.read_text().splitlines()]
            self.assertEqual([s["sim_time_s"] for s in samples], [.01, .51])
            self.assertEqual(samples[-1]["x_m"], 1.3)
            self.assertEqual(samples[-1]["yaw_rad"], -.4)

    def test_bag_trace_targets_twenty_samples_per_sim_second_at_physics_ticks(self):
        with tempfile.TemporaryDirectory() as tmp:
            scene_file = Path(tmp) / "scene.json"
            scene_file.write_text(json.dumps({"trajectory": {
                "samples_s_x_y_yaw": [[0, 0, 0, 0], [1, 1, 0, 0]], "travel_seconds": 120,
                "repeat": False, "trace_interval_s": .05, "route": "lap",
            }}))
            measured = SimpleNamespace(x=lambda: 2., y=lambda: 1., yaw=lambda: .3)
            model = SimpleNamespace(canonical_link=lambda _: None)
            link = SimpleNamespace(world_pose=lambda _: measured)
            sim = SimpleNamespace(Model=lambda _: model, Link=lambda _: link)
            motion = MonzaMotion()
            with patch.dict(sys.modules, {"gz.sim": sim, "gz.math": SimpleNamespace(Pose3d=object)}):
                motion.configure(None, SimpleNamespace(get_string=lambda _: scene_file), None, None)
            for i in range(180):
                motion.post_update(SimpleNamespace(sim_time=timedelta(seconds=i / 90), paused=False), None)
            samples = [json.loads(line) for line in motion.trace.read_text().splitlines()]
            self.assertEqual(len(samples), 40)
            self.assertEqual({s["x_m"] for s in samples}, {2.})
            ticks = [round(s["sim_time_s"] * 90) for s in samples]
            self.assertEqual({b - a for a, b in zip(ticks, ticks[1:])}, {4, 5})


if __name__ == "__main__":
    unittest.main()

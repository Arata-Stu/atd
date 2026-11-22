"""Headless, dependency-free checks for the generated Gazebo inputs."""
from datetime import timedelta
from hashlib import sha256
import json
from math import dist, isclose, pi
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from monza_motion import MonzaMotion, interpolate, travel_fraction
from sensor_profile import CAMERA_RATES, IMU_MODES, RESOLUTIONS, imu_streams
import world


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


class OdaibaWorldTest(unittest.TestCase):
    def test_geometry_and_sensors(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "scene"
            scene = world.build(output, height=.135, pitch_deg=4)
            self.assertNotIn("trace_interval_s", scene["trajectory"])
            sdf = ET.parse(output / "world.sdf")
            rig = sdf.find(".//model[@name='d455_rig']")
            left, right = rig.findall("./link/sensor")
            self.assertEqual([s.findtext("frame_id") for s in (left, right)],
                             ["camera_infra1_optical_frame", "camera_infra2_optical_frame"])
            self.assertEqual(left.findtext("topic"), "/odaiba/left/image")
            self.assertEqual(right.findtext("camera/camera_info_topic"), "/odaiba/right/camera_info")
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
            self.assertAlmostEqual(poses[-1][0], 79.66, places=1)
            self.assertEqual(scene["trajectory"]["end_s_m"], poses[-1][0])
            self.assertTrue(all(a[0] < b[0] for a, b in zip(poses, poses[1:])))

    def test_stereo_resolutions_and_imu_clock(self):
        for resolution, (width, height) in RESOLUTIONS.items():
            with self.subTest(resolution=resolution), tempfile.TemporaryDirectory() as tmp, patch.object(world, "odaiba_track"):
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
        for options in ({"resolution": "1280x800"}, {"imu_hz": -1}, {"camera_hz": 45}, {"camera_hz": 60.1}):
            with self.assertRaises(ValueError):
                world.build(Path("/unused"), **options)

    def test_stereo_period_is_a_whole_number_of_physics_ticks(self):
        for camera_hz in CAMERA_RATES:
            for imu_hz in IMU_MODES.values():
                with self.subTest(camera_hz=camera_hz, imu_hz=imu_hz), tempfile.TemporaryDirectory() as tmp:
                    with patch.object(world, "odaiba_track"):
                        scene = world.build(tmp, camera_hz=camera_hz, imu_hz=imu_hz)
                    sdf = ET.parse(Path(tmp) / "world.sdf")
                    self.assertEqual([s.text for s in sdf.findall(".//sensor/update_rate")], [str(camera_hz)] * 2)
                    self.assertEqual(scene["camera"]["rate_hz"], camera_hz)
                    step = float(sdf.findtext(".//physics/max_step_size"))
                    self.assertEqual(scene["trajectory"]["physics_step_s"], step)
                    ticks = 1 / (camera_hz * step)
                    self.assertAlmostEqual(ticks, round(ticks), delta=1e-5)
                    self.assertEqual(scene["imu"]["rate_hz"], imu_hz)
                    if imu_hz:
                        self.assertAlmostEqual(1 / step, 900, places=3)

    def test_asphalt_mesh_and_pinned_geometry(self):
        with tempfile.TemporaryDirectory() as tmp:
            scene = world.build(tmp)
            manifest = scene["geography"]
            for filename, checksum in (("file", "sha256"), ("scenery_file", "scenery_sha256")):
                self.assertEqual(manifest[checksum], sha256((world.HERE / manifest[filename]).read_bytes()).hexdigest())
            sdf = ET.parse(Path(tmp) / "world.sdf")
            self.assertIsNotNone(sdf.find(".//light[@type='directional']"))
            faces = ground_faces(sdf.find(".//visual[@name='floor']"))
        for s, x, y, yaw in scene["trajectory"]["samples_s_x_y_yaw"][::10]:
            self.assertTrue(on_surface(x, y, faces), f"走行軌跡が舗装から外れた位置 {s:.2f} m")

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
            scene = world.build(tmp, circuit="odaiba", speed_mps=3.5, laps=3, wait_for_capture=True,
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

    def test_odaiba_two_lap_bag_scene(self):
        with tempfile.TemporaryDirectory() as tmp:
            scene = world.build(tmp, circuit="odaiba", speed_mps=3, laps=2, resolution="1280x720",
                                camera_hz=30, record_ground_truth=True, wait_for_capture=True)
            self.assertEqual(scene["geography"]["horizontal_scale"], .2)
            self.assertNotIn("origin_lon_lat_deg", scene["geography"])
            route = scene["trajectory"]
            self.assertAlmostEqual(route["end_s_m"], 80, delta=.5)
            self.assertEqual(route["total_travel_seconds"], 2 * route["end_s_m"] / 3)
            self.assertTrue(route["wait_for_capture"])
            self.assertLessEqual(route["trace_interval_s"], .05)
            self.assertEqual(route["samples_s_x_y_yaw"][0][1:], route["samples_s_x_y_yaw"][-1][1:])
            sdf = ET.parse(Path(tmp) / "world.sdf")
            for name in ("odaiba_pit_columns", "odaiba_terrace_deck", "odaiba_grid_paint"):
                self.assertIsNotNone(sdf.find(f".//visual[@name='{name}']"))

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

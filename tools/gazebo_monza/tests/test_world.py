"""Headless, dependency-free checks for the generated Gazebo inputs."""
from datetime import timedelta
import json
from math import isclose, pi
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from monza_motion import MonzaMotion, interpolate, travel_fraction
import world


class MonzaWorldTest(unittest.TestCase):
    def test_monza_geometry_and_sensors(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            materials = root / "materials"
            materials.mkdir()
            for name in ("floor.jpg", "wall.jpg"):
                (materials / name).write_bytes(b"fixture")
            output = root / "scene"
            with patch.object(world, "HERE", root):
                scene = world.build(output, height=.135, pitch_deg=4)
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
            self.assertIn("vn 0 0 1", (output / "floor.obj").read_text())
            profile = json.loads((world.MAP / "lines/raceline.json").read_text())["profile"]
            poses = scene["trajectory"]["samples_s_x_y_yaw"]
            self.assertEqual(poses[0], profile[0][:4])
            self.assertEqual(poses[-1][0], 12.5)
            self.assertTrue(all(a[0] < b[0] for a, b in zip(poses, poses[1:])))
            headings = [p[3] for p in poses if p[0] > 8]
            self.assertGreater(max(headings)-min(headings), .30)  # Right-left chicane.

    def test_endpoints_and_benchmark_return_trip(self):
        self.assertEqual([travel_fraction(t, 10, False) for t in (0, 5, 10, 25)], [0, .5, 1, 1])
        self.assertEqual([travel_fraction(t, 10, True) for t in (0, 5, 10, 15, 20)], [0, .5, 1, .5, 0])
        x, y, yaw = interpolate([[0, 0, 0, pi-.1], [2, 1, 3, -pi+.1]], [0, 2], 1)
        self.assertEqual((x, y), (.5, 1.5))
        self.assertTrue(isclose(yaw, pi))

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


if __name__ == "__main__":
    unittest.main()

"""Depth 投影・3D 通過判定と既存の車両指令の接続を ROS なしで検証する。"""
from array import array
import math
from pathlib import Path
import sys
import tempfile
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from closed_loop_motion import ClosedLoopMotion, CommandPlant, STEERING_LIMIT, WHEELBASE_M
from depth_ftg import COMMAND_AGE_NS, DepthPublisher, DepthProjection, GapPlanner, optical_rotation
from atd_ftg.ros_input import optical_axes
from ftg_visualization import depth_heatmap
import world
from run import ftg_command


class DepthFTGTests(unittest.TestCase):
    def setUp(self):
        self.planner = GapPlanner(WHEELBASE_M, STEERING_LIMIT, .50, .50, 1.8)
        self.camera = DepthProjection.simulated(424, 240, world.HFOV, world.FORWARD_M, .12, 0.)
        self.clear = array("f", [math.inf]) * (424 * 240)

    def test_depth_heatmap_uses_fixed_scale_and_marks_invalid_pixels(self):
        depth = array("f", [.1, 1 / 2.7, 2.5, math.inf, math.nan, -math.inf, 0.])
        rgb = depth_heatmap(depth.tobytes())
        self.assertEqual(rgb, bytes([122, 4, 2, 161, 252, 61, 48, 18, 59, 48, 18, 59]
                                    + [70, 74, 80] * 3))

    def test_floor_passes_low_block_stops_and_high_crossbar_passes(self):
        def projection(image):
            return lambda x, y, z: self.camera.visible(image, x, y, z)

        rows = range(self.camera.height)
        for height, expected_speed in ((0., .49), (.045, 0.), (.30, .49)):
            with self.subTest(height=height):
                image = self.clear[:]
                # Use a transverse slab at the requested height above the floor.
                d = .17 if height == .045 else .5
                sampled_rows = 0
                for v in rows:
                    z = self.camera.origin[2] + d * (self.camera.cy - v) / self.camera.fy
                    if height <= z < height + (.008 if height == 0. else .028):
                        sampled_rows += 1
                        row = v * self.camera.width
                        image[row:row + self.camera.width] = array("f", [d]) * self.camera.width
                self.assertGreater(sampled_rows, 0)
                points = self.camera.obstacles(image)
                _, speed, _ = self.planner.plan(points, projection(image), 0., 0.)
                if expected_speed == 0.:
                    self.assertEqual(speed, 0.)
                    self.assertGreater(len(points), 0)
                else:
                    self.assertGreaterEqual(speed, expected_speed)
        bar = [(.50, iy * .02, .30) for iy in range(-25, 26)]
        self.assertGreater(self.planner.plan(bar, projection(self.clear), 0., 0.)[1], .49)

    def test_turn_uses_clear_side_and_unknown_stops(self):
        visible = lambda x, y, z: self.camera.visible(self.clear, x, y, z)
        points = [(.78, iy * .025, .075) for iy in range(-18, 3)]
        angle, speed, state = self.planner.plan(points, visible, .15, 0.)
        self.assertGreater(angle, 0.)
        self.assertGreater(speed, 0.)
        self.assertEqual(state["state"], "tracking_gap")
        self.assertEqual(self.planner.plan([], lambda *args: False, .15, 0.)[1], 0.)

    def test_native_frame_is_consumed_once_and_stale_frame_is_dropped(self):
        source = DepthPublisher.__new__(DepthPublisher)
        source.projection = self.camera
        source.lock, source.float32 = Lock(), 1
        source.pending, source.last_stamp = None, 0
        frame = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nsec=0)),
                                pixel_format_type=1, width=424, height=240, step=424 * 4,
                                data=self.clear.tobytes())
        source.receive(frame)
        self.assertEqual(source.take(10**9), (10**9, frame.data))
        self.assertIsNone(source.take(10**9 + 1))
        frame.header.stamp.sec = 2
        source.receive(frame)
        self.assertIsNone(source.take(2 * 10**9 + COMMAND_AGE_NS + 1))

    def test_ros_target_drives_plant_and_uses_matching_debug_frame(self):
        plugin = ClosedLoopMotion()
        plugin.plant = CommandPlant((0., 0., 0.), .30)
        plugin.now_ns, plugin.depth_age_ns = 10**9, COMMAND_AGE_NS
        plugin.ftg_command_topic = "/depth_ftg/target_twist"
        plugin.node = Mock()
        plugin.node.count_publishers.return_value = 1
        plugin.ftg_visualization = Mock()
        msg = SimpleNamespace(header=SimpleNamespace(frame_id="base_link", stamp=SimpleNamespace(sec=1, nanosec=0)),
                              twist=SimpleNamespace(linear=SimpleNamespace(x=.4), angular=SimpleNamespace(z=.25)))
        with tempfile.TemporaryDirectory() as temp:
            plugin.active_file = Path(temp) / "active"
            plugin.ftg_frames = []
            plugin.on_ftg_command(msg)
            self.assertFalse(plugin.active_file.exists())
            plugin.ftg_frames = [(10**9, b"depth")]
            plugin.on_ftg_command(msg)
            self.assertTrue(plugin.active_file.exists())
        plugin.plant.advance(10**9, .12)
        self.assertGreater(plugin.plant.vehicle.x, 0.)
        self.assertGreater(plugin.plant.vehicle.steering, 0.)
        plugin.on_ftg_status(SimpleNamespace(data='{"state":"tracking_gap","stamp_ns":1000000000,'
                                            '"clearance_m":1,"steering_rad":0.16,"target_speed_mps":0.4,'
                                            '"obstacles":[[0.5,0.2,0.1]]}'))
        self.assertNotIn("obstacles", plugin.control_state)
        self.assertEqual(plugin.ftg_visualization.observe.call_args.args[:3],
                         (10**9, b"depth", [[.5, .2, .1]]))

    def test_simulated_optical_tf_matches_native_projection(self):
        pitch = .19
        camera = DepthProjection.simulated(424, 240, world.HFOV, world.FORWARD_M, .12, pitch)
        for expected, observed in zip(camera.axes, optical_axes(optical_rotation(pitch))):
            for a, b in zip(expected, observed):
                self.assertAlmostEqual(a, b)

    def test_world_adds_native_depth_in_ftg_mode(self):
        with tempfile.TemporaryDirectory() as temp:
            scene = world.build(temp, command_driven=True, resolution="424x240", camera_hz=30, ftg_speed=.5, ftg_visualize=True)
            self.assertTrue(scene["depth"]["visualize"])
            command, env = ftg_command(scene, "use_sim_time:=true", True)
            self.assertTrue(Path(command[1]).is_file())
            self.assertIn("use_sim_time:=true", command)
            self.assertIn("allow_far_clip:=true", command)
            self.assertIn("publish_obstacles:=true", command)
            self.assertIn("atd_ftg", env["PYTHONPATH"])
            sdf = ET.parse(Path(temp) / "world.sdf")
            camera, = sdf.findall(".//model[@name='d455_rig']/link/sensor")
            self.assertEqual(camera.attrib["type"], "depth_camera")
            self.assertEqual(camera.findtext("topic"), scene["depth"]["topic"])
            self.assertEqual(camera.findtext("camera/image/width"), "424")
            self.assertEqual(camera.findtext("update_rate"), "30")
            self.assertIsNone(sdf.find(".//plugin/module_name[.='sensor_ingress']"))


if __name__ == "__main__":
    unittest.main()

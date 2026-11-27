"""Depth 投影・3D 通過判定と既存の車両指令の接続を ROS なしで検証する。"""
from array import array
import math
from pathlib import Path
import sys
import tempfile
from threading import Lock
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from closed_loop_motion import CommandPlant, JOINTS, STEERING_LIMIT, WHEELBASE_M
from depth_ftg import COMMAND_AGE_NS, DepthController, DepthProjection, GapPlanner
import world


class DepthFTGTests(unittest.TestCase):
    def setUp(self):
        self.planner = GapPlanner(WHEELBASE_M, STEERING_LIMIT, .50, .50, 1.8)
        self.camera = DepthProjection(424, 240, world.HFOV, world.FORWARD_M, .12, 0.)
        self.clear = array("f", [math.inf]) * (424 * 240)

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
                    z = self.camera.z + d * (self.camera.cy - v) / self.camera.focal
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

    def test_native_frame_drives_existing_plant_once_and_stale_frame_does_not_drive(self):
        controller = DepthController.__new__(DepthController)
        controller.projection, controller.planner = self.camera, self.planner
        controller.lock, controller.float32 = Lock(), 1
        controller.pending, controller.last_stamp = None, 0
        frame = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nsec=0)),
                                pixel_format_type=1, width=424, height=240, step=424 * 4,
                                data=self.clear.tobytes())
        controller.receive(frame)
        stamp, angle, speed, report = controller.update(10**9, 0., 0.)
        self.assertEqual(report["state"], "tracking_gap")
        plant = CommandPlant((0., 0., 0.), .30)
        plant.receive(stamp, stamp, JOINTS, (angle, speed / plant.vehicle.full_throttle_speed_mps, 1.))
        plant.advance(stamp, .12)
        self.assertGreater(plant.vehicle.x, 0.)
        self.assertIsNone(controller.update(stamp + 1, plant.vehicle.speed, plant.vehicle.steering))
        frame.header.stamp.sec = 2
        controller.receive(frame)
        self.assertEqual(controller.update(2 * 10**9 + COMMAND_AGE_NS + 1, 0., 0.)[2], 0.)

    def test_world_adds_native_depth_in_ftg_mode(self):
        with tempfile.TemporaryDirectory() as temp:
            scene = world.build(temp, command_driven=True, resolution="424x240", camera_hz=30, ftg_speed=.5)
            sdf = ET.parse(Path(temp) / "world.sdf")
            camera, = sdf.findall(".//model[@name='d455_rig']/link/sensor")
            self.assertEqual(camera.attrib["type"], "depth_camera")
            self.assertEqual(camera.findtext("topic"), scene["depth"]["topic"])
            self.assertEqual(camera.findtext("camera/image/width"), "424")
            self.assertEqual(camera.findtext("update_rate"), "30")
            self.assertIsNone(sdf.find(".//plugin/module_name[.='sensor_ingress']"))


if __name__ == "__main__":
    unittest.main()

"""Gazebo's command input, camera pose and run lifecycle without Gazebo/ROS."""
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from closed_loop_motion import ClosedLoopMotion, CommandPlant, JOINTS
from closed_loop_trial import Trial, read_samples, run_trial
from sensor_profile import REAR_AXLE_HEIGHT_M
import run
import world


class CommandMotionTests(unittest.TestCase):
    def test_no_motion_without_fresh_input_then_turn_and_expire(self):
        plant = CommandPlant((2., -1., math.pi / 2), .3)
        start = 10**9
        for i in range(100):
            plant.advance(i * 10**7, .01)
        self.assertEqual((plant.vehicle.x, plant.vehicle.y, plant.vehicle.speed), (2., -1., 0.))
        for i in range(200):
            now = start + i * 10**7
            plant.receive(now, now, JOINTS, [.2, .35, 1.])
            plant.advance(now, .01)
        self.assertGreater(plant.vehicle.y, -0.3)
        self.assertLess(plant.vehicle.x, 1.8)
        self.assertGreater(plant.vehicle.yaw, math.pi / 2)
        stamp = plant.command_stamp_ns
        for i in range(500):
            now = stamp + i * 10**7
            plant.advance(now, .01)
        self.assertEqual((plant.state, plant.reason), ("waiting_observation", "command_timeout"))
        self.assertLess(plant.vehicle.speed, .001)
        plant.receive(now, stamp, JOINTS, [0., .5, 1.])
        self.assertEqual(plant.vehicle.throttle_setpoint, 0.)
        plant.receive(now, now, JOINTS, [0., .5, 1.])
        self.assertEqual((plant.state, plant.reason), ("driving", ""))
        self.assertEqual(plant.vehicle.throttle_setpoint, .5)
        self.assertEqual(plant.first_active_ns, start)

    def test_disable_during_update_gap_latches_stop(self):
        plant = CommandPlant((0., 0., 0.), .3)
        stamp = 10**9
        plant.receive(stamp, stamp, JOINTS, [.1, .3, 1.])
        now = stamp + 500_000_000
        plant.advance(now, .01)
        self.assertEqual(plant.state, "waiting_observation")
        plant.receive(now, now, JOINTS, [0., 0., 0.])
        plant.receive(now + 1, now + 1, JOINTS, [0., .3, 1.])
        self.assertEqual((plant.state, plant.reason), ("stopped", "controller_disabled"))
        self.assertEqual(plant.vehicle.throttle_setpoint, 0.)

    def test_disabled_future_malformed_and_nonmonotonic(self):
        now = 30_000_000_000
        invalid = ((now, JOINTS, [0., .5, 0.]),
                   (now + 1, JOINTS, [0., .5, 1.]),
                   (now, reversed(JOINTS), [0., .5, 1.]),
                   (now, JOINTS, [math.nan, .5, 1.]),
                   (now, JOINTS, [0., -.1, 1.]))
        for stamp, names, values in invalid:
            plant = CommandPlant((0., 0., 0.), .3)
            plant.receive(now, now, JOINTS, [0., .3, 1.])
            plant.receive(now, now - 10, JOINTS, [.1, .9, 1.])
            self.assertEqual(plant.vehicle.throttle_setpoint, .3)
            plant.receive(now, stamp, names, values)
            self.assertEqual(plant.state, "stopped")
            self.assertEqual(plant.vehicle.throttle_setpoint, 0.)

    def test_competing_publishers_before_start_prevent_motion_and_report(self):
        plugin = ClosedLoopMotion()
        plugin.plant = CommandPlant((0., 0., 0.), .3)
        plugin.node = SimpleNamespace(count_publishers=lambda topic: 2)
        plugin.actuator_topic = "/atd/gazebo/actuators"
        plugin.on_command(None)
        plugin.plant.receive(42, 42, JOINTS, [0., .5, 1.])
        self.assertEqual(plugin.plant.vehicle.throttle_setpoint, 0.)
        row = dict(sim_time_s=1., first_active_ns=None, actuator_state=plugin.plant.state,
                   actuator_reason=plugin.plant.reason)
        trial = Trial(SimpleNamespace(report=lambda *args: args))
        self.assertEqual(trial.sample(row), ("control_stopped", 1., "multiple_actuator_publishers"))

    def test_independent_plugin_and_same_stereo_pose(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(world, "odaiba_track"):
            base = Path(tmp)
            normal = world.build(base/"bag")
            scene = world.build(base/"drive", command_driven=True, laps=2)
            geometry = json.loads((base/"drive/scene.json").read_text())["course_evaluation"]
            self.assertEqual(geometry["frame"], "gazebo_world")
            self.assertEqual(len(geometry["left"]), len(scene["trajectory"]["samples_s_x_y_yaw"]) - 1)
            self.assertEqual(len(geometry["left"]), len(geometry["right"]))
            self.assertEqual(geometry["footprint"]["vehicle_width"], .19)
            self.assertEqual(normal["camera"], scene["camera"])
            self.assertIsNone(scene["trajectory"]["speed_mps"])
            self.assertIsNone(scene["trajectory"]["travel_seconds"])
            self.assertEqual(list(scene["actuation"]["spawn_x_y_yaw"]), list(normal["trajectory"]["samples_s_x_y_yaw"][0][1:]))
            sdf = ET.parse(base/"drive/world.sdf")
            rig = sdf.find(".//model[@name='d455_rig']")
            self.assertEqual(rig.findtext("plugin/module_name"), "closed_loop_motion")
            self.assertEqual(len(rig.findall("link/sensor")), 2)
            with self.assertRaises(ValueError):
                world.build(base/"invalid", command_driven=True, speed_mps=1.)

    def test_live_gt_uses_measured_link_pose_and_trace_sim_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            plugin = ClosedLoopMotion()
            plugin.plant = CommandPlant((0., 0., 0.), .3)
            plugin.imu = plugin.control_state = None
            plugin.trace = Path(tmp) / "rig_motion.jsonl"
            plugin.next_trace_s, plugin.trace_interval_s = 0., .05
            observed = Mock(**{"x.return_value": 3., "y.return_value": -2., "z.return_value": 0.,
                               "yaw.return_value": math.pi / 2})
            plugin.link = Mock(**{"world_pose.return_value": observed})
            plugin.ground_truth = Mock(**{"get_subscription_count.return_value": 1})
            plugin.gt_pose = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace()),
                                             pose=SimpleNamespace(position=SimpleNamespace(),
                                                                  orientation=SimpleNamespace()))
            info = Mock(paused=False, **{"sim_time.total_seconds.return_value": .033333})
            plugin.plant.receive(30_000_000, 20_000_000, JOINTS, [0., .2, 1.])
            plugin.plant.receive(32_000_000, 20_000_000, JOINTS, [0., .3, 1.])
            plugin.post_update(info, None)
            plugin.ground_truth.publish.assert_called_once_with(plugin.gt_pose)
            pose, stamp = plugin.gt_pose.pose, plugin.gt_pose.header.stamp
            sample = json.loads(plugin.trace.read_text())
            self.assertEqual((sample["command_stamp_ns"], sample["command_applied_ns"]), (20_000_000, 30_000_000))
            self.assertEqual(stamp.sec * 10**9 + stamp.nanosec, round(sample["sim_time_s"] * 1e9))
            self.assertEqual((pose.position.x, pose.position.y, pose.position.z), (3., -2., REAR_AXLE_HEIGHT_M))
            self.assertAlmostEqual(2 * math.atan2(pose.orientation.z, pose.orientation.w), sample["yaw_rad"])
            info.sim_time.total_seconds.return_value = .04
            plugin.post_update(info, None)
            plugin.ground_truth.publish.assert_called_once()

    def test_partial_trace_line_waits_and_tracked_stop_exports_reason(self):
        stream = io.StringIO('{"a":1}\n{"b":')
        self.assertEqual(list(read_samples(stream)), [{"a": 1}])
        self.assertEqual(stream.read(), '{"b":')
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            (out/"scene").mkdir()
            trace = out/"scene/rig_motion.jsonl"
            rows = [dict(sim_time_s=i+1., first_active_ns=None if i == 0 else 2_000_000_000,
                         x_m=-.01*i, y_m=0., yaw_rad=math.pi, speed_mps=0.,
                         actuator_state=state, actuator_reason="controller_disabled",
                         control_state=dict(state="stopped", reason="lost tracking"))
                    for i, state in enumerate(("waiting", "driving", "stopped"))]
            trace.write_text("".join(json.dumps(r)+"\n" for r in rows))
            self.assertEqual(run_trial({}, out, dict(course="odaiba", trajectory=dict(laps=2)),
                                       {"imu_hz": 0}, lambda *a, **k: None, run.wait_for,
                                       run.check_children), 1)
            result = json.loads((out / "closed_loop.result.json").read_text())
            self.assertEqual(result["status"], "control_stopped")
            self.assertEqual(result["started_sim_s"], 2.)
            self.assertEqual(result["laps_completed"], 0)
            self.assertEqual(result["reason"], "lost tracking")

    def test_runner_selects_command_driven_mode_without_route_timing(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(sys, "argv", [
                "run.py", "--mode", "closed-loop", "--output", tmp]), \
                patch.object(run, "run", side_effect=lambda a, r: (r.update(closed_loop={"status": "completed"}) or 0)) as child:
            self.assertEqual(run.main(), 0)
            args = child.call_args.args[0]
            self.assertEqual((args.laps, args.duration, args.speed), (2, None, None))

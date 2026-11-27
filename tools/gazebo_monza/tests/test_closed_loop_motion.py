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
from closed_loop_eval import CourseEvaluation
from closed_loop_trial import Trial, read_samples, run_trial
from circuits import load_lap
from sensor_profile import REAR_AXLE_HEIGHT_M
import run
import world


class CommandMotionTests(unittest.TestCase):
    def test_hold_freezes_pose_and_requires_new_drive_observation(self):
        plant = CommandPlant((2., -1., .3), .3)
        now = 10**9
        plant.receive(now, now, JOINTS, (.15, .5, 1.))
        plant.advance(now, .1)
        v = plant.vehicle
        pose = (v.x, v.y, v.yaw)
        self.assertGreater(v.speed, 0.)
        self.assertGreater(v.steering, 0.)
        hold = now + 10**7
        received = hold + 10**7
        plant.receive(received, hold, JOINTS, (0., 0., 2.))
        # An in-flight plan even newer than the hold send time may precede its receipt.
        plant.receive(received + 1, received, JOINTS, (.15, .5, 1.))
        plant.advance(received + 10**9, 1.)
        self.assertEqual((v.x, v.y, v.yaw), pose)
        self.assertEqual((v.speed, v.steering, v.steering_setpoint, v.throttle_setpoint), (0., 0., 0., 0.))
        self.assertEqual(plant.state, "held")
        resume = received + 2 * 10**9
        plant.receive(resume, resume, JOINTS, (.1, .4, 1.))
        plant.receive(resume + 1, resume - 1, JOINTS, (0., 0., 2.))
        plant.advance(resume + 1, .1)
        self.assertEqual(plant.state, "driving")
        self.assertGreater(v.speed, 0.)
        self.assertEqual(plant.first_active_ns, now)

    def test_hold_before_first_drive_filters_queued_disable_and_faults_stay_terminal(self):
        for fault in (False, True):
            with self.subTest(fault=fault):
                plant = CommandPlant((0., 0., 0.), .3)
                for stamp in (100, 100, 110):
                    plant.receive(stamp, stamp, JOINTS, (0., 0., 2.))
                plant.receive(111, 99, JOINTS, (0., 0., 0.))
                self.assertEqual(plant.state, "held")
                self.assertIsNone(plant.first_active_ns)
                if fault:
                    plant.receive(112, 112, JOINTS, (0., 0., 0.))
                    plant.receive(113, 113, JOINTS, (0., 0., 2.))
                plant.receive(114, 114, JOINTS, (0., .4, 1.))
                self.assertEqual(plant.state, "stopped" if fault else "driving")
                self.assertEqual(plant.first_active_ns, None if fault else 114)

    def test_explicit_hold_exempts_stall_watchdogs_but_preserves_course_scoring(self):
        boundaries, route, _ = load_lap("odaiba")
        _, x, y, yaw = route[0]
        def sample(trial, t, actuator, controller, y_m=y):
            return trial.sample(dict(sim_time_s=t, first_active_ns=10**9, x_m=x, y_m=y_m,
                                     yaw_rad=yaw, actuator_state=actuator, control_state={"state": controller}))
        for boundary in (False, True):
            with self.subTest(boundary=boundary):
                evaluator = CourseEvaluation(boundaries, route, 1)
                trial = Trial(evaluator)
                self.assertIsNone(sample(trial, 1., "driving", "driving"))
                self.assertIsNone(sample(trial, 2., "held", "paused"))
                self.assertIsNone(sample(trial, 1002., "held", "paused"))
                self.assertEqual((evaluator.start, evaluator.previous[0], evaluator.distance), (1., 1002., 0.))
                if boundary:
                    self.assertEqual(sample(trial, 1003., "held", "paused", y + 4.)["status"], "off_course")
                else:
                    self.assertIsNone(sample(trial, 1003., "held", "waiting_observation"))
                    self.assertEqual(sample(trial, 1033., "held", "waiting_observation")["status"], "no_progress")

    def test_startup_pause_follows_operator_request_until_resume(self):
        with tempfile.TemporaryDirectory() as tmp:
            plugin = ClosedLoopMotion()
            plugin.control_state = None
            plugin.plant = CommandPlant((0., 0., 0.), .3)
            plugin.prestart_pause_file = Path(tmp) / "closed_loop.paused"
            for state, marked in (("waiting", False), ("paused", True), ("paused", True), ("waiting_observation", False)):
                plugin.on_control_state(SimpleNamespace(data=json.dumps({"state": state})))
                self.assertEqual(plugin.prestart_pause_file.exists(), marked)
            plugin.plant.receive(100, 100, JOINTS, (0., .2, 1.))
            plugin.on_control_state(SimpleNamespace(data=json.dumps({"state": "paused"})))
            self.assertFalse(plugin.prestart_pause_file.exists())

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
            scene = world.build(base/"drive", command_driven=True, laps=2, full_throttle_speed_mps=10.)
            saved = json.loads((base/"drive/scene.json").read_text())
            self.assertEqual(saved["actuation"]["full_throttle_speed_mps"], 10.)
            geometry = saved["course_evaluation"]
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

    def test_full_throttle_speed_is_reachable_with_fresh_gazebo_commands(self):
        plant = CommandPlant((0., 0., 0.), .3, full_throttle_speed_mps=10.)
        for tick in range(1, 2001):
            now = tick * 10**7
            plant.receive(now, now, JOINTS, (0., 1., 1.))
            plant.advance(now, .01)
        self.assertAlmostEqual(plant.vehicle.speed, 10., places=6)

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
            plugin.model_speed = Mock(**{"get_subscription_count.return_value": 1})
            plugin.speed_message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace()),
                                                   twist=SimpleNamespace(linear=SimpleNamespace()))
            plugin.hold_status = Mock(**{"get_subscription_count.return_value": 1})
            plugin.hold_message = SimpleNamespace(data="")
            plugin.gt_pose = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace()),
                                             pose=SimpleNamespace(position=SimpleNamespace(),
                                                                  orientation=SimpleNamespace()))
            info = Mock(paused=False, **{"sim_time.total_seconds.return_value": .033333})
            plugin.plant.receive(30_000_000, 20_000_000, JOINTS, [0., .2, 1.])
            plugin.plant.receive(32_000_000, 20_000_000, JOINTS, [0., .3, 1.])
            plugin.plant.vehicle.speed = .123
            plugin.post_update(info, None)
            plugin.model_speed.publish.assert_called_once_with(plugin.speed_message)
            self.assertEqual(plugin.speed_message.twist.linear.x, .123)
            self.assertEqual(plugin.speed_message.header.stamp.nanosec, 33_333_000)
            plugin.ground_truth.publish.assert_called_once_with(plugin.gt_pose)
            pose, stamp = plugin.gt_pose.pose, plugin.gt_pose.header.stamp
            sample = json.loads(plugin.trace.read_text())
            self.assertEqual((sample["command_stamp_ns"], sample["command_applied_ns"]), (20_000_000, 30_000_000))
            self.assertEqual(stamp.sec * 10**9 + stamp.nanosec, round(sample["sim_time_s"] * 1e9))
            self.assertEqual(json.loads(plugin.hold_message.data),
                             dict(held=False, stamp_ns=33_333_000, speed_mps=.123))
            self.assertEqual((pose.position.x, pose.position.y, pose.position.z), (3., -2., REAR_AXLE_HEIGHT_M))
            self.assertAlmostEqual(2 * math.atan2(pose.orientation.z, pose.orientation.w), sample["yaw_rad"])
            # The closed-loop IMU needs every measured pose between the 20 Hz GT samples.
            plugin.imu = Mock()
            info.sim_time.total_seconds.return_value = .04
            plugin.post_update(info, None)
            plugin.imu.update.assert_called_once_with(40_000_000, observed)
            plugin.ground_truth.publish.assert_called_once()
            plugin.plant.receive(45_000_000, 45_000_000, JOINTS, (0., 0., 2.))
            info.sim_time.total_seconds.return_value = .053
            plugin.post_update(info, None)
            self.assertEqual(json.loads(plugin.hold_message.data), dict(held=True, stamp_ns=53_000_000, speed_mps=0.))
            self.assertEqual(plugin.hold_status.publish.call_count, 2)

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

    def test_endurance_saves_completed_laps_and_partial_distance_on_manual_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            def start(children, stereo, course, check_stop, imu_hz, ingress_ready):
                scene = json.loads((out / "scene/scene.json").read_text())
                self.assertEqual(scene["trajectory"]["laps"], 0)
                self.assertIsNone(scene["trajectory"]["total_travel_seconds"])
                self.assertEqual(imu_hz, 400)
                self.assertEqual(scene["imu"]["streams"][run.IMU_TOPIC], {"frame_id": run.GYRO_FRAME, "rate_hz": 400})
                bridge = json.loads((out / "bridge.yaml").read_text())
                self.assertEqual([topic["ros_topic_name"] for topic in bridge], ["/clock"])
                self.assertEqual(ingress_ready, out.resolve() / "scene/sensor_ingress.ready")
                self.assertIn("tf_" + run.GYRO_FRAME, children)
                samples = scene["trajectory"]["samples_s_x_y_yaw"][:-1:2]
                with (out / "scene/rig_motion.jsonl").open("w") as trace:
                    for index, (_, x, y, yaw) in enumerate(samples * 3 + samples[:12]):
                        row = dict(sim_time_s=1. + index * .1, x_m=x, y_m=y, yaw_rad=yaw,
                                   actuator_state="driving", first_active_ns=10**9)
                        trace.write(json.dumps(row) + "\n")
            def stop_after_trace(_):
                (out / "closed_loop.cancel").write_text("ユーザーが走行を中断しました\n")

            argv = ["run.py", "--mode", "closed-loop", "--output", tmp, "--laps", "0", "--imu", "400"]
            with patch.object(sys, "argv", argv), patch.object(world, "odaiba_track"), \
                    patch.object(run.shutil, "which", return_value=None), patch.object(run.subprocess, "Popen"), \
                    patch.object(run, "check_children"), patch.object(run, "stop_group"), \
                    patch.object(run, "start_stream", side_effect=start), patch.object(run.time, "sleep", stop_after_trace):
                self.assertEqual(run.main(), 0)
            result = json.loads((out / "closed_loop.result.json").read_text())
            self.assertEqual(json.loads((out / "run.json").read_text())["closed_loop"], result)
            self.assertEqual(result["status"], "stopped")
            self.assertEqual((result["laps_completed"], result["laps_requested"]), (3, 0))
            self.assertEqual(len(result["lap_times_s"]), 3)
            self.assertGreater(result["finish_sim_s"] - result["started_sim_s"], sum(result["lap_times_s"]))

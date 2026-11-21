"""One-lap acquisition, recorder shutdown and publication without a ROS installation."""
import contextlib
import io
from pathlib import Path
import copy
import signal
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import lap_bag
import run


class LapBagTests(unittest.TestCase):
    def test_bag_cli_selects_a_full_lap_and_a_new_destination(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            final = root / "rosbags/gazebo-spa-001"
            argv = ["run.py", "--mode", "bag", "--output", str(root / "logs"), "--circuit", "spa"]
            with patch.object(sys, "argv", argv + ["--bag-output", str(final)]), \
                    patch.object(run, "run", return_value=0) as simulate, patch.object(signal, "signal"):
                self.assertEqual(run.main(), 0)
            args = simulate.call_args.args[0]
            self.assertEqual((args.route, args.duration, args.bag_output),
                             ("lap", run.world.default_travel_seconds("lap", "spa"), final))
            for invalid in ([], ["--route", "chicane", "--bag-output", str(final)]):
                with patch.object(sys, "argv", argv + invalid), contextlib.redirect_stderr(io.StringIO()), \
                        self.assertRaises(SystemExit):
                    run.main()

    def test_native_command_uses_sim_time_and_latched_static_transforms(self):
        cmd = lap_bag.command(Path("/some/new/bag"))
        self.assertEqual(cmd[:3], ["ros2", "bag", "record"])
        self.assertEqual(cmd[cmd.index("--storage") + 1], "mcap")
        self.assertIn("--use-sim-time", cmd)
        self.assertEqual(set(cmd[cmd.index("--topics") + 1:]),
                         {*lap_bag.TOPICS.values(), *lap_bag.INFO_TOPICS.values(), "/tf_static"})
        with_imu = lap_bag.command(Path("/some/new/bag"), imu_hz=400)
        self.assertEqual(set(with_imu[with_imu.index("--topics") + 1:]),
                         {*lap_bag.TOPIC_NAMES, *lap_bag.imu_streams(400)})
        qos = Path(cmd[cmd.index("--qos-profile-overrides-path") + 1]).read_text()
        self.assertIn("durability: transient_local", qos)
        self.assertIn("reliability: reliable", qos)

    def test_start_gate_requires_both_calibrated_eyes_and_all_latched_transforms(self):
        node = MagicMock()
        ros = MagicMock()
        ros.create_node.return_value.__enter__.return_value = node
        callbacks = {}
        node.create_subscription.side_effect = lambda typ, topic, fn, qos: callbacks.update({topic: fn})
        edges = {("base_link", "camera_link"), ("camera_link", "camera_infra1_optical_frame"),
                 ("camera_link", "camera_infra2_optical_frame")}
        graph = ["another_recorder"]
        node.get_subscriptions_info_by_topic.side_effect = lambda t: [SimpleNamespace(node_name=graph[0])]
        messages = []
        for side in lap_bag.TOPICS:
            hdr = SimpleNamespace(frame_id=lap_bag.FRAMES[side], stamp=SimpleNamespace(sec=7, nanosec=42))
            image = SimpleNamespace(header=hdr, encoding="mono8", width=640, height=480, step=640,
                                    data=bytes(640 * 480))
            p = [100., 0., 320., -9.5 if side == "right" else 0., 0., 100., 240., 0., 0., 0., 1., 0.]
            info = SimpleNamespace(header=hdr, k=[100., 0., 320., 0., 100., 240., 0., 0., 1.],
                                   p=p, width=640, height=480)
            messages.extend([(lap_bag.TOPICS[side], image), (lap_bag.INFO_TOPICS[side], info)])
        for p, c in sorted(edges):
            tf = SimpleNamespace(header=SimpleNamespace(frame_id=p), child_frame_id=c)
            messages.append(("/tf_static", SimpleNamespace(transforms=[tf])))
        for name, (frame, _) in lap_bag.imu_streams(400).items():
            header = SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=7, nanosec=42))
            vector = SimpleNamespace(x=0., y=0., z=1.)
            messages.append((name, SimpleNamespace(header=header, linear_acceleration=vector,
                                                   angular_velocity=vector, angular_velocity_covariance=[0.] * 9,
                                                   linear_acceleration_covariance=[0.] * 9)))
        start_frames = [(name, msg) for name, msg in messages if name != "/tf_static"]
        for sec in (8, 9):
            for name, msg in start_frames:
                later = copy.deepcopy(msg)
                later.header.stamp.sec = sec
                messages.append((name, later))
        def spin(*args, **kwargs):
            topic, msg = messages.pop(0)
            callbacks[topic](msg)
        ros.spin_once.side_effect = spin
        def wait(check, children, timeout, what, **kwargs):
            if not callbacks:
                self.assertFalse(check())
                graph[0] = lap_bag.RECORDER
                self.assertTrue(check())
                return
            while len(messages) > 1:
                self.assertFalse(check())
            self.assertTrue(check())
        modules = {"rclpy": ros, "rclpy.signals": SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)),
                   "rclpy.qos": SimpleNamespace(DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1),
                                                ReliabilityPolicy=SimpleNamespace(RELIABLE=1),
                                                QoSProfile=SimpleNamespace, qos_profile_sensor_data=object()),
                   "sensor_msgs.msg": SimpleNamespace(CameraInfo=object, Image=object, Imu=object),
                   "tf2_msgs.msg": SimpleNamespace(TFMessage=object)}
        with patch.dict(sys.modules, modules):
            lap_bag.wait_for_sample({}, wait, required_edges=edges, imu_hz=400, preroll_s=2)
        self.assertEqual(messages, [])

    def test_start_and_finish_are_observed_before_graceful_recorder_close(self):
        timing = {"start_sim_ns": 1_000_000_000, "finish_sim_ns": 3_000_000_000}
        for interrupted in (False, True):
            with self.subTest(interrupted=interrupted), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                (root / "rosbags").mkdir()
                final = root / "rosbags/gazebo-monza"
                events = []
                children, report = {}, {"geometry": [848, 480], "imu_hz": 400}
                proc = MagicMock()
                def gate(children, wait_for, **kwargs):
                    self.assertIs(children["recorder"], proc)
                    events.append(("sample", kwargs))
                def drive(*args):
                    events.append("lap")
                    if interrupted:
                        raise KeyboardInterrupt()
                    return timing
                with patch.object(run, "start_stream", side_effect=lambda *a, **kw: events.append("clock")), \
                        patch.object(run, "transforms", return_value=[("base_link", "camera_link", 0)]), \
                        patch.object(subprocess, "Popen", return_value=proc), \
                        patch.object(lap_bag, "wait_for_sample", side_effect=gate), \
                        patch.object(run, "drive_laps", side_effect=drive), \
                        patch.object(lap_bag, "close_recorder", side_effect=lambda p: events.append("close")):
                    if interrupted:
                        with self.assertRaises(KeyboardInterrupt):
                            run.record_laps(children, root, "monza", {"camera": {}}, report, final)
                    else:
                        pending, actual = run.record_laps(children, root, "monza", {"camera": {}}, report, final)
                        self.assertEqual(actual, timing)
                        self.assertEqual(pending, Path(report["partial_bag"]))
                profile = {"geometry": (848, 480), "imu_hz": 400}
                start_gate = {**profile, "required_edges": {("base_link", "camera_link")}, "preroll_s": 2}
                self.assertEqual(events[:3], ["clock", ("sample", start_gate), "lap"])
                self.assertEqual(events[3:], ["close"] if interrupted else
                                 [("sample", {**profile, "min_stamp": 3_250_000_000}), "close"])
                self.assertFalse(children)
                self.assertFalse(final.exists())

    def test_bag_mode_stops_the_renderer_before_reading_back_the_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            final = root / "rosbags/gazebo-spa"
            final.parent.mkdir()
            args = SimpleNamespace(mode="bag", route="lap", circuit="spa", output=root,
                                   bag_output=final, duration=17., speed=None, laps=2,
                                   camera_height=.12, camera_pitch_deg=0., resolution="424x240", imu="off")
            scene = {"camera": {"height_above_floor_m": .12, "midpoint_forward_from_rear_axle_m": .1,
                                "pitch_down_deg": 0., "baseline_m": .095, "optical_frames": lap_bag.FRAMES},
                     "trajectory": {"travel_seconds": 17., "laps": 2, "speed_mps": 1.2,
                                    "end_s_m": 20.4, "total_travel_seconds": 34.}}
            timing = {"start_sim_ns": 10, "finish_sim_ns": 20}
            processes = []
            def start(command, **kwargs):
                proc = SimpleNamespace(command=command, stopped=False)
                processes.append(proc)
                return proc
            def audit(*args):
                self.assertTrue(all(proc.stopped for proc in processes))
            with patch.object(lap_bag, "check_recorder"), patch.object(run.world, "build", return_value=scene) as build, \
                    patch.object(run.shutil, "which", return_value=None), patch.object(subprocess, "Popen", start), \
                    patch.object(run, "stop_group", side_effect=lambda p: setattr(p, "stopped", True)), \
                    patch.object(run, "record_laps", return_value=(root / ".gazebo123", timing)) as record, \
                    patch.object(lap_bag, "audit_and_publish", side_effect=audit) as publish:
                report = {"mode": "bag"}
                self.assertEqual(run.run(args, report), 0)
            self.assertEqual(len(processes), 5)  # Gazebo, bridge and three static TF publishers.
            self.assertTrue(all("use_sim_time:=true" in p.command for p in processes[1:]))
            self.assertEqual(build.call_args.kwargs["travel_seconds"], 17.)
            self.assertEqual(build.call_args.kwargs["laps"], 2)
            self.assertEqual(report["geometry"], [424, 240])
            self.assertEqual(build.call_args.kwargs["resolution"], "424x240")
            self.assertEqual(report["total_travel_seconds"], 34.)
            self.assertEqual(report["lap_length_m"], 20.4)
            self.assertFalse(build.call_args.kwargs["repeat"])
            self.assertTrue(build.call_args.kwargs["wait_for_capture"])
            self.assertTrue(build.call_args.kwargs["record_ground_truth"])
            record.assert_called_once()
            publish.assert_called_once_with(root / ".gazebo123", final, timing, report, root / "scene/rig_motion.jsonl")

    def test_partial_metadata_never_appears_in_the_web_library(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rosbags = root / "rosbags"
            rosbags.mkdir()
            final = rosbags / "gazebo-monza"
            pending = lap_bag.staging_path(final)
            pending.mkdir()
            (pending / "metadata.yaml").write_text("partial")
            trace = root / "rig_motion.jsonl"
            report = {"partial_bag": str(pending), "laps_requested": 1, "lap_duration_s": 1e-7}
            timing = {"start_sim_ns": 10, "finish_sim_ns": 123}
            audit = MagicMock(side_effect=ValueError("missing right image"))
            with patch.dict(sys.modules, {"bag_audit": SimpleNamespace(audit_bag=audit)}):
                with self.assertRaisesRegex(ValueError, "missing right"):
                    lap_bag.audit_and_publish(pending, final, timing, report, trace)
                self.assertEqual(list(rosbags.rglob("metadata.yaml")), [])
                self.assertTrue((Path(report["partial_bag"]) / "metadata.yaml").is_file())
                audit.assert_called_once_with(pending, 10, 123, laps=1, lap_duration_s=1e-7,
                                              geometry=(640, 480), imu_hz=0, preroll_s=0)
                audit.side_effect = None
                audit.return_value = {"coverage": "gaps_detected", "complete_stereo_samples": 2, "largest_gap_sim_s": .3,
                                      "file_bytes": {"test.mcap": 1234}}
                with self.assertRaises(FileNotFoundError):
                    lap_bag.audit_and_publish(pending, final, timing, report, trace)
                self.assertFalse(final.exists())
                trace.write_text('{"sim_time_s": 1, "x_m": 2, "y_m": 3, "yaw_rad": 0}\n')
                # A file or bag added during the lap is never replaced.
                final.mkdir()
                with self.assertRaises(FileExistsError):
                    lap_bag.audit_and_publish(pending, final, timing, report, trace)
                final.rmdir()
                lap_bag.audit_and_publish(pending, final, timing, report, trace)
                self.assertEqual((final / "atd_ground_truth/rig_motion.jsonl").read_bytes(), trace.read_bytes())
                self.assertEqual(report, {"bag_audit": audit.return_value, "bag_path": str(final),
                                          "laps_requested": 1, "lap_duration_s": 1e-7})
                self.assertEqual(list(rosbags.rglob("metadata.yaml")), [final / "metadata.yaml"])
                self.assertFalse(pending.parent.exists())

    def test_wait_for_native_writer_flush_and_reject_timeout(self):
        proc = MagicMock(pid=445)
        proc.poll.return_value = None
        proc.wait.return_value = 0
        with patch.object(lap_bag.os, "killpg") as kill:
            lap_bag.close_recorder(proc)
            kill.assert_called_once_with(445, signal.SIGTERM)
            proc.wait.side_effect = [subprocess.TimeoutExpired("ros2", 90), -9]
            with self.assertRaises(TimeoutError):
                lap_bag.close_recorder(proc)
            kill.assert_called_with(445, signal.SIGKILL)
            proc.poll.return_value = 1
            with self.assertRaises(RuntimeError):
                lap_bag.close_recorder(proc)


if __name__ == "__main__":
    unittest.main()

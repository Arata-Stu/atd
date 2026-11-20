"""配信ブリッジと走行判定を、実際のシーン・ROS受信設定と突き合わせる。"""
import json
from math import pi
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import capture
import run
import world


class RunContractTests(unittest.TestCase):
    def test_each_full_lap_world_connects_its_own_rig_to_the_same_ros_endpoints(self):
        for circuit in world.circuits.circuit_ids():
            with self.subTest(circuit=circuit), tempfile.TemporaryDirectory() as tmp:
                out = Path(tmp)
                scene = world.build(out, circuit=circuit, route="lap")
                sdf = ET.parse(out / "world.sdf").find("world")
                self.assertEqual(sdf.attrib["name"], circuit)
                self.assertEqual(scene["course"], circuit)
                asphalt = Path(sdf.findtext(".//visual[@name='floor']//albedo_map"))
                self.assertEqual(asphalt.name, "monza_asphalt.jpg" if circuit == "monza" else "floor.jpg")
                self.assertTrue(asphalt.is_file())
                routes = {t["gz_topic_name"]: t["ros_topic_name"] for t in run.bridge_topics(circuit)}
                self.assertEqual(routes[f"/world/{circuit}/clock"], "/clock")
                for sensor in sdf.findall(".//sensor"):
                    side = sensor.attrib["name"]
                    self.assertEqual(routes[sensor.findtext("topic")], capture.TOPICS[side])
                    self.assertEqual(routes[sensor.findtext("camera/camera_info_topic")], capture.INFO_TOPICS[side])
                for uri in sdf.findall(".//mesh/uri"):
                    self.assertTrue(Path(uri.text).is_file(), uri.text)
                self.assertEqual(scene["trajectory"]["samples_s_x_y_yaw"][0][1:],
                                 scene["trajectory"]["samples_s_x_y_yaw"][-1][1:])
                self.assertAlmostEqual(scene["trajectory"]["end_s_m"] / scene["trajectory"]["travel_seconds"],
                                       2.4, delta=.04)
                if circuit == "silverstone":
                    self.assertTrue((out / "pit_lane.obj").is_file())
                    self.assertTrue((out / "pit_garages.obj").is_file())
                if circuit == "austin":
                    self.assertTrue((out / "tower_deck.obj").is_file())

    def test_route_only_changes_the_default_video_duration(self):
        cases = [("video", "chicane", 10.), ("video", "lap", 240.),
                 ("diagnose", "lap", 20.), ("benchmark", "lap", 60.)]
        for mode, route, duration in cases:
            with self.subTest(mode=mode, route=route), tempfile.TemporaryDirectory() as tmp, \
                    patch.object(sys, "argv", ["run.py", "--mode", mode, "--output", tmp, "--route", route]), \
                    patch.object(run, "run", return_value=0) as simulate:
                self.assertEqual(run.main(), 0)
                args, report = simulate.call_args.args
                self.assertEqual((args.route, report["route"], args.duration), (route, route, duration))

    def test_map_uses_one_complete_lap_and_defaults_to_normal_course_speed(self):
        for circuit in ("monza", "spa"):
            with self.subTest(circuit=circuit), tempfile.TemporaryDirectory() as tmp, \
                    patch.object(sys, "argv", ["run.py", "--output", tmp, "--mode", "map", "--circuit", circuit]), \
                    patch.object(run, "run", return_value=0) as simulate:
                self.assertEqual(run.main(), 0)
                args, report = simulate.call_args.args
                self.assertEqual((args.route, args.duration), ("lap", world.default_travel_seconds("lap", circuit)))
                self.assertEqual(report["status"], "complete")

    def test_mapping_waits_for_tracking_then_measured_finish_then_acknowledgment(self):
        points = [[0, 0, 0, 0], [1, 1, 0, 0], [2, 0, 0, 0]]
        scene = {"trajectory": {"route": "lap", "travel_seconds": 8, "end_s_m": 2,
                                "samples_s_x_y_yaw": points, "wait_for_capture": True}}
        for valid in (True, False):
            with self.subTest(valid=valid), tempfile.TemporaryDirectory() as tmp:
                out = Path(tmp)
                directory = out / "scene"
                directory.mkdir()
                report = {}
                def next_phase(probe, children, timeout, label, check_mapper):
                    check_mapper()
                    self.assertFalse(probe())
                    if (out / "lap.complete.json").exists():
                        self.assertEqual(json.loads((out / "lap.complete.json").read_text()),
                                         {"start_sim_ns": 2_000_000_000, "finish_sim_ns": 10_000_000_000})
                        (out / "mapping.saved.json").write_text('{"map_id": "map-a"}')
                    elif (directory / "lap_capture.ready").exists():
                        self.assertTrue((out / "mapping.ready").exists())
                        (directory / "lap_start.json").write_text('{"sim_time_s": 2.0}')
                        with (directory / "rig_motion.jsonl").open("w") as f:
                            for i in range(17):
                                x, y, _ = run.interpolate(points, [0, 1, 2], i / 8 if valid else 0)
                                f.write(json.dumps({"sim_time_s": 2+i/2, "x_m": x, "y_m": y}) + "\n")
                        (directory / "lap_finished.ready").touch()
                    else:
                        self.assertTrue((out / "gazebo.ready").is_file())
                        (out / "mapping.ready").touch()
                    self.assertTrue(probe())

                with patch.object(run, "start_stream") as stream, patch.object(run, "wait_for", next_phase):
                    if valid:
                        self.assertEqual(run.map_one_lap({}, out, "spa", scene, report), 0)
                        self.assertTrue(report["motion"]["lap_completed"])
                        self.assertEqual(report["mapping"]["map_id"], "map-a")
                    else:
                        with self.assertRaisesRegex(RuntimeError, "全周"):
                            run.map_one_lap({}, out, "spa", scene, report)
                        self.assertFalse((out / "lap.complete.json").exists())
                    stream.assert_called_once()

    def test_mapping_peer_failure_and_cancel_never_release_the_rig(self):
        for signal, payload in [("mapping.failed.json", '{"error":"tracking lost"}'), ("mapping.cancel", "")]:
            with self.subTest(signal=signal), tempfile.TemporaryDirectory() as tmp, patch.object(run, "start_stream"):
                out = Path(tmp)
                (out / signal).write_text(payload)
                with self.assertRaisesRegex(RuntimeError, "tracking lost|中断"):
                    run.map_one_lap({}, out, "monza", {}, {})
                self.assertFalse((out / "scene/lap_capture.ready").exists())
                self.assertFalse((out / "lap.complete.json").exists())

    def test_generated_rig_matches_ros_endpoints_and_transforms(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            scene = world.build(out)
            sensors = ET.parse(out / "world.sdf").findall(".//sensor")
        topics = {t["gz_topic_name"]: t for t in run.bridge_topics()}
        by_child = {entry[1]: entry for entry in run.transforms(scene["camera"])}
        for sensor in sensors:
            side = sensor.attrib["name"]
            self.assertEqual(topics[sensor.findtext("topic")]["ros_topic_name"], capture.TOPICS[side])
            info = topics[sensor.findtext("camera/camera_info_topic")]
            self.assertEqual(info["ros_topic_name"], capture.INFO_TOPICS[side])
            frame = sensor.findtext("frame_id")
            self.assertEqual(frame, capture.FRAMES[side])
            pose = list(map(float, sensor.findtext("pose").split()))
            self.assertEqual(by_child[frame][3], pose[1])
            self.assertEqual(by_child[frame][5:], (-pi/2, 0, -pi/2))
        self.assertEqual(topics["/world/monza/clock"]["ros_topic_name"], "/clock")

    def test_serve_waits_for_compatible_stereo_subscribers_and_advancing_ros_clock(self):
        ros = MagicMock()
        node = ros.create_node.return_value.__enter__.return_value
        node.count_publishers.return_value = 1
        stamps = [0, 11_111_111, 11_111_111, 22_222_222]
        node.get_clock.return_value.now.side_effect = [SimpleNamespace(nanoseconds=s) for s in stamps]
        sensor_types = {t["ros_topic_name"]: t["ros_type_name"] for t in run.bridge_topics()[1:]}
        node.get_publishers_info_by_topic.side_effect = lambda name: [
            SimpleNamespace(topic_type=sensor_types[name], qos_profile="offered")]
        # An incompatible subscriber on one eye must not start the trajectory.
        seen = {name: 0 for name in sensor_types}

        def subscriptions(name):
            seen[name] += 1
            return [SimpleNamespace(topic_type=sensor_types[name],
                                    qos_profile="incompatible" if seen[name] == 1 else "matched")]

        node.get_subscriptions_info_by_topic.side_effect = subscriptions
        def play_when_ready(circuit):
            self.assertEqual(circuit, "spa")
            self.assertTrue(all(n >= 2 for n in seen.values()))
            return True

        modules = {"rclpy": ros, "rclpy.parameter": SimpleNamespace(Parameter=lambda name, value: (name, value)),
                   "rclpy.signals": SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)),
                   "rclpy.qos": SimpleNamespace(QoSCompatibility=SimpleNamespace(ERROR=2),
                       qos_check_compatible=lambda pub, sub: (2 if sub == "incompatible" else 0, ""))}
        with patch.dict(sys.modules, modules), patch.object(run.time, "sleep"), \
                patch.object(run, "unpause", side_effect=play_when_ready) as play:
            run.start_stream({}, True, "spa")
        play.assert_called_once()
        self.assertEqual(ros.create_node.call_args.kwargs["parameter_overrides"], [("use_sim_time", True)])
        self.assertEqual(ros.spin_once.call_count, len(stamps))
        self.assertFalse(node.create_subscription.called)  # No diagnostic image subscription in the data path.

    def test_stationary_or_incomplete_video_is_not_accepted_as_driving(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            scene = world.build(out)
            path = out / "rig_motion.jsonl"
            metrics = {"mode": "video", "pair_span_sim_s": 10}
            samples = scene["trajectory"]["samples_s_x_y_yaw"]

            def write(samples):
                path.write_text("".join(json.dumps({"x_m": s[1], "y_m": s[2]}) + "\n" for s in samples))

            # A trace the length of the retired 12.5 m compact course still
            # falls well short of the georeferenced T2 exit.
            for poses in ([samples[0]] * 20, [s for s in samples if s[0] <= 12.5]):
                write(poses)
                with self.assertRaises(RuntimeError):
                    run.inspect_motion(out, scene, metrics)
            write(samples)
            result = run.inspect_motion(out, scene, metrics)
            self.assertTrue(result["reached_corner_exit"])
            self.assertAlmostEqual(result["sampled_distance_m"], scene["trajectory"]["end_s_m"], places=6)

    def test_lap_verifies_pose_at_intermediate_stations_and_capture_boundaries(self):
        profile = [[0, 0, 0, 0], [10, 10, 0, pi/2], [20, 10, 10, pi],
                   [30, 0, 10, -pi/2], [40, 0, 0, 0]]
        duration = 10.
        scene = {"trajectory": {"samples_s_x_y_yaw": profile, "route": "lap",
                                "end_s_m": 40., "travel_seconds": duration}}
        start = 4.
        metrics = {"mode": "video", "lap_start_sim_ns": 4_000_000_000,
                   "first_encoded_pair_sim_ns": 3_990_000_000,
                   "last_encoded_pair_sim_ns": 14_011_111_110}
        stations = [p[0] for p in profile]
        with tempfile.TemporaryDirectory() as dirname:
            out = Path(dirname)
            (out / "lap_start.json").write_text(json.dumps({"sim_time_s": start}))
            def trace(direction=1):
                samples = []
                for i in range(21):
                    s = i * 2 if direction >= 0 else 40 - i * 2
                    x, y, _ = run.interpolate(profile, stations, s if direction else 0.)
                    samples.append({"sim_time_s": start + i/2, "x_m": x, "y_m": y})
                (out / "rig_motion.jsonl").write_text("".join(json.dumps(p) + "\n" for p in samples))

            trace()
            verified = run.inspect_motion(out, scene, metrics)
            self.assertTrue(verified["lap_completed"])
            self.assertTrue(verified["video_endpoints_covered"])
            self.assertAlmostEqual(verified["sampled_distance_m"], 40)
            late = dict(metrics, first_encoded_pair_sim_ns=5_000_000_000)
            self.assertFalse(run.inspect_motion(out, scene, late)["video_endpoints_covered"])
            for direction in (0, -1):
                trace(direction)
                rejected = run.inspect_motion(out, scene, metrics)
                self.assertFalse(rejected["lap_completed"])
                self.assertTrue(rejected["video_endpoints_covered"])


if __name__ == "__main__":
    unittest.main()

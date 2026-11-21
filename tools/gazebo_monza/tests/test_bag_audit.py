from collections import Counter
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bag_audit as audit


STEP = round(audit.NANO / audit.TARGET_HZ)
PIXELS = bytes(640 * 480)


def frame(topic, index, geometry=(640, 480)):
    side = audit.CAMERAS[topic]
    w, h = geometry
    sec, ns = divmod(index * STEP, audit.NANO)
    header = Obj(stamp=Obj(sec=sec, nanosec=ns), frame_id=audit.FRAMES[side])
    if topic == audit.TOPICS[side]:
        return Obj(header=header, width=w, height=h, encoding="mono8", step=w,
                   data=PIXELS if geometry == (640, 480) else bytes(w * h))
    tx = -337 * audit.BASELINE_M if side == "right" else 0.
    return Obj(header=header, width=w, height=h, k=[337, 0, w / 2, 0, 337, h / 2, 0, 0, 1],
               p=[337, 0, w / 2, tx, 0, 337, h / 2, 0, 0, 0, 1, 0])


def static_tf(imu_hz=0):
    transforms = []
    imu_frames = {frame for frame, _ in audit.imu_streams(imu_hz).values()}
    links = [("camera_link", "base_link", 0.), (audit.FRAMES["left"], "camera_link", .0475),
             (audit.FRAMES["right"], "camera_link", -.0475)]
    links.extend((frame, "camera_link", 0.) for frame in imu_frames)
    for child, parent, y in links:
        q = (0, 0, 0, 1) if child == "camera_link" else (-.5, .5, -.5, .5)
        t = Obj(translation=Obj(x=0., y=y, z=0.), rotation=Obj(x=q[0], y=q[1], z=q[2], w=q[3]))
        transforms.append(Obj(header=Obj(frame_id=parent), child_frame_id=child, transform=t))
    return Obj(transforms=transforms)


def recording(indices=range(1, 8), *, geometry=(640, 480), imu_hz=0):
    messages = [(audit.TF, static_tf(imu_hz))]
    messages.extend((topic, frame(topic, i, geometry)) for i in indices for topic in audit.CAMERAS)
    for name, (frame_id, rate) in audit.imu_streams(imu_hz).items():
        for ns in range(audit.NANO // rate, 8 * STEP, audit.NANO // rate):
            header = Obj(frame_id=frame_id, stamp=Obj(sec=0, nanosec=ns))
            messages.append((name, Obj(header=header, linear_acceleration=Obj(x=0., y=-9.8, z=0.),
                                       angular_velocity=Obj(x=.5, y=0., z=0.),
                                       linear_acceleration_covariance=[-1. if name == audit.GYRO_TOPIC else 0.] + [0.]*8,
                                       angular_velocity_covariance=[-1. if name == audit.ACCEL_TOPIC else 0.] + [0.]*8)))
    return messages


def receipt_time(name, msg):
    if name == audit.TF:
        return 1
    tm = msg.header.stamp
    return tm.sec * audit.NANO + tm.nanosec + 500_000


class AuditTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.bag = Path(temp.name)
        self.file = self.bag / "record_0.mcap"
        self.file.write_bytes(audit.MCAP_MAGIC + b"fixture" + audit.MCAP_MAGIC)

    def metadata(self, messages):
        rows = []
        for topic, count in Counter(name for name, _ in messages).items():
            meta = {"name": topic, "type": audit.TYPES[topic], "serialization_format": "cdr"}
            if topic == audit.TF:
                meta["offered_qos_profiles"] = [{"durability": "TRANSIENT_LOCAL"}]
            rows.append({"topic_metadata": meta, "message_count": count})
        return {"storage_identifier": "mcap", "relative_file_paths": [self.file.name],
                "message_count": len(messages), "topics_with_message_count": rows}

    def audit(self, messages, start=2 * STEP, finish=6 * STEP, meta=None, clock=receipt_time, **options):
        stored = ((name, msg, clock(name, msg)) for name, msg in messages)
        with patch.object(audit, "load_bag_metadata", return_value=self.metadata(messages) if meta is None else meta), \
             patch.object(audit, "read_messages", return_value=stored):
            return audit.audit_bag(self.bag, start, finish, **options)

    def test_requires_stereo_inside_each_requested_lap(self):
        options = {"start": 2 * STEP, "finish": 8 * STEP, "laps": 2, "lap_duration_s": 3 * STEP / audit.NANO}
        result = self.audit(recording(range(1, 10)), **options)
        self.assertEqual(result["interior_stereo_samples_per_lap"], [2, 2])
        with self.assertRaisesRegex(ValueError, "2周目"):
            self.audit(recording([1, 2, 3, 4, 5, 8, 9]), **options)

    def test_selected_geometry_and_all_imu_streams_are_verified_in_sim_time(self):
        geometry = (424, 240)
        for rate in (200, 400):
            messages = recording(geometry=geometry, imu_hz=rate)
            result = self.audit(messages, geometry=geometry, imu_hz=rate)
            self.assertEqual(result["geometry"], list(geometry))
            self.assertEqual(result["stereo_calibration"]["right"]["width"], geometry[0])
            for name in (audit.ACCEL_TOPIC, audit.GYRO_TOPIC, audit.IMU_TOPIC):
                stream = result["imu_streams"][name]
                self.assertEqual(stream["measured_sim_hz"], 200 if name == audit.ACCEL_TOPIC else rate)
                self.assertEqual(stream["missing_slots_estimate"], 0)
                self.assertGreater(stream["interior_samples_per_lap"][0], 0)
                self.assertEqual(result["static_tf_parents"][stream["frame"]], "camera_link")
        with self.assertRaisesRegex(ValueError, "mono8"):
            self.audit(messages, imu_hz=400)
        with self.assertRaisesRegex(ValueError, "必須の topic"):
            self.audit([(name, m) for name, m in messages if name != audit.GYRO_TOPIC], geometry=geometry, imu_hz=400)
        with self.assertRaisesRegex(ValueError, "周期"):
            self.audit(recording(imu_hz=200), imu_hz=400)

    def test_imu_timing_gaps_payload_and_tf(self):
        messages = recording(imu_hz=400)
        short = [(name, m) for name, m in messages
                 if name != audit.IMU_TOPIC or m.header.stamp.nanosec != 30_000_000]
        result = self.audit(short, imu_hz=400)
        self.assertEqual(result["imu_streams"][audit.IMU_TOPIC]["missing_slots_estimate"], 1)
        self.assertEqual(result["imu_streams"][audit.GYRO_TOPIC]["missing_slots_estimate"], 0)
        with self.assertRaisesRegex(ValueError, "両端"):
            self.audit([(n, m) for n, m in messages if n != audit.IMU_TOPIC or m.header.stamp.nanosec < 60_000_000],
                       imu_hz=400)
        bad = next(m for n, m in messages if n == audit.IMU_TOPIC)
        bad.angular_velocity.x = float("nan")
        with self.assertRaisesRegex(ValueError, "角速度"):
            self.audit(messages, imu_hz=400)
        bad.angular_velocity.x = 0.
        bad.header.frame_id = audit.FRAMES["left"]
        with self.assertRaisesRegex(ValueError, "frame"):
            self.audit(messages, imu_hz=400)
        messages = recording(imu_hz=400)
        messages[0][1].transforms.pop()
        with self.assertRaisesRegex(ValueError, "固定 TF"):
            self.audit(messages, imu_hz=400)

    def test_audits_common_sensor_preroll_on_the_bag_clock(self):
        messages = recording(imu_hz=400)
        start = 4 * STEP
        result = self.audit(messages, imu_hz=400, start=start, preroll_s=.03)
        self.assertAlmostEqual(result["prestart_sensor_span_s"], (start - STEP) / audit.NANO)
        with self.assertRaisesRegex(ValueError, "走行前記録"):
            self.audit(messages, imu_hz=400, start=start, preroll_s=.04)

    def test_lap_counts_use_exact_header_stamps_and_bracket_both_edges(self):
        messages = recording([1, 2, 3, 4, 6, 7, 8])
        # 5 枠目は全ストリーム欠落。4 枠目は右 CameraInfo のみ欠落。
        missing = audit.INFO_TOPICS["right"]
        messages = [(t, m) for t, m in messages if t != missing or m.header.stamp.nanosec != 4 * STEP]
        duplicate = (audit.INFO_TOPICS["left"], frame(audit.INFO_TOPICS["left"], 4))
        messages.insert(next(i for i, (t, m) in enumerate(messages) if t in audit.CAMERAS and
                             m.header.stamp.nanosec >= 6 * STEP), duplicate)
        result = self.audit(messages, start=2 * STEP + 1, finish=7 * STEP - 1)
        self.assertEqual(result["complete_stereo_samples"], 4)  # 2, 3, 6, 7
        self.assertEqual((result["first_covering_sim_ns"], result["last_covering_sim_ns"]), (2 * STEP, 7 * STEP))
        self.assertEqual(result["missing_complete_slots_estimate"], 2)
        self.assertEqual(result["largest_gap_sim_s"], 3 * STEP / audit.NANO)
        self.assertEqual(result["coverage"], "continuous")
        self.assertEqual(result["gaps"], [])
        self.assertEqual(result["streams"][missing]["missing_at_observed_stamps"], 1)
        self.assertEqual(result["streams"][missing]["missing_slots_estimate"], 2)
        self.assertEqual(result["streams"][duplicate[0]]["duplicates"], 1)
        self.assertEqual(result["streams"][audit.TOPICS["right"]]["duplicates"], 0)
        self.assertAlmostEqual(result["stereo_calibration"]["baseline_from_projection_m"], .095)
        self.assertEqual(result["largest_receive_header_offset_s"], .0005)
        self.assertEqual(result["message_counts"][audit.TF], 1)
        self.assertEqual(result["static_tf_parents"]["camera_link"], "base_link")
        json.dumps(result, allow_nan=False)

    def test_requires_covering_endpoints_and_samples_during_movement(self):
        for start, finish in ((STEP - 1, 2 * STEP), (2 * STEP, 7 * STEP + 1)):
            with self.subTest(start=start, finish=finish), self.assertRaisesRegex(ValueError, "両端"):
                self.audit(recording(), start, finish)
        with self.assertRaisesRegex(ValueError, "移動区間"):
            self.audit(recording([1, 2, 6, 7]))

    def test_gap_and_jitter_are_reported_without_discarding_a_completed_lap(self):
        def clock(name, msg):
            ns = receipt_time(name, msg)
            return ns + 2 * audit.NANO if ns > 50 * STEP else ns

        result = self.audit(recording([1, 2, 3, 50, 51]), finish=50 * STEP, clock=clock)
        self.assertEqual(result["coverage"], "gaps_detected")
        self.assertEqual(result["gaps"], [{"from_sim_ns": 3 * STEP, "to_sim_ns": 50 * STEP,
                                           "duration_s": 47 * STEP / audit.NANO}])
        self.assertEqual(result["largest_gap_sim_s"], 47 * STEP / audit.NANO)
        self.assertEqual(result["receive_header_offset_over_1s"], 8)
        self.assertEqual(result["largest_receive_header_offset_s"], 2.0005)

    def test_fails_on_different_right_camera_info_stamp_at_every_frame(self):
        messages = recording()
        for t, m in messages:
            if t == audit.INFO_TOPICS["right"]:
                m.header.stamp.nanosec += 1
        with self.assertRaisesRegex(ValueError, "両端"):
            self.audit(messages)

    def test_duplicate_image_even_outside_lap_and_invalid_replay_clock_fail(self):
        messages = recording()
        messages.insert(1, (audit.TOPICS["right"], frame(audit.TOPICS["right"], 1)))
        with self.assertRaisesRegex(ValueError, "重複"):
            self.audit(messages)
        for bad_receipt in (0, 12 * audit.NANO, 1_800_000_000_000_000_000):
            with self.subTest(receipt=bad_receipt), self.assertRaisesRegex(ValueError, "記録時刻"):
                self.audit(recording(), clock=lambda name, msg: 1 if name == audit.TF else bad_receipt)
        with self.assertRaisesRegex(ValueError, "実時計"):
            self.audit(recording(), clock=lambda name, msg: audit.UNIX_2000_NS if
                       receipt_time(name, msg) > 7 * STEP else receipt_time(name, msg))
        result = self.audit(recording(), clock=lambda name, msg: 3 * STEP if name == audit.TF else receipt_time(name, msg))
        self.assertFalse(result["tf_ready_before_lap"])

    def test_refuses_overreported_count_partial_mcap_and_volatile_tf(self):
        messages = recording()
        wrong = self.metadata(messages + messages[:1])
        with self.assertRaisesRegex(ValueError, "件数"):
            self.audit(messages, meta=wrong)
        wrong = self.metadata(messages)
        wrong["topics_with_message_count"][0]["topic_metadata"]["offered_qos_profiles"] = [{"durability": 2}]
        with self.assertRaisesRegex(ValueError, "transient_local"):
            self.audit(messages, meta=wrong)
        self.file.write_bytes(audit.MCAP_MAGIC + b"truncated MCAP")
        with self.assertRaisesRegex(ValueError, "閉じ"):
            self.audit(messages)

    def test_rejects_missing_or_invalid_static_extrinsics(self):
        for corruption in ("missing", "baseline", "rotation", "frame"):
            messages = recording()
            transforms = messages[0][1].transforms
            if corruption == "missing":
                transforms.pop()
            elif corruption == "baseline":
                transforms[-1].transform.translation.y *= -1
            elif corruption == "rotation":
                transforms[-1].transform.rotation.w = float("nan")
            else:
                transforms[-1].header.frame_id = "odom"
            with self.subTest(corruption=corruption), self.assertRaisesRegex(ValueError, "TF"):
                self.audit(messages)

    def test_rejects_broken_pixels_bad_projection_changed_info_and_rewound_headers(self):
        for corruption, error in (("pixels", "mono8"), ("baseline", "projection"),
                                  ("calibration", "校正"), ("rewind", "逆行")):
            messages = recording()
            right_info = [m for t, m in messages if t == audit.INFO_TOPICS["right"]]
            if corruption == "pixels":
                next(m for t, m in messages if t == audit.TOPICS["left"]).data = b""
            elif corruption == "baseline":
                for m in right_info:
                    m.p[3] = 0.
            elif corruption == "calibration":
                right_info[-1].k[0] += 1
            else:
                right_info[-1].header.stamp.nanosec = STEP
            with self.subTest(corruption=corruption), self.assertRaisesRegex(ValueError, error):
                self.audit(messages)

    @unittest.skipUnless(importlib.util.find_spec("yaml"), "metadata.yaml の読込には PyYAML が必要")
    def test_uses_the_offline_mapping_metadata_parser(self):
        meta = self.metadata(recording())
        (self.bag / "metadata.yaml").write_text(json.dumps({"rosbag2_bagfile_information": meta}))
        counts, sizes = audit.inspect_metadata(self.bag)
        self.assertEqual(counts[audit.TF], 1)
        self.assertEqual(sizes[self.file.name], self.file.stat().st_size)

    @unittest.skipUnless(importlib.util.find_spec("rosbag2_py"), "Notebook の ROS 2 Lyrical と MCAP が必要")
    def test_native_mcap_writer_to_audit_reader(self):
        import rosbag2_py
        from geometry_msgs.msg import TransformStamped
        from rclpy.serialization import serialize_message
        from rosbag2_py._storage import QoS
        from sensor_msgs.msg import CameraInfo, Image, Imu
        from tf2_msgs.msg import TFMessage

        bag = self.bag / "rosbag"
        options = rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap")
        writer = rosbag2_py.SequentialWriter()
        writer.open(options, rosbag2_py.ConverterOptions("", ""))
        try:
            qos = QoS(10)
            qos.transient_local()
            for idx, (topic, kind) in enumerate(audit.TYPES.items(), 1):
                writer.create_topic(rosbag2_py.TopicMetadata(
                    id=idx, name=topic, type=kind, serialization_format="cdr",
                    offered_qos_profiles=[qos] if topic == audit.TF else []))
            geometry = (848, 480)
            messages = recording(range(1, 5), geometry=geometry, imu_hz=400)
            for name, msg in sorted(messages, key=lambda item: receipt_time(*item)):
                if name == audit.TF:
                    message = TFMessage()
                    for source in msg.transforms:
                        tf = TransformStamped()
                        tf.header.frame_id, tf.child_frame_id = source.header.frame_id, source.child_frame_id
                        tf.transform.translation.y = source.transform.translation.y
                        q = source.transform.rotation
                        rotation = tf.transform.rotation
                        rotation.x, rotation.y, rotation.z, rotation.w = map(float, (q.x, q.y, q.z, q.w))
                        message.transforms.append(tf)
                elif name in audit.CAMERAS:
                    if name in audit.TOPICS.values():
                        message = Image(width=msg.width, height=msg.height, step=msg.step, encoding="mono8",
                                        data=msg.data)
                    else:
                        message = CameraInfo(width=msg.width, height=msg.height, k=list(map(float, msg.k)),
                                             p=list(map(float, msg.p)))
                else:
                    message = Imu(angular_velocity_covariance=msg.angular_velocity_covariance,
                                  linear_acceleration_covariance=msg.linear_acceleration_covariance)
                    message.angular_velocity.x = msg.angular_velocity.x
                    message.linear_acceleration.y = msg.linear_acceleration.y
                if name != audit.TF:
                    message.header.frame_id = msg.header.frame_id
                    message.header.stamp.nanosec = msg.header.stamp.nanosec
                writer.write(name, serialize_message(message), receipt_time(name, msg))
        finally:
            writer.close()
        result = audit.audit_bag(bag, STEP + 1, 4 * STEP - 1, geometry=geometry, imu_hz=400)
        self.assertEqual(result["complete_stereo_samples"], 4)
        self.assertEqual(sum(result["message_counts"].values()), len(messages))
        self.assertEqual(result["imu_streams"][audit.IMU_TOPIC]["measured_sim_hz"], 400)
        self.assertAlmostEqual(result["stereo_calibration"]["baseline_from_projection_m"], .095)


if __name__ == "__main__":
    unittest.main()

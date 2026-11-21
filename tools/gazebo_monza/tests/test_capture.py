import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import capture


def stereo_stamp(index):
    return round(index * capture.NANO / capture.TARGET_HZ)


class PairTests(unittest.TestCase):
    def test_exact_stamp_duplicate_missing_and_out_of_order(self):
        pairs = capture.StereoPairs()
        for i in (1, 2, 4):
            t = stereo_stamp(i)
            self.assertIsNone(pairs.feed("right", t, i, b"R"))
            self.assertEqual(pairs.feed("left", t, i, b"L"), (b"L", b"R"))
            self.assertIsNone(pairs.feed("right", t, i, b"R"))
        t = stereo_stamp(3)
        self.assertIsNone(pairs.feed("right", t + 1, 5))
        self.assertIsNone(pairs.feed("right", t, 5))
        self.assertIsNone(pairs.feed("left", t, 6))
        report = pairs.summary(6)
        self.assertEqual(report["unique_ordered_pairs"], 3)
        self.assertEqual(report["duplicate_images"], {"left": 0, "right": 3})
        self.assertEqual(report["unpaired_images"], {"left": 0, "right": 1})
        self.assertEqual(report["out_of_order_pairs"], 1)
        self.assertEqual(report["pairing"]["shared_image_stamps"], 4)
        self.assertEqual(report["pairing"]["shared_stamps_evicted_before_match"], 0)
        self.assertEqual(report["missing_sim_slots_between_pairs_estimate"], 1)
        self.assertFalse(capture.benchmark_passes(report))

    def test_simulation_rate_does_not_substitute_for_wall_delivery(self):
        pairs = capture.StereoPairs()
        for i in range(90):
            for side in capture.TOPICS:
                pairs.feed(side, stereo_stamp(i), i / 45)
        self.assertTrue(capture.benchmark_passes(pairs.summary(1)))
        slow = pairs.summary(2)
        self.assertEqual(slow["paired_wall_hz"], 45)
        self.assertAlmostEqual(slow["paired_sim_hz"], 90)
        self.assertFalse(capture.benchmark_passes(slow))

    def test_per_eye_delivery_and_overflow_of_live_pairing_buffer(self):
        pairs = capture.StereoPairs()
        for i in range(20):
            pairs.feed("left", stereo_stamp(i), i / 90)
        for i in (0, 1, 4, 9, 19):
            pairs.feed("right", stereo_stamp(i), 0.5 + i / 90)
        result = pairs.summary(1)
        self.assertEqual(result["received_images"], {"left": 20, "right": 5})
        self.assertEqual(result["unique_ordered_pairs"], 3)
        self.assertEqual(result["pairing"]["shared_image_stamps"], 5)
        self.assertEqual(result["pairing"]["shared_stamps_evicted_before_match"], 2)
        left, right = (result["image_streams"][side] for side in capture.TOPICS)
        self.assertAlmostEqual(left["unique_sim_hz"], 90)
        self.assertEqual(left["unique_wall_hz"], 20)
        self.assertEqual(right["unique_wall_hz"], 5)
        self.assertEqual(right["missing_sim_slots_between_images_estimate"], 15)
        self.assertEqual((right["first_sim_ns"], right["last_sim_ns"]), (0, stereo_stamp(19)))
        self.assertFalse(capture.benchmark_passes(result))

    def test_udp_counters_linux_and_unavailable(self):
        with tempfile.TemporaryDirectory() as directory:
            snmp = Path(directory) / "snmp"
            self.assertIsNone(capture.udp_error_counters(snmp))
            snmp.write_text("Udp: InDatagrams InErrors RcvbufErrors SndbufErrors\nUdp: 200 5 4 1\n")
            self.assertEqual(capture.udp_error_counters(snmp), {"InErrors": 5, "RcvbufErrors": 4, "SndbufErrors": 1})
            snmp.write_text("Icmp: InErrors\nIcmp: 0\n")
            self.assertIsNone(capture.udp_error_counters(snmp))

    def test_input_geometry_and_row_padding(self):
        for width, height in capture.RESOLUTIONS.values():
            pixel_row = b"\x01" * width
            image = SimpleNamespace(width=width, height=height, encoding="mono8", step=width + 2,
                                    data=(pixel_row + b"XX") * height,
                                    header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=4)))
            self.assertEqual(capture.check_image(image, (width, height)), 1_000_000_004)
            self.assertEqual(capture.packed_pixels(image), pixel_row * height)
            if width != capture.WIDTH:
                with self.assertRaises(ValueError):
                    capture.check_image(image)
        image.encoding = "rgb8"
        with self.assertRaises(ValueError):
            capture.check_image(image, (width, height))


class CaptureLoopTests(unittest.TestCase):
    def run_receiver(self, directory, frame_count, matched=True, right_baseline=0.095,
                     deliver_info=True, lap_video=False):
        clock = SimpleNamespace(now=3.0, next_frame=0)
        callbacks = {}
        raw = bytes(640 * 480)

        def spin_once(timeout_sec):
            if matched and clock.next_frame < frame_count:
                stamp = stereo_stamp(clock.next_frame)
                if lap_video:
                    stamp += 4 * capture.NANO
                    scene_dir = directory / "scene"
                    origin_file = scene_dir / "lap_start.json"
                    if (scene_dir / "lap_capture.ready").is_file() and not origin_file.is_file():
                        origin_file.write_text(json.dumps({"sim_time_s": stamp / capture.NANO}))
                clock.now += 1.0 / 90
                for side, topic in capture.TOPICS.items():
                    header = SimpleNamespace(frame_id=capture.FRAMES[side], stamp=SimpleNamespace(
                        sec=stamp // capture.NANO, nanosec=stamp % capture.NANO))
                    msg = SimpleNamespace(width=640, height=480, step=640, data=raw, encoding="mono8", header=header)
                    callbacks[topic](msg)
                    topic = capture.INFO_TOPICS[side]
                    if deliver_info and topic in callbacks:
                        tx = -337 * right_baseline if side == "right" else 0
                        callbacks[topic](SimpleNamespace(width=640, height=480, header=header,
                            k=[337, 0, 320, 0, 337, 240, 0, 0, 1],
                            p=[337, 0, 320, tx, 0, 337, 240, 0, 0, 0, 1, 0]))
                clock.next_frame += 1
            else:
                clock.now += timeout_sec

        def subscribe(typ, topic, callback, qos):
            callbacks[topic] = callback
            return SimpleNamespace(get_publisher_count=lambda: int(matched), topic=topic)

        node = SimpleNamespace(create_subscription=subscribe, destroy_node=lambda: None,
                               destroy_subscription=lambda sub: callbacks.pop(sub.topic))
        executor = Mock(spin_once=spin_once)
        ros = SimpleNamespace(init=lambda: None, create_node=lambda name: node,
                              ok=lambda: True, try_shutdown=lambda: None)
        fake_modules = {"rclpy": ros, "rclpy.qos": SimpleNamespace(qos_profile_sensor_data=object()),
                        "rclpy.executors": SimpleNamespace(SingleThreadedExecutor=lambda: executor),
                        "sensor_msgs": SimpleNamespace(), "sensor_msgs.msg": SimpleNamespace(Image=object, CameraInfo=object)}
        args = SimpleNamespace(output=directory, mode="video" if lap_video else "benchmark",
                               route="lap" if lap_video else "chicane", duration=.2 if lap_video else 1.0, warmup=0.2,
                               startup_timeout=0.5, video_timeout=100.0, resolution="640x480")
        report = {}
        counters = [{"InErrors": i, "RcvbufErrors": i, "SndbufErrors": 0} for i in (5, 7)]
        with patch.dict(sys.modules, fake_modules), patch.object(capture.time, "monotonic", lambda: clock.now), \
                patch.object(capture, "udp_error_counters", side_effect=counters):
            status = capture.capture(args, report)
        executor.add_node.assert_called_once_with(node)
        executor.shutdown.assert_called_once_with()
        return status, report

    @unittest.skipUnless(shutil.which("ffmpeg"), "ffmpeg required")
    def test_lap_video_waits_for_stereo_before_arming_and_captures_finish(self):
        with tempfile.TemporaryDirectory() as dirname:
            out = Path(dirname)
            (out / "scene").mkdir()
            status, metrics = self.run_receiver(out, 100, lap_video=True)
            self.assertEqual(status, 0)
            origin = metrics["lap_start_sim_ns"]
            self.assertGreater(origin, 4 * capture.NANO)
            self.assertLessEqual(metrics["first_encoded_pair_sim_ns"], origin)
            self.assertGreaterEqual(metrics["last_encoded_pair_sim_ns"],
                                    origin + round(.2 * capture.NANO) + stereo_stamp(2))
            self.assertEqual(metrics["video_frames"], 7)
            self.assertTrue((out / "left.mp4").is_file())

    def test_fixed_window_after_warmup_and_silent_startup(self):
        with tempfile.TemporaryDirectory() as root:
            status, report = self.run_receiver(Path(root), 200)
            self.assertEqual(status, 0)
            self.assertTrue((Path(root) / "benchmark.ready").is_file())
            self.assertTrue(report["pass_90hz"])
            self.assertEqual(report["stereo_calibration"]["stamp_ns"], 0)
            self.assertAlmostEqual(report["stereo_calibration"]["baseline_from_projection_m"], 0.095)
            self.assertAlmostEqual(report["measurement_wall_s"], 1)
            self.assertAlmostEqual(report["measurement_end_monotonic_s"] - report["measurement_start_monotonic_s"], 1)
            self.assertEqual(report["udp_namespace_counter_deltas"], {"InErrors": 2, "RcvbufErrors": 2, "SndbufErrors": 0})
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaisesRegex(TimeoutError, "No synchronized"):
                self.run_receiver(Path(root), 0)

    def test_missing_publishers_calibration_or_wrong_baseline(self):
        with tempfile.TemporaryDirectory() as dirname:
            output = Path(dirname)
            with self.assertRaisesRegex(TimeoutError, "No matched ROS publisher"):
                self.run_receiver(output, 10, matched=False)
            self.assertFalse((output / "benchmark.ready").exists())
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaisesRegex(TimeoutError, "CameraInfo"):
                self.run_receiver(Path(root), 100, deliver_info=False)
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaisesRegex(ValueError, "0.000000 m baseline"):
                self.run_receiver(Path(root), 100, right_baseline=0.0)

    def test_stalled_stream_fails_fixed_window(self):
        with tempfile.TemporaryDirectory() as root:
            status, report = self.run_receiver(Path(root), 35)
            self.assertEqual(status, 2)
            self.assertFalse(report["pass_90hz"])


@unittest.skipUnless(shutil.which("ffmpeg") and shutil.which("ffprobe"), "ffmpeg required")
class StereoVideoTests(unittest.TestCase):
    def test_lap_window_uses_the_starting_pose_and_fresh_finish_frame(self):
        black = (bytes(capture.WIDTH * capture.HEIGHT),) * 2
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            video = capture.StereoVideo(output, .2, include_finish=True)
            # The stream came online long after sim time zero. The rig is parked
            # at this first observed pose until the agreed 5 s release.
            video.origin = 5 * capture.NANO
            video.last_stamp, video.last_frames = video.origin - 1_000_000, black
            for elapsed_ns in (7_000_000, 70_000_000, 199_000_000):
                video.add(video.origin + elapsed_ns, black)
            self.assertEqual(video.written, 6)
            self.assertEqual(video.first_encoded_stamp, video.origin - 1_000_000)
            video.add(video.origin + 208_000_000, black)
            self.assertEqual(video.written, 6)
            video.add(video.origin + 225_000_000, black)
            self.assertEqual(video.written, 7)
            self.assertEqual(video.last_encoded_stamp, video.origin + 225_000_000)
            video.close(True)

    def test_sync_video_files_with_different_pixels(self):
        width, height = capture.RESOLUTIONS["424x240"]
        count = width * height
        frames = (bytes(count), bytes([255]) * count)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            video = capture.StereoVideo(output, 0.2, geometry=(width, height))
            for index in (0, 3, 12, 18):
                video.add(stereo_stamp(index), frames)
            self.assertEqual(video.written, 6)
            self.assertGreater(video.repeated_frames, 0)
            video.close(True)
            for name, expected in zip(("left", "right"), (0, 255)):
                path = str(output / f"{name}.mp4")
                data = subprocess.check_output(["ffprobe", "-v", "error", "-show_streams", "-of", "json", path])
                stream = json.loads(data)["streams"][0]
                self.assertEqual(int(stream["nb_frames"]), 6)
                self.assertEqual((stream["width"], stream["height"]), (width, height))
                pixel = subprocess.check_output(["ffmpeg", "-v", "error", "-i", path, "-frames:v", "1",
                                                 "-vf", "scale=1:1", "-f", "rawvideo", "-pix_fmt", "gray", "-"])
                self.assertLessEqual(abs(pixel[0] - expected), 2)


if __name__ == "__main__":
    unittest.main()

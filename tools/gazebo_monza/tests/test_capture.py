import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

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

    def test_input_geometry_and_row_padding(self):
        pixel_row = bytes(range(256)) * 2 + bytes(128)
        image = SimpleNamespace(width=640, height=480, encoding="mono8", step=642,
                                data=(pixel_row + b"XX") * 480,
                                header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=4)))
        self.assertEqual(capture.check_image(image), 1_000_000_004)
        self.assertEqual(capture.packed_pixels(image), pixel_row * 480)
        image.encoding = "rgb8"
        with self.assertRaises(ValueError):
            capture.check_image(image)


class CaptureLoopTests(unittest.TestCase):
    def run_receiver(self, directory, frame_count, matched=True, right_baseline=0.095,
                     deliver_info=True):
        clock = SimpleNamespace(now=3.0, next_frame=0)
        callbacks = {}
        raw = bytes(640 * 480)

        def spin_once(node, timeout_sec):
            if matched and clock.next_frame < frame_count:
                stamp = stereo_stamp(clock.next_frame)
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
        ros = SimpleNamespace(init=lambda: None, create_node=lambda name: node,
                              spin_once=spin_once, ok=lambda: True, try_shutdown=lambda: None)
        fake_modules = {"rclpy": ros, "rclpy.qos": SimpleNamespace(qos_profile_sensor_data=object()),
                        "sensor_msgs": SimpleNamespace(), "sensor_msgs.msg": SimpleNamespace(Image=object, CameraInfo=object)}
        args = SimpleNamespace(output=directory, mode="benchmark", duration=1.0, warmup=0.2,
                               startup_timeout=0.5, video_timeout=100.0)
        report = {}
        with patch.dict(sys.modules, fake_modules), patch.object(capture.time, "monotonic", lambda: clock.now):
            status = capture.capture(args, report)
        return status, report

    def test_fixed_window_after_warmup_and_silent_startup(self):
        with tempfile.TemporaryDirectory() as root:
            status, report = self.run_receiver(Path(root), 200)
            self.assertEqual(status, 0)
            self.assertTrue((Path(root) / "benchmark.ready").is_file())
            self.assertTrue(report["pass_90hz"])
            self.assertEqual(report["stereo_calibration"]["stamp_ns"], 0)
            self.assertAlmostEqual(report["stereo_calibration"]["baseline_from_projection_m"], 0.095)
            self.assertAlmostEqual(report["measurement_wall_s"], 1)
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
    def test_sync_video_files_with_different_pixels(self):
        count = capture.WIDTH * capture.HEIGHT
        frames = (bytes(count), bytes([255]) * count)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            video = capture.StereoVideo(output, 0.2)
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
                self.assertEqual((stream["width"], stream["height"]), (640, 480))
                pixel = subprocess.check_output(["ffmpeg", "-v", "error", "-i", path, "-frames:v", "1",
                                                 "-vf", "scale=1:1", "-f", "rawvideo", "-pix_fmt", "gray", "-"])
                self.assertLessEqual(abs(pixel[0] - expected), 2)


if __name__ == "__main__":
    unittest.main()

"""配信ブリッジと走行判定を、実際のシーン・ROS受信設定と突き合わせる。"""
import json
from math import pi
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import capture
import run
import world


class RunContractTests(unittest.TestCase):
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

    def test_stationary_or_incomplete_video_is_not_accepted_as_driving(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            scene = world.build(out)
            path = out / "rig_motion.jsonl"
            metrics = {"mode": "video", "pair_span_sim_s": 10}
            samples = scene["trajectory"]["samples_s_x_y_yaw"]

            def write(samples):
                path.write_text("".join(json.dumps({"x_m": s[1], "y_m": s[2]}) + "\n" for s in samples))

            for poses in ([samples[0]] * 20, samples[:10]):
                write(poses)
                with self.assertRaises(RuntimeError):
                    run.inspect_motion(out, scene, metrics)
            write(samples)
            result = run.inspect_motion(out, scene, metrics)
            self.assertTrue(result["reached_corner_exit"])
            self.assertAlmostEqual(result["sampled_distance_m"], 12.5, places=1)


if __name__ == "__main__":
    unittest.main()

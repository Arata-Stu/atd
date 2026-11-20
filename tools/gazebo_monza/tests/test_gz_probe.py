import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import gz_probe


class ProbeTests(unittest.TestCase):
    def test_uses_exact_stamps_inside_same_half_open_wall_window(self):
        s = gz_probe.NANO
        images = {"left": [(3.9, 0), (4, s), (4.1, s), (5, 2*s), (6, 3*s)],
                  "right": [(4.5, s), (4.6, s+1), (5.1, 2*s), (6, 3*s)]}
        world = [(3.9, 0), (4.1, s), (5.1, s + s//2), (6, 2*s)]
        metrics = gz_probe.summarize(images, world, 4, 6)
        self.assertEqual(metrics["received_images"], {"left": 3, "right": 3})
        self.assertEqual(metrics["eye_wall_hz"], {"left": 1.5, "right": 1.5})
        self.assertEqual(metrics["duplicate_images"], {"left": 1, "right": 0})
        self.assertEqual(metrics["unpaired_unique_stamps"], {"left": 0, "right": 1})
        self.assertEqual(metrics["matching_stamps"], 2)
        self.assertEqual(metrics["paired_wall_hz"], 1)
        self.assertEqual(metrics["paired_sim_hz"], 1)
        self.assertEqual(metrics["largest_sim_gap_ms"], 1000)
        self.assertAlmostEqual(metrics["world"]["real_time_factor"], .5)

    def test_missing_ros_measurement_is_reported_separately_even_without_messages(self):
        topics = []

        class Node:
            def subscribe(self, msg_type, topic, callback):
                topics.append(topic)
                return True

        modules = {"gz.transport": SimpleNamespace(Node=Node),
                   "gz.msgs.image_pb2": SimpleNamespace(Image=object),
                   "gz.msgs.world_stats_pb2": SimpleNamespace(WorldStatistics=object)}
        with tempfile.TemporaryDirectory() as temp, patch.dict(sys.modules, modules):
            output = Path(temp)
            (output / "benchmark.json").write_text('{"status": "error"}')
            gz_probe.measure(output, "spa")
            self.assertTrue((output / "gz_probe.ready").is_file())
            report = json.loads((output / "gz_probe.json").read_text())
            self.assertEqual(set(topics), {"/spa/left/image", "/spa/right/image", "/world/spa/stats"})
            self.assertEqual(report["window"], "probe_lifetime_without_ros_window")
            self.assertEqual(report["matching_stamps"], 0)
            self.assertIsNone(report["paired_sim_hz"])
            self.assertIsNone(report["world"]["real_time_factor"])

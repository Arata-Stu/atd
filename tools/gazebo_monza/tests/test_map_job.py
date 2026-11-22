"""Run both sides of the host coordinator with local process fixtures."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import map_job


class MapJobTests(unittest.TestCase):
    def make_output(self, root):
        out = root / "runs/gazebo/odaiba/trial"
        out.mkdir(parents=True)
        return out

    def test_finite_lap_is_published_only_after_save_and_both_exits(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            out = self.make_output(root)
            common = """
from pathlib import Path
import sys, time
p = Path(sys.argv[1])
def wait(name):
    deadline = time.monotonic() + 5
    while not (p/name).exists():
        if time.monotonic() > deadline: raise TimeoutError(name)
        time.sleep(.01)
"""
            gazebo = common + """
assert not any(p.iterdir())
(p/'gazebo.ready').touch()
wait('mapping.ready')
(p/'scene').mkdir()
(p/'scene/scene.json').write_text('{"geography":{"fixture":true}}')
(p/'lap.complete.json').write_text('{"start_sim_ns":1,"finish_sim_ns":2}')
wait('mapping.saved.json')
(p/'run.json').write_text('{"status":"complete","circuit":"odaiba","motion":{"lap_completed":true}}')
"""
            mapper = common + """
assert (p/'gazebo.ready').exists()
(p/'mapping.ready').touch()
wait('lap.complete.json')
db = p/'map-staging/nvidia/cuvslam_map'
db.mkdir(parents=True)
(db/'landmarks.mdb').write_bytes(b'map')
(p/'mapping.saved.json').write_text('{"status":"complete","tracking":{"start_sim_ns":1,"finish_sim_ns":2},"databases":{"landmarks.mdb":3}}')
"""
            command = lambda script: [sys.executable, "-c", script, str(out)]
            map_job.coordinate(out, command(gazebo), command(mapper), lambda: self.fail("unexpected stop"))
            self.assertFalse((root / "maps").exists())
            saved = out / "mapping.saved.json"
            witness = saved.read_text()
            altered = json.loads(witness)
            altered["tracking"]["finish_sim_ns"] += 1
            saved.write_text(json.dumps(altered))
            with self.assertRaisesRegex(RuntimeError, "全周"):
                map_job.verify_and_publish(root, out, "odaiba")
            saved.write_text(witness)
            target = map_job.verify_and_publish(root, out, "odaiba")
            self.assertEqual(target, root / "maps/gazebo-odaiba-trial")
            self.assertEqual((target / "nvidia/cuvslam_map/landmarks.mdb").read_bytes(), b"map")
            self.assertEqual(json.loads((target / "mapping.json").read_text())["run"], "runs/gazebo/odaiba/trial")
            self.assertFalse((out / "map-staging").exists())

    def test_mapper_exit_is_detected_even_if_launch_returns_zero(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = self.make_output(Path(tmp))
            sim = [sys.executable, "-c", """
from pathlib import Path
import sys,time
p = Path(sys.argv[1])
(p/'gazebo.ready').touch()
while not (p/'mapping.cancel').exists(): time.sleep(.01)
""", str(out)]
            with self.assertRaisesRegex(RuntimeError, "地図保存前"):
                map_job.coordinate(out, sim, [sys.executable, "-c", "pass"], lambda: None)
            self.assertTrue((out / "mapping.cancel").is_file())
            self.assertFalse((Path(tmp) / "maps").exists())

    def test_two_lap_map_requires_both_laps_and_records_the_drive_settings(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            out = self.make_output(root)
            (out / "scene").mkdir()
            scene = {"geography": {}, "trajectory": {"laps": 2, "speed_mps": 3.0}}
            (out / "scene/scene.json").write_text(json.dumps(scene))
            timing = {"start_sim_ns": 10, "finish_sim_ns": 1000}
            (out / "lap.complete.json").write_text(json.dumps(timing))
            db = out / "map-staging/nvidia/cuvslam_map"
            db.mkdir(parents=True)
            (db / "landmarks.mdb").write_bytes(b"map")
            saved = {"status": "complete", "tracking": timing, "databases": {"landmarks.mdb": 3},
                     "slam_max_map_size": 0}
            (out / "mapping.saved.json").write_text(json.dumps(saved))
            motion = {"lap_completed": True, "laps_requested": 2, "laps_completed": 1}
            report = {"status": "complete", "circuit": "odaiba", "motion": motion}
            (out / "run.json").write_text(json.dumps(report))
            with self.assertRaisesRegex(RuntimeError, "全周"):
                map_job.verify_and_publish(root, out, "odaiba")
            self.assertFalse((root / "maps").exists())
            motion["laps_completed"] = 2
            (out / "run.json").write_text(json.dumps(report))
            target = map_job.verify_and_publish(root, out, "odaiba")
            metadata = json.loads((target / "mapping.json").read_text())
            self.assertEqual(metadata["lap"], {**timing, "speed_mps": 3.0, "laps_completed": 2})
            self.assertEqual(metadata["save"]["slam_max_map_size"], 0)

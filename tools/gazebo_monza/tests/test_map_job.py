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
        out = root / "runs/gazebo/spa/trial"
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
(p/'run.json').write_text('{"status":"complete","circuit":"spa","motion":{"lap_completed":true}}')
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
                map_job.verify_and_publish(root, out, "spa")
            saved.write_text(witness)
            target = map_job.verify_and_publish(root, out, "spa")
            self.assertEqual(target, root / "maps/gazebo-spa-trial")
            self.assertEqual((target / "nvidia/cuvslam_map/landmarks.mdb").read_bytes(), b"map")
            self.assertEqual(json.loads((target / "mapping.json").read_text())["run"], "runs/gazebo/spa/trial")
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

"""専用イメージの選択と容量計測。Docker のビルドを伴わないホスト検証。"""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("build_storage", ROOT / "docker/gazebo_monza/build_storage.py")
storage = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(storage)


class GazeboHostTests(unittest.TestCase):
    def test_run_previews_isolated_modes_and_storage_without_creating_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run(
                ["bash", str(ROOT / "scripts/gazebo.sh"), "run", "--dry-run", "--camera-height", ".14"],
                env={**os.environ, "ATD_RESULT_DIR": tmp, "ATD_GAZEBO_ROS_DOMAIN_ID": "91"},
                text=True, check=True, capture_output=True)
            self.assertEqual(list(Path(tmp).iterdir()), [])
        for mode in ("video", "benchmark"):
            self.assertIn(f"--mode {mode}", result.stdout)
        self.assertIn("atd-gazebo-monza:jetty-lyrical", result.stdout)
        self.assertIn("ROS_DOMAIN_ID=91", result.stdout)
        self.assertIn("--camera-height .14", result.stdout)
        self.assertIn("--pull=never", result.stdout)
        self.assertNotIn("atd_pc_dev", result.stdout)

    def test_storage_records_peak_and_failure_without_adding_shared_layers(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            filesystems = [{"mount": tmp, "paths": {"docker_root": tmp}}]
            child = mock.MagicMock()
            child.__enter__.return_value = child
            child.poll.side_effect = [None, 125]
            child.returncode = 125
            with (mock.patch.object(storage.subprocess, "check_output", return_value=tmp),
                  mock.patch.object(storage, "mount_points", return_value=[out]),
                  mock.patch.object(storage, "measured_filesystems", return_value=filesystems),
                  mock.patch.object(storage, "docker_snapshot") as snapshot,
                  mock.patch.object(storage, "image_size", side_effect=[None, None]),
                  mock.patch.object(storage.subprocess, "Popen", return_value=child),
                  mock.patch.object(storage, "free_bytes", side_effect=[100, 25, 60]),
                  mock.patch.object(storage.time, "sleep")):
                self.assertEqual(storage.measure_build(out, "atd-test", out), 125)
            report = json.loads((out / "storage.json").read_text())
            self.assertEqual(report["build_exit_code"], 125)
            self.assertEqual(report["filesystems"][0]["net_used_bytes"], 40)
            self.assertEqual(report["filesystems"][0]["peak_added_bytes_sampled"], 75)
            self.assertEqual([call.args[1] for call in snapshot.call_args_list], ["before", "after"])


if __name__ == "__main__":
    unittest.main()

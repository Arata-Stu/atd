"""専用イメージの選択と容量計測。Docker のビルドを伴わないホスト検証。"""
import importlib.util
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("build_storage", ROOT / "docker/gazebo_monza/build_storage.py")
storage = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(storage)


class GazeboHostTests(unittest.TestCase):
    def test_bag_uses_configured_record_mount_and_runs_one_simulator(self):
        command = ["bash", str(ROOT / "scripts/gazebo.sh"), "bag", "--dry-run"]
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = root / "storage.env"
            configured = root / "pc records"
            config.write_text(f"ATD_PC_RECORD_DIR={shlex.quote(str(configured))}\n")
            env = {**os.environ, "ATD_STORAGE_CONFIG": str(config), "ATD_RESULT_DIR": str(root / "runs")}
            env.pop("ATD_RECORD_DIR", None)
            for record_path in (configured, root / "override"):
                if record_path != configured:
                    env["ATD_RECORD_DIR"] = str(record_path)
                output = subprocess.check_output([*command, "--circuit", "odaiba", "--duration", "240"],
                                                 env=env, text=True)
                commands = [shlex.split(line) for line in output.splitlines() if line.startswith("docker ")]
                self.assertEqual(len(commands), 1)
                args = commands[0]
                self.assertIn(f"type=bind,src={record_path},dst=/data/record", args)
                self.assertIn("--shm-size=128m", args)
                self.assertFalse(any(arg.startswith("--ipc=") for arg in args))
                for flag, expected in (("--mode", "bag"), ("--route", "lap"), ("--duration", "240")):
                    self.assertEqual(args[args.index(flag) + 1], expected)
                bag = Path(args[args.index("--bag-output") + 1])
                self.assertEqual(bag.parent, Path("/data/record/rosbags"))
                stamp = r"\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}"
                self.assertRegex(bag.name, rf"^gazebo-odaiba-{stamp}_640x480-90Hz_imu-off_1lap_0\.3319mps$")
                self.assertTrue(any(arg.endswith(f"/{bag.name},dst=/output") for arg in args))
                self.assertNotIn("gazebo_mapping.launch.py", output)
                self.assertEqual(list(root.iterdir()), [config])
            for extra, record in ((["--route", "chicane"], str(configured)), ([], "relative"),
                                  ([], str(root / "has,comma"))):
                result = subprocess.run([*command, *extra], env={**env, "ATD_RECORD_DIR": record}, capture_output=True)
                self.assertNotEqual(result.returncode, 0)

    def test_bag_names_in_tokyo_time_use_sensor_settings_and_reserve_unique_paths(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scripts = root / "scripts"
            (scripts / "lib").mkdir(parents=True)
            shutil.copy2(ROOT / "scripts/gazebo.sh", scripts / "gazebo.sh")
            (root / "tools").symlink_to(ROOT / "tools", target_is_directory=True)
            (scripts / "lib/detect_host.sh").write_text("detect_atd_host() { echo pc; }\natd_in_container() { return 1; }\n")
            bin_dir = root / "bin"
            bin_dir.mkdir()
            # 2026-11-21 18:04:05 UTC is 2026-11-22 03:04:05 in Tokyo.
            fixtures = {
                "date": """#!/usr/bin/env python3
import os, sys, time
if '-u' in sys.argv:
    os.environ['TZ'] = 'UTC'
    time.tzset()
print(time.strftime(sys.argv[-1].removeprefix('+'), time.localtime(1795284245)))
""",
                "uname": '#!/bin/sh\ncase "$1" in -s) echo Linux;; -m) echo x86_64;; *) exit 1;; esac\n',
                "flock": "#!/bin/sh\nexit 0\n",
                "docker": """#!/bin/sh
set -eu
if [ "$1" = image ]; then test "$2" = inspect; exit; fi
test "$1" = run
while [ "$1" != --bag-output ]; do shift; done
mkdir "$ATD_RECORD_DIR/rosbags/${2##*/}"
""",
            }
            for name, content in fixtures.items():
                shim = bin_dir / name
                shim.write_text(content)
                shim.chmod(0o755)
            records = root / "record"
            results = root / "results"
            env = {"HOME": str(root), "TZ": "UTC", "PATH": f"{bin_dir}{os.pathsep}{os.environ['PATH']}",
                   "DOCKER_HOST": "unix:///unused", "ATD_RECORD_DIR": str(records), "ATD_RESULT_DIR": str(results)}
            command = ["bash", str(scripts / "gazebo.sh"), "bag"]
            prefix = "gazebo-odaiba-2026-11-22_03-04-05"
            for settings, profile in (([], "640x480-90Hz_imu-off_1lap_2.414mps"),
                                      (["--resolution", "848x480", "--camera-hz", "60", "--imu", "200",
                                        "--speed", "5.0", "--laps", "2"], "848x480-60Hz_imu-200Hz_2laps_5mps"),
                                      (["--resolution", "1280x720", "--camera-hz", "30", "--imu", "400",
                                        "--duration", "120", "--laps", "3"], "1280x720-30Hz_imu-400Hz_3laps_0.6638mps")):
                with self.subTest(profile=profile):
                    output = subprocess.check_output([*command, *settings, "--dry-run"], env=env, text=True)
                    self.assertIn(f"--bag-output /data/record/rosbags/{prefix}_{profile} ", output)
            self.assertFalse(records.exists())
            self.assertFalse(results.exists())

            bag_root = records / "rosbags"
            run_root = results / "runs/gazebo/odaiba"
            base = f"{prefix}_640x480-90Hz_imu-off_1lap_2.414mps"
            # A previously transferred bag may exist without its original run log.
            (bag_root / base).mkdir(parents=True)
            for suffix in ("-2", "-3"):
                name = base + suffix
                output = subprocess.check_output(command, env=env, text=True)
                self.assertIn(f"rosbag 保存先 {bag_root / name}\n", output)
                self.assertTrue((bag_root / name).is_dir())
                self.assertTrue((run_root / name / "output-size-kib.txt").is_file())
            # An interrupted run may have a log directory but no published bag.
            (run_root / (base + "-4")).mkdir()
            preview = subprocess.check_output([*command, "--dry-run"], env=env, text=True)
            self.assertIn(f"--bag-output /data/record/rosbags/{base}-5 ", preview)
            self.assertFalse((run_root / (base + "-5")).exists())
            self.assertFalse((bag_root / (base + "-5")).exists())

    def test_map_connects_two_containers_to_one_lap_and_results_mount(self):
        command = ["bash", str(ROOT / "scripts/gazebo.sh"), "map", "--dry-run"]
        with tempfile.TemporaryDirectory() as tmp:
            for dds in ("shm", "udp"):
                out = subprocess.check_output([*command, "--circuit", "odaiba", "--dds", dds],
                                              text=True, env={**os.environ, "ATD_RESULT_DIR": tmp,
                                                              "ATD_GAZEBO_ROS_DOMAIN_ID": "74"})
                self.assertIn("--mode map --output /output --circuit odaiba --route lap", out)
                self.assertIn("gazebo_mapping.launch.py", out)
                self.assertIn("/map-staging/nvidia/cuvslam_map", out)
                processes = [line for line in out.splitlines() if line.startswith("docker ")]
                self.assertEqual(len(processes), 2)
                self.assertTrue(all("ROS_DOMAIN_ID=74" in line for line in processes))
                self.assertEqual("--ipc=host" in out, dds == "shm")
                self.assertNotIn("--mode video", out)
            self.assertEqual(list(Path(tmp).iterdir()), [])
        self.assertNotEqual(subprocess.run([*command, "--route", "chicane"], capture_output=True).returncode, 0)

    def test_all_circuits_can_be_selected_from_the_host(self):
        command = ["bash", str(ROOT / "scripts/gazebo.sh")]
        listed = subprocess.check_output([*command, "list"], text=True).splitlines()
        self.assertEqual(listed, ["odaiba"])
        for circuit in listed:
            out = subprocess.check_output([*command, "video", "--dry-run", "--circuit", circuit], text=True)
            self.assertIn(f"--circuit {circuit}", out)
            self.assertIn(f"/gazebo/{circuit}/", out)
        for args in (["video", "--circuit", "../odaiba"], ["serve", "--route", "chicane", "--circuit", "odaiba"],
                     ["bag", "--course", "monza"], ["video", "--course", "spa"], ["prepare", "--circuit", "odaiba"]):
            self.assertNotEqual(subprocess.run([*command, *args, "--dry-run"], capture_output=True).returncode, 0)

    def test_lap_choice_reaches_simulation_and_rejects_unknown_routes(self):
        command = ["bash", str(ROOT / "scripts/gazebo.sh")]
        result = subprocess.run([*command, "video", "--route", "lap", "--dry-run"],
                                capture_output=True, check=True, text=True)
        self.assertIn("--route lap", result.stdout)
        for action, args in (("video", []), ("video", ["unknown"]), ("prepare", ["lap"])):
            result = subprocess.run([*command, action, "--dry-run", "--route", *args], capture_output=True)
            self.assertNotEqual(result.returncode, 0)

    def test_drive_settings_reach_supported_modes_and_run_measurement(self):
        command = ["bash", str(ROOT / "scripts/gazebo.sh")]
        for mode in ("run", "bag", "map", "video", "serve"):
            with self.subTest(mode=mode):
                out = subprocess.check_output([*command, mode, "--dry-run", "--course", "odaiba",
                                               "--speed", "1.5", "--laps", "3",
                                               "--resolution", "1280x720", "--camera-hz", "60", "--imu", "400"], text=True)
                drives = [shlex.split(line) for line in out.splitlines()
                          if line.startswith("docker ") and "gazebo_monza/run.py" in line]
                self.assertEqual(len(drives), 2 if mode == "run" else 1)
                for args in drives:
                    for flag, expected in (("--circuit", "odaiba"), ("--route", "lap"), ("--speed", "1.5"),
                                           ("--resolution", "1280x720"), ("--camera-hz", "60"), ("--imu", "400")):
                        self.assertEqual(args[args.index(flag) + 1], expected)
                    if args[args.index("--mode") + 1] == "benchmark":
                        self.assertNotIn("--laps", args)
                    else:
                        self.assertEqual(args[args.index("--laps") + 1], "3")

    def test_invalid_drive_settings_fail_before_start_and_diagnostics_accept_speed(self):
        command = ["bash", str(ROOT / "scripts/gazebo.sh")]
        cases = [("bag", "--resolution", "320x240"), ("bag", "--imu", "100"),
                 ("bag", "--camera-hz", "45"), ("run", "--camera-hz"), ("prepare", "--camera-hz", "30"),
                 ("prepare", "--imu", "400"), ("prepare", "--speed", "2"), ("video", "--speed", "nan"),
                 ("bag", "--speed", "-1"), ("video", "--speed", "1e500"),
                 ("bag", "--full-throttle-speed", "10"), ("serve", "--full-throttle-speed", "10"),
                 ("map", "--laps", "0"), ("video", "--laps", "1.2"), ("bag", "--laps"),
                 ("run", "--duration", "2", "--speed", "1"), ("serve", "--route", "chicane", "--laps", "1"),
                 ("benchmark", "--laps", "2"), ("diagnose", "--laps", "2"), ("video", "--course", "../spa")]
        for mode, *settings in cases:
            with self.subTest(mode=mode, settings=settings):
                result = subprocess.run([*command, mode, "--dry-run", *settings], capture_output=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn(b"docker run", result.stdout)
        out = subprocess.check_output([*command, "diagnose", "--speed", "3", "--duration", "10", "--dry-run"],
                                      text=True)
        self.assertIn("--duration 10", out)
        self.assertIn("--speed 3", out)

    def test_server_uses_same_ipc_as_isaac_ros(self):
        shell = """
            set -eu
            source "$1"
            docker() {
                case "$2" in
                    ls) printf '%s' "$IPC_FIXTURE_ID" ;;
                    inspect) printf '%s' "$IPC_FIXTURE_MODE" ;;
                    *) return 99 ;;
                esac
            }
            sim_ipc_args=()
            atd_gazebo_ipc_args false
            printf '%s' "${sim_ipc_args[@]}"
        """
        for container_id, mode, expected in [
            ("", "", "--ipc=host"), ("abc", "host host", "--ipc=host"),
            ("abc", "host shareable", "--ipc=container:abc"),
            ("abc", "host private", None), ("abc", "bridge shareable", None),
        ]:
            with self.subTest(container_id=container_id, mode=mode):
                result = subprocess.run(
                    ["bash", "-c", shell, "test", str(ROOT / "scripts/lib/gazebo_ipc.sh")],
                    env={**os.environ, "IPC_FIXTURE_ID": container_id, "IPC_FIXTURE_MODE": mode},
                    capture_output=True, text=True)
                if expected is None:
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("atd_pc_dev", result.stderr)
                else:
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(result.stdout, expected)

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
        self.assertIn("--shm-size=128m", result.stdout)
        self.assertIn("FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/atd/docker/gazebo_monza/fastdds.xml", result.stdout)
        self.assertNotIn("atd_pc_dev", result.stdout)

    def test_diagnostics_can_reproduce_original_transport_without_shared_memory(self):
        command = ["bash", str(ROOT / "scripts/gazebo.sh"), "diagnose", "--duration", "15", "--dry-run", "--dds"]
        result = subprocess.run([*command, "udp"], text=True, capture_output=True, check=True)
        self.assertIn("--mode diagnose", result.stdout)
        self.assertIn("--duration 15", result.stdout)
        self.assertIn("docker/middleware_profiles/fastdds.xml", result.stdout)
        self.assertNotIn("--shm-size", result.stdout)
        for value in ([], ["invalid"]):
            self.assertNotEqual(subprocess.run([*command, *value], capture_output=True).returncode, 0)

    def test_continuous_stream_has_no_duration_or_benchmark(self):
        command = ["bash", str(ROOT / "scripts/gazebo.sh"), "serve", "--dds", "udp", "--dry-run"]
        result = subprocess.run([*command, "--wait-for-stereo"], check=True, capture_output=True, text=True)
        self.assertIn("--mode serve", result.stdout)
        self.assertIn("--wait-for-stereo", result.stdout)
        self.assertNotIn("--mode benchmark", result.stdout)
        self.assertNotIn("--duration", result.stdout)
        self.assertNotEqual(subprocess.run([*command, "--duration", "5"], capture_output=True).returncode, 0)
        self.assertNotEqual(subprocess.run([*command[:2], "benchmark", "--wait-for-stereo", "--dry-run"],
                                           capture_output=True).returncode, 0)

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

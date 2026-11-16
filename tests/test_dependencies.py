"""起動時依存導入の契約。apt / dpkg は偽物で、ホストへは導入しない。"""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "docker" / "scripts" / "install-dependencies.sh"
MANIFEST = PROJECT / "docker" / "dependencies.apt"

FAKE_COMMAND = r'''
import json
import os
from pathlib import Path
import sys

command = Path(sys.argv[0]).name
args = sys.argv[1:]
if command == "id":
    print(os.environ.get("ATD_TEST_UID", "0"))
    raise SystemExit(0)
if command == "dpkg-query":
    installed = set(Path(os.environ["ATD_TEST_INSTALLED"]).read_text().splitlines())
    if args[-1] in installed:
        print("install ok installed", end="")
        raise SystemExit(0)
    raise SystemExit(1)
if command != "apt-get":
    raise SystemExit("Unexpected command")
with open(os.environ["ATD_TEST_LOG"], "a") as output:
    output.write(json.dumps(args) + "\n")
if os.environ.get("ATD_TEST_FAIL") == args[0]:
    raise SystemExit(42)
if args[0] == "install":
    with open(os.environ["ATD_TEST_INSTALLED"], "a") as output:
        for package in args[1:]:
            if not package.startswith("-"):
                output.write(package + "\n")
'''


class DependencyScriptTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="atd-deps-test-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.manifest = self.root / "dependencies.apt"
        self.manifest.write_text("# Example dependencies\ncmake\nfzf\nros-lyrical-joy\n")
        self.installed = self.root / "installed.txt"
        self.installed.write_text("")
        self.log = self.root / "apt.jsonl"
        for name in ("id", "apt-get", "dpkg-query"):
            executable = self.bin / name
            executable.write_text(f"#!{sys.executable}\n" + FAKE_COMMAND)
            executable.chmod(0o755)
        self.env = {
            "PATH": os.pathsep.join((str(self.bin), str(Path(sys.executable).parent), os.defpath)),
            "HOME": str(self.root),
            "ROS_DISTRO": "lyrical",
            "ATD_TEST_INSTALLED": str(self.installed),
            "ATD_TEST_LOG": str(self.log),
        }

    def invoke(self, env=None, manifest=None):
        return subprocess.run(
            ["/bin/bash", str(SCRIPT), str(manifest or self.manifest)],
            env={**self.env, **(env or {})}, text=True, capture_output=True, timeout=10,
        )

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def test_complete_environment_does_not_access_apt(self):
        self.installed.write_text("cmake\nfzf\nros-lyrical-joy\n")
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls(), [])

    def test_only_missing_packages_are_installed_and_second_start_skips_apt(self):
        self.installed.write_text("cmake\n")
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls(), [
            ["update"], ["install", "-y", "--no-install-recommends", "fzf", "ros-lyrical-joy"],
        ])
        self.log.unlink()
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls(), [])

    def test_apt_failure_stops_startup_and_retries_on_next_invocation(self):
        for stage in ("update", "install"):
            with self.subTest(stage=stage):
                self.log.unlink(missing_ok=True)
                result = self.invoke({"ATD_TEST_FAIL": stage})
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("導入完了", result.stdout)
                self.assertEqual(self.calls()[0], ["update"])
                self.assertEqual(len(self.calls()), 1 if stage == "update" else 2)
        self.log.unlink()
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(self.calls()), 2)

    def test_wrong_ros_distribution_and_non_root_do_not_install(self):
        for override in ({"ROS_DISTRO": "jazzy"}, {"ATD_TEST_UID": "1000"}):
            with self.subTest(override=override):
                result = self.invoke(override)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.calls(), [])

    def test_invalid_or_disallowed_manifest_fails_before_installation(self):
        for text in ("", "# no packages\n", "cmake\nfzf extra\n", "--allow-unauthenticated\n",
                     "ros-lyrical-event-camera-msgs\n", "openeb\n", "metavision-sdk\n"):
            with self.subTest(manifest=text):
                self.manifest.write_text(text)
                result = self.invoke()
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.calls(), [])

    def test_project_manifest_is_accepted_and_contains_core_tools(self):
        packages = [line.split("#", 1)[0].strip() for line in MANIFEST.read_text().splitlines()]
        packages = [package for package in packages if package]
        self.installed.write_text("\n".join(packages) + "\n")
        result = self.invoke(manifest=MANIFEST)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls(), [])
        for package in ("fzf", "ros-lyrical-ament-cmake-auto", "ros-lyrical-rclcpp-components",
                        "ros-lyrical-rosbag2-storage-mcap", "ros-lyrical-joy",
                        "ros-lyrical-foxglove-bridge", "ros-lyrical-rmw-fastrtps-cpp"):
            self.assertIn(package, packages)


if __name__ == "__main__":
    unittest.main()

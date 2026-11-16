"""ホスト準備を一時 OS 情報と偽コマンドで検証し、実 sudo / apt は実行しない。"""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "scripts/setup/host.sh"

FAKE = r'''
import json, os, pathlib, shutil, sys
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
root = pathlib.Path(os.environ["ATD_TEST_DIR"])
with (root / "calls").open("a") as stream:
    stream.write(json.dumps([name, *args]) + "\n")
if name == "sudo":
    os.environ["ATD_TEST_PRIVILEGED"] = "1"
    os.execvp(args[0], args)
elif name == "uname":
    print("Linux" if args == ["-s"] else os.environ.get("ATD_TEST_ARCH", "x86_64"))
elif name == "id":
    print("1000" if args == ["-u"] else "tester" if args == ["-un"] else "tester docker input plugdev dialout")
elif name == "getent":
    print(args[-1] + ":x:999:")
elif name == "dpkg-query":
    if args[-1] in os.environ.get("ATD_TEST_MISSING", "").split(","):
        sys.exit(1)
    print("install ok installed")
elif name == "dpkg":
    print("arm64" if os.environ.get("ATD_TEST_ARCH") == "aarch64" else "amd64")
elif name == "apt-get":
    if args[0] == os.environ.get("ATD_TEST_APT_FAIL"):
        sys.exit(9)
    for package, executable in (("isaac-ros-cli", "isaac-ros"), ("nvidia-container-toolkit", "nvidia-ctk"), ("docker-ce-cli", "docker")):
        if args[0] == "install" and package in args:
            target = root / "bin" / executable
            target.write_text(pathlib.Path(__file__).read_text())
            target.chmod(0o755)
elif name == "curl":
    value = "official rule or key\n"
    if "-o" in args:
        pathlib.Path(args[args.index("-o") + 1]).write_text(value)
    else:
        print("deb https://nvidia.github.io/libnvidia-container/stable/deb/$(ARCH) /")
elif name == "gpg":
    pathlib.Path(args[args.index("-o") + 1]).write_text("dearmored key")
elif name == "install":
    source, target = map(pathlib.Path, args[-2:])
    if not target.is_relative_to(root):
        sys.exit("test refused write outside fixture")
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
elif name == "docker":
    if args[0] == "info":
        if os.environ.get("ATD_TEST_INFO_NEEDS_SUDO") and not os.environ.get("ATD_TEST_PRIVILEGED"):
            sys.exit("permission denied")
        print('{"nvidia":{}}' if os.environ.get("ATD_TEST_RUNTIME") == "registered" else '{"runc":{}}')
    elif args[:2] == ["ps", "-q"]:
        print(os.environ.get("ATD_TEST_RUNNING", ""))
elif name == "isaac-ros":
    if args[0] == "status":
        if os.environ.get("ATD_TEST_STATUS_FAIL"):
            sys.exit(8)
        print(json.dumps({"mode": os.environ.get("ATD_TEST_CLI_MODE", "docker")}))
elif name == "systemctl":
    if args[0] == "is-active":
        sys.exit(0 if os.environ.get("ATD_TEST_SERVICE_ACTIVE") else 3)
'''


class HostSetupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="atd-host-setup-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.os_file = self.root / "etc/os-release"
        self.os_file.parent.mkdir()
        self.os_file.write_text('ID=ubuntu\nVERSION_ID="24.04"\n')
        self.log = self.root / "calls"
        for name in ("sudo", "uname", "id", "getent", "dpkg-query", "dpkg", "apt-get", "curl", "gpg", "install", "docker", "nvidia-ctk", "isaac-ros", "systemctl", "usermod", "udevadm"):
            executable = self.bin / name
            executable.write_text(f"#!{sys.executable}\n" + FAKE)
            executable.chmod(0o755)
        (self.bin / "python3").symlink_to(sys.executable)
        self.env = {
            "PATH": str(self.bin) + os.pathsep + os.defpath,
            "HOME": str(self.root),
            "ATD_SETUP_ROOT": str(self.root),
            "ATD_TEST_DIR": str(self.root),
        }

    def invoke(self, action, host="pc", dry=False, env=None):
        args = ["/bin/bash", str(HOST), "--host", host, "--action", action]
        if dry:
            args.append("--dry-run")
        return subprocess.run(args, env={**self.env, **(env or {})}, text=True,
                              capture_output=True, timeout=15)

    def calls(self, command=None):
        calls = [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []
        return [call for call in calls if command is None or call[0] == command]

    def reset(self):
        self.log.unlink(missing_ok=True)

    def test_tools_skip_apt_when_complete_and_only_install_missing(self):
        result = self.invoke("tools")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls("apt-get"), [])
        self.reset()
        result = self.invoke("tools", env={"ATD_TEST_MISSING": "git-lfs,fzf,rsync"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls("apt-get"), [
            ["apt-get", "update"],
            ["apt-get", "install", "-y", "--no-install-recommends", "git-lfs", "fzf", "rsync"],
        ])

    def test_dry_run_does_not_call_sudo_network_or_write_configuration(self):
        for action in ("tools", "docker", "runtime", "isaac", "devices"):
            with self.subTest(action=action):
                self.reset()
                result = self.invoke(action, env={"ATD_TEST_MISSING": "fzf"}, dry=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                for command in ("sudo", "apt-get", "curl", "gpg", "install", "systemctl", "usermod", "udevadm"):
                    self.assertEqual(self.calls(command), [], command)
        self.assertFalse((self.root / "etc/apt").exists())
        self.assertFalse((self.root / "etc/udev").exists())

    def test_fresh_dry_run_does_not_bootstrap_keys_packages_or_cli(self):
        conflicts = "docker.io,docker-compose,docker-compose-v2,docker-doc,docker-buildx,podman-docker,containerd,runc"
        missing = conflicts + ",docker-ce,docker-ce-cli,containerd.io,docker-buildx-plugin,docker-compose-plugin,nvidia-container-toolkit,isaac-ros-cli"
        for action, executable in (("docker", "docker"), ("runtime", "nvidia-ctk"), ("isaac", "isaac-ros")):
            with self.subTest(action=action):
                self.reset()
                source = (self.bin / executable).read_text()
                (self.bin / executable).unlink()
                result = self.invoke(action, dry=True, env={"ATD_TEST_MISSING": missing})
                (self.bin / executable).write_text(source)
                (self.bin / executable).chmod(0o755)
                self.assertEqual(result.returncode, 0, result.stderr)
                for command in ("sudo", "curl", "gpg", "apt-get", "install", "systemctl", "usermod"):
                    self.assertEqual(self.calls(command), [])
        self.assertFalse((self.root / "etc/apt").exists())

    def test_check_reports_unsupported_os_without_mutation(self):
        self.os_file.write_text('ID=other\nVERSION_ID="1"\n')
        result = self.invoke("check")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("[要確認]", result.stdout)
        self.assertEqual(self.calls("sudo"), [])
        self.assertEqual(self.calls("apt-get"), [])

    def test_wrong_os_and_architecture_refuse_mutation(self):
        self.os_file.write_text('ID=ubuntu\nVERSION_ID="22.04"\n')
        result = self.invoke("tools")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Ubuntu 24.04", result.stderr)
        self.assertEqual(self.calls("sudo"), [])
        self.os_file.write_text('ID=ubuntu\nVERSION_ID="24.04"\n')
        result = self.invoke("tools", host="jetson")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("aarch64", result.stderr)

    def test_apt_failure_stops_before_install(self):
        result = self.invoke("tools", env={"ATD_TEST_MISSING": "fzf", "ATD_TEST_APT_FAIL": "update"})
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls("apt-get"), [["apt-get", "update"]])

    def test_existing_docker_is_not_reinstalled(self):
        result = self.invoke("docker")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls("apt-get"), [])
        self.assertEqual(self.calls("curl"), [])
        self.assertEqual(self.calls("systemctl"), [])

    def test_new_docker_uses_official_repository_without_removing_packages(self):
        (self.bin / "docker").unlink()
        conflicts = "docker.io,docker-compose,docker-compose-v2,docker-doc,docker-buildx,podman-docker,containerd,runc"
        missing = conflicts + ",docker-ce,docker-ce-cli,containerd.io,docker-buildx-plugin,docker-compose-plugin"
        result = self.invoke("docker", env={"ATD_TEST_MISSING": missing})
        self.assertEqual(result.returncode, 0, result.stderr)
        source = (self.root / "etc/apt/sources.list.d/atd-docker.sources").read_text()
        self.assertIn("https://download.docker.com/linux/ubuntu", source)
        self.assertIn("Suites: noble", source)
        self.assertFalse(any("remove" in call for call in self.calls("apt-get")))

    def test_existing_docker_repository_is_not_duplicated(self):
        (self.bin / "docker").unlink()
        source = self.root / "etc/apt/sources.list.d/docker.sources"
        source.parent.mkdir(parents=True)
        source.write_text("Types: deb\nURIs: https://download.docker.com/linux/ubuntu\nSuites: noble\n")
        conflicts = "docker.io,docker-compose,docker-compose-v2,docker-doc,docker-buildx,podman-docker,containerd,runc"
        result = self.invoke("docker", env={"ATD_TEST_MISSING": conflicts})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls("curl"), [])
        self.assertFalse((source.parent / "atd-docker.sources").exists())

    def test_runtime_preserves_registered_runtime_and_refuses_running_containers(self):
        result = self.invoke("runtime", env={"ATD_TEST_RUNTIME": "registered"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls("systemctl"), [])
        self.reset()
        result = self.invoke("runtime", env={"ATD_TEST_RUNNING": "running-container"})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("稼働中", result.stderr)
        self.assertEqual(self.calls("nvidia-ctk"), [])
        self.assertEqual(self.calls("systemctl"), [])

    def test_runtime_reads_privileged_info_before_changing_registered_runtime(self):
        result = self.invoke("runtime", env={
            "ATD_TEST_INFO_NEEDS_SUDO": "1", "ATD_TEST_RUNTIME": "registered",
        })
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["sudo", "docker", "info", "--format", "{{json .Runtimes}}"], self.calls("sudo"))
        self.assertEqual(len(self.calls("docker")), 2)
        self.assertEqual(self.calls("nvidia-ctk"), [])
        self.assertEqual(self.calls("systemctl"), [])
        self.reset()
        result = self.invoke("runtime", dry=True, env={"ATD_TEST_INFO_NEEDS_SUDO": "1"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls("sudo"), [])
        self.assertEqual(self.calls("systemctl"), [])

    def test_missing_toolkit_uses_stable_repository_then_configures_runtime(self):
        (self.bin / "nvidia-ctk").unlink()
        result = self.invoke("runtime", env={"ATD_TEST_MISSING": "nvidia-container-toolkit"})
        self.assertEqual(result.returncode, 0, result.stderr)
        source = (self.root / "etc/apt/sources.list.d/atd-nvidia-container-toolkit.list").read_text()
        self.assertIn("stable/deb", source)
        self.assertNotIn("experimental", source)
        self.assertIn(["nvidia-ctk", "runtime", "configure", "--runtime=docker", "--set-as-default"], self.calls("nvidia-ctk"))
        self.assertIn(["systemctl", "restart", "docker"], self.calls("systemctl"))

    def test_cli_does_not_overwrite_existing_modes_or_status_errors(self):
        for mode in ("docker", "venv", "baremetal"):
            with self.subTest(mode=mode):
                self.reset()
                result = self.invoke("isaac", env={"ATD_TEST_CLI_MODE": mode})
                self.assertEqual(result.returncode == 0, mode == "docker", result.stderr)
                self.assertFalse(any(call[1:2] == ["init"] for call in self.calls("isaac-ros")))
        self.reset()
        result = self.invoke("isaac", env={"ATD_TEST_STATUS_FAIL": "1"})
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(call[1:2] == ["init"] for call in self.calls("isaac-ros")))
        self.reset()
        result = self.invoke("isaac", env={"ATD_TEST_CLI_MODE": "uninitialized"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["isaac-ros", "init", "docker"], self.calls("isaac-ros"))

    def test_new_cli_uses_platform_suite_and_apt(self):
        for host, arch, suite in (("pc", "x86_64", "noble"), ("jetson", "aarch64", "noble-jetpack")):
            with self.subTest(host=host):
                self.reset()
                (self.bin / "isaac-ros").unlink(missing_ok=True)
                (self.root / "etc/apt/sources.list.d/atd-isaac-ros.list").unlink(missing_ok=True)
                result = self.invoke("isaac", host=host, env={
                    "ATD_TEST_ARCH": arch, "ATD_TEST_MISSING": "isaac-ros-cli",
                    "ATD_TEST_CLI_MODE": "uninitialized",
                })
                self.assertEqual(result.returncode, 0, result.stderr)
                source = (self.root / "etc/apt/sources.list.d/atd-isaac-ros.list").read_text()
                self.assertIn(f"release-5.0 {suite} main", source)
                self.assertIn(["isaac-ros", "init", "docker"], self.calls("isaac-ros"))

    def test_devices_install_fixed_udev_and_preserve_existing_rules(self):
        result = self.invoke("devices")
        self.assertEqual(result.returncode, 0, result.stderr)
        rule = self.root / "etc/udev/rules.d/99-realsense-libusb.rules"
        self.assertTrue(rule.exists())
        self.assertTrue(any("v2.56.3" in arg for call in self.calls("curl") for arg in call))
        self.assertIn(["udevadm", "control", "--reload-rules"], self.calls("udevadm"))
        rule.write_text("existing rules")
        self.reset()
        result = self.invoke("devices")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(rule.read_text(), "existing rules")
        self.assertEqual(self.calls("curl"), [])
        self.assertEqual(self.calls("udevadm"), [])


if __name__ == "__main__":
    unittest.main()

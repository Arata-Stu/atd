"""初回セットアップの操作契約を、temp project と偽コマンドで確認する。"""

import json
import os
from pathlib import Path
import pty
import shlex
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]
WORKSPACE_CONFIGS = (
    "docker/isaac_ros/config.yaml",
    "docker/isaac_ros/common-config.sh",
    "docker/isaac_ros/build_image_layers.yaml",
)


RECORDER = r'''
import json
import os
import sys

tool, args = sys.argv[1], sys.argv[2:]
entry = {"tool": tool, "args": args}
if tool == "dev":
    entry["workspace"] = os.environ.get("ATD_ROS_WS")
with open(os.environ["ATD_TEST_LOG"], "a") as stream:
    stream.write(json.dumps(entry) + "\n")
if tool == "host":
    action = args[args.index("--action") + 1]
    if action == os.environ.get("ATD_TEST_HOST_FAIL"):
        sys.exit(7)
elif tool == "docker":
    if args[:1] == ["info"]:
        if os.environ.get("ATD_TEST_DOCKER_FAIL") == "info":
            sys.exit(8)
        print('{"nvidia":{"path":"nvidia-container-runtime"},"runc":{"path":"runc"}}')
    elif args[:1] == ["run"]:
        if os.environ.get("ATD_TEST_DOCKER_FAIL") == "run":
            sys.exit(9)
    else:
        sys.exit("unexpected Docker operation")
elif tool == "isaac-ros":
    if args[:1] != ["status"]:
        sys.exit("unexpected CLI mutation")
    if os.environ.get("ATD_TEST_CLI_FAIL"):
        sys.exit(10)
    print(json.dumps({"mode": os.environ.get("ATD_TEST_CLI_MODE", "docker"), "activation": "inactive"}))
elif tool in ("sudo", "apt-get", "curl", "systemctl"):
    sys.exit("real host mutation is forbidden in this test")
'''


FZF = r'''
import json
import os
import sys

with open(os.environ["ATD_TEST_LOG"], "a") as stream:
    stream.write(json.dumps({"tool": "fzf", "args": sys.argv[1:]}) + "\n")
lines = sys.stdin.read().splitlines()
action = os.environ.get("ATD_TEST_FZF_ACTION", "cancel")
if action == "cancel":
    sys.exit(130)
for line in lines:
    if line.startswith(action + "\t") or line.startswith(action + " ") or line == action:
        print(line)
        break
else:
    sys.exit("requested action missing from fzf candidates: " + action)
'''


class SetupScriptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="atd-setup-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.project = self.root / "project"
        (self.project / "scripts/setup").mkdir(parents=True)
        (self.project / "config").mkdir()
        (self.project / 'ros2_ws/src').mkdir(parents=True)
        (self.project / 'ros2_ws/src/README.md').write_text('ROS package source fixture\n')
        self.script = self.project / "scripts/setup.sh"
        shutil.copyfile(PROJECT / "scripts/setup.sh", self.script)
        shutil.copytree(PROJECT / "scripts/lib", self.project / "scripts/lib")
        shutil.copyfile(PROJECT / "config/storage.env.example", self.project / "config/storage.env.example")
        shutil.copyfile(PROJECT / "config/workspace.env.example", self.project / "config/workspace.env.example")
        for relative in WORKSPACE_CONFIGS:
            destination = self.project / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(PROJECT / relative, destination)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.log = self.root / "calls.jsonl"
        self.config = self.project / "config/storage.env"
        self.home = self.root / "home"
        self.home.mkdir()
        self.workspace = self.project / "ros2_ws"
        self.workspace_config = self.project / "config/workspace.env"
        self.recorder = self.root / "record.py"
        self.recorder.write_text(RECORDER)
        for tool in ("docker", "isaac-ros", "sudo", "apt-get", "curl", "systemctl"):
            executable = self.bin / tool
            executable.write_text(f"#!{sys.executable}\n" + RECORDER.replace(
                'tool, args = sys.argv[1], sys.argv[2:]',
                f'tool, args = {tool!r}, sys.argv[1:]'))
            executable.chmod(0o755)
        (self.bin / "python3").symlink_to(sys.executable)
        # CI が root の場合も dispatcher の通常ユーザー経路を検証する。
        (self.bin / "id").write_text('#!/bin/sh\n[ "$1" = -u ] || exit 1\nprintf "1000\\n"\n')
        (self.bin / "id").chmod(0o755)
        for relative, tool in (("scripts/setup/host.sh", "host"), ("scripts/dev.sh", "dev")):
            executable = self.project / relative
            executable.write_text(
                '#!/bin/bash\nexec "$ATD_TEST_PYTHON" "$ATD_TEST_RECORDER" '
                + shlex.quote(tool) + ' "$@"\n'
            )
            executable.chmod(0o755)
        (self.project / "scripts/lib/detect_host.sh").write_text(
            'detect_atd_host() { printf "%s\\n" "${ATD_TEST_HOST:-pc}"; }\n')
        self.env = {
            "PATH": os.pathsep.join((str(self.bin), "/usr/bin", "/bin", "/usr/sbin", "/sbin")),
            "HOME": str(self.home), "USER": "atd-test",
            "ATD_SETUP_ROOT": str(self.root),
            "ATD_TEST_PYTHON": sys.executable,
            "ATD_TEST_RECORDER": str(self.recorder),
            "ATD_TEST_LOG": str(self.log),
            "ATD_STORAGE_CONFIG": str(self.config),
            "ATD_WORKSPACE_CONFIG": str(self.workspace_config),
        }

    def invoke(self, *args, env=None, input=None):
        return subprocess.run(
            ["/bin/bash", str(self.script), *args],
            env={**self.env, **(env or {})}, text=True, input=input,
            capture_output=True, timeout=15,
        )

    def calls(self, tool=None):
        if not self.log.exists():
            return []
        entries = [json.loads(line) for line in self.log.read_text().splitlines()]
        return [entry for entry in entries if tool is None or entry["tool"] == tool]

    def reset_calls(self):
        self.log.unlink(missing_ok=True)

    def action_calls(self):
        return [entry["args"][entry["args"].index("--action") + 1]
                for entry in self.calls("host")]

    def read_storage(self):
        names = ("ATD_PC_RECORD_DIR", "ATD_PC_RESULT_DIR", "ATD_JETSON_RECORD_DIR", "ATD_JETSON_RESULT_DIR")
        result = subprocess.run(
            ["/bin/bash", "-c", 'source "$1"; shift; for key in "$@"; do printf "%s\\0" "${!key-}"; done',
             "storage-test", str(self.config), *names],
            env=self.env, capture_output=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        return dict(zip(names, result.stdout.decode().split("\0")))

    def read_workspace(self):
        names = ("ATD_PC_ROS_WS", "ATD_JETSON_ROS_WS")
        result = subprocess.run(
            ["/bin/bash", "-c", 'source "$1"; shift; for key in "$@"; do printf "%s\\0" "${!key-}"; done',
             "workspace-test", str(self.workspace_config), *names],
            env=self.env, capture_output=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        return dict(zip(names, result.stdout.decode().split("\0")))

    def test_dry_run_all_never_calls_tools_or_writes_configuration(self):
        record, results = self.root / "new record", self.root / "new results"
        before_paths = set(self.project.rglob("*"))
        before_files = {path: path.read_bytes() for path in before_paths if path.is_file()}
        result = self.invoke("--action", "all", "--dry-run", "--record-dir", str(record), "--result-dir", str(results))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls(), [])
        self.assertFalse(self.config.exists())
        self.assertFalse(record.exists())
        self.assertFalse(results.exists())
        self.assertFalse((self.project / ".cache").exists())
        self.assertFalse((self.workspace / '.isaac-ros-cli').exists())
        self.assertFalse(self.workspace_config.exists())
        self.assertEqual(set(self.project.rglob("*")), before_paths)
        self.assertEqual({path: path.read_bytes() for path in before_files}, before_files)
        positions = [result.stdout.index("--- " + action + " ---") for action in
                     ("tools", "docker", "runtime", "isaac", "devices", "storage", "workspace", "verify", "build")]
        self.assertEqual(positions, sorted(positions))

    def test_dry_run_does_not_source_user_configuration(self):
        sentinel = self.root / "should-not-be-created"
        self.config.write_text('touch ' + shlex.quote(str(sentinel)) + '\n')
        before = self.config.read_bytes()
        result = self.invoke("--action", "storage", "--dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.config.read_bytes(), before)
        self.assertFalse(sentinel.exists())
        self.assertEqual(self.calls(), [])

    def test_auto_host_and_explicit_override_reach_host_helper(self):
        for args, detected, expected in (
            ([], "pc", "pc"), ([], "jetson", "jetson"),
            (["--host", "pc"], "jetson", "pc"), (["--host", "jetson"], "pc", "jetson"),
        ):
            with self.subTest(args=args, detected=detected):
                self.reset_calls()
                result = self.invoke("--action", "tools", *args, env={"ATD_TEST_HOST": detected})
                self.assertEqual(result.returncode, 0, result.stderr)
                call, = self.calls("host")
                self.assertEqual(call["args"], ["--host", expected, "--action", "tools"])

    def test_notebook_alias_normalizes_to_pc(self):
        result = self.invoke("--host", "notebook", "--action", "tools",
                             env={"ATD_TEST_HOST": "jetson"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls("host")[0]["args"], ["--host", "pc", "--action", "tools"])
        self.reset_calls()
        result = self.invoke("--host", "notebook", "--action", "build")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls("dev")[0]["args"], ["--host", "pc", "--build"])

    def test_workspace_prepares_src_and_reports_host_and_container_paths(self):
        bashrc = self.home / ".bashrc"
        bashrc.write_text("# existing host configuration\n")
        before = {relative: (self.project / relative).read_bytes() for relative in WORKSPACE_CONFIGS}
        for host in ("pc", "jetson"):
            with self.subTest(host=host):
                result = self.invoke("--host", host, "--action", "workspace")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertTrue((self.workspace / "src").is_dir())
                self.assertIn(str(self.workspace), result.stdout)
                self.assertEqual(self.read_workspace()[f"ATD_{host.upper()}_ROS_WS"], str(self.workspace))
                self.assertIn("/workspaces/isaac_ros-dev", result.stdout)
                self.assertIn("/workspaces/atd", result.stdout)
                self.assertEqual(bashrc.read_text(), "# existing host configuration\n")
                self.assertEqual({relative: (self.project / relative).read_bytes()
                                  for relative in WORKSPACE_CONFIGS}, before)
                self.assertEqual(self.calls(), [])
                self.assertFalse((self.project / 'src').exists())
                self.assertFalse((self.workspace / 'src/atd').exists())

    def test_workspace_saves_distinct_hosts_and_preserves_unrelated_settings(self):
        pc_workspace, jetson_workspace = self.root / "notebook-ws", self.root / "jetson-ws"
        self.workspace_config.write_text('ATD_CUSTOM_WORKSPACE_SETTING=keep\n')
        for host, path in (("notebook", pc_workspace), ("jetson", jetson_workspace)):
            result = self.invoke("--host", host, "--action", "workspace", "--workspace", str(path))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((path / "src").is_dir())
        values = self.read_workspace()
        self.assertEqual(values["ATD_PC_ROS_WS"], str(pc_workspace))
        self.assertEqual(values["ATD_JETSON_ROS_WS"], str(jetson_workspace))
        self.assertIn("ATD_CUSTOM_WORKSPACE_SETTING=keep", self.workspace_config.read_text())
        self.assertEqual(stat.S_IMODE(self.workspace_config.stat().st_mode), 0o600)
        first = self.workspace_config.read_bytes()
        result = self.invoke("--host", "pc", "--action", "workspace")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(str(pc_workspace), result.stdout)
        self.assertEqual(self.workspace_config.read_bytes(), first)

    def test_workspace_interactive_input_expands_tilde(self):
        master, slave = pty.openpty()
        try:
            process = subprocess.Popen(
                ["/bin/bash", str(self.script), "--host", "notebook", "--action", "workspace"],
                env=self.env, stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            )
            os.write(master, b"~/interactive-workspace\n")
            stdout, stderr = process.communicate(timeout=15)
        finally:
            os.close(slave)
            os.close(master)
        self.assertEqual(process.returncode, 0, stderr)
        expected = self.home / "interactive-workspace"
        self.assertTrue((expected / "src").is_dir())
        self.assertEqual(self.read_workspace()["ATD_PC_ROS_WS"], str(expected))
        self.assertIn(str(expected), stdout)
        self.assertIn(str(self.workspace), stderr)

    def test_workspace_can_replace_an_invalid_saved_path(self):
        self.workspace_config.write_text('ATD_PC_ROS_WS=relative/old\nATD_JETSON_ROS_WS=/srv/jetson-keep\n')
        replacement = self.root / "replacement-workspace"
        result = self.invoke("--action", "workspace", "--workspace", str(replacement))
        self.assertEqual(result.returncode, 0, result.stderr)
        values = self.read_workspace()
        self.assertEqual(values["ATD_PC_ROS_WS"], str(replacement))
        self.assertEqual(values["ATD_JETSON_ROS_WS"], "/srv/jetson-keep")

    def test_workspace_rejects_invalid_paths_before_writing(self):
        for path in ("relative/ws", str(self.root / "workspace with spaces"), str(self.root / "$unsafe")):
            with self.subTest(path=path):
                result = self.invoke("--action", "workspace", "--workspace", path)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.workspace_config.exists())
                self.assertFalse((Path(path) / "src").exists())
                self.assertEqual(self.calls(), [])

    def test_workspace_override_reaches_start_without_saving(self):
        selected = str(self.root / "temporary-workspace")
        result = self.invoke("--action", "start", "--workspace", selected)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls("dev")[0]["workspace"], selected)
        self.assertFalse(self.workspace_config.exists())

    def test_workspace_dry_run_does_not_source_local_configuration(self):
        sentinel = self.root / "workspace-should-not-be-created"
        self.workspace_config.write_text('touch ' + shlex.quote(str(sentinel)) + '\n')
        before = self.workspace_config.read_bytes()
        for action in ("check", "workspace"):
            result = self.invoke("--action", action, "--dry-run")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(str(self.workspace), result.stdout)
        self.assertEqual(self.workspace_config.read_bytes(), before)
        self.assertFalse(sentinel.exists())
        self.assertEqual(self.calls(), [])

    def test_workspace_missing_configuration_fails(self):
        for relative in WORKSPACE_CONFIGS:
            with self.subTest(missing=relative):
                path = self.project / relative
                path.unlink()
                try:
                    result = self.invoke("--action", "workspace")
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(relative, result.stdout + result.stderr)
                    self.assertEqual(self.calls(), [])
                finally:
                    shutil.copyfile(PROJECT / relative, path)

    def test_default_workspace_keeps_existing_ros_package_source(self):
        workspace = self.project / 'ros2_ws'
        package = workspace / 'src/atd_fixture/package.xml'
        package.parent.mkdir()
        package.write_text('<package format="3"><name>atd_fixture</name></package>\n')
        before = package.read_bytes()
        result = self.invoke('--action', 'workspace')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.read_workspace()['ATD_PC_ROS_WS'], str(workspace))
        self.assertEqual(package.read_bytes(), before)
        self.assertFalse((workspace / 'src/atd').exists())
        self.assertTrue((workspace / '.isaac-ros-cli/config.yaml').is_file())

    def test_workspace_dry_run_does_not_prepare_directories(self):
        before_paths = set(self.project.rglob("*"))
        result = self.invoke("--action", "workspace", "--dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(set(self.project.rglob("*")), before_paths)
        self.assertFalse((self.project / ".cache").exists())
        self.assertFalse((self.workspace / '.isaac-ros-cli').exists())
        self.assertFalse(self.workspace_config.exists())
        self.assertFalse((self.home / ".bashrc").exists())
        self.assertEqual(self.calls(), [])

    def test_build_and_start_forward_only_explicit_host(self):
        for action, extra, expected in (
            ("build", [], ["--build"]),
            ("start", [], []),
            ("build", ["--host", "jetson"], ["--host", "jetson", "--build"]),
            ("start", ["--host", "pc"], ["--host", "pc"]),
        ):
            with self.subTest(action=action, extra=extra):
                self.reset_calls()
                result = self.invoke("--action", action, *extra)
                self.assertEqual(result.returncode, 0, result.stderr)
                call, = self.calls("dev")
                self.assertEqual(call["args"], expected)
                self.assertEqual(self.calls("host"), [])

    def test_storage_creates_spaced_paths_and_private_config_idempotently(self):
        record, results = self.root / "record with spaces", self.root / "results with spaces"
        result = self.invoke("--action", "storage", "--record-dir", str(record), "--result-dir", str(results))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(record.is_dir())
        self.assertTrue(results.is_dir())
        self.assertEqual(stat.S_IMODE(self.config.stat().st_mode), 0o600)
        first = self.read_storage()
        self.assertEqual(first["ATD_PC_RECORD_DIR"], str(record))
        self.assertEqual(first["ATD_PC_RESULT_DIR"], str(results))
        result = self.invoke("--action", "storage")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.read_storage(), first)
        self.assertEqual(self.calls(), [])

    def test_storage_defaults_to_the_actual_clone_and_saves_only_the_selected_host(self):
        for host in ('pc', 'jetson'):
            with self.subTest(host=host):
                self.config.unlink(missing_ok=True)
                result = self.invoke('--host', host, '--action', 'storage')
                self.assertEqual(result.returncode, 0, result.stderr)
                values = self.read_storage()
                selected = host.upper()
                other = 'JETSON' if host == 'pc' else 'PC'
                self.assertEqual(values[f'ATD_{selected}_RECORD_DIR'], str(self.project / 'data/record'))
                self.assertEqual(values[f'ATD_{selected}_RESULT_DIR'], str(self.project / 'data/results'))
                self.assertEqual(values[f'ATD_{other}_RECORD_DIR'], '')
                self.assertEqual(values[f'ATD_{other}_RESULT_DIR'], '')
                self.assertTrue((self.project / 'data/record').is_dir())
                self.assertTrue((self.project / 'data/results').is_dir())
                self.assertFalse((self.home / 'atd-data').exists())
                self.assertEqual(self.calls(), [])

    def test_storage_interactive_enter_uses_clone_defaults_and_labels_their_purpose(self):
        master, slave = pty.openpty()
        try:
            process = subprocess.Popen(
                ['/bin/bash', str(self.script), '--host', 'jetson', '--action', 'storage'],
                env=self.env, stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            os.write(master, b'\n\n')
            stdout, stderr = process.communicate(timeout=15)
        finally:
            os.close(slave)
            os.close(master)
        self.assertEqual(process.returncode, 0, stdout + stderr)
        self.assertIn('record 保存先（rosbag）', stderr)
        self.assertIn('results 保存先（地図・モデル・解析）', stderr)
        self.assertIn(str(self.project / 'data/record'), stderr)
        self.assertEqual(self.read_storage()['ATD_JETSON_RESULT_DIR'], str(self.project / 'data/results'))

    def test_storage_changed_paths_do_not_move_or_delete_existing_recordings(self):
        previous_record = self.root / 'old-record'
        previous_result = self.root / 'old-results'
        previous_record.mkdir()
        previous_result.mkdir()
        bag = previous_record / 'recording.mcap'
        bag.write_bytes(b'preserved recording fixture')
        self.config.write_text(f'ATD_JETSON_RECORD_DIR={previous_record}\n'
                               f'ATD_JETSON_RESULT_DIR={previous_result}\n')
        result = self.invoke('--host', 'jetson', '--action', 'storage')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.read_storage()['ATD_JETSON_RECORD_DIR'], str(previous_record))
        new_record, new_results = self.project / 'data/record', self.project / 'data/results'
        result = self.invoke('--host', 'jetson', '--action', 'storage',
                             '--record-dir', str(new_record), '--result-dir', str(new_results))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.read_storage()['ATD_JETSON_RECORD_DIR'], str(new_record))
        self.assertEqual(bag.read_bytes(), b'preserved recording fixture')
        self.assertFalse((new_record / bag.name).exists())

    def test_storage_preserves_other_host_and_unrelated_settings(self):
        existing = self.root / "existing PC record"
        existing_results = self.root / "existing PC results"
        self.config.write_text(
            f'ATD_PC_RECORD_DIR={shlex.quote(str(existing))}\n'
            f'ATD_PC_RESULT_DIR={shlex.quote(str(existing_results))}\n'
            'ATD_JETSON_RECORD_DIR=/srv/jetson/record\n'
            'ATD_JETSON_RESULT_DIR=/srv/jetson/results\n'
            'ATD_CUSTOM_SETTING=keep\n')
        replacement = self.root / "replacement results"
        result = self.invoke("--action", "storage", "--result-dir", str(replacement))
        self.assertEqual(result.returncode, 0, result.stderr)
        values = self.read_storage()
        self.assertEqual(values["ATD_PC_RECORD_DIR"], str(existing))
        self.assertEqual(values["ATD_PC_RESULT_DIR"], str(replacement))
        self.assertEqual(values["ATD_JETSON_RECORD_DIR"], "/srv/jetson/record")
        self.assertEqual(values["ATD_JETSON_RESULT_DIR"], "/srv/jetson/results")
        self.assertIn("ATD_CUSTOM_SETTING=keep", self.config.read_text())

    def test_storage_rejects_unsafe_paths_without_creating_anything(self):
        for value in ("relative/path", str(self.root / "$bad"), str(self.root / "bad,comma"),
                      str(self.root / 'bad"quote'), str(self.root / "bad\nline")):
            with self.subTest(path=value):
                result = self.invoke("--action", "storage", "--record-dir", value,
                                     "--result-dir", str(self.root / "valid-results"))
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.config.exists())
                self.assertFalse((self.root / "valid-results").exists())
                self.assertEqual(self.calls(), [])

    def test_same_and_parent_child_storage_paths_are_allowed(self):
        record = self.root / "shared"
        for results in (record, record / "results"):
            with self.subTest(results=results):
                result = self.invoke("--action", "storage", "--record-dir", str(record), "--result-dir", str(results))
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(self.read_storage()["ATD_PC_RESULT_DIR"], str(results))

    def test_all_stops_after_failed_host_step(self):
        result = self.invoke("--action", "all", env={"ATD_TEST_HOST_FAIL": "runtime"})
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.action_calls(), ["tools", "docker", "runtime"])
        self.assertEqual(self.calls("dev"), [])
        self.assertEqual(self.calls("docker"), [])
        self.assertFalse(self.config.exists())

    def test_all_completes_setup_before_build(self):
        result = self.invoke("--action", "all")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.action_calls(), ["tools", "docker", "runtime", "isaac", "devices"])
        self.assertTrue(self.config.exists())
        self.assertTrue((self.workspace / "src").is_dir())
        self.assertEqual([call["tool"] for call in self.calls()],
                         ["host"] * 5 + ["docker", "isaac-ros", "docker", "dev"])
        self.assertEqual(self.calls("dev")[0]["args"], ["--build"])

    def test_all_stops_before_verify_and_build_when_workspace_is_incomplete(self):
        (self.project / WORKSPACE_CONFIGS[0]).unlink()
        result = self.invoke("--action", "all")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.action_calls(), ["tools", "docker", "runtime", "isaac", "devices"])
        self.assertEqual(self.calls("docker"), [])
        self.assertEqual(self.calls("isaac-ros"), [])
        self.assertEqual(self.calls("dev"), [])

    def test_verify_requires_docker_and_docker_cli_mode_before_gpu_run(self):
        cases = (
            ({"ATD_TEST_DOCKER_FAIL": "info"}, False),
            ({"ATD_TEST_CLI_FAIL": "1"}, True),
            ({"ATD_TEST_CLI_MODE": "venv"}, True),
            ({"ATD_TEST_CLI_MODE": "baremetal"}, True),
        )
        for env, cli_called in cases:
            with self.subTest(env=env):
                self.reset_calls()
                result = self.invoke("--action", "verify", env=env)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(any(call["args"][0] == "run" for call in self.calls("docker")))
                self.assertEqual(bool(self.calls("isaac-ros")), cli_called)
                self.assertEqual(self.calls("host"), [])

    def test_verify_stops_before_gpu_probe_for_kart_launcher(self):
        launcher = self.root / "usr/lib/isaac-ros-cli/run_dev.py"
        launcher.parent.mkdir(parents=True)
        launcher.write_text('raise ValueError("ISAAC_ROS_WS must be <kart project>/ros2_ws beside packages.repos")\n')
        result = self.invoke("--action", "verify")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("kart", result.stderr)
        self.assertEqual(self.calls("docker"), [])

    def test_verify_runs_only_gpu_runtime_probe(self):
        result = self.invoke("--action", "verify")
        self.assertEqual(result.returncode, 0, result.stderr)
        runs = [call["args"] for call in self.calls("docker") if call["args"][0] == "run"]
        self.assertEqual(runs, [["run", "--rm", "--gpus", "all", "ubuntu:24.04", "/bin/true"]])
        self.assertEqual(self.calls("dev"), [])

    def test_verify_propagates_gpu_probe_failure(self):
        result = self.invoke("--action", "verify", env={"ATD_TEST_DOCKER_FAIL": "run"})
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(len(self.calls("docker")), 2)
        self.assertEqual(self.calls("dev"), [])

    def test_fzf_cancel_performs_no_action(self):
        fzf = self.bin / "fzf"
        fzf.write_text(f"#!{sys.executable}\n" + FZF)
        fzf.chmod(0o755)
        master, slave = pty.openpty()
        try:
            result = subprocess.run(
                ["/bin/bash", str(self.script), "--action", "menu"], env=self.env,
                stdin=slave, stdout=slave, stderr=slave, timeout=15,
            )
        finally:
            os.close(slave)
            os.close(master)
        self.assertEqual(result.returncode, 0)
        self.assertEqual([call["tool"] for call in self.calls()], ["fzf"])
        self.assertFalse(self.config.exists())


if __name__ == "__main__":
    unittest.main()

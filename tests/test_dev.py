"""起動ラッパーの契約を検証する。CLI / Docker は一時ディレクトリの偽物を使う。"""

import json
import os
from pathlib import Path
import shlex
import shutil
import socket
import subprocess
import sys
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]
DEV_SCRIPT = PROJECT / "scripts" / "dev.sh"
HOST_DETECTOR = PROJECT / "scripts" / "detect_host.sh"


FAKE_CLI = r'''
import json
import os
from pathlib import Path
import shlex
import sys

entry = {"tool": "isaac-ros", "args": sys.argv[1:]}
if sys.argv[1:2] == ["activate"]:
    entry["record"] = os.environ["ATD_RECORD_DIR"]
    entry["results"] = os.environ["ATD_RESULT_DIR"]
    entry["workspace"] = os.environ["ISAAC_ROS_WS"]
    entry["project"] = os.environ["ATD_PROJECT_DIR"]
    entry["runtime"] = os.environ["ATD_RUNTIME_REV"]
    generated = Path(os.environ["ATD_GENERATED_DOCKER_DIR"])
    entry["generated_directory"] = str(generated)
    entry['docker_args_file'] = os.environ['DOCKER_ARGS_FILE']
    entry["generated_dockerfile"] = (generated / "Dockerfile.atd").read_text()
    entry["generated_context"] = str((generated / "../../../docker").resolve())
    # 上流 loader と同じ変数展開を行い、shell=True に渡る引数を字句解析する。
    entry["mount_args"] = [
        shlex.split(os.path.expandvars(line))
        for line in Path(os.environ["DOCKER_ARGS_FILE"]).read_text().splitlines()
    ]
with open(os.environ["ATD_TEST_LOG"], "a") as log:
    log.write(json.dumps(entry) + "\n")
if sys.argv[1:] == ["status", "--output", "json"]:
    print(json.dumps({"mode": os.environ.get("ATD_TEST_CLI_MODE", "docker")}))
elif sys.argv[1:2] != ["activate"]:
    sys.exit(2)
'''


FAKE_DOCKER = r'''
import json
import os
import sys

with open(os.environ["ATD_TEST_LOG"], "a") as log:
    log.write(json.dumps({"tool": "docker", "args": sys.argv[1:]}) + "\n")
if sys.argv[1:2] == ["exec"]:
    if sys.argv[-2:] == ["cat", "/etc/atd/runtime.ready"]:
        ready = os.environ.get("ATD_TEST_READY", "matching")
        if ready == "missing":
            sys.exit(1)
        print(os.environ["ATD_RUNTIME_REV"] if ready == "matching" else "outdated")
    elif sys.argv[-1:] != ["/usr/local/bin/atd-shell"]:
        sys.exit("想定外の Docker exec")
    sys.exit(0)
if sys.argv[1:3] != ["container", "inspect"]:
    sys.exit("Docker 実起動は禁止")
state = os.environ.get("ATD_TEST_CONTAINER", "missing")
if state == "missing":
    sys.exit(1)
expected = [
    ("/workspaces/isaac_ros-dev", os.environ["ISAAC_ROS_WS"], True),
    ("/workspaces/atd", os.environ["ATD_PROJECT_DIR"], False),
    ("/data/record", os.environ["ATD_RECORD_DIR"], True),
    ("/data/results", os.environ["ATD_RESULT_DIR"], True),
]
source_root = os.path.join(os.environ['ATD_PROJECT_DIR'], 'ros2_ws', 'src')
native = os.path.realpath(os.path.join(os.environ['ISAAC_ROS_WS'], 'src')) == os.path.realpath(source_root)
if not native or state == 'stale_nested_source':
    expected.append(('/workspaces/isaac_ros-dev/src/atd', source_root, True))
if os.environ.get('ATD_HOST_SSH_DIR') and state != 'missing_ssh':
    expected.append(('/home/admin/.ssh', os.environ['ATD_HOST_SSH_DIR'], False))
if os.environ.get('ATD_HOST_SSH_AGENT') and state != 'missing_agent':
    expected.append(('/run/atd/ssh-agent', os.environ['ATD_HOST_SSH_AGENT'], False))
mounts = [
    {"Destination": dst, "Source": src, "RW": writable, "Type": "bind"}
    for dst, src, writable in expected
]
if state == "different_path":
    mounts[2]["Source"] += "-old"
if state == "different_access":
    mounts[2]["RW"] = not mounts[2]["RW"]
if state == "different_source_path":
    mounts[4]["Source"] += "-old"
if state == "different_source_access":
    mounts[4]["RW"] = False
if state == "missing_source":
    mounts.pop()
environment = {
    "ATD_RUNTIME_API": "1",
    "ATD_RUNTIME_REV": os.environ["ATD_RUNTIME_REV"],
    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    "FASTDDS_DEFAULT_PROFILES_FILE": "/etc/atd/fastdds.xml",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "SYSTEM_DEFAULT",
    "ROS_STATIC_PEERS": "",
    "RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
}
if os.environ.get('ATD_HOST_SSH_DIR'):
    environment['ATD_HOST_SSH_SHARED'] = '1'
if os.environ.get('ATD_HOST_SSH_AGENT'):
    environment.update(SSH_AUTH_SOCK='/run/atd/ssh-agent', ATD_HOST_SSH_AGENT_SHARED='1')
if state == "different_dds":
    environment["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
if state == "different_runtime":
    environment["ATD_RUNTIME_REV"] = "outdated"
if state == "legacy":
    environment.clear()
print(json.dumps([{
    "State": {"Running": state != "stopped"}, "Mounts": mounts,
    "Config": {"Env": [key + "=" + value for key, value in environment.items()]},
}]))
'''


class DevScriptTests(unittest.TestCase):
    def test_optional_host_ssh_is_readonly_and_required_on_reconnect(self):
        ssh = self.root / '.ssh'
        ssh.mkdir(mode=0o700)
        (ssh / 'known_hosts').write_text('test-host-key\n')
        result = self.invoke('pc')
        self.assertEqual(result.returncode, 0, result.stderr)
        activation, = self.activations()
        self.assertIn(['--mount', f'type=bind,src={ssh},dst=/home/admin/.ssh,readonly'], activation['mount_args'])
        self.assertEqual((ssh / 'known_hosts').read_text(), 'test-host-key\n')
        self.reset_calls()
        result = self.invoke('pc', env={'ATD_TEST_CONTAINER': 'running'})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.activations(), [])
        self.reset_calls()
        result = self.invoke('pc', env={'ATD_TEST_CONTAINER': 'missing_ssh'})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('SSH共有 mount', result.stderr)
        self.assertEqual(self.activations(), [])

    def test_optional_pc_ssh_agent_is_forwarded_and_missing_mount_refused(self):
        with tempfile.TemporaryDirectory(prefix='atd-agent-', dir='/tmp') as directory:
            agent_path = str(Path(directory) / 'agent.sock')
            agent = socket.socket(socket.AF_UNIX)
            self.addCleanup(agent.close)
            agent.bind(agent_path)
            result = self.invoke('pc', env={'SSH_AUTH_SOCK': agent_path})
            self.assertEqual(result.returncode, 0, result.stderr)
            activation, = self.activations()
            self.assertIn(['--mount', f'type=bind,src={agent_path},dst=/run/atd/ssh-agent,readonly'], activation['mount_args'])
            self.assertIn(['-e', 'SSH_AUTH_SOCK=/run/atd/ssh-agent'], activation['mount_args'])
            self.reset_calls()
            result = self.invoke('pc', env={'SSH_AUTH_SOCK': agent_path, 'ATD_TEST_CONTAINER': 'missing_agent'})
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(self.activations(), [])

    def test_existing_matching_ssh_mount_is_reused_and_writable_one_refused(self):
        ssh = self.root / '.ssh'
        ssh.mkdir()
        additional = self.root / '.isaac_ros_dev-dockerargs'
        additional.write_text('--mount=type=bind,src="${HOME}/.ssh",dst=/home/admin/.ssh,readonly\n')
        result = self.invoke('pc')
        self.assertEqual(result.returncode, 0, result.stderr)
        activation, = self.activations()
        self.assertFalse(any('dst=/home/admin/.ssh' in ' '.join(v) for v in activation['mount_args']))
        additional.write_text('--mount type=bind,src="${HOME}/.ssh",dst=/home/admin/.ssh\n')
        self.reset_calls()
        result = self.invoke('pc')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('SSH mount', result.stderr)
        self.assertEqual(self.activations(), [])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="atd-dev-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.log = self.root / "calls.jsonl"
        self.config = self.root / "storage.env"
        self.paths = {}
        for host in ("pc", "jetson"):
            for kind in ("record", "results"):
                path = self.root / host / (kind + " with spaces")
                path.mkdir(parents=True)
                self.paths[(host, kind)] = path
        self.config.write_text("\n".join(
            f"ATD_{host.upper()}_{'RESULT' if kind == 'results' else 'RECORD'}_DIR={shlex.quote(str(path))}"
            for (host, kind), path in self.paths.items()
        ) + "\n")
        self.workspace_config = self.root / 'workspace.env'
        self.workspaces = {host: self.root / host / 'ws' for host in ('pc', 'jetson')}
        for workspace in self.workspaces.values():
            workspace.mkdir()
        self.workspace_config.write_text(''.join(
            f'ATD_{host.upper()}_ROS_WS={shlex.quote(str(path))}\n'
            for host, path in self.workspaces.items()))
        for name, source in (("isaac-ros", FAKE_CLI), ("docker", FAKE_DOCKER)):
            executable = self.bin / name
            executable.write_text(f"#!{sys.executable}\n" + source)
            executable.chmod(0o755)
        # ホスト環境や credentials を引き継がない。
        self.env = {
            "PATH": os.pathsep.join((str(self.bin), str(Path(sys.executable).parent), os.defpath)),
            "HOME": str(self.root),
            "ATD_STORAGE_CONFIG": str(self.config),
            "ATD_WORKSPACE_CONFIG": str(self.workspace_config),
            "ATD_TEST_LOG": str(self.log),
        }

    def invoke(self, host="pc", *extra, env=None, script=DEV_SCRIPT):
        task_env = {**self.env, **(env or {})}
        args = ["/bin/bash", str(script)]
        if host is not None:
            args.extend(("--host", host))
        args.extend(extra)
        return subprocess.run(
            args,
            env=task_env, text=True, capture_output=True, timeout=10,
        )

    def calls(self):
        if not self.log.exists():
            return []
        return [json.loads(line) for line in self.log.read_text().splitlines()]

    def activations(self):
        return [entry for entry in self.calls()
                if entry["tool"] == "isaac-ros" and entry["args"][0] == "activate"]

    def reset_calls(self):
        self.log.unlink(missing_ok=True)

    def detector_fixture(self, name, files):
        root = self.root / name
        root.mkdir()
        for relative, content in files:
            marker = root / relative
            marker.parent.mkdir(parents=True, exist_ok=True)
            marker.write_bytes(content)
        return root

    def auto_project(self, name, fixture):
        project = self.root / name
        (project / "scripts").mkdir(parents=True)
        shutil.copytree(PROJECT / "docker", project / "docker")
        (project / "ros2_ws/src").mkdir(parents=True)
        (project / 'ros2_ws/src/README.md').write_text('ROS package source fixture\n')
        shutil.copytree(PROJECT / 'scripts/lib', project / 'scripts/lib')
        script = project / "scripts" / "dev.sh"
        script.write_text(DEV_SCRIPT.read_text())
        # 本体には test hook を入れず、コピー上で実 detector の root 引数を差し替える。
        (project / "scripts" / "detect_host.sh").write_text(
            "detect_atd_host() {\n"
            "  /bin/bash -c 'source \"$1\"; detect_atd_host \"$2\"' atd-test "
            + shlex.quote(str(HOST_DETECTOR)) + " " + shlex.quote(str(fixture)) + "\n}\n")
        return script

    def test_detector_uses_jetson_markers_and_not_arm_architecture(self):
        # ARM Linux でも Jetson 固有の marker がなければ PC として扱う。
        uname = self.bin / "uname"
        uname.write_text("#!/bin/sh\nprintf 'aarch64\\n'\n")
        uname.chmod(0o755)
        cases = (
            ("no-markers", [], "pc"),
            ("generic-arm", [
                ("proc/device-tree/model", b"Raspberry Pi 5 Model B\0"),
                ("proc/device-tree/compatible", b"raspberrypi,5-model-b\0brcm,bcm2712\0"),
            ], "pc"),
            ("proc-model", [("proc/device-tree/model", b"NVIDIA Jetson Orin Nano Developer Kit\0")], "jetson"),
            ("sys-model", [("sys/firmware/devicetree/base/model", b"NVIDIA Jetson Orin Nano\0")], "jetson"),
            ("proc-compatible", [("proc/device-tree/compatible", b"nvidia,p3768\0nvidia,tegra234\0")], "jetson"),
            ("sys-compatible", [("sys/firmware/devicetree/base/compatible", b"nvidia,tegra234\0")], "jetson"),
            ("bsp-marker", [("etc/nv_tegra_release", b"# R39 (release), REVISION: 2.0\n")], "jetson"),
        )
        for name, files, expected in cases:
            with self.subTest(marker=name):
                fixture = self.detector_fixture(name, files)
                result = subprocess.run(
                    ["/bin/bash", "-c", 'source "$1"; detect_atd_host "$2"',
                     "atd-test", str(HOST_DETECTOR), str(fixture)],
                    env=self.env, text=True, capture_output=True, timeout=10,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout.strip(), expected)
                self.assertEqual(self.calls(), [])

    def test_omitting_host_selects_detected_storage_and_container(self):
        for host, files in (("pc", []), ("jetson", [("etc/nv_tegra_release", b"R39\n")])):
            with self.subTest(host=host):
                self.reset_calls()
                fixture = self.detector_fixture(f"auto-{host}-fixture", files)
                script = self.auto_project(f"auto-{host}-project", fixture)
                result = self.invoke(None, script=script)
                self.assertEqual(result.returncode, 0, result.stderr)
                activation, = self.activations()
                self.assertIn(f"docker.run.container_name=atd_{host}_dev", activation["args"])
                self.assertEqual(activation["record"], str(self.paths[(host, "record")]))
                self.assertEqual(activation["results"], str(self.paths[(host, "results")]))

    def test_explicit_host_overrides_hardware_detection(self):
        for host, files in (("pc", [("etc/nv_tegra_release", b"R39\n")]), ("jetson", [])):
            with self.subTest(host=host):
                self.reset_calls()
                fixture = self.detector_fixture(f"override-{host}-fixture", files)
                script = self.auto_project(f"override-{host}-project", fixture)
                result = self.invoke(host, script=script)
                self.assertEqual(result.returncode, 0, result.stderr)
                activation, = self.activations()
                self.assertIn(f"docker.run.container_name=atd_{host}_dev", activation["args"])
                self.assertEqual(activation["record"], str(self.paths[(host, "record")]))

    def test_host_selects_paths_and_mounts_record_and_results_writable(self):
        for host in ("pc", "jetson"):
            with self.subTest(host=host):
                self.reset_calls()
                result = self.invoke(host)
                self.assertEqual(result.returncode, 0, result.stderr)
                activation, = self.activations()
                self.assertEqual(activation["args"], [
                    "activate", "--config", f"docker.run.container_name=atd_{host}_dev",
                ])
                self.assertEqual(activation["workspace"], str(self.workspaces[host]))
                self.assertEqual(activation["mount_args"], [
                    ["--mount", f"type=bind,src={PROJECT},dst=/workspaces/atd,readonly"],
                    ["--mount", f"type=bind,src={PROJECT / 'ros2_ws/src'},dst=/workspaces/isaac_ros-dev/src/atd"],
                    ["--mount", f"type=bind,src={self.paths[(host, 'record')]},dst=/data/record"],
                    ["--mount", f"type=bind,src={self.paths[(host, 'results')]},dst=/data/results"],
                    ["-e", "ATD_RUNTIME_REV=" + activation["runtime"]],
                ])
                self.assertRegex(activation["runtime"], r"^[a-f0-9]{64}$")
                self.assertEqual(activation['generated_directory'], str(PROJECT / '.cache/docker' / activation['runtime']))
                self.assertEqual(activation['docker_args_file'], str(PROJECT / '.cache/docker' / activation['runtime'] / f'dev.{host}.dockerargs'))
                self.assertEqual(activation["generated_context"], str(PROJECT / "docker"))
                self.assertTrue(activation["generated_dockerfile"].startswith(
                    (PROJECT / "docker/Dockerfile.atd").read_text()))
                self.assertIn(activation["runtime"], activation["generated_dockerfile"])

    def test_one_invocation_environment_overrides_selected_host(self):
        record = self.root / "override record"
        results = self.root / "override results"
        record.mkdir()
        results.mkdir()
        result = self.invoke("jetson", env={
            "ATD_RECORD_DIR": str(record), "ATD_RESULT_DIR": str(results),
        })
        self.assertEqual(result.returncode, 0, result.stderr)
        activation, = self.activations()
        self.assertEqual(activation["record"], str(record))
        self.assertEqual(activation["results"], str(results))
        self.reset_calls()
        result = self.invoke("jetson")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.activations()[0]["record"], str(self.paths[("jetson", "record")]))

    def test_workspace_default_override_and_notebook_alias(self):
        fixture = self.detector_fixture('default-workspace-fixture', [])
        script = self.auto_project('default-workspace-project', fixture)
        project = script.parent.parent
        default = project / 'ros2_ws'
        result = self.invoke('notebook', script=script,
                             env={'ATD_WORKSPACE_CONFIG': str(self.root / 'absent.env')})
        self.assertEqual(result.returncode, 0, result.stderr)
        activation, = self.activations()
        self.assertEqual(activation['workspace'], str(default))
        self.assertFalse(any('dst=/workspaces/isaac_ros-dev/src/atd' in str(args)
                             for args in activation['mount_args']))
        self.assertEqual((project / 'scripts/.build_image_layers.yaml').resolve(),
                         project / 'docker/isaac_ros/build_image_layers.yaml')
        self.assertFalse((self.root / 'workspaces/atd').exists())
        self.reset_calls()
        override = self.root / 'override_ws'
        override.mkdir()
        before = self.workspace_config.read_bytes()
        result = self.invoke('jetson', '--workspace', str(override), script=script)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.activations()[0]['workspace'], str(override))
        self.assertEqual(self.workspace_config.read_bytes(), before)
        self.reset_calls()
        result = self.invoke('jetson', env={'ATD_ROS_WS': str(override)}, script=script)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.activations()[0]['workspace'], str(override))

    def test_workspace_path_and_conflicting_cli_configuration_stop_before_activation(self):
        for path in ('relative', str(self.root / 'bad path'), str(self.root / 'missing_ws')):
            with self.subTest(path=path):
                result = self.invoke('pc', '--workspace', path)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.calls(), [])
        conflict = self.workspaces['pc'].parent / 'scripts/.build_image_layers.yaml'
        conflict.parent.mkdir()
        conflict.write_text('image_key_order: [other]\n')
        result = self.invoke('pc')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('衝突', result.stderr)
        self.assertEqual(conflict.read_text(), 'image_key_order: [other]\n')
        self.assertEqual(self.calls(), [])

    def test_invalid_data_paths_fail_before_cli(self):
        for override, message in (
            ({"ATD_RECORD_DIR": "relative/record"}, "絶対パス"),
            ({"ATD_RECORD_DIR": str(self.root / "missing")}, "ディレクトリがありません"),
        ):
            with self.subTest(override=override):
                self.reset_calls()
                result = self.invoke(env=override)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)
                self.assertEqual(self.calls(), [])

    def test_same_nested_and_symlink_data_directories_are_allowed(self):
        record = self.paths[("pc", "record")]
        child = record / "nested results"
        child.mkdir()
        alias = self.root / "results alias"
        alias.symlink_to(record, target_is_directory=True)
        for record_path, result_path in ((record, record), (record, child), (child, record), (record, alias)):
            with self.subTest(record=record_path, results=result_path):
                self.reset_calls()
                result = self.invoke(env={"ATD_RECORD_DIR": str(record_path), "ATD_RESULT_DIR": str(result_path)})
                self.assertEqual(result.returncode, 0, result.stderr)
                activation, = self.activations()
                self.assertEqual(activation["record"], str(record_path.resolve()))
                self.assertEqual(activation["results"], str(result_path.resolve()))

    def test_unsafe_project_path_fails_before_loading_config_or_cli(self):
        for name in ("project space", "project;bad", "project$bad"):
            with self.subTest(name=name):
                script = self.root / name / "scripts" / "dev.sh"
                script.parent.mkdir(parents=True)
                script.write_text(DEV_SCRIPT.read_text())
                result = subprocess.run(
                    ["/bin/bash", str(script), "--host", "pc", "--dry-run"],
                    env=self.env, text=True, capture_output=True, timeout=10,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("プロジェクトパス", result.stderr)
                self.assertEqual(self.calls(), [])

    def test_build_requests_local_cli_build_and_dry_run_executes_nothing(self):
        result = self.invoke("pc", "--build")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--build-local", self.activations()[0]["args"])
        self.reset_calls()
        result = self.invoke("jetson", "--build", "--dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--build-local", result.stdout)
        self.assertIn("read-write", result.stdout)
        self.assertEqual(self.calls(), [])

    def test_existing_container_requires_matching_path_and_access(self):
        for state in ("different_path", "different_access"):
            with self.subTest(state=state):
                self.reset_calls()
                result = self.invoke("pc", env={"ATD_TEST_CONTAINER": state})
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("mount が設定と異なります", result.stderr)
                self.assertEqual(self.activations(), [])
        self.reset_calls()
        result = self.invoke("pc", env={"ATD_TEST_CONTAINER": "matching"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.activations(), [])
        docker_execs = [entry["args"] for entry in self.calls()
                       if entry["tool"] == "docker" and entry["args"][0] == "exec"]
        self.assertEqual(len(docker_execs), 2)
        self.assertEqual(docker_execs[-1][-1], "/usr/local/bin/atd-shell")

    def test_stale_runtime_or_dds_never_attaches_or_activates(self):
        for state in ("different_dds", "different_runtime", "legacy"):
            with self.subTest(state=state):
                self.reset_calls()
                result = self.invoke("pc", env={"ATD_TEST_CONTAINER": state})
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("依存・DDS 設定が古いか異なります", result.stderr)
                self.assertEqual(self.activations(), [])
                self.assertFalse(any(entry["tool"] == "docker" and entry["args"][0] == "exec"
                                     for entry in self.calls()))

    def test_native_workspace_preserves_packages_and_uses_four_mounts(self):
        fixture = self.detector_fixture('native-fixture', [])
        script = self.auto_project('native-project', fixture)
        workspace = script.parent.parent / 'ros2_ws'
        package = workspace / 'src/atd_fixture/package.xml'
        package.parent.mkdir()
        package.write_text('<package format="3"><name>atd_fixture</name></package>\n')
        before = package.read_bytes()
        result = self.invoke('pc', '--workspace', str(workspace), script=script)
        self.assertEqual(result.returncode, 0, result.stderr)
        activation, = self.activations()
        self.assertEqual(activation['workspace'], str(workspace))
        self.assertEqual(len([args for args in activation['mount_args'] if args[0] == '--mount']), 3)
        self.assertFalse(any('dst=/workspaces/isaac_ros-dev/src/atd' in str(args) for args in activation['mount_args']))
        self.assertEqual(package.read_bytes(), before)
        self.assertFalse((workspace / 'src/atd').exists())
        self.reset_calls()
        result = self.invoke('pc', '--workspace', str(workspace), script=script,
                             env={'ATD_TEST_CONTAINER': 'matching'})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.activations(), [])
        self.assertEqual(package.read_bytes(), before)

    def test_native_workspace_rejects_stale_nested_source_mount(self):
        fixture = self.detector_fixture('native-stale-fixture', [])
        script = self.auto_project('native-stale-project', fixture)
        workspace = script.parent.parent / 'ros2_ws'
        result = self.invoke('pc', '--workspace', str(workspace), script=script,
                             env={'ATD_TEST_CONTAINER': 'stale_nested_source'})
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.activations(), [])
        self.assertFalse(any(entry['tool'] == 'docker' and entry['args'][0] == 'exec'
                             for entry in self.calls()))

    def test_existing_container_requires_current_writable_ros_source_mount(self):
        for state in ('different_source_path', 'different_source_access', 'missing_source'):
            with self.subTest(state=state):
                self.reset_calls()
                result = self.invoke('pc', env={'ATD_TEST_CONTAINER': state})
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('mount が設定と異なります', result.stderr)
                self.assertEqual(self.activations(), [])
                self.assertFalse(any(entry['tool'] == 'docker' and entry['args'][0] == 'exec'
                                     for entry in self.calls()))

    def test_setup_must_be_finished_before_attaching(self):
        for ready in ("missing", "outdated"):
            with self.subTest(ready=ready):
                self.reset_calls()
                result = self.invoke("pc", env={"ATD_TEST_CONTAINER": "matching", "ATD_TEST_READY": ready})
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("初期設定", result.stderr)
                self.assertEqual(self.activations(), [])
                self.assertFalse(any(entry["tool"] == "docker" and entry["args"][-1] == "/usr/local/bin/atd-shell"
                                     for entry in self.calls()))

    def test_runtime_manifest_change_changes_creation_revision(self):
        fixture = self.detector_fixture("revision-fixture", [])
        script = self.auto_project("revision-project", fixture)
        result = self.invoke(script=script)
        self.assertEqual(result.returncode, 0, result.stderr)
        first = self.activations()[0]["runtime"]
        self.reset_calls()
        manifest = script.parent.parent / "docker" / "dependencies.apt"
        manifest.write_text(manifest.read_text() + "\n# changed runtime configuration\n")
        result = self.invoke(script=script)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotEqual(self.activations()[0]["runtime"], first)

    def test_dry_run_does_not_generate_build_files(self):
        fixture = self.detector_fixture("dry-fixture", [])
        script = self.auto_project("dry-project", fixture)
        result = self.invoke("pc", "--dry-run", script=script)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse((script.parent.parent / ".cache/docker").exists())
        self.assertEqual(self.calls(), [])

    def test_python_lock_changes_revision_but_generated_python_files_do_not(self):
        fixture = self.detector_fixture('python-revision-fixture', [])
        script = self.auto_project('python-revision-project', fixture)

        def revision():
            result = self.invoke('pc', '--dry-run', script=script)
            self.assertEqual(result.returncode, 0, result.stderr)
            return next(line for line in result.stdout.splitlines() if line.startswith('runtime: '))

        first = revision()
        project = script.parent.parent / 'docker/python/ros'
        for relative in ('.venv/lib/generated.py', '__pycache__/generated.pyc'):
            generated = project / relative
            generated.parent.mkdir(parents=True, exist_ok=True)
            generated.write_text('not a build input\n')
        self.assertEqual(revision(), first)
        lock = project / 'uv.lock'
        lock.write_text(lock.read_text() + '\n# revised lockfile\n')
        self.assertNotEqual(revision(), first)
        self.assertEqual(self.calls(), [])

    def test_docker_configuration_templates_change_runtime_revision(self):
        fixture = self.detector_fixture('docker-config-revision-fixture', [])
        script = self.auto_project('docker-config-revision-project', fixture)

        def revision():
            result = self.invoke('pc', '--dry-run', script=script)
            self.assertEqual(result.returncode, 0, result.stderr)
            return next(line for line in result.stdout.splitlines() if line.startswith('runtime: '))

        previous = revision()
        for relative in ('docker/dev.dockerargs', 'docker/isaac_ros/build_image_layers.yaml'):
            path = script.parent.parent / relative
            path.write_text(path.read_text() + '\n# revised Docker configuration\n')
            current = revision()
            self.assertNotEqual(current, previous, relative)
            previous = current
        self.assertEqual(self.calls(), [])

    def test_build_rejects_running_container_but_accepts_stopped_container(self):
        result = self.invoke("pc", "--build", env={"ATD_TEST_CONTAINER": "matching"})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("実行中コンテナを終了", result.stderr)
        self.assertEqual(self.activations(), [])
        self.reset_calls()
        result = self.invoke("pc", "--build", env={"ATD_TEST_CONTAINER": "stopped"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(self.activations()), 1)

    def test_non_docker_cli_mode_never_activates(self):
        result = self.invoke(env={"ATD_TEST_CLI_MODE": "venv"})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Docker モードではありません", result.stderr)
        self.assertEqual(self.activations(), [])
        self.assertEqual([entry["tool"] for entry in self.calls()], ["isaac-ros"])


if __name__ == "__main__":
    unittest.main()

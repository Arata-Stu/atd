"""起動 hook と専用 shell を、コンテナやホスト設定を変更せず検証する。"""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
HOOK = ROOT / "docker/scripts/10-atd-runtime.sh"
SHELL = ROOT / "docker/scripts/atd-shell.sh"
BASHRC = ROOT / "docker/scripts/atd.bashrc"
PYTHON_HELPER = ROOT / "docker/scripts/atd-python.sh"
VGL_HELPER = ROOT / "docker/scripts/atd-vgl.sh"


class StartupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="atd-startup-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.runtime = self.root / "runtime with spaces"
        self.config = self.root / "config with spaces"
        self.image = self.root / "image with spaces"
        self.baked_shell = self.root / "image shell"
        self.baked_python = self.root / "image python helper"
        self.baked_vgl = self.root / 'image vgl helper'
        self.bin = self.root / "bin"
        self.bin.mkdir()
        (self.runtime / "scripts").mkdir(parents=True)
        self.config.mkdir()
        self.image.mkdir()
        (self.runtime / "Dockerfile.atd").write_text("ARG BASE_IMAGE\n")
        (self.image / "Dockerfile.atd").write_text("ARG BASE_IMAGE\n")
        (self.runtime / "scripts/10-atd-runtime.sh").write_text(HOOK.read_text())
        (self.runtime / "scripts/atd-shell.sh").write_text(SHELL.read_text())
        (self.runtime / "scripts/atd.bashrc").write_text(BASHRC.read_text())
        (self.runtime / "scripts/atd-python.sh").write_text(PYTHON_HELPER.read_text())
        (self.runtime / 'scripts/atd-vgl.sh').write_text(VGL_HELPER.read_text())
        self.baked_shell.write_text(SHELL.read_text())
        self.baked_python.write_text(PYTHON_HELPER.read_text())
        self.baked_vgl.write_text(VGL_HELPER.read_text())
        (self.config / "bashrc").write_text(BASHRC.read_text())
        self.log = self.root / "calls"
        (self.bin / "id").write_text('#!/bin/sh\nprintf "%s\\n" "${ATD_TEST_UID:-0}"\n')
        (self.bin / "id").chmod(0o755)
        (self.bin / 'uv').write_text('#!/bin/sh\nprintf "uv %s\\n" "${ATD_TEST_UV_VERSION:-0.13.8}"\n')
        (self.bin / 'uv').chmod(0o755)
        (self.bin / 'gosu').write_text(
            '#!/bin/bash\n'
            '[[ "$1" = admin && "$2" = env && "$3" = "ATD_PYTHON_PROJECTS_DIR=$ATD_RUNTIME_DIR/python" ]] || exit 10\n'
            '[[ "$4" = "$ATD_PYTHON_PATH" && "$5" = ros && "$6" = sync ]] || exit 11\n'
            'printf "python\\n" >> "$ATD_TEST_LOG"\n'
            '[[ ${ATD_TEST_FAIL:-} != python ]] || exit 12\n')
        (self.bin / 'gosu').chmod(0o755)
        (self.runtime / "scripts/install-dependencies.sh").write_text(
            '#!/bin/bash\nprintf "dependencies\\n" >> "$ATD_TEST_LOG"\n'
            '[[ ${ATD_TEST_FAIL:-} != dependencies ]] || exit 7\n'
        )
        (self.runtime / "scripts/configure-dds.sh").write_text(
            '#!/bin/bash\nprintf "dds\\n" >> "$ATD_TEST_LOG"\n'
            '[[ ${ATD_TEST_FAIL:-} != dds ]] || exit 8\n'
            '[[ ${ATD_TEST_FAIL:-} != missing_env ]] || exit 0\n'
            'if [[ ${ATD_TEST_FAIL:-} == env ]]; then '
            'printf "return 9\\n" > "$ATD_DDS_CONFIG_DIR/dds.env.sh"; exit 0; fi\n'
            "printf '%s\\n' 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' "
            "'unset CYCLONEDDS_URI' > \"$ATD_DDS_CONFIG_DIR/dds.env.sh\"\n"
        )
        self.env = {
            "PATH": os.pathsep.join((str(self.bin), str(Path(sys.executable).parent), os.defpath)),
            "ATD_RUNTIME_DIR": str(self.runtime),
            "ATD_DDS_CONFIG_DIR": str(self.config),
            "ATD_IMAGE_DIR": str(self.image),
            "ATD_SHELL_PATH": str(self.baked_shell),
            "ATD_PYTHON_PATH": str(self.baked_python),
            "ATD_VGL_PATH": str(self.baked_vgl),
            "ATD_RUNTIME_REV": "fixture-v1",
            "ATD_TEST_LOG": str(self.log),
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "CYCLONEDDS_URI": "old.xml",
        }

    def run_hook(self, env=None, command=None):
        return subprocess.run(
            ["/bin/bash", "-c", command or
             'task_hook=$1; set -- /bin/bash; source "$task_hook"; printf "result=%s|%s|%s\\n" '
             '"$1" "$RMW_IMPLEMENTATION" "${CYCLONEDDS_URI-unset}"',
             "startup-test", str(HOOK)],
            env={**self.env, **(env or {})}, capture_output=True, text=True,
        )

    def test_order_environment_ready_marker_and_initial_shell(self):
        # 保存済み marker があっても各 startup task は必ず走る。
        (self.config / "runtime.ready").write_text("old\n")
        result = self.run_hook()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.log.read_text(), "dependencies\ndds\npython\n")
        self.assertIn(f"result={self.baked_shell}|rmw_fastrtps_cpp|unset", result.stdout)
        self.assertEqual((self.config / "runtime.ready").read_text(), "fixture-v1\n")

    def test_failure_stops_entrypoint_and_removes_stale_marker(self):
        for stage, expected in (("dependencies", "dependencies\n"), ("dds", "dependencies\ndds\n"),
                                ("missing_env", "dependencies\ndds\n"), ("env", "dependencies\ndds\n"),
                                ("python", "dependencies\ndds\npython\n")):
            with self.subTest(stage=stage):
                (self.config / "runtime.ready").write_text("stale\n")
                self.log.unlink(missing_ok=True)
                (self.config / "dds.env.sh").unlink(missing_ok=True)
                result = self.run_hook({"ATD_TEST_FAIL": stage})
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("result=", result.stdout)
                self.assertFalse((self.config / "runtime.ready").exists())
                self.assertEqual(self.log.read_text(), expected)

    def test_non_root_never_runs_tasks(self):
        result = self.run_hook({"ATD_TEST_UID": "1000"})
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.log.exists())

    def test_image_specific_changes_require_rebuild_before_tasks(self):
        for relative in ("Dockerfile.atd", "scripts/10-atd-runtime.sh", "scripts/atd-shell.sh",
                         "scripts/atd.bashrc", "scripts/atd-python.sh", "scripts/atd-vgl.sh"):
            with self.subTest(path=relative):
                path = self.runtime / relative
                original = path.read_text()
                path.write_text(original + "\n# changed since image build\n")
                (self.config / "runtime.ready").write_text("stale\n")
                result = self.run_hook()
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("--build", result.stderr)
                self.assertFalse(self.log.exists())
                self.assertFalse((self.config / "runtime.ready").exists())
                path.write_text(original)

    def test_runtime_dependency_helper_can_change_without_rebuild(self):
        helper = self.runtime / "scripts/install-dependencies.sh"
        helper.write_text(helper.read_text() + 'printf "new dependency helper\\n"\n')
        result = self.run_hook()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("new dependency helper", result.stdout)

    def test_wrong_uv_version_stops_before_dependencies_and_ready(self):
        (self.config / 'runtime.ready').write_text('stale\n')
        result = self.run_hook({'ATD_TEST_UV_VERSION': '0.1.0'})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('uv 0.13.8', result.stderr)
        self.assertFalse(self.log.exists())
        self.assertFalse((self.config / 'runtime.ready').exists())

    def test_explicit_command_preserved_and_image_revision_default(self):
        command = ('task_hook=$1; set -- /usr/bin/example "arg with spaces"; '
                   'source "$task_hook"; printf "args=%s|%s\\n" "$1" "$2"')
        result = self.run_hook({"ATD_RUNTIME_REV": ""}, command)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("args=/usr/bin/example|arg with spaces", result.stdout)
        self.assertEqual((self.config / "runtime.ready").read_text(), "image\n")

    def test_shell_command_receives_dds_and_exact_arguments(self):
        result = self.run_hook()
        self.assertEqual(result.returncode, 0, result.stderr)
        result = subprocess.run(
            ["/bin/bash", str(SHELL), sys.executable, "-c",
             'import os,sys; print(os.environ["RMW_IMPLEMENTATION"]); print(sys.argv[1])',
             "arg with spaces"],
            env=self.env, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "rmw_fastrtps_cpp\narg with spaces\n")

    def test_shell_command_loads_workspace_path_before_applying_dds(self):
        self.assertEqual(self.run_hook().returncode, 0)
        workspace = self.root / "workspace"
        tools = workspace / "bin"
        tools.mkdir(parents=True)
        (workspace / "install").mkdir()
        (workspace / "install/setup.bash").write_text(
            'export PATH="${ISAAC_ROS_WS}/bin:$PATH"\nexport RMW_IMPLEMENTATION=overlay-dds\n'
        )
        command = tools / "ros-command"
        command.write_text('#!/bin/sh\nprintf "%s|%s\\n" "$RMW_IMPLEMENTATION" "$1"\n')
        command.chmod(0o755)
        result = subprocess.run(
            ["/bin/bash", str(SHELL), "ros-command", "argument with spaces"],
            env={**self.env, "ISAAC_ROS_WS": str(workspace)},
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "rmw_fastrtps_cpp|argument with spaces\n")

    def test_interactive_shell_ignores_host_rc_and_restores_overlay_dds(self):
        self.assertEqual(self.run_hook().returncode, 0)
        (self.config / "bashrc").write_text(BASHRC.read_text())
        home = self.root / "isolated-home"
        home.mkdir()
        for name in (".bashrc", ".bash_profile", ".profile"):
            (home / name).write_text('export ATD_HOST_RC=read\nexport RMW_IMPLEMENTATION=host-dds\n')
        workspace = self.root / "workspace"
        (workspace / "install").mkdir(parents=True)
        (workspace / "install/setup.bash").write_text(
            'export ATD_FROM_WORKSPACE=read\nexport RMW_IMPLEMENTATION=overlay-dds\n'
        )
        result = subprocess.run(
            ["/bin/bash", str(SHELL)],
            input='printf "ATD_RESULT=%s|%s|%s\\n" "$RMW_IMPLEMENTATION" '
                  '"$ATD_FROM_WORKSPACE" "${ATD_HOST_RC-unset}"\nexit\n',
            env={**self.env, "HOME": str(home), "ISAAC_ROS_WS": str(workspace)},
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("ATD_RESULT=rmw_fastrtps_cpp|read|unset", result.stdout)

    def test_shell_without_runtime_configuration_fails(self):
        result = subprocess.run(["/bin/bash", str(SHELL), "true"], env=self.env,
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("DDS 設定がありません", result.stderr)


if __name__ == "__main__":
    unittest.main()

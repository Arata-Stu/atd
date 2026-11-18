"""起動 hook と専用 shell を、コンテナやホスト設定を変更せず検証する。"""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
HOOK = ROOT / "docker/scripts/10-atd-runtime.sh"
ENTRYPOINT = ROOT / "docker/scripts/atd-entrypoint.sh"
DISPATCH = ROOT / "docker/scripts/atd-dispatch.sh"
SHELL = ROOT / "docker/scripts/atd-shell.sh"
BASHRC = ROOT / "docker/scripts/atd.bashrc"


class StartupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="atd-startup-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.runtime = self.root / "runtime with spaces"
        self.config = self.root / "config with spaces"
        self.image = self.root / "image with spaces"
        self.bin = self.root / "bin"
        self.bin.mkdir()
        (self.runtime / "scripts").mkdir(parents=True)
        self.config.mkdir()
        self.image.mkdir()
        (self.runtime / "Dockerfile.atd").write_text("ARG BASE_IMAGE\n")
        (self.image / "Dockerfile.atd").write_text("ARG BASE_IMAGE\n")
        (self.image / "scripts").mkdir()
        for launcher in (DISPATCH, ENTRYPOINT):
            for directory in (self.runtime, self.image):
                shutil.copy2(launcher, directory / "scripts" / launcher.name)
        (self.runtime / "scripts/10-atd-runtime.sh").write_text(HOOK.read_text())
        shutil.copy2(SHELL, self.runtime / "scripts/atd-shell.sh")
        (self.runtime / "scripts/atd.bashrc").write_text(BASHRC.read_text())
        for command in ('atd-python', 'atd-shell', 'atd-web'):
            (self.bin / command).symlink_to(DISPATCH)
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
        for directory in (self.runtime, self.image):
            (directory / "dependencies.apt").write_text("cmake\n")
            (directory / "scripts/install-dependencies.sh").write_text(
                '#!/bin/bash\n'
                '[[ "$0" == "$ATD_IMAGE_DIR/scripts/install-dependencies.sh" && "$1" == --check ]] || exit 14\n'
                'printf "dependencies\\n" >> "$ATD_TEST_LOG"\n'
                '[[ ${ATD_TEST_FAIL:-} != dependencies ]] || exit 7\n'
            )
        (self.runtime / "scripts/configure-serial-access.sh").write_text(
            '#!/bin/bash\nprintf "serial\\n" >> "$ATD_TEST_LOG"\n'
            '[[ ${ATD_TEST_FAIL:-} != serial ]] || exit 13\n'
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
            "ATD_SHELL_PATH": str(self.bin / "atd-shell"),
            "ATD_PYTHON_PATH": str(self.bin / "atd-python"),
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
             "startup-test", str(ENTRYPOINT)],
            env={**self.env, **(env or {})}, capture_output=True, text=True,
        )

    def test_order_environment_ready_marker_and_initial_shell(self):
        # 保存済み marker があっても各 startup task は必ず走る。
        (self.config / "runtime.ready").write_text("old\n")
        result = self.run_hook()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.log.read_text(), "dependencies\nserial\ndds\npython\n")
        self.assertIn(f"result={self.bin / 'atd-shell'}|rmw_fastrtps_cpp|unset", result.stdout)
        self.assertEqual((self.config / "runtime.ready").read_text(), "fixture-v1\n")

    def test_failure_stops_entrypoint_and_removes_stale_marker(self):
        for stage, expected in (("dependencies", "dependencies\n"), ("serial", "dependencies\nserial\n"),
                                ("dds", "dependencies\nserial\ndds\n"),
                                ("missing_env", "dependencies\nserial\ndds\n"), ("env", "dependencies\nserial\ndds\n"),
                                ("python", "dependencies\nserial\ndds\npython\n")):
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
        for relative in ("Dockerfile.atd", "scripts/atd-entrypoint.sh", "scripts/atd-dispatch.sh",
                         "dependencies.apt", "scripts/install-dependencies.sh"):
            with self.subTest(file=relative):
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

    def test_next_start_reads_updated_hook_from_checkout(self):
        helper = self.runtime / "scripts/10-atd-runtime.sh"
        with helper.open("a") as stream:
            stream.write('printf "updated mounted startup\\n"\n')
        result = self.run_hook()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("updated mounted startup", result.stdout)

    def test_installed_command_runs_edited_checkout_with_original_arguments(self):
        command = self.runtime / "scripts/atd-web.sh"
        for version in ("first", "second"):
            command.write_text(f'#!/bin/bash\nprintf "%s|%s|%s\\n" {version} "$1" "$2"\n')
            command.chmod(0o755)
            result = subprocess.run([str(self.bin / "atd-web"), "serve", "value with spaces"],
                                    env=self.env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout, f"{version}|serve|value with spaces\n")

    def test_missing_mounted_command_fails_even_if_snapshot_has_a_copy(self):
        fallback = self.image / "scripts/atd-web.sh"
        fallback.write_text('#!/bin/bash\nprintf "image snapshot\\n"\n')
        result = subprocess.run([str(self.bin / "atd-web")], env=self.env,
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("image snapshot", result.stdout)

    @unittest.skipIf(Path("/workspaces/atd").is_dir(), "実際の checkout がマウントされている")
    def test_entrypoint_and_commands_use_image_snapshot_without_a_checkout(self):
        shutil.copytree(self.runtime, self.image, dirs_exist_ok=True)
        (self.image / "scripts/atd-web.sh").write_text('printf "image snapshot\\n"\n')
        environment = {"ATD_RUNTIME_DIR": "", "ATD_RUNTIME_REV": ""}
        result = self.run_hook(environment)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((self.config / "runtime.ready").read_text(), "image\n")
        command = subprocess.run([str(self.bin / "atd-web")], env={**self.env, **environment},
                                 capture_output=True, text=True)
        self.assertEqual(command.returncode, 0, command.stderr)
        self.assertEqual(command.stdout, "image snapshot\n")

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

    def test_interactive_shell_opens_checkout_from_default_workspace(self):
        self.assertEqual(self.run_hook().returncode, 0)
        workspace = self.root.resolve() / "ros workspace"
        checkout = self.root.resolve() / "atd checkout"
        workspace.mkdir()
        checkout.mkdir()
        workspace_alias = self.root / "workspace-link"
        workspace_alias.symlink_to(workspace, target_is_directory=True)
        env = {**self.env, "ISAAC_ROS_WS": str(workspace_alias), "ATD_PROJECT_DIR": str(checkout)}
        for args, cwd, expected in (([], workspace, checkout), ([], self.root, self.root),
                                    (["pwd"], workspace, workspace)):
            with self.subTest(args=args, cwd=cwd):
                result = subprocess.run(
                    ["/bin/bash", str(SHELL), *args], cwd=cwd, env=env,
                    input="pwd\nexit\n", capture_output=True, text=True,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(str(expected.resolve()), result.stdout.splitlines())

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

    def test_interactive_rc_does_not_repeat_system_initialization(self):
        self.assertEqual(self.run_hook().returncode, 0)
        system_rc = self.root / 'system.bashrc'
        system_rc.write_text('printf "SYSTEM_GUIDANCE\\n"\n')
        # Ubuntu Bash のシステムrc→--rcfileという順序を再現する。
        # ホストの /etc は変更せず、ATD側の明示sourceだけをfixtureへ向ける。
        (self.config / 'bashrc').write_text(
            BASHRC.read_text().replace('/etc/bash.bashrc', str(system_rc)))
        startup = self.root / 'ubuntu-startup.bash'
        startup.write_text('source "$ATD_TEST_SYSTEM_RC"\nsource "$ATD_DDS_CONFIG_DIR/bashrc"\n')
        result = subprocess.run(
            ['/bin/bash', '--noprofile', '--rcfile', str(startup), '-i'],
            input='printf "ATD_RESULT=%s\\n" "$RMW_IMPLEMENTATION"\nexit\n',
            env={**self.env, 'ATD_TEST_SYSTEM_RC': str(system_rc)},
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.count('SYSTEM_GUIDANCE'), 1)
        self.assertIn('ATD_RESULT=rmw_fastrtps_cpp', result.stdout)

    def test_shell_without_runtime_configuration_fails(self):
        result = subprocess.run(["/bin/bash", str(SHELL), "true"], env=self.env,
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("DDS 設定がありません", result.stderr)


if __name__ == "__main__":
    unittest.main()

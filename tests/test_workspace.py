"""外部 workspace の準備と、既存設定・解決後パスの保護を確認する。"""

import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


PROJECT = Path(__file__).resolve().parents[1]
HELPER = PROJECT / "scripts/lib/workspace.sh"
TEMPLATES = (
    "docker/isaac_ros/build_image_layers.yaml",
    "docker/isaac_ros/common-config.sh",
    "docker/isaac_ros/config.yaml",
    "config/workspace.env.example",
)


class WorkspaceHelperTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="atd-workspace-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.project = self.root / "project"
        for relative in TEMPLATES:
            output = self.project / relative
            output.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(PROJECT / relative, output)
        (self.project / 'ros2_ws/src').mkdir(parents=True)
        (self.project / 'ros2_ws/src/README.md').write_text('ROS package source\n')
        self.home = self.root / "home"
        self.home.mkdir()
        self.workspace = self.home / "workspaces/atd"
        self.config = self.project / "config/workspace.env"
        self.env = {
            "HOME": str(self.home),
            "PATH": os.pathsep.join((str(Path(sys.executable).parent), os.defpath)),
            "ATD_WORKSPACE_CONFIG": str(self.config),
        }

    def call(self, function, workspace=None, project=None):
        arguments = [function, str(project or self.project)]
        if function == "atd_save_workspace":
            arguments.append("pc")
        arguments.append(str(workspace or self.workspace))
        return subprocess.run(
            ["/bin/bash", "-c", 'source "$1"; shift; "$@"', "workspace-test", str(HELPER), *arguments],
            env=self.env, text=True, capture_output=True, timeout=10,
        )

    def snapshot(self):
        result = {}
        for path in sorted(self.root.rglob("*")):
            relative = str(path.relative_to(self.root))
            if path.is_symlink():
                result[relative] = ("symlink", os.readlink(path))
            elif path.is_dir():
                result[relative] = ("directory",)
            else:
                result[relative] = ("file", path.read_bytes())
        return result

    def test_default_workspace_metadata_and_second_prepare_are_idempotent(self):
        result = subprocess.run(
            ["/bin/bash", "-c",
             'source "$1"; atd_load_workspace "$2" pc && atd_prepare_workspace "$2" "$ATD_RESOLVED_ROS_WS"',
             "workspace-test", str(HELPER), str(self.project)],
            env=self.env, text=True, capture_output=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        destinations = {
            self.workspace.parent / "scripts/.build_image_layers.yaml": self.project / TEMPLATES[0],
            self.workspace / "scripts/.isaac_ros_common-config": self.project / TEMPLATES[1],
            self.workspace / ".isaac-ros-cli/config.yaml": self.project / TEMPLATES[2],
        }
        for destination, source in destinations.items():
            self.assertTrue(destination.is_symlink(), str(destination))
            self.assertEqual(destination.resolve(), source)
        self.assertTrue((self.workspace / "src").is_dir())
        self.assertTrue((self.workspace / 'src/atd').is_dir())
        self.assertEqual(list((self.workspace / 'src/atd').iterdir()), [])
        self.assertFalse(self.config.exists())
        first = self.snapshot()
        inodes = {path: path.lstat().st_ino for path in destinations}
        result = self.call("atd_prepare_workspace")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.snapshot(), first)
        self.assertEqual({path: path.lstat().st_ino for path in destinations}, inodes)

    def test_resolved_root_paths_are_rejected_before_any_write(self):
        # 検証が退行しても実際の / へ metadata を作らないよう、source を1つ隠す。
        (self.project / TEMPLATES[0]).unlink()
        resolved_tmp = Path(tempfile.gettempdir()).resolve()
        root_aliases = ["/.", str(resolved_tmp) + "/.." * len(resolved_tmp.parts[1:])]
        if Path("/tmp/..").resolve() == Path("/"):
            root_aliases.append("/tmp/..")
        before = self.snapshot()
        for function in ("atd_prepare_workspace", "atd_save_workspace"):
            for alias in root_aliases:
                with self.subTest(function=function, path=alias):
                    result = self.call(function, workspace=alias)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("/ 以外の絶対パス", result.stderr)
                    self.assertEqual(self.snapshot(), before)
                    self.assertFalse(self.config.exists())

    def test_space_in_symlink_target_is_rejected_without_metadata_or_saved_config(self):
        target = self.root / "workspace with spaces"
        target.mkdir()
        alias = self.root / "safe-workspace-name"
        alias.symlink_to(target, target_is_directory=True)
        before = self.snapshot()
        for function in ("atd_prepare_workspace", "atd_save_workspace"):
            with self.subTest(function=function):
                result = self.call(function, workspace=alias)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("空白や shell 特殊文字", result.stderr)
                self.assertEqual(self.snapshot(), before)
                self.assertFalse(self.config.exists())

    def test_parent_shell_configuration_conflict_is_preserved(self):
        existing = self.workspace.parent / "scripts/.isaac_ros_common-config"
        existing.parent.mkdir(parents=True)
        existing.write_text("CONFIG_DOCKER_SEARCH_DIRS=(/another/project/docker)\n")
        before = self.snapshot()
        result = self.call("atd_prepare_workspace")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(str(existing), result.stderr)
        self.assertEqual(self.snapshot(), before)
        self.assertFalse(self.workspace.exists())

    def test_parent_layer_yaml_conflict_is_preserved(self):
        existing = self.workspace.parent / "scripts/.build_image_layers.yaml"
        existing.parent.mkdir(parents=True)
        existing.write_text("image_key_order: [another.project]\n")
        before = self.snapshot()
        result = self.call("atd_prepare_workspace")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(str(existing), result.stderr)
        self.assertEqual(self.snapshot(), before)
        self.assertFalse(self.workspace.exists())

    def test_workspace_cli_configuration_conflict_does_not_partially_prepare(self):
        existing = self.workspace / ".isaac-ros-cli/config.yaml"
        existing.parent.mkdir(parents=True)
        existing.write_text("docker:\n  run:\n    container_name: another_project\n")
        before = self.snapshot()
        result = self.call("atd_prepare_workspace")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(str(existing), result.stderr)
        self.assertEqual(self.snapshot(), before)
        self.assertFalse((self.workspace / "src").exists())
        self.assertFalse((self.workspace.parent / "scripts/.build_image_layers.yaml").exists())

    def test_unsafe_project_path_is_rejected_before_preparing_workspace(self):
        unsafe_project = self.root / "project with spaces"
        shutil.copytree(self.project, unsafe_project)
        before = self.snapshot()
        result = self.call("atd_prepare_workspace", project=unsafe_project)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("空白や shell 特殊文字", result.stderr)
        self.assertEqual(self.snapshot(), before)
        self.assertFalse(self.workspace.exists())

    def test_legacy_template_symlinks_are_migrated_even_when_old_targets_are_gone(self):
        legacy = {
            self.workspace.parent / 'scripts/.build_image_layers.yaml': self.project / 'scripts/.build_image_layers.yaml',
            self.workspace / 'scripts/.isaac_ros_common-config': self.project / 'ros2_ws/scripts/.isaac_ros_common-config',
            self.workspace / '.isaac-ros-cli/config.yaml': self.project / 'ros2_ws/.isaac-ros-cli/config.yaml',
            self.workspace.parent / 'scripts/.isaac_ros_common-config': self.project / 'ros2_ws/scripts/.isaac_ros_common-config',
        }
        for destination, target in legacy.items():
            destination.parent.mkdir(parents=True, exist_ok=True)
            # 相対の旧 symlink も移行対象にする。
            destination.symlink_to(os.path.relpath(target, destination.parent))
            self.assertFalse(destination.exists())
        result = self.call('atd_prepare_workspace')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((self.workspace.parent / 'scripts/.build_image_layers.yaml').resolve(), self.project / TEMPLATES[0])
        self.assertEqual((self.workspace / 'scripts/.isaac_ros_common-config').resolve(), self.project / TEMPLATES[1])
        self.assertEqual((self.workspace / '.isaac-ros-cli/config.yaml').resolve(), self.project / TEMPLATES[2])
        self.assertEqual((self.workspace.parent / 'scripts/.isaac_ros_common-config').resolve(), self.project / TEMPLATES[1])

    def test_unknown_symlink_conflict_prevents_partial_legacy_migration(self):
        legacy = self.workspace.parent / 'scripts/.build_image_layers.yaml'
        legacy.parent.mkdir(parents=True)
        legacy.symlink_to(self.project / 'scripts/.build_image_layers.yaml')
        unknown = self.workspace / '.isaac-ros-cli/config.yaml'
        unknown.parent.mkdir(parents=True)
        unknown.symlink_to(self.root / 'another-project/missing.yaml')
        before = self.snapshot()
        result = self.call('atd_prepare_workspace')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('衝突', result.stderr)
        self.assertEqual(self.snapshot(), before)

    def test_existing_source_mountpoint_data_and_symlinks_are_preserved(self):
        mountpoint = self.workspace / 'src/atd'
        mountpoint.mkdir(parents=True)
        important = mountpoint / 'user.cpp'
        important.write_text('user source\n')
        before = self.snapshot()
        result = self.call('atd_prepare_workspace')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('既存データ', result.stderr)
        self.assertEqual(self.snapshot(), before)
        important.unlink()
        mountpoint.rmdir()
        mountpoint.symlink_to(self.project / 'ros2_ws/src', target_is_directory=True)
        before = self.snapshot()
        result = self.call('atd_prepare_workspace')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.snapshot(), before)

    def test_source_tree_cannot_overlap_the_workspace_mountpoint(self):
        for workspace in (self.project / 'ros2_ws/src', self.project / 'ros2_ws/src/nested'):
            with self.subTest(workspace=workspace):
                before = self.snapshot()
                result = self.call('atd_prepare_workspace', workspace=workspace)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('重複', result.stderr)
                self.assertEqual(self.snapshot(), before)
        nested_project = self.root / 'other-ws/src/atd/checkout'
        shutil.copytree(self.project, nested_project)
        before = self.snapshot()
        result = self.call('atd_prepare_workspace', workspace=self.root / 'other-ws', project=nested_project)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('重複', result.stderr)
        self.assertEqual(self.snapshot(), before)

    def test_native_workspace_preserves_packages_without_nested_source_reservation(self):
        workspace = self.project / 'ros2_ws'
        package = workspace / 'src/atd_example'
        package.mkdir()
        source = package / 'example.cpp'
        source.write_text('int example;\n')
        result = self.call('atd_prepare_workspace', workspace=workspace)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(source.read_text(), 'int example;\n')
        self.assertFalse((workspace / 'src/atd').exists())
        self.assertIn('-> /workspaces/isaac_ros-dev/src (Docker 内)', result.stdout)
        before = self.snapshot()
        self.assertEqual(self.call('atd_prepare_workspace', workspace=workspace).returncode, 0)
        self.assertEqual(self.snapshot(), before)

    def test_native_workspace_keeps_existing_atd_source_directory(self):
        workspace = self.project / 'ros2_ws'
        existing = workspace / 'src/atd'
        existing.mkdir()
        (existing / 'source.cpp').write_text('user-owned source\n')
        result = self.call('atd_prepare_workspace', workspace=workspace)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual((existing / 'source.cpp').read_text(), 'user-owned source\n')

    def test_native_workspace_alias_resolves_without_nested_source_mountpoint(self):
        alias = self.root / 'native-alias'
        alias.symlink_to(self.project / 'ros2_ws', target_is_directory=True)
        result = self.call('atd_prepare_workspace', workspace=alias)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse((self.project / 'ros2_ws/src/atd').exists())

    def test_source_or_workspace_src_symlinks_are_not_accepted_as_native_layout(self):
        self.workspace.mkdir(parents=True)
        (self.workspace / 'src').symlink_to(self.project / 'ros2_ws/src', target_is_directory=True)
        before = self.snapshot()
        result = self.call('atd_prepare_workspace')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('symlink', result.stderr)
        self.assertEqual(self.snapshot(), before)
        (self.workspace / 'src').unlink()
        original = self.project / 'ros2_ws/src'
        original.rename(self.project / 'ros2_ws/real-source')
        original.symlink_to(self.project / 'ros2_ws/real-source', target_is_directory=True)
        before = self.snapshot()
        result = self.call('atd_prepare_workspace', workspace=self.project / 'ros2_ws')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.snapshot(), before)


if __name__ == "__main__":
    unittest.main()

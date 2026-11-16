"""ROS Python を引き継がない venv でも ament index から package を探せる。"""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from tools.vgl.build_engine import package_paths


class VglPackagePathTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='atd-vgl-ament-index-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()

    def installation(self, name, *, marker=True, share=True):
        prefix = self.root / name
        prefix.mkdir()
        if marker:
            index = prefix / 'share/ament_index/resource_index/packages/isaac_ros_visual_mapping'
            index.parent.mkdir(parents=True)
            index.write_text('')
        if share:
            (prefix / 'share/isaac_ros_visual_mapping').mkdir(parents=True)
        return prefix

    def find(self, value):
        with patch.dict(os.environ, {'AMENT_PREFIX_PATH': value}, clear=True):
            return package_paths()

    def test_first_indexed_prefix_wins_overlay_order(self):
        overlay = self.installation('overlay')
        base = self.installation('base')
        self.assertEqual(self.find(os.pathsep.join((str(overlay), str(base)))),
                         (overlay, overlay / 'share/isaac_ros_visual_mapping'))
        self.assertEqual(self.find(os.pathsep.join((str(base), str(overlay)))),
                         (base, base / 'share/isaac_ros_visual_mapping'))

    def test_empty_nonexistent_and_unindexed_prefixes_are_skipped(self):
        unindexed = self.installation('unindexed', marker=False)
        base = self.installation('base')
        value = os.pathsep.join(('', str(self.root / 'missing'), str(unindexed), '', str(base), ''))
        self.assertEqual(self.find(value), (base, base / 'share/isaac_ros_visual_mapping'))

    def test_relative_prefix_is_rejected_even_after_a_valid_overlay(self):
        base = self.installation('base')
        for value in ('relative/install', os.pathsep.join((str(base), 'relative/install'))):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, '相対 prefix'):
                self.find(value)

    def test_unset_or_empty_environment_is_clear_error(self):
        for value in ('', os.pathsep):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'AMENT_PREFIX_PATH.*空'):
                self.find(value)
        with patch.dict(os.environ, {}, clear=True), self.assertRaisesRegex(ValueError, 'AMENT_PREFIX_PATH'):
            package_paths()

    def test_indexed_package_without_share_fails_instead_of_falling_back(self):
        broken = self.installation('broken', share=False)
        base = self.installation('base')
        with self.assertRaisesRegex(ValueError, 'share ディレクトリがありません'):
            self.find(os.pathsep.join((str(broken), str(base))))

    def test_share_must_be_directory_and_marker_must_be_file(self):
        invalid = self.installation('invalid', share=False)
        (invalid / 'share/isaac_ros_visual_mapping').write_text('file')
        with self.assertRaisesRegex(ValueError, 'share ディレクトリがありません'):
            self.find(str(invalid))
        marker = invalid / 'share/ament_index/resource_index/packages/isaac_ros_visual_mapping'
        marker.unlink()
        marker.mkdir()
        base = self.installation('base')
        self.assertEqual(self.find(os.pathsep.join((str(invalid), str(base)))),
                         (base, base / 'share/isaac_ros_visual_mapping'))

    def test_share_without_package_marker_is_not_a_match(self):
        unindexed = self.installation('unindexed', marker=False)
        with self.assertRaisesRegex(ValueError, 'AMENT_PREFIX_PATH 内に isaac_ros_visual_mapping がありません'):
            self.find(str(unindexed))

    def test_symlink_install_prefix_keeps_ament_prefix_spelling(self):
        base = self.installation('base')
        alias = self.root / 'overlay alias'
        alias.symlink_to(base, target_is_directory=True)
        self.assertEqual(self.find(str(alias)), (alias, alias / 'share/isaac_ros_visual_mapping'))

    def test_isolated_python_without_ros_path_or_commands_can_resolve_package(self):
        prefix = self.installation('isolated')
        code = '''
from pathlib import Path
import sys
sys.path.insert(0, sys.argv[1])
from tools.vgl.build_engine import package_paths
prefix, share = package_paths()
assert prefix == Path(sys.argv[2])
assert share == prefix / 'share/isaac_ros_visual_mapping'
assert not any(name in sys.modules for name in ('ros2cli', 'rclpy', 'ament_index_python'))
'''
        result = subprocess.run([sys.executable, '-I', '-S', '-c', code, str(PROJECT), str(prefix)],
                                env={'AMENT_PREFIX_PATH': str(prefix), 'PATH': ''}, text=True,
                                capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()

import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

from atd_mapping.capture_config import DEFAULT_CAPTURE, resolve_capture
from atd_mapping.capture_snapshot import publisher_signature


class CaptureConfigTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        self.config = Path(__file__).parents[1] / 'config'
        for relative in ('nvidia/cuvslam_map/data.mdb', 'nvidia/cuvgl_map/keyframes/frames_meta.json',
            'nvidia/cuvgl_map/bow_index.pb', 'nvidia/cuvgl_map/vocabulary/voc.pb',
            'models/aliked_lightglue/aliked_x.engine', 'models/aliked_lightglue/lightglue_aliked_x.engine',
            'models/aliked_lightglue/aliked.onnx', 'vgl_runtime_config/keypoint_creation_config.pb.txt',
            'vgl_runtime_config/localizer_config.pb.txt', 'vgl_runtime_config/matching_config.pb.txt',
            'vgl_runtime_config/query_map_config.pb.txt'):
            path = root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('fixture')
        self.job = dict(map_id='map1', map_dir=str(root / 'nvidia/cuvslam_map'), bag=str(root/'bag'), output=str(root/'output'),
            workflow=dict(left_image='/left', right_image='/right', left_info='/left_info', right_info='/right_info',
                          base_frame='base_link', left_frame='left_optical', right_frame='right_optical', ros_domain_id=94,
                          ground_constraint=False),
            capture=dict(DEFAULT_CAPTURE), vgl=dict(map_root=str(root/'nvidia'), model_dir=str(root/'models'),
                          config_dir=str(root/'vgl_runtime_config'), input_shape=[1,3,480,640]))

    def test_explicit_assets_and_remaps_without_bringup_dependency(self):
        called = []
        resolved = resolve_capture(self.config, self.job, lambda engines, shape: called.append((engines, shape)))
        slam, vgl = resolved['parameters']['cuvslam'], resolved['parameters']['vgl']
        self.assertEqual(slam['load_map_folder_path'], str(Path(self.job['map_dir']).resolve()))
        self.assertEqual(slam['save_map_folder_path'], '')
        self.assertFalse(slam['localize_on_startup'])
        self.assertEqual(slam['slam_max_map_size'], 0)
        self.assertTrue(slam['enable_request_hint'])
        self.assertFalse(slam['enable_ground_constraint_in_odometry'])
        self.assertFalse(slam['enable_ground_constraint_in_slam'])
        self.assertEqual(slam['camera_optical_frames'], ['left_optical', 'right_optical'])
        self.assertTrue(slam['use_sim_time'])
        self.assertTrue(vgl['use_sim_time'])
        self.assertFalse(vgl['publish_map_to_odom_tf'])
        self.assertEqual(called[0][1], [1,3,480,640])
        self.assertIn(('/diagnostics','/atd_mapping/vgl_diagnostics'), resolved['remaps']['vgl'])

    def test_no_origin_fallback_and_missing_model_refused(self):
        bad = dict(self.job, vgl=None)
        with self.assertRaisesRegex(ValueError, 'fallback'):
            resolve_capture(self.config, bad)
        model = Path(self.job['vgl']['model_dir']) / 'aliked_lightglue/aliked.onnx'
        model.unlink()
        with self.assertRaisesRegex(ValueError, '通常ファイル'):
            resolve_capture(self.config, self.job)

    def test_ground_constraint_is_explicit_and_matches_recorded_recipe(self):
        self.job['workflow']['ground_constraint'] = True
        resolved = resolve_capture(self.config, self.job)
        self.assertTrue(resolved['parameters']['cuvslam']['enable_ground_constraint_in_odometry'])
        self.assertTrue(resolved['parameters']['cuvslam']['enable_ground_constraint_in_slam'])
        del self.job['workflow']['ground_constraint']
        with self.assertRaisesRegex(ValueError, 'ground_constraint'):
            resolve_capture(self.config, self.job)

    def test_capture_defaults_match_config_and_unknown_options_fail(self):
        self.assertEqual(json.loads((self.config/'capture.json').read_text()), DEFAULT_CAPTURE)
        self.job['capture']['arbitrary_option'] = 1
        with self.assertRaisesRegex(ValueError, '未知'):
            resolve_capture(self.config, self.job)

    def test_duplicate_or_replaced_publishers_are_not_success(self):
        class Node:
            publishers = [SimpleNamespace(node_name='example', node_namespace='/', endpoint_gid=[1,2,3])]
            def get_publishers_info_by_topic(self, topic):
                return self.publishers
        node = Node()
        first = publisher_signature(node, ['/cloud'])
        node.publishers = [SimpleNamespace(node_name='example', node_namespace='/', endpoint_gid=[4,5,6])]
        self.assertNotEqual(publisher_signature(node, ['/cloud']), first)
        node.publishers = node.publishers * 2
        with self.assertRaises(ValueError):
            publisher_signature(node, ['/cloud'])


if __name__ == '__main__':
    unittest.main()

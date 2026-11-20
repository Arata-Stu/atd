import unittest
from types import SimpleNamespace

from atd_mapping.capture_snapshot import capture_readiness, publisher_signature
from atd_mapping.capture_state import require_tf_publisher


HINT = '/atd_mapping/pose_hint'
WORKFLOW = dict(left_image='/left/image', right_image='/right/image',
                left_info='/left/camera_info', right_info='/right/camera_info')
OUTPUTS = ['/tf', '/diagnostics', '/visual_slam/vis/landmarks_cloud',
           '/visual_slam/tracking/slam_path', HINT]


def endpoint(name, gid, namespace='/'):
    return SimpleNamespace(node_name=name, node_namespace=namespace, endpoint_gid=gid)


class CaptureGraphTests(unittest.TestCase):
    def setUp(self):
        self.slam = endpoint('visual_slam', [1, 2])
        self.vgl = endpoint('visual_global_localization', [3, 4])
        self.publishers = {topic: [endpoint('visual_slam', [index + 10])]
                           for index, topic in enumerate(OUTPUTS)}
        self.publishers['/tf'] = [self.vgl, self.slam]
        self.publishers[HINT] = [endpoint('visual_global_localization', [5])]
        self.subscribers = dict.fromkeys([HINT, *WORKFLOW.values()], 2)
        self.node = SimpleNamespace(
            get_node_names=lambda: ['atd_map_collector', 'visual_slam', 'visual_global_localization'],
            count_subscribers=self.subscribers.__getitem__,
            get_publishers_info_by_topic=self.publishers.__getitem__,
        )
        self.client = SimpleNamespace(service_is_ready=lambda: True)

    def readiness(self):
        return capture_readiness(self.node, self.client, WORKFLOW, HINT, OUTPUTS)

    def test_bag_can_start_with_both_nvidia_tf_endpoints(self):
        sources, pending = self.readiness()
        self.assertEqual(pending, [])
        self.assertEqual(sources['/tf'], '0102')
        self.assertEqual(set(sources), set(OUTPUTS))
        self.publishers['/tf'] = [self.slam]
        self.assertEqual(self.readiness(), (sources, []))

    def test_tf_identity_changes_when_slam_writer_restarts(self):
        original = publisher_signature(self.node, ['/tf'])
        self.publishers['/tf'] = [endpoint('visual_slam', [5, 6]), self.vgl]
        self.assertNotEqual(publisher_signature(self.node, ['/tf']), original)

    def test_received_tf_must_identify_the_selected_writer(self):
        selected = publisher_signature(self.node, ['/tf'])['/tf']

        def info(gid):
            return dict(publisher_gid=dict(implementation_identifier='rmw_fastrtps_cpp', data=gid))

        self.assertIsNone(require_tf_publisher(info(bytes(self.slam.endpoint_gid)), selected))
        for message_info in (info(bytes(self.vgl.endpoint_gid)), dict(publisher_gid=None), info(b'\x00\x00')):
            with self.subTest(message_info=message_info), self.assertRaises(ValueError):
                require_tf_publisher(message_info, selected)

    def test_ambiguous_missing_or_unidentified_tf_writer_is_refused(self):
        for publishers in (
            [self.vgl],
            [self.slam, endpoint('visual_slam', [8])],
            [self.slam, self.vgl, endpoint('visual_global_localization', [6])],
            [self.slam, endpoint('visual_global_localization', [9], namespace='/other')],
            [endpoint('visual_slam', [])],
            [endpoint('visual_slam', [0, 0])],
        ):
            with self.subTest(publishers=publishers):
                self.publishers['/tf'] = publishers
                with self.assertRaisesRegex(ValueError, '/tf'):
                    publisher_signature(self.node, ['/tf'])

    def test_pending_conditions_name_all_missing_connections_and_unexpected_writer(self):
        self.subscribers[WORKFLOW['right_info']] = 1
        self.subscribers[HINT] = 0
        self.client.service_is_ready = lambda: False
        self.publishers['/tf'].append(endpoint('rosbag2_player', [7]))
        _, pending = self.readiness()
        details = '\n'.join(pending)
        for item in (WORKFLOW['right_info'], HINT, '/visual_slam/localize_in_map',
                     '/tf', 'rosbag2_player'):
            self.assertIn(item, details)
        self.assertNotIn(WORKFLOW['left_image'], details)

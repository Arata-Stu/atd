import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from atd_bringup.replay_inputs import (
    GROUND_TRUTH_ERROR, GROUND_TRUTH_NODE, GROUND_TRUTH_PATH, SLAM_ODOMETRY, STEREO_WORKFLOW,
)
from atd_bringup.replay_readiness import _missing_endpoints, wait_and_resume


def endpoint(name, kind, qos='compatible', gid=b'\x10\x20'):
    namespace, _, leaf = name.rpartition('/')
    return NS(node_namespace=namespace or '/', node_name=leaf, topic_type=kind, qos_profile=qos, endpoint_gid=gid)


class GateTests(unittest.TestCase):
    def setUp(self):
        self.pubs = {}
        self.subs = {}
        self.player = '/preview/bag'
        self.rviz = '/rviz2'
        for role, topic in STEREO_WORKFLOW.items():
            if role == 'tf_static':
                kind = 'tf2_msgs/msg/TFMessage'
                reader = '/transform_listener_impl_2a'
            else:
                kind = 'sensor_msgs/msg/' + ('Image' if role.endswith('image') else 'CameraInfo')
                reader = '/visual_slam'
            self.pubs[topic] = [endpoint(self.player, kind)]
            self.subs[topic] = [endpoint(reader, kind)]
        self.pubs['/clock'] = [endpoint(self.player, 'rosgraph_msgs/msg/Clock')]
        self.subs['/clock'] = [endpoint('/visual_slam', 'rosgraph_msgs/msg/Clock')]
        self.subs['/visual_slam/tracking/slam_path'] = [endpoint(self.rviz, 'nav_msgs/msg/Path')]
        self.subs['/visual_slam/vis/landmarks_cloud'] = [endpoint(self.rviz, 'sensor_msgs/msg/PointCloud2')]

        self.node = Mock()
        self.node.get_node_names_and_namespaces.return_value = [('bag', '/preview'), ('visual_slam', '/')]
        self.node.get_service_names_and_types.return_value = []
        self.node.get_publishers_info_by_topic.side_effect = self.pubs.__getitem__
        self.node.get_subscriptions_info_by_topic.side_effect = self.subs.__getitem__
        self.paused, self.resume = Mock(), Mock()
        self.node.create_client.side_effect = [self.paused, self.resume]
        self.paused.call_async.return_value.result.return_value = NS(paused=True)
        self.resume.call_async.return_value.result.return_value = NS(return_code=0, error_string='')
        srv = NS(IsPaused=NS(Request=object), Resume=NS(Request=object, Response=NS(RETURN_CODE_SUCCESS=0)))
        qos = NS(QoSCompatibility=NS(ERROR='bad'),
                 qos_check_compatible=lambda pub, sub: ('ok' if pub == sub else 'bad', ''))
        self.ros = Mock()
        modules = patch.dict(sys.modules, {
            'rclpy': self.ros, 'rclpy.qos': qos, 'rosbag2_interfaces.srv': srv,
        })
        modules.start()
        self.addCleanup(modules.stop)

    def test_vio_waits_for_bag_and_tracker_imu_endpoints(self):
        topic, kind = '/camera/camera/imu', 'sensor_msgs/msg/Imu'
        self.pubs[topic], self.subs[topic] = [endpoint(self.player, kind)], []
        self.assertFalse(_missing_endpoints(self.node, self.player, self.rviz))
        self.assertIn(topic, ' '.join(_missing_endpoints(self.node, self.player, self.rviz, vio=True)))
        self.subs[topic].append(endpoint('/visual_slam', kind))
        self.assertFalse(_missing_endpoints(self.node, self.player, self.rviz, vio=True))

    def test_waits_for_configured_rviz_displays_before_resuming(self):
        path = '/visual_slam/tracking/slam_path'
        ready = self.subs[path]
        self.subs[path] = [endpoint('/another/rviz2', 'nav_msgs/msg/Path')]

        def enable_display(node, timeout_sec):
            self.resume.call_async.assert_not_called()
            self.subs[path] = ready

        self.ros.spin_once.side_effect = enable_display
        wait_and_resume(self.node, self.player)
        self.ros.spin_once.assert_called_once()
        self.paused.call_async.assert_called_once()
        self.resume.call_async.assert_called_once()
        self.assertEqual(self.node.destroy_client.call_count, 2)

    def test_stale_other_nodes_and_wrong_qos_do_not_satisfy_gate(self):
        left = STEREO_WORKFLOW['left_image']
        self.pubs[left] = [endpoint(self.player, 'sensor_msgs/msg/CameraInfo')]
        self.subs[left] = [endpoint('/another/visual_slam', 'sensor_msgs/msg/Image')]
        right = STEREO_WORKFLOW['right_info']
        self.subs[right][0].qos_profile = 'incompatible'
        self.subs['/tf_static'] = [endpoint('/rviz2', 'tf2_msgs/msg/TFMessage')]
        self.assertEqual(_missing_endpoints(self.node, self.player, self.rviz), [
            f'{left} の /preview/bag publisher', f'{left} の /visual_slam subscriber',
            f'{right} の QoS が /visual_slam と非互換', '/tf_static の TF listener subscriber',
        ])
        with self.assertRaisesRegex(TimeoutError, 'tf_static'):
            wait_and_resume(self.node, self.player, timeout_s=0)
        self.resume.call_async.assert_not_called()

    def test_unknown_image_writer_becomes_player_before_resume(self):
        topic = STEREO_WORKFLOW['left_image']
        writer = self.pubs[topic][0]
        self.pubs[topic] = [endpoint('_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_', 'sensor_msgs/msg/Image')]

        def discover_owner(node, timeout_sec):
            self.resume.call_async.assert_not_called()
            self.pubs[topic] = [writer]

        self.ros.spin_once.side_effect = discover_owner
        wait_and_resume(self.node, self.player)
        self.ros.spin_once.assert_called_once()
        self.resume.call_async.assert_called_once()

    def test_unresolved_extra_writer_does_not_pass_ready_or_masquerade_as_conflict(self):
        topic = STEREO_WORKFLOW['right_image']
        self.pubs[topic].append(endpoint('_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_', 'sensor_msgs/msg/Image'))
        with patch.dict('os.environ', {'ROS_DOMAIN_ID': '79', 'FASTDDS_DEFAULT_PROFILES_FILE': '/replay.xml'}):
            with self.assertRaises(TimeoutError) as error:
                wait_and_resume(self.node, self.player, timeout_s=0)
        diagnostic = str(error.exception)
        self.assertIn(f'{topic} の配信元を DDS 探索中（GID 10.20）', diagnostic)
        self.assertIn(f'ROS_DOMAIN_ID=79 FASTDDS_DEFAULT_PROFILES_FILE=/replay.xml ros2 topic info --verbose {topic}',
                      diagnostic)
        self.assertNotIn('競合', diagnostic)
        self.resume.call_async.assert_not_called()

    def test_unresolved_clock_becomes_second_identified_publisher(self):
        self.pubs['/clock'].append(endpoint('_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_', 'rosgraph_msgs/msg/Clock'))
        self.ros.spin_once.side_effect = lambda *a, **k: self.pubs['/clock'].__setitem__(
            1, endpoint('/gz_bridge', 'rosgraph_msgs/msg/Clock'))
        with self.assertRaisesRegex(RuntimeError, '/clock.*競合.*gz_bridge'):
            wait_and_resume(self.node, self.player)
        self.resume.call_async.assert_not_called()

    def test_gazebo_overlay_needs_map_odometry_clock_and_matching_rviz_qos(self):
        overlay = '/' + GROUND_TRUTH_NODE
        self.pubs[SLAM_ODOMETRY] = [endpoint('/visual_slam', 'nav_msgs/msg/Odometry')]
        self.subs[SLAM_ODOMETRY] = []
        self.pubs[GROUND_TRUTH_PATH] = [endpoint(overlay, 'nav_msgs/msg/Path')]
        self.subs[GROUND_TRUTH_PATH] = [endpoint(self.rviz, 'nav_msgs/msg/Path', 'incompatible')]
        self.pubs[GROUND_TRUTH_ERROR] = [endpoint(overlay, 'visualization_msgs/msg/Marker')]
        self.subs[GROUND_TRUTH_ERROR] = [endpoint(self.rviz, 'visualization_msgs/msg/Marker')]
        issues = _missing_endpoints(self.node, self.player, self.rviz, ground_truth=True)
        self.assertEqual(issues, [f'/clock の {overlay} subscriber',
                                  f'{SLAM_ODOMETRY} の {overlay} subscriber',
                                  f'{GROUND_TRUTH_PATH} の QoS が {self.rviz} と非互換'])
        self.subs['/clock'].append(endpoint(overlay, 'rosgraph_msgs/msg/Clock'))
        self.subs[SLAM_ODOMETRY] = [endpoint(overlay, 'nav_msgs/msg/Odometry')]
        self.subs[GROUND_TRUTH_PATH] = [endpoint(self.rviz, 'nav_msgs/msg/Path')]
        self.assertFalse(_missing_endpoints(self.node, self.player, self.rviz, ground_truth=True))
        self.pubs[GROUND_TRUTH_PATH].append(endpoint('_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_', 'nav_msgs/msg/Path'))
        self.assertTrue(any(GROUND_TRUTH_PATH in issue and 'DDS 探索中' in issue
                            for issue in _missing_endpoints(self.node, self.player, self.rviz, ground_truth=True)))
        self.pubs[GROUND_TRUTH_PATH].pop()
        self.pubs[GROUND_TRUTH_PATH].append(endpoint('/old_replay', 'nav_msgs/msg/Path'))
        with self.assertRaisesRegex(RuntimeError, GROUND_TRUTH_PATH):
            wait_and_resume(self.node, self.player, ground_truth=True)
        self.resume.call_async.assert_not_called()

    def test_headless_resumes_only_after_gt_clock_and_odometry_are_ready(self):
        overlay = '/' + GROUND_TRUTH_NODE
        del self.subs['/visual_slam/tracking/slam_path'], self.subs['/visual_slam/vis/landmarks_cloud']
        self.pubs[SLAM_ODOMETRY] = [endpoint('/visual_slam', 'nav_msgs/msg/Odometry')]
        self.subs[SLAM_ODOMETRY] = []
        self.assertEqual(_missing_endpoints(self.node, self.player, None, ground_truth=True), [
            f'/clock の {overlay} subscriber', f'{SLAM_ODOMETRY} の {overlay} subscriber',
        ])
        self.subs['/clock'].append(endpoint(overlay, 'rosgraph_msgs/msg/Clock'))
        self.subs[SLAM_ODOMETRY].append(endpoint(overlay, 'nav_msgs/msg/Odometry'))
        wait_and_resume(self.node, self.player, rviz_name=None, ground_truth=True)
        self.resume.call_async.assert_called_once()

    def test_conflicting_clock_discovered_during_pause_query_never_resumes(self):
        def another_clock():
            self.pubs['/clock'].append(endpoint('/other/bag', 'rosgraph_msgs/msg/Clock'))
            return NS(paused=True)

        self.paused.call_async.return_value.result.side_effect = another_clock
        with self.assertRaisesRegex(RuntimeError, '/clock.*競合.*other/bag'):
            wait_and_resume(self.node, self.player)
        self.resume.call_async.assert_not_called()

    def test_two_players_with_the_same_name_never_receive_resume(self):
        self.node.get_node_names_and_namespaces.return_value = [('bag', '/preview')] * 2
        with self.assertRaisesRegex(RuntimeError, '/preview/bag.*複数'):
            wait_and_resume(self.node, self.player)
        self.resume.call_async.assert_not_called()

    def test_second_tracker_appears_before_resume(self):
        def another_tracker():
            self.node.get_node_names_and_namespaces.return_value.append(('visual_slam', '/'))
            return NS(paused=True)

        self.paused.call_async.return_value.result.side_effect = another_tracker
        with self.assertRaisesRegex(RuntimeError, '/visual_slam.*複数'):
            wait_and_resume(self.node, self.player)
        self.resume.call_async.assert_not_called()

    def test_tracker_static_broadcaster_can_coexist_but_another_bag_cannot(self):
        static = self.pubs['/tf_static']
        static.append(endpoint('/visual_slam', 'tf2_msgs/msg/TFMessage'))
        self.assertFalse(_missing_endpoints(self.node, self.player, self.rviz))
        static.append(endpoint('/other/bag', 'tf2_msgs/msg/TFMessage'))
        self.node.get_service_names_and_types.return_value = [
            ('/other/bag/is_paused', ['rosbag2_interfaces/srv/IsPaused'])]
        with self.assertRaisesRegex(RuntimeError, '/tf_static.*競合.*other/bag'):
            wait_and_resume(self.node, self.player)
        self.resume.call_async.assert_not_called()

    def test_requires_paused_state(self):
        self.paused.call_async.return_value.result.return_value.paused = False
        with self.assertRaisesRegex(RuntimeError, '一時停止'):
            wait_and_resume(self.node, self.player)
        self.resume.call_async.assert_not_called()

    def test_child_failure_while_waiting_never_resumes(self):
        self.resume.service_is_ready.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'RViz 終了'):
            wait_and_resume(self.node, self.player, check_running=Mock(side_effect=RuntimeError('RViz 終了')))
        self.resume.call_async.assert_not_called()

    def test_short_bag_can_finish_before_resume_response_arrives(self):
        self.resume.call_async.return_value.done.side_effect = [False, True]

        def check():
            self.resume.call_async.assert_not_called()

        wait_and_resume(self.node, self.player, check_running=check)
        self.ros.spin_once.assert_called_once()

    def test_failed_service_response_is_not_reported_as_playback(self):
        self.resume.call_async.return_value.result.return_value = NS(return_code=3, error_string='failed')
        with self.assertRaisesRegex(RuntimeError, 'failed'):
            wait_and_resume(self.node, self.player)

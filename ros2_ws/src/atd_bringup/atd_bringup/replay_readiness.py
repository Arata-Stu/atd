"""停止中の bag player と cuVSLAM・RViz の購読準備を確認してから再生する。"""

from atd_localization.replay_player import (
    _check_publishers, _connection_issues, _endpoints, _graph_command, _name, resume_when_connected,
)

from atd_bringup.replay_inputs import (
    GROUND_TRUTH_ERROR, GROUND_TRUTH_NODE, GROUND_TRUTH_PATH, IMU_TOPIC, SLAM_ODOMETRY, STEREO_WORKFLOW,
)


def _missing_endpoints(node, player_name, rviz_name, ground_truth=False, vio=False, orb=None):
    inputs = {
        STEREO_WORKFLOW['left_image']: 'sensor_msgs/msg/Image',
        STEREO_WORKFLOW['right_image']: 'sensor_msgs/msg/Image',
        STEREO_WORKFLOW['left_info']: 'sensor_msgs/msg/CameraInfo',
        STEREO_WORKFLOW['right_info']: 'sensor_msgs/msg/CameraInfo',
        STEREO_WORKFLOW['tf_static']: 'tf2_msgs/msg/TFMessage',
        '/clock': 'rosgraph_msgs/msg/Clock',
    }
    if vio:
        inputs[IMU_TOPIC] = 'sensor_msgs/msg/Imu'
    missing = []
    nodes = [_name(name, namespace) for name, namespace in node.get_node_names_and_namespaces()]
    overlay = '/' + GROUND_TRUTH_NODE
    tracker = orb.tracker if orb else '/visual_slam'
    required_nodes = [player_name, tracker]
    if orb:
        required_nodes.append(orb.sensor_source)
    if ground_truth:
        required_nodes.append(overlay)
    for name in required_nodes:
        if nodes.count(name) > 1:
            raise RuntimeError(f'同じ ROS domain に {name} が複数あります。Notebook コンテナで確認してください。'
                               f'\n{_graph_command()}')
    for topic, kind in inputs.items():
        producer = player_name
        if orb and topic != '/clock':
            producer = orb.seed_source if topic in orb.seed_topics else orb.sensor_source
        publishers = _check_publishers(node, topic, kind, producer, missing)
        infos = node.get_subscriptions_info_by_topic(topic)
        if topic == STEREO_WORKFLOW['tf_static']:
            # cuVSLAM builds its TF buffer with tf2's independently named listener node.
            subscribers = [sub for sub in _endpoints(infos, kind, None)
                           if sub.node_name.startswith('transform_listener_impl_')
                           or _name(sub.node_name, sub.node_namespace) == tracker]
            consumer = 'TF listener'
        else:
            consumer = tracker
            subscribers = _endpoints(infos, kind, consumer)
        missing.extend(_connection_issues(topic, publishers, subscribers, producer, consumer))
        if ground_truth and topic == '/clock':
            readers = _endpoints(infos, kind, overlay)
            missing.extend(_connection_issues(topic, publishers, readers, player_name, overlay))

    if rviz_name:
        # RViz creates both display subscriptions while loading the preset, before the first TF.
        for topic, kind in ((SLAM_ODOMETRY, 'nav_msgs/msg/Odometry'),
                            ('/visual_slam/vis/landmarks_cloud', 'sensor_msgs/msg/PointCloud2')):
            if orb and kind == 'sensor_msgs/msg/PointCloud2':
                continue
            if not _endpoints(node.get_subscriptions_info_by_topic(topic), kind, rviz_name):
                missing.append(f'{topic} の {rviz_name} subscriber')
    links = []
    if ground_truth:
        links.append((SLAM_ODOMETRY, 'nav_msgs/msg/Odometry', tracker, overlay))
        if rviz_name:
            links.extend([(GROUND_TRUTH_PATH, 'nav_msgs/msg/Path', overlay, rviz_name),
                          (GROUND_TRUTH_ERROR, 'visualization_msgs/msg/Marker', overlay, rviz_name)])
    if orb:
        links.append((orb.sensor_source + '/finished', 'std_msgs/msg/Bool', orb.sensor_source, orb.seed_source))
        feedback = [(SLAM_ODOMETRY, 'nav_msgs/msg/Odometry'), ('/orbslam3/ready', 'std_msgs/msg/Bool'),
                    ('/orbslam3/processed', 'std_msgs/msg/Header')]
        if vio:
            feedback.append(('/orbslam3/imu_skipped', 'std_msgs/msg/Header'))
        if orb.metadata:
            feedback.append(('/localization/status', 'atd_interfaces/msg/LocalizationStatus'))
        links.extend((topic, kind, tracker, orb.seed_source) for topic, kind in feedback)
        if orb.gyro_topic:
            from .orb_gap_bridge import CONTINUOUS_ODOMETRY, FRAME_STATUS
            links.extend([(FRAME_STATUS, 'atd_interfaces/msg/LocalizationStatus', tracker, orb.seed_source),
                          (orb.gyro_topic, 'sensor_msgs/msg/Imu', orb.sensor_source, orb.seed_source),
                          (STEREO_WORKFLOW['tf_static'], 'tf2_msgs/msg/TFMessage', orb.seed_source, orb.seed_source)])
            if rviz_name:
                links.append((CONTINUOUS_ODOMETRY, 'nav_msgs/msg/Odometry', orb.seed_source, rviz_name))
    for topic, kind, producer, consumer in links:
        publishers = _check_publishers(node, topic, kind, producer, missing)
        missing.extend(_connection_issues(
            topic, publishers,
            _endpoints(node.get_subscriptions_info_by_topic(topic), kind, consumer), producer, consumer))
    return list(dict.fromkeys(missing))


def wait_and_resume(node, player_name='/rosbag2_player', timeout_s=90.0, *,
                    rviz_name='/rviz2', check_running=None, ground_truth=False, vio=False, orb=None):
    player_name = '/' + player_name.strip('/')
    if rviz_name:
        rviz_name = '/' + rviz_name.strip('/')
    def missing():
        return _missing_endpoints(node, player_name, rviz_name, ground_truth, vio, orb)

    initialize = (lambda: orb.seed(check_running)) if orb else None
    resume_when_connected(node, player_name, timeout_s, missing, check_running, initialize)

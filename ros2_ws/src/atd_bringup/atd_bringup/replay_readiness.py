"""停止中の bag player と cuVSLAM・RViz の購読準備を確認してから再生する。"""

from time import monotonic

from atd_bringup.replay_inputs import (
    GROUND_TRUTH_ERROR, GROUND_TRUTH_NODE, GROUND_TRUTH_PATH, SLAM_ODOMETRY, STEREO_WORKFLOW,
)


def _name(name, namespace):
    return f'{namespace.rstrip("/")}/{name}'


def _endpoints(infos, kind, node_name):
    return [info for info in infos if info.topic_type == kind and (
        node_name is None or _name(info.node_name, info.node_namespace) == node_name)]


def _connection_issues(topic, publishers, subscribers, producer, consumer):
    from rclpy.qos import QoSCompatibility, qos_check_compatible

    missing = []
    if not publishers:
        missing.append(f'{topic} の {producer} publisher')
    if not subscribers:
        missing.append(f'{topic} の {consumer} subscriber')
    if publishers and subscribers and all(
        qos_check_compatible(pub.qos_profile, sub.qos_profile)[0] == QoSCompatibility.ERROR
        for pub in publishers for sub in subscribers
    ):
        missing.append(f'{topic} の QoS が {consumer} と非互換')
    return missing


def _missing_endpoints(node, player_name, rviz_name, ground_truth=False):
    inputs = {
        STEREO_WORKFLOW['left_image']: 'sensor_msgs/msg/Image',
        STEREO_WORKFLOW['right_image']: 'sensor_msgs/msg/Image',
        STEREO_WORKFLOW['left_info']: 'sensor_msgs/msg/CameraInfo',
        STEREO_WORKFLOW['right_info']: 'sensor_msgs/msg/CameraInfo',
        STEREO_WORKFLOW['tf_static']: 'tf2_msgs/msg/TFMessage',
        '/clock': 'rosgraph_msgs/msg/Clock',
    }
    missing = []
    nodes = [_name(name, namespace) for name, namespace in node.get_node_names_and_namespaces()]
    overlay = '/' + GROUND_TRUTH_NODE
    required_nodes = [player_name, '/visual_slam']
    if ground_truth:
        required_nodes.append(overlay)
    for name in required_nodes:
        if nodes.count(name) > 1:
            raise RuntimeError(f'同じ ROS domain に {name} が複数あります。別の再生・追跡セッションを終了してください')
    for topic, kind in inputs.items():
        offered = node.get_publishers_info_by_topic(topic)
        publishers = _endpoints(offered, kind, player_name)
        others = {_name(info.node_name, info.node_namespace) for info in offered} - {player_name}
        if others and topic == STEREO_WORKFLOW['tf_static']:
            # cuVSLAM also advertises /tf_static. Only competing bag players matter here.
            services = dict(node.get_service_names_and_types())
            others = {name for name in others if 'rosbag2_interfaces/srv/IsPaused'
                      in services.get(name + '/is_paused', [])}
        if len(publishers) > 1 or others:
            sources = '、'.join(sorted(others or {player_name}))
            raise RuntimeError(f'{topic} の配信元が競合しています（{sources}）。別の配信を終了してください')
        infos = node.get_subscriptions_info_by_topic(topic)
        if topic == STEREO_WORKFLOW['tf_static']:
            # cuVSLAM builds its TF buffer with tf2's independently named listener node.
            subscribers = [sub for sub in _endpoints(infos, kind, None)
                           if sub.node_name.startswith('transform_listener_impl_')
                           or _name(sub.node_name, sub.node_namespace) == '/visual_slam']
            consumer = 'TF listener'
        else:
            consumer = '/visual_slam'
            subscribers = _endpoints(infos, kind, consumer)
        missing.extend(_connection_issues(topic, publishers, subscribers, player_name, consumer))
        if ground_truth and topic == '/clock':
            readers = _endpoints(infos, kind, overlay)
            missing.extend(_connection_issues(topic, publishers, readers, player_name, overlay))

    # RViz creates both display subscriptions while loading the preset, before the first TF.
    for topic, kind in (('/visual_slam/tracking/slam_path', 'nav_msgs/msg/Path'),
                        ('/visual_slam/vis/landmarks_cloud', 'sensor_msgs/msg/PointCloud2')):
        if not _endpoints(node.get_subscriptions_info_by_topic(topic), kind, rviz_name):
            missing.append(f'{topic} の {rviz_name} subscriber')
    if ground_truth:
        for topic, kind, producer, consumer in (
            (SLAM_ODOMETRY, 'nav_msgs/msg/Odometry', '/visual_slam', overlay),
            (GROUND_TRUTH_PATH, 'nav_msgs/msg/Path', overlay, rviz_name),
            (GROUND_TRUTH_ERROR, 'visualization_msgs/msg/Marker', overlay, rviz_name),
        ):
            infos = node.get_publishers_info_by_topic(topic)
            if any(_name(info.node_name, info.node_namespace) != producer for info in infos):
                raise RuntimeError(f'{topic} に別の配信元があります。別の追跡・再生セッションを終了してください')
            missing.extend(_connection_issues(
                topic, _endpoints(infos, kind, producer),
                _endpoints(node.get_subscriptions_info_by_topic(topic), kind, consumer), producer, consumer))
    return list(dict.fromkeys(missing))


def wait_and_resume(node, player_name='/rosbag2_player', timeout_s=90.0, *,
                    rviz_name='/rviz2', check_running=None, ground_truth=False):
    """再生開始の応答まで待つ。check_running は子プロセスの異常終了時に例外を送出する。"""
    import rclpy
    from rosbag2_interfaces.srv import IsPaused, Resume

    player_name = '/' + player_name.strip('/')
    rviz_name = '/' + rviz_name.strip('/')
    deadline = monotonic() + timeout_s

    def poll(reason, *, check_children=True):
        if check_children and check_running is not None:
            check_running()
        remaining = deadline - monotonic()
        if remaining <= 0:
            raise TimeoutError(f'bag 再生の準備がタイムアウトしました（待機中: {reason}）')
        rclpy.spin_once(node, timeout_sec=min(0.15, remaining))

    def call(client, request, service, *, check_children=True):
        future = client.call_async(request)
        while not future.done():
            poll(service, check_children=check_children)
        return future.result()

    paused = node.create_client(IsPaused, player_name + '/is_paused')
    resume = node.create_client(Resume, player_name + '/resume')
    try:
        while True:
            waiting = _missing_endpoints(node, player_name, rviz_name, ground_truth)
            if not paused.service_is_ready():
                waiting.append(player_name + '/is_paused')
            if not resume.service_is_ready():
                waiting.append(player_name + '/resume')
            if not waiting:
                if not call(paused, IsPaused.Request(), player_name + '/is_paused').paused:
                    raise RuntimeError(f'{player_name} が開始待ちの一時停止状態ではありません')
                waiting = _missing_endpoints(node, player_name, rviz_name, ground_truth)
                if not waiting:
                    break
            poll('、'.join(waiting))

        if check_running is not None:
            check_running()
        # A short bag may reach EOF while the final service reply is in flight.
        result = call(resume, Resume.Request(), player_name + '/resume', check_children=False)
        if result.return_code != Resume.Response.RETURN_CODE_SUCCESS:
            raise RuntimeError(f'{player_name} の再生開始に失敗しました: {result.error_string}')
    finally:
        node.destroy_client(resume)
        node.destroy_client(paused)

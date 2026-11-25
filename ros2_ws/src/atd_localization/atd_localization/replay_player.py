"""rosbag2 再生の接続・QoS・一時停止状態を検証する。"""
import os
import shlex
from time import monotonic


def _name(name, namespace):
    return f'{namespace.rstrip("/")}/{name}'


def _endpoints(infos, kind, node_name):
    return [info for info in infos if info.topic_type == kind and (
        node_name is None or _name(info.node_name, info.node_namespace) == node_name)]


def _graph_command(topic=None):
    env = ['env', 'ROS_DOMAIN_ID=' + os.environ.get('ROS_DOMAIN_ID', '0')]
    if profile := os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE'):
        env.append('FASTDDS_DEFAULT_PROFILES_FILE=' + profile)
    query = ['topic', 'info', '--verbose', topic] if topic else ['node', 'list']
    return shlex.join([*env, 'ros2', *query, '--no-daemon', '--spin-time', '2'])


def _check_publishers(node, topic, kind, producer, missing):
    offered = node.get_publishers_info_by_topic(topic)
    owners = []
    for info in offered:
        # Fast DDS can announce a writer before rmw_dds_common associates its GID
        # with a ROS node. An unnamed writer cannot establish either owner or conflict.
        if (info.node_name in ('', '_NODE_NAME_UNKNOWN_', '_CREATED_BY_BARE_DDS_APP_')
                or info.node_namespace in ('', '_NODE_NAMESPACE_UNKNOWN_', '_CREATED_BY_BARE_DDS_APP_')):
            gid = bytes(info.endpoint_gid).hex('.')
            missing.append(f'{topic} の配信元を DDS 探索中（GID {gid}）')
        else:
            owners.append(_name(info.node_name, info.node_namespace))

    others = set(owners) - {producer}
    if others and topic == '/tf_static':
        # cuVSLAM also advertises /tf_static. Only competing bag players matter here.
        services = dict(node.get_service_names_and_types())
        others = {name for name in others if 'rosbag2_interfaces/srv/IsPaused'
                  in services.get(name + '/is_paused', [])}
    if owners.count(producer) > 1 or others:
        sources = '、'.join(sorted(others or {producer}))
        raise RuntimeError(f'{topic} の配信元が競合しています（{sources}、再生の配信元 {producer}）。'
                           f'Notebook コンテナで確認してください。\n{_graph_command(topic)}')
    return _endpoints(offered, kind, producer)


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


def resume_when_connected(node, player_name, timeout_s, missing_endpoints, check_running, initialize=None):
    """必要な接続と初期化を検証したうえで、一時停止中の bag を再生する。"""
    import rclpy
    from rosbag2_interfaces.srv import IsPaused, Resume

    player_name = '/' + player_name.strip('/')
    start = monotonic()
    deadline = start + timeout_s
    notified = False

    def poll(reason, *, check_children=True):
        nonlocal notified
        if check_children and check_running is not None:
            check_running()
        now = monotonic()
        remaining = deadline - now
        first_word = reason.split(' ', 1)[0]
        command = _graph_command(first_word) if first_word.startswith('/') and ' の' in reason else _graph_command()
        if remaining <= 0:
            raise TimeoutError(f'bag 再生の準備がタイムアウトしました（待機中: {reason}）。'
                               f'Notebook コンテナで探索状態を確認できます。\n{command}')
        if not notified and now - start >= 3:
            node.get_logger().info(f'bag 再生の準備を待っています（{reason}）。'
                                   f'待機中に別端末の Notebook コンテナで確認できます。\n{command}')
            notified = True
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
            waiting = missing_endpoints()
            if not paused.service_is_ready():
                waiting.append(player_name + '/is_paused')
            if not resume.service_is_ready():
                waiting.append(player_name + '/resume')
            if not waiting:
                if not call(paused, IsPaused.Request(), player_name + '/is_paused').paused:
                    raise RuntimeError(f'{player_name} が開始待ちの一時停止状態ではありません')
                waiting = missing_endpoints()
                if not waiting:
                    break
            poll('、'.join(waiting))

        if check_running is not None:
            check_running()
        if initialize is not None:
            initialize()
            deadline = monotonic() + timeout_s
        # A short bag may reach EOF while the final service reply is in flight.
        result = call(resume, Resume.Request(), player_name + '/resume', check_children=False)
        if result.return_code != Resume.Response.RETURN_CODE_SUCCESS:
            raise RuntimeError(f'{player_name} の再生開始に失敗しました: {result.error_string}')
    finally:
        node.destroy_client(resume)
        node.destroy_client(paused)

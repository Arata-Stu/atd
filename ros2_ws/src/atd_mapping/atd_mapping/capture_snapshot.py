"""保存地図の複製へVGLでlocalizeし、末尾処理を確認してsnapshotを保存する。"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from .replay_dds import select_replay_dds


def stop(process):
    """root worker の5秒grace以内で、launchの子プロセス群も片付ける。"""
    if not process or process.poll() is not None:
        return
    for signum, timeout in ((signal.SIGINT, 1.), (signal.SIGTERM, .5), (signal.SIGKILL, .5)):
        try:
            os.killpg(process.pid, signum)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue
    raise ValueError('capture子プロセスの終了を確認できません')


def last_stereo_stamp(bag, workflow, shape):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    from .frames import stamp_ns
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag)), rosbag2_py.ConverterOptions('', ''))
    topics = [workflow['left_image'], workflow['right_image']]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    last = {}
    counts = dict.fromkeys(topics, 0)
    max_gaps = dict.fromkeys(topics, 0)
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        image = deserialize_message(raw, Image)
        if [image.height, image.width] != shape[2:]:
            raise ValueError('VGLモデルとbag画像の解像度が一致しません: ' + topic)
        stamp = stamp_ns(image.header.stamp)
        if stamp <= 0:
            raise ValueError('左右画像stampが不正です: ' + topic)
        previous = last.get(topic, stamp)
        max_gaps[topic] = max(max_gaps[topic], stamp - previous)
        last[topic] = max(previous, stamp)
        counts[topic] += 1
    if len(last) != 2:
        raise ValueError('左右画像の有効な末尾stampがありません')
    summary = '、'.join(f'{side} {counts[topic]} 枚・最大間隔 {max_gaps[topic] / 1e6:.1f} ms'
                        for side, topic in zip(('左', '右'), topics))
    print(f'bag内の画像stamp: {summary}', flush=True)
    return min(last.values())


def publisher_gid(node, topic):
    publishers = node.get_publishers_info_by_topic(topic)
    names = [f'{p.node_namespace.rstrip("/")}/{p.node_name}' for p in publishers]
    if topic == '/tf':
        # VGL registers a broadcaster even when both of its TF outputs are disabled.
        allowed = {'/visual_slam', '/visual_global_localization'}
        if (names.count('/visual_slam') != 1 or names.count('/visual_global_localization') > 1
                or set(names) - allowed):
            raise ValueError(f'/tf の発行元が起動構成と一致しません（/visual_slam は1つ、'
                             f'/visual_global_localization は最大1つ、検出: {names}）')
        publisher = publishers[names.index('/visual_slam')]
    else:
        if len(publishers) != 1:
            raise ValueError(f'{topic} の発行元を一意に確認できません（検出: {names}）')
        publisher = publishers[0]
    gid = bytes(publisher.endpoint_gid)
    if not any(gid):
        raise ValueError(f'{topic} の publisher GID を確認できません')
    return gid.hex()


def publisher_signature(node, topics):
    return {topic: publisher_gid(node, topic) for topic in topics}


def capture_readiness(node, client, workflow, hint_topic, outputs):
    pending = []
    if not client.service_is_ready():
        pending.append('service /visual_slam/localize_in_map')
    if 'visual_global_localization' not in node.get_node_names():
        pending.append('node /visual_global_localization')
    stereo = [workflow[key] for key in ('left_image', 'right_image', 'left_info', 'right_info')]
    for topic in [hint_topic, *stereo]:
        count = node.count_subscribers(topic)
        if count < 2:
            pending.append(f'{topic} の購読者 {count}/2')
    sources = {}
    for topic in outputs:
        try:
            sources[topic] = publisher_gid(node, topic)
        except ValueError as exc:
            pending.append(str(exc))
    return sources, pending


def drain_capture(node, launch, spin_once, expected, tolerance, options):
    """Let queued diagnostics, hints, cloud, path and TF arrive after playback."""
    deadline = time.monotonic() + options['drain_timeout_s']
    while True:
        if launch.poll() is not None:
            raise ValueError('点群収集中にVSLAMが終了しました')
        spin_once(node, timeout_sec=.1)
        now = time.monotonic()
        quiet_s = now - node.last_update
        if quiet_s < options['settle_s'] and now < deadline:
            continue
        try:
            node.check_final_inputs(expected, tolerance, hint_required=True)
        except ValueError:
            if now >= deadline:
                raise
            continue
        if quiet_s >= options['settle_s']:
            return
        raise ValueError(f"末尾データは揃いましたが点群・軌跡・TFの更新が続いています"
                         f"（最終更新から {quiet_s:.3f} 秒、必要 {options['settle_s']:.3f} 秒）")


def run(job_file):
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from isaac_ros_visual_slam_interfaces.srv import LocalizeInMap
    from .bag_inputs import inspect_bag
    from .capture_collector import CaptureCollector
    from .capture_config import resolve_capture, inspect_engines
    select_replay_dds()
    job = json.loads(job_file.read_text())
    config = Path(get_package_share_directory('atd_mapping')) / 'config'
    resolved = resolve_capture(config, job, inspect_engines)
    workflow, options = resolved['workflow'], resolved['capture']
    bag, output = Path(job['bag']), Path(job['output'])
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) != workflow['ros_domain_id']:
        raise ValueError('隔離 ROS domain がjobと一致しません')
    inputs, metadata = inspect_bag(bag, workflow, resolved['parameters']['cuvslam'])
    expected = last_stereo_stamp(bag, workflow, resolved['input_shape'])
    duration = metadata.get('duration', {}).get('nanoseconds')
    if type(duration) is not int or duration <= 0:
        raise ValueError('bag duration が不正です')
    if not output.is_dir():
        raise ValueError('capture出力directoryを準備してください')
    rclpy.init()
    node = CaptureCollector(hint_topic=resolved['hint_topic'])
    launch = player = None
    previous_handler = signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        discovery_deadline = time.monotonic() + 1.5
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if set(node.get_node_names()) & {'visual_slam', 'visual_global_localization', 'atd_mapping_container'}:
                raise ValueError('隔離domainに既存のVSLAM/VGLがあります')
        launch = subprocess.Popen(['ros2', 'launch', 'atd_mapping', 'capture.launch.py', f'job_file:={job_file}'], start_new_session=True)
        client = node.create_client(LocalizeInMap, '/visual_slam/localize_in_map')
        outputs = ['/diagnostics', '/tf', '/visual_slam/vis/landmarks_cloud',
                   '/visual_slam/tracking/slam_path', resolved['hint_topic']]
        deadline = time.monotonic() + options['ready_timeout_s']
        next_report = time.monotonic() + 5.
        while True:
            if launch.poll() is not None:
                raise ValueError(f'bag再生前にVSLAM/VGL launchが終了しました（終了コード {launch.returncode}）')
            rclpy.spin_once(node, timeout_sec=.1)
            sources, pending = capture_readiness(node, client, workflow, resolved['hint_topic'], outputs)
            if not pending:
                node.tf_publisher_gid = sources['/tf']
                break
            now = time.monotonic()
            if now >= deadline:
                raise ValueError('bag再生前の接続待ちがタイムアウトしました: ' + '、'.join(pending))
            if now >= next_report:
                print('bag再生前の接続待ち: ' + '、'.join(pending), flush=True)
                next_report = now + 15.
        print(f'VSLAM/VGL の接続を確認しました。bag再生を開始します: {bag}', flush=True)
        player = subprocess.Popen(['ros2', 'bag', 'play', str(bag), '--clock', '--rate', str(options['replay_rate']),
            '--qos-profile-overrides-path', str(config / 'replay_qos.yaml'), '--topics', *inputs], start_new_session=True)
        deadline = time.monotonic() + duration / 1e9 / options['replay_rate'] * 2 + options['ready_timeout_s']
        next_identity_check = time.monotonic() + 1.
        while player.poll() is None:
            if launch.poll() is not None or time.monotonic() > deadline:
                raise ValueError('bag再生中にVSLAM終了またはtimeout')
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() >= next_identity_check:
                if publisher_signature(node, outputs) != sources:
                    raise ValueError('capture中にpublisherが再接続/交代しました。再取得してください')
                next_identity_check = time.monotonic() + 1.
        if player.returncode:
            raise ValueError('bag再生が失敗しました')
        tolerance = int(options['tail_tolerance_s'] * 1e9)
        drain_capture(node, launch, rclpy.spin_once, expected, tolerance, options)
        if publisher_signature(node, outputs) != sources:
            raise ValueError('capture出力publisherの識別情報が変わりました')
        snapshot = node.final_snapshot(expected, tolerance, dict(source='saved_cuvslam_map_replay',
            initial_localization='vgl_pose_hint', vgl_input_shape=resolved['input_shape'],
            bag=str(bag), map_id=job['map_id'], workflow=workflow, capture=options,
            parameters=resolved['parameters'], assets=resolved['assets'], publisher_gids=sources))
        temporary = output / '.snapshot.tmp'
        with temporary.open('w') as stream:
            json.dump(snapshot, stream, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output / 'snapshot.json')
        print(f"保存地図にlocalizeした点群: {len(snapshot['points'])} points", flush=True)
    finally:
        try:
            stop(player)
        finally:
            try:
                stop(launch)
            finally:
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                signal.signal(signal.SIGTERM, previous_handler)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--job', required=True)
    run(Path(parser.parse_args().job).resolve())


if __name__ == '__main__':
    main()

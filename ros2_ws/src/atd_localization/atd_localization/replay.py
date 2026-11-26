"""Notebook の隔離 ROS domain で別走行を再生し、オンラインと共通の Localization を検証する。"""
import argparse
from contextlib import ExitStack
import csv
from datetime import datetime, timezone
import fcntl
import json
import math
import os
import re
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

from atd_mapping.capture_snapshot import publisher_signature, stop
from atd_mapping.frames import stamp_ns

from .replay_dds import replay_dds_profile
from .replay_inputs import independent_bag, inspect_inputs
from .replay_report import TrackingReport
from .replay_player import _check_publishers, _connection_issues, _endpoints, resume_when_connected

STATUS = '/localization/status'
DOMAIN = 95


def choose(label, choices):
    choices = sorted(choices)
    if not choices:
        raise ValueError(label + 'の候補がありません')
    if not sys.stdin.isatty():
        raise ValueError('非対話端末では --map と --bag を明示してください')
    if shutil.which('fzf'):
        selected = subprocess.run(['fzf', '--prompt', label + '> '], input='\n'.join(choices),
                                  text=True, stdout=subprocess.PIPE)
        if selected.returncode or selected.stdout.rstrip('\n') not in choices:
            raise ValueError('選択を中止しました')
        return selected.stdout.rstrip('\n')
    for index, value in enumerate(choices, 1):
        print(f'{index}. {value}')
    answer = input(label + ' の番号: ').strip()
    if not answer.isdecimal() or not 1 <= int(answer) <= len(choices):
        raise ValueError('候補の番号を選択してください')
    return choices[int(answer) - 1]


def launch_command(results, map_id, engine):
    args = ['ros2', 'launch', 'atd_localization', 'localization.launch.py',
            'map_id:=' + map_id, 'results_root:=' + str(results), 'use_sim_time:=true']
    if engine:
        args.append('engine_dir:=' + engine)
    return args


def play_command(bag, topics, qos, rate, *, paused_player=None):
    return ['ros2', 'bag', 'play', str(bag), '--clock', '--rate', str(rate),
            '--qos-profile-overrides-path', str(qos),
            *(['--start-paused', '--disable-keyboard-controls', '--remap', f'__node:={paused_player}'] if paused_player else []),
            '--topics', *topics]


def validation_connections(node, player_name, workflow, *, orb, observe_images=False):
    """停止中の bag と実際に追跡・再定位を行う購読先の型・QoS を確認する。"""
    cameras = ['/orbslam3_node'] if orb else ['/visual_slam', '/visual_global_localization']
    clocks = cameras if orb else [*cameras, '/atd_localization_monitor']
    readers = [*cameras, node.get_fully_qualified_name()] if observe_images else cameras
    links = [(workflow[side + '_image'], 'sensor_msgs/msg/Image', readers) for side in ('left', 'right')]
    if not orb:
        # These component names and remapped CameraInfo inputs are shared with localization.launch.py.
        links += [(workflow[side + '_info'], 'sensor_msgs/msg/CameraInfo', clocks) for side in ('left', 'right')]
        # tf2 can create its own listener node; validate the actual typed reader in this isolated domain.
        links.append((workflow['tf_static'], 'tf2_msgs/msg/TFMessage', [None]))
    links.append(('/clock', 'rosgraph_msgs/msg/Clock', clocks))
    issues = []
    for topic, kind, consumers in links:
        pubs = _check_publishers(node, topic, kind, player_name, issues)
        readers = node.get_subscriptions_info_by_topic(topic)
        for consumer in consumers:
            subs = _endpoints(readers, kind, consumer)
            issues.extend(_connection_issues(topic, pubs, subs, player_name, consumer or 'TF listener'))
    return issues


def trace_connections(node):
    """画像の処理時間を配信するノードと診断用の購読先を開始前に確認する。"""
    missing = []
    observer = node.get_fully_qualified_name()
    links = [('/visual_slam/status', 'isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus',
              '/visual_slam', [observer]),
             ('/visual_slam/tracking/odometry', 'nav_msgs/msg/Odometry', '/visual_slam', [observer]),
             ('/localization/pose_hint', 'geometry_msgs/msg/PoseWithCovarianceStamped', '/visual_global_localization',
              [observer, '/atd_localization_monitor', '/visual_slam'])]
    for topic, kind, source, consumers in links:
        publishers = _check_publishers(node, topic, kind, source, missing)
        subscribers = node.get_subscriptions_info_by_topic(topic)
        for consumer in consumers:
            missing.extend(_connection_issues(topic, publishers, _endpoints(subscribers, kind, consumer),
                                               source, consumer))
    return missing


def run_ros(args, meta, bag, topics, duration, report, output, *, trace=None):
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from atd_interfaces.msg import LocalizationStatus
    from rclpy.signals import SignalHandlerOptions

    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')
    def interrupt(*_):
        raise KeyboardInterrupt
    old_handler = signal.signal(signal.SIGTERM, interrupt)
    with ExitStack() as stack:
        stack.callback(signal.signal, signal.SIGTERM, old_handler)
        lock_dir = args.results_root / 'runs/localization'
        lock_dir.mkdir(parents=True, exist_ok=True)
        lock = stack.enter_context((lock_dir / f'.domain-{args.domain}.lock').open('a'))
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as e:
            raise ValueError('同じ ROS domain で別の検証を実行中です') from e
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        stack.callback(rclpy.try_shutdown)
        node = rclpy.create_node('atd_localization_replay_report')
        stack.callback(node.destroy_node)
        stream = stack.enter_context((output / 'trajectory.csv').open('x', newline=''))
        writer = csv.writer(stream)
        writer.writerow(['stamp_ns', 'epoch', 'x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw',
                         'speed_mps', 'yaw_rate_rps'])

        def observe(msg):
            if (msg.map_id != meta['map_id'] or msg.map_sha256 != meta['map_sha256']
                    or msg.hdmap_sha256 != meta['hdmap_sha256'] or msg.header.frame_id != 'map'):
                raise ValueError('選択した地図と異なる /localization/status を受信しました')
            stamp = stamp_ns(msg.header.stamp)
            sample = None
            if msg.state == 'tracking':
                p, q = msg.pose.position, msg.pose.orientation
                sample = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, msg.speed_mps, msg.yaw_rate_rps)
            if report.observe(msg.session_id, msg.epoch, msg.state, msg.reason, stamp, sample):
                writer.writerow([stamp, msg.epoch, *sample])

        node.create_subscription(LocalizationStatus, STATUS, observe, 20)
        if trace is not None:
            trace.subscribe(node, meta['workflow'])
        orb = meta.get('backend') == 'orbslam3'
        if not orb:
            from isaac_ros_visual_slam_interfaces.srv import LocalizeInMap
            client = node.create_client(LocalizeInMap, '/visual_slam/localize_in_map')
        start = time.monotonic()
        while time.monotonic() - start < 1.:
            rclpy.spin_once(node, timeout_sec=.1)
        feedback = [STATUS, *(['/orbslam3/ready', '/orbslam3/processed'] if orb else [])]
        if any(node.count_publishers(t) for t in [*topics, *feedback, '/tf', '/clock']):
            raise ValueError('検証 domain に既存の sensor/TF/clock/localization publisher があります')
        playback_topics = topics
        paused_player = f'atd_validation_{os.getpid()}'
        bootstrap = None
        if orb:
            from .orb_replay import OrbBootstrap, recorded_calibration
            from rclpy.qos import QoSProfile, DurabilityPolicy
            from std_msgs.msg import Header

            calibration, _ = recorded_calibration(bag)
            bootstrap = OrbBootstrap(node, calibration)
            playback_topics = [topic for topic in topics if topic not in bootstrap.topics]
            processed_stamp = None

            def processed(msg):
                nonlocal processed_stamp
                processed_stamp = stamp_ns(msg.stamp)
            node.create_subscription(Header, '/orbslam3/processed', processed,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        command = launch_command(args.results_root, args.map_id, args.engine_dir)
        launch = subprocess.Popen(command, start_new_session=True)
        stack.callback(stop, launch)
        deadline = time.monotonic() + args.ready_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if launch.poll() is not None:
                raise ValueError('自己位置推定 launch の起動に失敗しました')
            if (all(node.count_publishers(topic) == 1 for topic in feedback)
                    and (orb or (client.service_is_ready() and node.count_subscribers('/localization/pose_hint') >= 2))
                    and all(node.count_subscribers(meta['workflow'][key]) >= (1 if orb else 2)
                            for key in ('left_image', 'right_image', 'left_info', 'right_info'))):
                break
        else:
            raise ValueError('保存地図の自己位置推定ノードの準備がタイムアウトしました')
        identity = publisher_signature(node, feedback)
        qos = Path(get_package_share_directory('atd_mapping')) / 'config/replay_qos.yaml'
        player = subprocess.Popen(play_command(bag, playback_topics, qos, args.rate, paused_player=paused_player),
                                  start_new_session=True)
        stack.callback(stop, player)
        def check_children():
            if launch.poll() is not None or player.poll() is not None:
                raise ValueError('自己位置推定または bag player が再生開始前に終了しました')

        def missing():
            issues = validation_connections(node, '/' + paused_player, meta['workflow'], orb=orb,
                                            observe_images=trace is not None)
            if trace is not None:
                issues.extend(trace_connections(node))
            return issues

        initialize = (lambda: bootstrap.wait(check_children, args.ready_timeout)) if bootstrap else None
        resume_when_connected(node, paused_player, args.ready_timeout, missing, check_children, initialize)
        deadline = time.monotonic() + duration / args.rate * 2 + args.ready_timeout
        next_check = time.monotonic()
        checked = [*topics, '/clock']
        if not orb:
            checked.remove(meta['workflow']['tf_static'])
        while player.poll() is None:
            rclpy.spin_once(node, timeout_sec=.05)
            if launch.poll() is not None or time.monotonic() > deadline:
                raise ValueError('再生中に自己位置推定が終了、またはタイムアウトしました')
            if time.monotonic() >= next_check:
                if not orb:
                    # cuVSLAM also advertises /tf_static. Permit its writer while still detecting competing players.
                    _check_publishers(node, meta['workflow']['tf_static'], 'tf2_msgs/msg/TFMessage', '/' + paused_player, [])
                if publisher_signature(node, feedback) != identity or any(node.count_publishers(t) > 1 for t in checked):
                    raise ValueError('再生中に入力または自己位置 publisher が変わりました')
                next_check = time.monotonic() + 1.
        if player.returncode:
            raise ValueError(f'rosbag 再生が失敗しました（exit {player.returncode}）')
        deadline = time.monotonic() + args.drain_timeout
        def still_draining():
            if trace is not None:
                # tail_tracked can already be true half a second before the final image.
                # Drain independent DDS queues through the full observation deadline.
                return True
            return (processed_stamp != report.stereo['last_stamp_ns'] if orb else not report.summary()['tail_tracked'])

        while time.monotonic() < deadline and still_draining():
            rclpy.spin_once(node, timeout_sec=.05)
            if launch.poll() is not None:
                raise ValueError('再生末尾の処理中に自己位置推定が終了しました')
        if orb and still_draining():
            raise ValueError(f"末尾の ORB-SLAM3 画像処理を確認できません: {processed_stamp} / {report.stereo['last_stamp_ns']} ns")
        if orb:
            # Separate DDS writers can deliver the final status just after the last processed-frame receipt.
            deadline = time.monotonic() + .5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
        if publisher_signature(node, feedback) != identity or launch.poll() is not None:
            raise ValueError('検証完了前に自己位置推定が終了・再起動しました')
        if not report.session_id:
            raise ValueError('状態ノードから検証結果を受信できませんでした')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', dest='map_id', help='保存済み NVIDIA または ORB-SLAM3 地図の ID')
    parser.add_argument('--bag', help='記録ルートからの相対パス（地図作成とは別の走行）')
    parser.add_argument('--results-root', type=Path, default=Path('/data/results'))
    parser.add_argument('--record-root', type=Path, default=Path('/data/record/rosbags'))
    parser.add_argument('--engine-dir', help='複数の候補がある場合の Notebook engine build')
    parser.add_argument('--run-id', help='Web ジョブが指定する新規 run ID（小文字16進32桁）')
    parser.add_argument('--rate', type=float, default=1., help='bag 再生倍率（0.05〜4）')
    parser.add_argument('--domain', type=int, default=DOMAIN, help='隔離する ROS domain（既定95）')
    parser.add_argument('--ready-timeout', type=float, default=120., help='起動準備待ち秒')
    parser.add_argument('--drain-timeout', type=float, default=8., help='最終入力の処理待ち秒')
    parser.add_argument('--diagnose-frame-gaps', action='store_true',
                        help='cuVSLAM の画像到達と処理完了、地図照合の時刻を同じ再生で記録する')
    args = parser.parse_args(argv)
    if not 1 <= args.domain <= 101 or args.domain in (93, 94) or not .05 <= args.rate <= 4:
        parser.error('domain は 1〜101（地図作成用93・94を除く）、rate は0.05〜4です')
    if any(not math.isfinite(v) or v <= 0 for v in (args.ready_timeout, args.drain_timeout)):
        parser.error('待ち時間は正の有限値が必要です')
    if args.run_id is not None and not re.fullmatch('[0-9a-f]{32}', args.run_id):
        parser.error('run-id は小文字の16進32桁で指定してください')
    args.results_root = args.results_root.resolve()
    args.record_root = args.record_root.resolve()
    from .assets import map_metadata, save_validation_hdmap
    output = None
    result = {}
    try:
        if not args.map_id:
            args.map_id = choose('保存地図', (p.parent.parent.name for p in
                (args.results_root / 'maps').glob('*/hdmap/map.json') if not p.parents[1].name.startswith('.') and
                ((p.parents[1] / 'nvidia/cuvgl_map').is_dir() or (p.parents[1] / 'orbslam3/manifest.json').is_file())))
        meta = map_metadata(args.results_root, args.map_id)
        if args.diagnose_frame_gaps and meta.get('backend') == 'orbslam3':
            raise ValueError('フレーム間隔の診断には cuVSLAM 地図を選択してください')
        if meta.get('backend') == 'orbslam3' and args.engine_dir:
            raise ValueError('ORB-SLAM3 地図に VGL engine は使用しません')
        if not args.bag:
            ids = (p.parent.relative_to(args.record_root).as_posix() for p in args.record_root.rglob('metadata.yaml'))
            args.bag = choose('別走行の bag', (i for i in ids if i != meta['source_bag']))
        bag, digest = independent_bag(args.record_root, args.bag, meta['source_bag'], meta['source_metadata_sha256'])
        run_id = args.run_id or datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid.uuid4().hex[:12]
        category = 'cuvslam_frame_gaps' if args.diagnose_frame_gaps else 'localization'
        destination = args.results_root / 'runs' / category / run_id
        destination.mkdir(parents=True, exist_ok=False)
        output = destination
        print('検証結果 ' + str(output), flush=True)
        result = dict(schema='atd.localization.validation.v1', backend=meta.get('backend', 'cuvslam'),
                      map_id=args.map_id, map_sha256=meta['map_sha256'],
                      hdmap_sha256=meta['hdmap_sha256'], source_bag=meta['source_bag'],
                      source_metadata_sha256=meta['source_metadata_sha256'], bag=args.bag, metadata_sha256=digest,
                      frame='map', body_frame='base_link', replay_rate=args.rate, ros_domain_id=args.domain)
        result['dds_profile'] = replay_dds_profile()
        save_validation_hdmap(args.results_root, meta, output)
        recorded_stamps = {}
        if args.diagnose_frame_gaps:
            from .replay_frame_trace import FrameTrace, print_frame_summary
            result.update(schema='atd.cuvslam.frame_diagnosis.v1')
            topics, duration, stereo = inspect_inputs(bag, meta, recorded_stamps=recorded_stamps)
        else:
            topics, duration, stereo = inspect_inputs(bag, meta)
        result.update(inputs=topics, input_shape=meta['input_shape'], stereo=stereo)
        report = TrackingReport(stereo, max(.5, .3 * args.rate))
        if args.diagnose_frame_gaps:
            trace = FrameTrace()
            try:
                run_ros(args, meta, bag, topics, duration, report, output, trace=trace)
            finally:
                measured = trace.write(output, recorded_stamps)
                result['frame_trace'] = 'frame_trace.json'
            print_frame_summary(measured)
        else:
            run_ros(args, meta, bag, topics, duration, report, output)
        result.update(report.summary(), trajectory='trajectory.csv')
        print(f"{result['outcome']}  追跡時間率 {result['tracking_time_fraction']:.1%}  "
              f"有効姿勢 {result['pose_samples']} 件", flush=True)
        return 0 if result['outcome'] == 'tracked_to_end' else 2
    except KeyboardInterrupt:
        result.update(outcome='cancelled')
        print('検証を中止しました', file=sys.stderr)
        return 130
    except (ValueError, OSError, RuntimeError) as e:
        result.update(outcome='failed', error=str(e))
        print(f'検証失敗: {e}', file=sys.stderr)
        return 1
    finally:
        if output and result.get('outcome'):
            temporary = output / '.report.tmp'
            temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
            temporary.replace(output / 'report.json')

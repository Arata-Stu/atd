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
from atd_mapping.replay_dds import select_replay_dds

from .replay_dds import replay_dds_profile
from .replay_inputs import independent_bag, inspect_inputs
from .replay_report import TrackingReport
from .replay_player import TypedBagInput, _check_publishers, _connection_issues, _endpoints, resume_when_connected

STATUS = '/localization/status'
DOMAIN = 95
SENSOR_INPUT = '/atd_sensor_bag_input'


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


def launch_command(results, map_id, engine, bag, rate, player_name, *, map_correction_source='cuvslam',
                   vgl_frequency=None, trace_publications=False, rviz=False):
    args = ['ros2', 'launch', 'atd_localization', 'localization.launch.py',
            'map_id:=' + map_id, 'results_root:=' + str(results), 'use_sim_time:=true',
            'replay_bag:=' + str(bag), 'replay_rate:=' + str(rate), 'replay_player_name:=' + player_name,
            'map_correction_source:=' + map_correction_source]
    if vgl_frequency is not None:
        args.append('vgl_frequency:=' + str(vgl_frequency))
    if engine:
        args.append('engine_dir:=' + engine)
    if trace_publications:
        args.append('trace_publications:=true')
    if rviz and map_correction_source == 'cuvslam':
        args.append('visualize_landmarks:=true')
    return args


def validation_connections(node, player_name, workflow, *, map_correction_source='cuvslam', rviz=False):
    """停止中の bag と実際に追跡・再定位を行う購読先の型・QoS を確認する。"""
    cameras = ['/visual_slam', '/visual_global_localization']
    clocks = [*cameras, '/atd_localization_monitor']
    links = [(workflow[side + '_image'], 'sensor_msgs/msg/Image', SENSOR_INPUT, cameras)
             for side in ('left', 'right')]
    # These component names and remapped CameraInfo inputs are shared with localization.launch.py.
    links += [(workflow[side + '_info'], 'sensor_msgs/msg/CameraInfo', SENSOR_INPUT, clocks)
              for side in ('left', 'right')]
    hint_consumers = ['/atd_localization_monitor']
    if map_correction_source == 'cuvslam':
        hint_consumers.append('/visual_slam')
        if rviz:
            # The initial saved-map match can finish before a late RViz subscriber joins.
            links += [('/visual_slam/vis/' + cloud, 'sensor_msgs/msg/PointCloud2', '/visual_slam', ['/rviz2'])
                      for cloud in ('landmarks_cloud', 'localizer_map_cloud')]
    links.append(('/localization/pose_hint', 'geometry_msgs/msg/PoseWithCovarianceStamped',
                  '/visual_global_localization', hint_consumers))
    # tf2 can create its own listener node; validate the actual typed reader in this isolated domain.
    links.append((workflow['tf_static'], 'tf2_msgs/msg/TFMessage', player_name, [None]))
    links.append(('/clock', 'rosgraph_msgs/msg/Clock', player_name, clocks))
    issues = []
    for topic, kind, producer, consumers in links:
        pubs = _check_publishers(node, topic, kind, producer, issues)
        readers = node.get_subscriptions_info_by_topic(topic)
        for consumer in consumers:
            subs = _endpoints(readers, kind, consumer)
            issues.extend(_connection_issues(topic, pubs, subs, producer, consumer or 'TF listener'))
    return issues


def trace_connections(node):
    """画像の処理時間を配信するノードと診断用の購読先を開始前に確認する。"""
    missing = []
    observer = node.get_fully_qualified_name()
    links = [(SENSOR_INPUT + '/' + side + '_published', 'std_msgs/msg/Header', SENSOR_INPUT, [observer])
             for side in ('left', 'right')]
    links += [('/visual_slam/status', 'isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus',
              '/visual_slam', [observer]),
             ('/visual_slam/tracking/odometry', 'nav_msgs/msg/Odometry', '/visual_slam', [observer]),
             ('/localization/pose_hint', 'geometry_msgs/msg/PoseWithCovarianceStamped', '/visual_global_localization',
              [observer])]
    for topic, kind, source, consumers in links:
        publishers = _check_publishers(node, topic, kind, source, missing)
        subscribers = node.get_subscriptions_info_by_topic(topic)
        for consumer in consumers:
            missing.extend(_connection_issues(topic, publishers, _endpoints(subscribers, kind, consumer),
                                               source, consumer))
    return missing


def start_rviz(domain):
    from ament_index_python.packages import get_package_share_directory

    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise ValueError('RViz の画面出力先がありません。Notebook のデスクトップから atd dev を起動してください')
    if not shutil.which('rviz2'):
        raise ValueError('Notebook コンテナに rviz2 が見つかりません')
    config = Path(get_package_share_directory('atd_localization')) / 'config/fixed_map_replay.rviz'
    if not config.is_file():
        raise ValueError(f'RViz 設定がありません: {config}。atd-build --packages-up-to atd_localization を実行してください')
    return subprocess.Popen(['rviz2', '-d', str(config), '--ros-args', '-p', 'use_sim_time:=true',
                             '-r', '__node:=rviz2'],
                            env=dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_STATIC_PEERS=''),
                            stdin=subprocess.DEVNULL, start_new_session=True)


def check_rviz(rviz):
    if rviz is not None:
        code = rviz.poll()
        if code == 0:
            raise KeyboardInterrupt  # RViz を閉じた操作で再生も中止する。
        if code is not None:
            raise RuntimeError(f'RViz が終了しました（exit {code}）。画面接続と起動ログを確認してください')


def run_ros(args, meta, bag, topics, duration, report, output, *, trace=None, rviz=None):
    import rclpy
    from atd_interfaces.msg import LocalizationStatus
    from rclpy.signals import SignalHandlerOptions

    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_STATIC_PEERS='')
    with ExitStack() as stack:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        stack.callback(rclpy.try_shutdown)
        node = rclpy.create_node('atd_localization_replay_report')
        stack.callback(node.destroy_node)
        view = None
        if rviz is not None:
            from .replay_view import ReplayView
            view = ReplayView(node, output / 'hdmap.json')
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
                if view is not None:
                    view.observe(msg, report.intervals[-1])
            elif view is not None and msg.state != 'tracking':
                view.observe(msg, None)

        node.create_subscription(LocalizationStatus, STATUS, observe, 20)
        if trace is not None:
            trace.subscribe(node, SENSOR_INPUT, vgl_corrections=args.map_correction_source == 'vgl')
        localize_service = None
        if args.map_correction_source == 'cuvslam':
            from isaac_ros_visual_slam_interfaces.srv import LocalizeInMap
            localize_service = node.create_client(LocalizeInMap, '/visual_slam/localize_in_map')
        sensor = TypedBagInput(node, source=SENSOR_INPUT)
        stack.callback(sensor.close)
        cu_frames = set()
        if trace is None:
            from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
            from rclpy.qos import qos_profile_sensor_data
            node.create_subscription(VisualSlamStatus, '/visual_slam/status',
                lambda msg: cu_frames.add(stamp_ns(msg.header.stamp)), qos_profile_sensor_data)
        start = time.monotonic()
        while time.monotonic() - start < 1.:
            rclpy.spin_once(node, timeout_sec=.1)
        feedback = [STATUS]
        if any(node.count_publishers(t) for t in [*topics, *feedback, '/tf', '/clock']):
            raise ValueError('検証 domain に既存の sensor/TF/clock/localization publisher があります')
        paused_player = f'atd_validation_{os.getpid()}'
        command = launch_command(args.results_root, args.map_id, args.engine_dir, bag, args.rate, paused_player,
                                 map_correction_source=args.map_correction_source,
                                 vgl_frequency=args.vgl_frequency, trace_publications=trace is not None,
                                 rviz=rviz is not None)
        launch = subprocess.Popen(command, start_new_session=True)
        stack.callback(stop, launch)
        deadline = time.monotonic() + args.ready_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            check_rviz(rviz)
            if launch.poll() is not None:
                raise ValueError('自己位置推定 launch の起動に失敗しました')
            if (node.count_publishers(STATUS) == 1 and (localize_service is None or localize_service.service_is_ready())
                    and all(node.count_subscribers(meta['workflow'][key]) >= 2
                            for key in ('left_image', 'right_image', 'left_info', 'right_info'))):
                break
        else:
            raise ValueError('保存地図の自己位置推定ノードの準備がタイムアウトしました')
        identity = publisher_signature(node, feedback)
        def check_launch():
            check_rviz(rviz)
            if launch.poll() is not None:
                raise ValueError('自己位置推定または bag 入力の component container が終了しました')

        def missing():
            issues = validation_connections(node, '/' + paused_player, meta['workflow'],
                                            map_correction_source=args.map_correction_source, rviz=rviz is not None)
            if trace is not None:
                issues.extend(trace_connections(node))
            if view is not None and not view.ready():
                issues.append('RViz の地図・推定軌跡の購読')
            return issues

        initial_connections = {}

        def initialize():
            initial_connections.update(sensor.wait_for_connections(check_launch, args.ready_timeout))

        resume_when_connected(node, paused_player, args.ready_timeout, missing, check_launch, initialize)
        deadline = time.monotonic() + duration / args.rate * 2 + args.ready_timeout
        next_check = time.monotonic()
        checked = [*topics, '/clock']
        checked.remove(meta['workflow']['tf_static'])
        while not sensor.finished:
            check_launch()
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() > deadline:
                raise ValueError('bag の再生がタイムアウトしました')
            if time.monotonic() >= next_check:
                # cuVSLAM also advertises /tf_static. Permit its writer while still detecting competing players.
                _check_publishers(node, meta['workflow']['tf_static'], 'tf2_msgs/msg/TFMessage', '/' + paused_player, [])
                if publisher_signature(node, feedback) != identity or any(node.count_publishers(t) > 1 for t in checked):
                    raise ValueError('再生中に入力または自己位置 publisher が変わりました')
                next_check = time.monotonic() + 1.
        deadline = time.monotonic() + args.drain_timeout
        def still_draining():
            # cuVSLAM may stamp either side of the last validated (<= 5 ms) stereo pair.
            end = report.stereo['last_stamp_ns']
            if trace is not None:
                # End once the actual final input, processed frame and map pose have arrived.
                # After EOF the live monitor's freshness watchdog will correctly report stale.
                return not (trace.observed_tail(report.stereo) and report.last_unique_pose
                            and report.last_unique_pose[1] >= end - 5_000_000)
            return not (report.last_unique_pose and report.last_unique_pose[1] >= end - 5_000_000
                        and cu_frames and max(cu_frames) >= end - 5_000_000)

        while time.monotonic() < deadline and still_draining():
            check_launch()
            rclpy.spin_once(node, timeout_sec=.05)
        if publisher_signature(node, feedback) != identity or launch.poll() is not None:
            raise ValueError('検証完了前に自己位置推定が終了・再起動しました')
        if not report.session_id:
            raise ValueError('状態ノードから検証結果を受信できませんでした')
        counts = sensor.counts(check_launch, args.ready_timeout)
        for side in ('left', 'right'):
            if counts[side + '_published'] != report.stereo[side + '_images']:
                raise ValueError(f'bag の {side} 画像の発行数が記録と一致しません: '
                                 f'{counts[side + "_published"]} / {report.stereo[side + "_images"]}')
        print(f'型付き bag 入力: 発行 左 {counts["left_published"]} / 右 {counts["right_published"]} 枚、'
              f'再生開始時のプロセス内購読 左 {initial_connections["left_intra_readers"]} / '
              f'右 {initial_connections["right_intra_readers"]}', flush=True)
        evidence = dict(sensor_input_transport='ros2_intra_process', bag_input=dict(
            before_replay=initial_connections, after_replay=counts))
        if trace is None:
            evidence['processed_frames_observed'] = len(cu_frames)
            print(f'cuVSLAM の処理完了通知 {len(cu_frames)} 時刻', flush=True)
        if view is not None:
            view.flush(wait_for_ack=True)
        return evidence


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', dest='map_id', help='保存済み NVIDIA 地図の ID')
    parser.add_argument('--bag', help='記録ルートからの相対パス（地図作成とは別の走行）')
    parser.add_argument('--results-root', type=Path, default=Path('/data/results'))
    parser.add_argument('--record-root', type=Path, default=Path('/data/record/rosbags'))
    parser.add_argument('--engine-dir', help='複数の候補がある場合の Notebook engine build')
    parser.add_argument('--map-correction-source', choices=('cuvslam', 'vgl'), default='cuvslam',
                        help='map → odom の補正元（既定 cuvslam）')
    parser.add_argument('--vgl-frequency', type=float, help='vgl 方式の照合頻度 [Hz]（既定1、0は初回成功後に手動）')
    parser.add_argument('--run-id', help='Web ジョブが指定する新規 run ID（小文字16進32桁）')
    parser.add_argument('--rate', type=float, default=1., help='bag 再生倍率（0.05〜4）')
    parser.add_argument('--rviz', action='store_true', help='保存地図と推定軌跡を RViz で表示する')
    parser.add_argument('--domain', type=int, default=DOMAIN, help='隔離する ROS domain（既定95）')
    parser.add_argument('--ready-timeout', type=float, default=120., help='起動準備待ち秒')
    parser.add_argument('--drain-timeout', type=float, default=8., help='最終入力の処理待ち秒')
    parser.add_argument('--diagnose-frame-gaps', action='store_true',
                        help='型付き画像の発行通知・cuVSLAM 処理完了・地図照合の時刻を記録する')
    args = parser.parse_args(argv)
    if args.vgl_frequency is not None:
        if args.map_correction_source != 'vgl':
            parser.error('--vgl-frequency は --map-correction-source vgl と併用してください')
        if not math.isfinite(args.vgl_frequency) or args.vgl_frequency < 0:
            parser.error('--vgl-frequency は有限の0以上の値が必要です')
    elif args.map_correction_source == 'vgl':
        args.vgl_frequency = 1.
    if not 1 <= args.domain <= 101 or args.domain in (93, 94) or not .05 <= args.rate <= 4:
        parser.error('domain は 1〜101（地図作成用93・94を除く）、rate は0.05〜4です')
    if any(not math.isfinite(v) or v <= 0 for v in (args.ready_timeout, args.drain_timeout)):
        parser.error('待ち時間は正の有限値が必要です')
    if args.run_id is not None and not re.fullmatch('[0-9a-f]{32}', args.run_id):
        parser.error('run-id は小文字の16進32桁で指定してください')
    args.results_root = args.results_root.resolve()
    args.record_root = args.record_root.resolve()

    def interrupt(*_):
        raise KeyboardInterrupt

    with ExitStack() as children:
        old_handler = signal.signal(signal.SIGTERM, interrupt)
        children.callback(signal.signal, signal.SIGTERM, old_handler)
        return validate(args, children)


def validate(args, children):
    from .assets import map_metadata, save_validation_hdmap
    output = None
    rviz = None
    result = {}
    try:
        if not args.map_id:
            args.map_id = choose('保存地図', (p.parent.parent.name for p in
                (args.results_root / 'maps').glob('*/hdmap/map.json') if not p.parents[1].name.startswith('.') and
                (p.parents[1] / 'nvidia/cuvgl_map').is_dir()))
        meta = map_metadata(args.results_root, args.map_id)
        if not args.bag:
            ids = (p.parent.relative_to(args.record_root).as_posix() for p in args.record_root.rglob('metadata.yaml'))
            args.bag = choose('別走行の bag', (i for i in ids if i != meta['source_bag']))
        bag, digest = independent_bag(args.record_root, args.bag, meta['source_bag'], meta['source_metadata_sha256'])
        select_replay_dds()
        os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
        run_id = args.run_id or datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid.uuid4().hex[:12]
        category = 'cuvslam_frame_gaps' if args.diagnose_frame_gaps else 'localization'
        destination = args.results_root / 'runs' / category / run_id
        destination.mkdir(parents=True, exist_ok=False)
        output = destination
        print('検証結果 ' + str(output), flush=True)
        result = dict(schema='atd.localization.validation.v1', backend='cuvslam',
                      map_correction_source=args.map_correction_source,
                      map_id=args.map_id, map_sha256=meta['map_sha256'],
                      hdmap_sha256=meta['hdmap_sha256'], source_bag=meta['source_bag'],
                      source_metadata_sha256=meta['source_metadata_sha256'], bag=args.bag, metadata_sha256=digest,
                      frame='map', body_frame='base_link', tracking='vo', replay_rate=args.rate,
                      ros_domain_id=args.domain, rviz_enabled=args.rviz)
        result['dds_profile'] = replay_dds_profile()
        if args.map_correction_source == 'vgl':
            result['vgl_frequency_hz'] = args.vgl_frequency
        save_validation_hdmap(args.results_root, meta, output)
        recorded_stamps = {}
        if args.diagnose_frame_gaps:
            from .replay_frame_trace import FrameTrace, print_frame_summary
            result.update(schema='atd.cuvslam.frame_diagnosis.v2')
            topics, duration, stereo = inspect_inputs(bag, meta, recorded_stamps=recorded_stamps)
        else:
            topics, duration, stereo = inspect_inputs(bag, meta)
        result.update(inputs=topics, input_shape=meta['input_shape'], stereo=stereo)
        report = TrackingReport(stereo, max(.5, .3 * args.rate))
        lock_dir = args.results_root / 'runs/localization'
        lock_dir.mkdir(parents=True, exist_ok=True)
        lock = children.enter_context((lock_dir / f'.domain-{args.domain}.lock').open('a'))
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as e:
            raise ValueError('同じ ROS domain で別の検証を実行中です') from e
        if args.rviz:
            rviz = start_rviz(args.domain)
            children.callback(stop, rviz)
            print('RViz の表示準備を待ってから再生します。再生中にウィンドウを閉じると検証を中止します。', flush=True)
        if args.diagnose_frame_gaps:
            trace = FrameTrace()
            try:
                result.update(run_ros(args, meta, bag, topics, duration, report, output, trace=trace, rviz=rviz))
            finally:
                measured = trace.write(output, recorded_stamps)
                result['frame_trace'] = 'frame_trace.json'
                result['processed_frames_observed'] = measured['processed_frames']
                if args.map_correction_source == 'vgl':
                    result['map_to_odom_updates_observed'] = len({event.stamp_ns for event in trace.events
                                                                   if event.stream == 'map_to_odom'})
            print_frame_summary(measured)
        else:
            result.update(run_ros(args, meta, bag, topics, duration, report, output, rviz=rviz))
        result.update(report.summary(), trajectory='trajectory.csv')
        print(f"{result['outcome']}  追跡時間率 {result['tracking_time_fraction']:.1%}  "
              f"有効姿勢 {result['pose_samples']} 件", flush=True)
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
    if rviz is not None:
        print(f'結果を保存しました: {output}。最終軌跡を確認後、RViz を閉じてください。', flush=True)
        try:
            rviz.wait()
        except KeyboardInterrupt:
            pass  # 結果の保存後は Ctrl+C でも表示を終了できる。
    return 0 if result['outcome'] == 'tracked_to_end' else 2

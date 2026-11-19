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

from .replay_inputs import independent_bag, inspect_inputs
from .replay_report import TrackingReport

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


def play_command(bag, topics, qos, rate):
    return ['ros2', 'bag', 'play', str(bag), '--clock', '--rate', str(rate),
            '--qos-profile-overrides-path', str(qos), '--topics', *topics]


def run_ros(args, meta, bag, topics, duration, report, output):
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from atd_interfaces.msg import LocalizationStatus
    from rclpy.signals import SignalHandlerOptions
    from isaac_ros_visual_slam_interfaces.srv import LocalizeInMap

    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')
    def interrupt(*_):
        raise KeyboardInterrupt
    old_handler = signal.signal(signal.SIGTERM, interrupt)
    with ExitStack() as stack:
        stack.callback(signal.signal, signal.SIGTERM, old_handler)
        lock = stack.enter_context((output.parent / f'.domain-{args.domain}.lock').open('a'))
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
        client = node.create_client(LocalizeInMap, '/visual_slam/localize_in_map')
        start = time.monotonic()
        while time.monotonic() - start < 1.:
            rclpy.spin_once(node, timeout_sec=.1)
        if any(node.count_publishers(t) for t in [*topics, '/clock', STATUS, '/tf']):
            raise ValueError('検証 domain に既存の sensor/TF/clock/localization publisher があります')
        launch = subprocess.Popen(launch_command(args.results_root, args.map_id, args.engine_dir), start_new_session=True)
        stack.callback(stop, launch)
        deadline = time.monotonic() + args.ready_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if launch.poll() is not None:
                raise ValueError('自己位置推定 launch の起動に失敗しました')
            if (client.service_is_ready() and node.count_publishers(STATUS) == 1
                    and node.count_subscribers('/localization/pose_hint') >= 2
                    and all(node.count_subscribers(meta['workflow'][key]) >= 2
                            for key in ('left_image', 'right_image', 'left_info', 'right_info'))):
                break
        else:
            raise ValueError('cuVSLAM・VGL・状態ノードの準備がタイムアウトしました')
        identity = publisher_signature(node, [STATUS])
        qos = Path(get_package_share_directory('atd_mapping')) / 'config/replay_qos.yaml'
        player = subprocess.Popen(play_command(bag, topics, qos, args.rate), start_new_session=True)
        stack.callback(stop, player)
        deadline = time.monotonic() + duration / args.rate * 2 + args.ready_timeout
        next_check = time.monotonic()
        while player.poll() is None:
            rclpy.spin_once(node, timeout_sec=.05)
            if launch.poll() is not None or time.monotonic() > deadline:
                raise ValueError('再生中に自己位置推定が終了、またはタイムアウトしました')
            if time.monotonic() >= next_check:
                if (publisher_signature(node, [STATUS]) != identity
                        or any(node.count_publishers(t) > 1 for t in [*topics, '/clock'])):
                    raise ValueError('再生中に入力または自己位置 publisher が変わりました')
                next_check = time.monotonic() + 1.
        if player.returncode:
            raise ValueError(f'rosbag 再生が失敗しました（exit {player.returncode}）')
        deadline = time.monotonic() + args.drain_timeout
        while time.monotonic() < deadline and not report.summary()['tail_tracked']:
            rclpy.spin_once(node, timeout_sec=.05)
            if launch.poll() is not None:
                raise ValueError('再生末尾の処理中に自己位置推定が終了しました')
        if publisher_signature(node, [STATUS]) != identity or launch.poll() is not None:
            raise ValueError('検証完了前に自己位置推定が終了・再起動しました')
        if not report.session_id:
            raise ValueError('状態ノードから検証結果を受信できませんでした')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', dest='map_id', help='保存済み NVIDIA 地図の ID')
    parser.add_argument('--bag', help='記録ルートからの相対パス（地図作成とは別の走行）')
    parser.add_argument('--results-root', type=Path, default=Path('/data/results'))
    parser.add_argument('--record-root', type=Path, default=Path('/data/record/rosbags'))
    parser.add_argument('--engine-dir', help='複数の候補がある場合の Notebook engine build')
    parser.add_argument('--run-id', help='Web ジョブが指定する新規 run ID（小文字16進32桁）')
    parser.add_argument('--rate', type=float, default=1., help='bag 再生倍率（0.05〜4）')
    parser.add_argument('--domain', type=int, default=DOMAIN, help='隔離する ROS domain（既定95）')
    parser.add_argument('--ready-timeout', type=float, default=120., help='起動準備待ち秒')
    parser.add_argument('--drain-timeout', type=float, default=8., help='最終入力の処理待ち秒')
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
                (args.results_root / 'maps').glob('*/hdmap/map.json') if (p.parents[1] / 'nvidia/cuvgl_map').is_dir()))
        meta = map_metadata(args.results_root, args.map_id)
        if not args.bag:
            ids = (p.parent.relative_to(args.record_root).as_posix() for p in args.record_root.rglob('metadata.yaml'))
            args.bag = choose('別走行の bag', (i for i in ids if i != meta['source_bag']))
        bag, digest = independent_bag(args.record_root, args.bag, meta['source_bag'], meta['source_metadata_sha256'])
        run_id = args.run_id or datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid.uuid4().hex[:12]
        destination = args.results_root / 'runs/localization' / run_id
        destination.mkdir(parents=True, exist_ok=False)
        output = destination
        print('検証結果 ' + str(output), flush=True)
        result = dict(schema='atd.localization.validation.v1', map_id=args.map_id, map_sha256=meta['map_sha256'],
                      hdmap_sha256=meta['hdmap_sha256'], source_bag=meta['source_bag'],
                      source_metadata_sha256=meta['source_metadata_sha256'], bag=args.bag, metadata_sha256=digest,
                      frame='map', body_frame='base_link', replay_rate=args.rate, ros_domain_id=args.domain)
        save_validation_hdmap(args.results_root, meta, output)
        topics, duration, stereo = inspect_inputs(bag, meta)
        result.update(inputs=topics, input_shape=meta['input_shape'], stereo=stereo)
        report = TrackingReport(stereo, max(.5, .3 * args.rate))
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

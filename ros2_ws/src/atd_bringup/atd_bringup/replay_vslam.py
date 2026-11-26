"""Notebook でステレオ bag を SLAM 再生し、Gazebo の正解軌跡と比較する。"""
import argparse
from copy import deepcopy
from contextlib import ExitStack
import fcntl
import hashlib
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from tempfile import TemporaryDirectory
import time

import yaml
from atd_mapping.capture_snapshot import stop
from atd_mapping.replay_dds import select_replay_dds

from .replay_gt_course import load_course
from .replay_gt_geometry import Trace
from .replay_gt_node import ground_truth_viewer
from .replay_gt_source import find_ground_truth
from .replay_inputs import (GROUND_TRUTH_ERROR, GROUND_TRUTH_PATH, IMU_TOPIC, SLAM_ODOMETRY, STEREO_TOPICS,
                            gap_gyro_topic, select_bag, validate_bag)
from .replay_result import ReplayResult


class ViewerClosed(Exception):
    """RViz のウィンドウを閉じて再生セッションを終了した。"""


def _show_node_output(reader, *, drain=False):
    # The node writes straight to a regular file, so teardown never waits on a log pipe.
    while chunk := reader.read(64 * 1024):
        try:
            print(chunk, end='', flush=True)
        except BrokenPipeError:
            return  # The original bytes remain in the run even if the console is closed.
        if not drain:
            break  # Bound work between ROS spins even during an ORB warning burst.


def replay_config(preset, directory, hidden, *, bridge=False):
    """今回配信しない topic を RViz の表示から取り除く。"""
    config = yaml.safe_load(preset.read_text())
    manager = config['Visualization Manager']
    manager['Displays'] = [display for display in manager['Displays']
                           if display.get('Topic', {}).get('Value') not in hidden]
    if bridge:
        from .orb_gap_bridge import CONTINUOUS_ODOMETRY
        raw = next(d for d in manager['Displays'] if d.get('Topic', {}).get('Value') == SLAM_ODOMETRY)
        raw['Name'] = 'ORB visual observations'
        continuous = deepcopy(raw)
        continuous['Name'] = 'ORB with gyro gap prediction'
        continuous['Topic']['Value'] = CONTINUOUS_ODOMETRY
        continuous['Shape'].update({'Color': '120; 235; 145', 'Shaft Radius': .03, 'Head Radius': .08})
        manager['Displays'].append(continuous)
    output = Path(directory) / preset.name
    output.write_text(yaml.safe_dump(config, sort_keys=False))
    return output


def estimated_only_config(preset, directory):
    return replay_config(preset, directory, {GROUND_TRUTH_PATH, GROUND_TRUTH_ERROR})


def run_session(node, bag, rate, rviz_config, qos, ground_truth=None, result=None, *, slam_max_map_size=0,
                imu=None, orb=None):
    from .replay_readiness import wait_and_resume

    with ExitStack() as children:
        def start(command, **streams):
            process = subprocess.Popen(command, start_new_session=True, stdin=subprocess.DEVNULL, **streams)
            children.callback(stop, process)
            return process

        name = 'ORB-SLAM3' if orb else 'cuVSLAM'
        if orb:
            log_path = orb.directory / 'orbslam3.log'
            output = children.enter_context(log_path.open('xb'))
            reader = children.enter_context(log_path.open(encoding='utf-8', errors='replace'))
            # LIFO: drain the last output after the tracker has stopped, before closing the file.
            children.callback(_show_node_output, reader, drain=True)
            orb.summary.update(node_log=log_path.name)
            launch = start(orb.command, stdout=output, stderr=subprocess.STDOUT)
        else:
            launch_args = ['ros2', 'launch', 'atd_bringup', 'bag_vslam.launch.py',
                           f'slam_max_map_size:={slam_max_map_size}']
            if imu:
                launch_args.extend(['tracking_mode:=1', f'imu_frame:={imu.frame}', f'imu_frequency:={imu.frequency}'])
            launch = start(launch_args)
        rviz = None
        if rviz_config is not None:
            rviz = start(['rviz2', '-d', str(rviz_config), '--ros-args',
                          '-p', 'use_sim_time:=true', '-r', '__node:=rviz2'])
        spin_overlay = None
        if ground_truth is not None:
            observer = children.enter_context(ground_truth_viewer(
                ground_truth, result, visualize=rviz is not None, **({'orb': orb} if orb else {})))
            spin_overlay = observer.spin_once
        player_name = orb.player_name if orb else f'atd_replay_player_{os.getpid()}'
        player = None
        if not orb:
            topics = [*STEREO_TOPICS, IMU_TOPIC] if imu else STEREO_TOPICS
            player = start(['ros2', 'bag', 'play', str(bag), '--start-paused', '--disable-keyboard-controls',
                            '--clock', '--rate', str(rate), '--qos-profile-overrides-path', str(qos),
                            '--remap', f'__node:={player_name}', '--topics', *topics])

        def check_viewer():
            if orb:
                _show_node_output(reader)
            if launch.poll() is not None:
                if orb:
                    code = launch.returncode
                    detail = f'exit {code}'
                    orb.summary.update(node_exit_code=code)
                    if code < 0:
                        try:
                            signame = signal.Signals(-code).name
                        except ValueError:
                            signame = f'signal {-code}'
                        detail += f', {signame}'
                    raise RuntimeError(f'{name} が終了しました（{detail}）。ノードログ: {log_path}')
                raise RuntimeError(f'{name} が終了しました（exit {launch.returncode}）。起動ログを確認してください')
            if rviz is not None and rviz.poll() is not None:
                if rviz.returncode:
                    raise RuntimeError(f'RViz が終了しました（exit {rviz.returncode}）。画面接続と起動ログを確認してください')
                raise ViewerClosed()

        def check_startup():
            check_viewer()
            if player and player.poll() is not None:
                raise RuntimeError(f'再生開始前に rosbag2 が終了しました（exit {player.returncode}）')

        tracking = f'VIO (IMU {imu.frequency:g} Hz, {imu.frame})' if imu else 'VO'
        consumers = f'{name} と RViz' if rviz is not None else name
        settings = orb.params['mode'] if orb else f'slam_max_map_size={slam_max_map_size}'
        print(f'{bag} ({rate:g} 倍速)  {tracking}  {settings}'
              f'\n{consumers} の購読準備を待っています。', flush=True)

        def spin(timeout_sec):
            if orb:
                orb.spin(timeout_sec=min(.005, timeout_sec))
                if spin_overlay:
                    spin_overlay(timeout_sec=0.)
            elif spin_overlay:
                spin_overlay(timeout_sec=timeout_sec)
            else:
                time.sleep(timeout_sec)

        finished = False
        try:
            wait_and_resume(node, player_name=f'/{player_name}', check_running=check_startup,
                            ground_truth=spin_overlay is not None, vio=imu is not None,
                            rviz_name='/rviz2' if rviz is not None else None, **({'orb': orb} if orb else {}))
            print('bag を再生しています。' + ('終了するには RViz を閉じるか Ctrl+C を押してください。'
                                           if rviz is not None else '末尾で比較結果を保存して終了します。'), flush=True)
            while True:
                if not finished and (orb.playback_finished if orb else player.poll() is not None):
                    if player and player.returncode:
                        raise RuntimeError(f'rosbag2 の再生に失敗しました（exit {player.returncode}）')
                    if orb:
                        orb.finish(spin, check_viewer, result)
                    elif rviz is None:
                        # rosbag2 の終了後も GPU から遅れて届く姿勢は比較に含める。
                        stamp = result.last_stamp
                        deadline = time.monotonic() + 2.
                        while time.monotonic() < deadline:
                            check_viewer()
                            spin_overlay(timeout_sec=.05)
                            if result.last_stamp != stamp:
                                stamp = result.last_stamp
                                deadline = time.monotonic() + 2.
                        if stamp is None:
                            raise RuntimeError('GT と時刻を照合できた cuVSLAM の推定姿勢が 0 件です')
                    finished = True
                    if result is not None:
                        if not orb:
                            observer.inspect_loop_graph()
                            print(result.closure_report(), flush=True)
                        result.mark_eof()
                    if rviz is None:
                        print(f'bag の末尾に到達しました。結果を保存しました: {result.directory if result else orb.directory}', flush=True)
                        return
                    print('bag の末尾に到達しました。RViz で最終表示を確認できます。', flush=True)
                check_viewer()
                spin(.15)
        except ViewerClosed:
            if orb and not finished:
                raise RuntimeError('ORB-SLAM3 の処理が完了する前に RViz が閉じられました')
            # An estimate can arrive just after the last image was played.
            if finished and spin_overlay is not None:
                deadline = time.monotonic() + .5
                while time.monotonic() < deadline:
                    spin_overlay(timeout_sec=.05)


def run(bag, rate, *, slam_max_map_size=0, tracking='vo', headless=False, orb_options=None):
    if orb_options:
        from .replay_orbslam3 import OrbReplay, orb_executable
        executable = orb_executable()

    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    config = None
    if not headless:
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            raise ValueError('RViz の画面出力先がありません。Notebook のデスクトップ端末で atd dev を起動してください')
        if not shutil.which('rviz2'):
            raise ValueError('コンテナに rviz2 がありません。Notebook の RViz 依存を準備してください')
        config = Path(get_package_share_directory('atd_bringup')) / 'config/bag_vslam.rviz'
    qos = Path(get_package_share_directory('atd_mapping')) / 'config/replay_qos.yaml'
    for path in (config, qos):
        if path is not None and not path.is_file():
            raise ValueError(f'{path} がありません。atd build --packages-up-to atd_bringup を実行してください')

    def interrupt(*_):
        raise KeyboardInterrupt

    with ExitStack() as stack:
        previous = signal.signal(signal.SIGTERM, interrupt)
        stack.callback(signal.signal, signal.SIGTERM, previous)
        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        lock = stack.enter_context(Path(f'/tmp/atd-replay-domain-{domain}.lock').open('a'))
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError(f'ROS domain {domain} では既に atd replay が実行中です') from exc
        imu = validate_bag(bag, vio=tracking == 'vio')
        trace = None
        root = Path(os.environ.get('ATD_REPLAY_RESULTS_ROOT', '/data/results')).expanduser().resolve()
        try:
            ground_truth = find_ground_truth(bag, root)
            if ground_truth is not None:
                trace = Trace.from_file(ground_truth)
        except (OSError, ValueError) as exc:
            if headless or orb_options:
                raise ValueError(f'Gazebo 正解軌跡を照合できません: {exc}') from exc
            ground_truth = None
            print(f'Gazebo 正解軌跡を照合できません。cuVSLAM の推定軌跡のみ表示します。{exc}',
                  file=sys.stderr, flush=True)
        else:
            if ground_truth is None:
                if headless and not orb_options:
                    raise ValueError('--headless の比較には、この bag に対応する Gazebo 正解軌跡が必要です')
                print('対応する Gazebo 正解軌跡がありません。推定結果を観測します。', flush=True)
        select_replay_dds()
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        stack.callback(rclpy.try_shutdown)
        node = rclpy.create_node('atd_bag_replay')
        stack.callback(node.destroy_node)
        result = None
        course = load_course(bag, ground_truth, root) if trace else {}
        orb = None
        hidden = set()
        if orb_options:
            gyro = None
            if orb_options.map and orb_options.gyro_bridge != 'off':
                gyro = gap_gyro_topic(bag, required=orb_options.gyro_bridge == 'on')
                if gyro is None:
                    print('bag にジャイロがありません。ORB の観測姿勢で再生します。', flush=True)
            orb = OrbReplay(stack, root, node, bag, orb_options, imu, course, executable,
                            gyro_topic=gyro, gazebo=ground_truth is not None)
            hidden.update(('/visual_slam/vis/landmarks_cloud', '/visual_slam/vis/loop_closure_cloud'))
            if trace and orb.metadata and not orb.fixed_alignment:
                print('このコースと照合できる地図作成時の GT 座標変換がないため、絶対誤差は評価しません。', flush=True)
                trace = None
        if not trace:
            hidden.update((GROUND_TRUTH_PATH, GROUND_TRUTH_ERROR))
        if config and hidden:
            config = replay_config(config, stack.enter_context(TemporaryDirectory(prefix='atd-replay-rviz-')),
                                   hidden, bridge=bool(orb and orb.bridge))
        if trace is not None:
            result = stack.enter_context(ReplayResult(root, bag, rate, trace, course,
                                                     slam_max_map_size=slam_max_map_size, imu=imu))
            bag_hash = orb.bag_hash if orb else hashlib.sha256((bag / 'metadata.yaml').read_bytes()).hexdigest()
            result.summary.update(backend='cuvslam', mode='mapping', alignment_source='first_estimate',
                                  start_offset_s=0., metadata_sha256=bag_hash,
                                  sensor_input_transport=orb.summary['sensor_input_transport'] if orb else 'ros2_inter_process')
            if orb:
                result.summary.pop('slam_max_map_size')
                result.summary.update(backend='orbslam3', mode=orb.params['mode'], map_id=orb_options.map or orb_options.save_map,
                                      start_offset_s=orb_options.start_offset, alignment_source=(
                                          'saved_map' if orb_options.map else 'first_estimate'))
                result.summary['loop_closure_graph'] = {'status': 'unavailable', 'reason': 'ORB-SLAM3 replay'}
                orb.summary['gt_replay_id'] = result.directory.name
            print(f'Gazebo 正解軌跡: {ground_truth}\n比較結果の保存先: {result.directory}', flush=True)
        run_session(node, bag, rate, config, qos, trace, result, slam_max_map_size=slam_max_map_size, imu=imu, orb=orb)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?', help='/data/record/rosbags からの相対パス、または絶対パス')
    parser.add_argument('--rate', type=float, default=0.5, help='再生倍率（既定 0.5）')
    parser.add_argument('--headless', action='store_true', help='RViz を開かずに GT と比較し、bag 末尾で保存して終了')
    parser.add_argument('--tracking', choices=('vo', 'vio'), default='vo',
                        help='vo はステレオのみ（既定）、vio は統合 IMU も使用')
    parser.add_argument('--slam-max-map-size', type=int, default=0,
                        help='SLAM 地図の姿勢数上限（既定 0 は無制限）')
    parser.add_argument('--backend', choices=('cuvslam', 'orbslam3'), default='cuvslam')
    maps = parser.add_mutually_exclusive_group()
    maps.add_argument('--save-map', metavar='ID', help='新しい ORB-SLAM3 コース地図を保存')
    maps.add_argument('--map', metavar='ID', help='保存済み ORB-SLAM3 地図に別走行から再定位')
    parser.add_argument('--gyro-bridge', choices=('auto', 'on', 'off'), default='auto',
                        help='固定 ORB 地図の途絶補完。auto は収録ジャイロがあれば有効（既定）、on は記録必須')
    parser.add_argument('--start-offset', type=float, default=0., help='固定地図を検証する bag の途中開始位置 [s]')
    parser.add_argument('--imu-calibration', type=Path,
                        help='ORB-SLAM3 VIO 用 IMU ノイズ校正 YAML。ATD Gazebo bag は省略時に試験値を使用')
    args = parser.parse_args(argv)
    if args.gyro_bridge != 'auto' and not (args.backend == 'orbslam3' and args.map):
        parser.error('--gyro-bridge は --backend orbslam3 --map ID と一緒に指定してください')
    if args.backend != 'orbslam3' and (args.map or args.save_map or args.start_offset or args.imu_calibration):
        parser.error('地図・途中開始・IMU校正の指定には --backend orbslam3 が必要です')
    if not math.isfinite(args.start_offset) or args.start_offset < 0 or (args.start_offset and not args.map):
        parser.error('--start-offset は --map と一緒に、0 以上の有限値で指定してください')
    if args.backend == 'orbslam3':
        if args.slam_max_map_size:
            parser.error('--slam-max-map-size は cuVSLAM の設定です')
        if args.tracking == 'vio' and (args.map or args.save_map):
            parser.error('固定地図の作成・再定位には --tracking vo を指定してください')
        if args.imu_calibration and args.tracking != 'vio':
            parser.error('--imu-calibration は --tracking vio と一緒に指定してください')
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('--rate には正の有限値を指定してください')
    if not 0 <= args.slam_max_map_size <= 2147483647:
        parser.error('--slam-max-map-size には 0〜2147483647 の整数を指定してください')
    try:
        roots = {}
        if os.environ.get('ATD_REPLAY_RECORD_ROOT'):
            roots['root'] = Path(os.environ['ATD_REPLAY_RECORD_ROOT']).expanduser().resolve()
        if os.environ.get('ATD_REPLAY_RESULTS_ROOT'):
            roots['results_root'] = Path(os.environ['ATD_REPLAY_RESULTS_ROOT']).expanduser().resolve()
        bag = select_bag(args.bag, **roots)
        if bag is not None:
            run(bag, args.rate, slam_max_map_size=args.slam_max_map_size, tracking=args.tracking, headless=args.headless,
                **({'orb_options': args} if args.backend == 'orbslam3' else {}))
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'atd replay: {exc}', file=sys.stderr)
        return 1

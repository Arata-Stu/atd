"""Notebook でステレオ bag を cuVSLAM 再生し、Gazebo の正解軌跡と比較する。"""
import argparse
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
from atd_localization.replay_player import TypedBagInput
from atd_mapping.capture_snapshot import stop
from atd_mapping.replay_dds import select_replay_dds

from .replay_gt_course import load_course
from .replay_gt_geometry import Trace
from .replay_gt_node import ground_truth_viewer
from .replay_gt_source import find_ground_truth
from .replay_inputs import GROUND_TRUTH_ERROR, GROUND_TRUTH_PATH, select_bag, validate_bag
from .replay_result import ReplayResult


class ViewerClosed(Exception):
    """RViz のウィンドウを閉じて再生セッションを終了した。"""


def estimated_only_config(preset, directory):
    """正解を照合できない bag では RViz も正解用の topic を購読しない。"""
    config = yaml.safe_load(preset.read_text())
    manager = config['Visualization Manager']
    manager['Displays'] = [display for display in manager['Displays']
                           if display.get('Topic', {}).get('Value') not in (GROUND_TRUTH_PATH, GROUND_TRUTH_ERROR)]
    output = Path(directory) / preset.name
    output.write_text(yaml.safe_dump(config, sort_keys=False))
    return output


def run_session(node, bag, rate, rviz_config, ground_truth=None, result=None, *, slam_max_map_size=0, imu=None):
    from .replay_readiness import wait_and_resume

    with ExitStack() as children:
        def start(command):
            process = subprocess.Popen(command, start_new_session=True, stdin=subprocess.DEVNULL)
            children.callback(stop, process)
            return process

        bag_input = TypedBagInput(node)
        children.callback(bag_input.close)
        player_name = f'atd_replay_player_{os.getpid()}'
        launch_args = ['ros2', 'launch', 'atd_bringup', 'bag_vslam.launch.py',
                       f'bag_path:={bag}', f'rate:={rate}', f'player_name:={player_name}',
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
            observer = children.enter_context(ground_truth_viewer(ground_truth, result, visualize=rviz is not None))
            spin_overlay = observer.spin_once

        def check_viewer():
            if launch.poll() is not None:
                raise RuntimeError(f'cuVSLAM が終了しました（exit {launch.returncode}）。起動ログを確認してください')
            if rviz is not None and rviz.poll() is not None:
                if rviz.returncode:
                    raise RuntimeError(f'RViz が終了しました（exit {rviz.returncode}）。画面接続と起動ログを確認してください')
                raise ViewerClosed()

        tracking = f'VIO (IMU {imu.frequency:g} Hz, {imu.frame})' if imu else 'VO'
        consumers = 'cuVSLAM と RViz' if rviz is not None else 'cuVSLAM と GT 比較用ノード'
        print(f'{bag} ({rate:g} 倍速)  {tracking}  slam_max_map_size={slam_max_map_size}'
              f'\n{consumers} の購読準備を待っています。', flush=True)
        finished = False
        try:
            connections = wait_and_resume(
                node, player_name=f'/{player_name}', check_running=check_viewer,
                ground_truth=spin_overlay is not None, vio=imu is not None,
                rviz_name='/rviz2' if rviz is not None else None, bag_input=bag_input)
            if result is not None:
                result.summary.update(sensor_input_transport='ros2_intra_process', bag_input_connections=connections)
            print('bag を再生しています。' + ('終了するには RViz を閉じるか Ctrl+C を押してください。'
                                           if rviz is not None else '末尾で比較結果を保存して終了します。'), flush=True)
            while True:
                if not finished and bag_input.finished:
                    counts = bag_input.counts(check_viewer, 5.)
                    print(f"bag センサー発行: 左 {counts['left_published']} / 右 {counts['right_published']}"
                          f" / IMU {counts['imu_published']} 件", flush=True)
                    if result is not None:
                        result.summary['sensor_input_counts'] = counts
                    if rviz is None:
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
                        observer.inspect_loop_graph()
                        result.mark_eof()
                        print(result.closure_report(), flush=True)
                    if rviz is None:
                        print(f'bag の末尾に到達しました。比較結果を保存しました: {result.directory}', flush=True)
                        return
                    print('bag の末尾に到達しました。RViz で最終表示を確認できます。', flush=True)
                check_viewer()
                bag_input.spin(0. if spin_overlay else .15)
                if spin_overlay:
                    spin_overlay(timeout_sec=.15)
        except ViewerClosed:
            # An estimate can arrive just after the last image was played.
            if finished and spin_overlay is not None:
                deadline = time.monotonic() + .5
                while time.monotonic() < deadline:
                    spin_overlay(timeout_sec=.05)


def run(bag, rate, *, slam_max_map_size=0, tracking='vo', headless=False):
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
    if config is not None and not config.is_file():
        raise ValueError(f'{config} がありません。atd build --packages-up-to atd_bringup を実行してください')

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
        root = Path(os.environ.get('ATD_REPLAY_RESULTS_ROOT', '/data/results')).expanduser().resolve()
        trace = None
        try:
            ground_truth = find_ground_truth(bag, root)
            if ground_truth is not None:
                trace = Trace.from_file(ground_truth)
        except (OSError, ValueError) as exc:
            if headless:
                raise ValueError(f'Gazebo 正解軌跡を照合できません: {exc}') from exc
            ground_truth = None
            print(f'Gazebo 正解軌跡を照合できません。cuVSLAM の推定軌跡のみ表示します。{exc}',
                  file=sys.stderr, flush=True)
        else:
            if ground_truth is None:
                if headless:
                    raise ValueError('--headless の比較には、この bag に対応する Gazebo 正解軌跡が必要です')
                print('対応する Gazebo 正解軌跡が見つからないため、cuVSLAM の推定軌跡のみ表示します。', flush=True)
        if ground_truth is None:
            config = estimated_only_config(config, stack.enter_context(TemporaryDirectory(prefix='atd-replay-rviz-')))
        select_replay_dds()
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        stack.callback(rclpy.try_shutdown)
        node = rclpy.create_node('atd_bag_replay')
        stack.callback(node.destroy_node)
        result = None
        if trace is not None:
            course = load_course(bag, ground_truth, root)
            result = stack.enter_context(ReplayResult(root, bag, rate, trace, course,
                                                     slam_max_map_size=slam_max_map_size, imu=imu))
            result.summary.update(backend='cuvslam', mode='mapping', alignment_source='first_estimate',
                                  metadata_sha256=hashlib.sha256((bag / 'metadata.yaml').read_bytes()).hexdigest())
            print(f'Gazebo 正解軌跡: {ground_truth}\n比較結果の保存先: {result.directory}', flush=True)
        run_session(node, bag, rate, config, trace, result, slam_max_map_size=slam_max_map_size, imu=imu)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?', help='/data/record/rosbags からの相対パス、または絶対パス')
    parser.add_argument('--rate', type=float, default=0.5, help='再生倍率（既定 0.5）')
    parser.add_argument('--headless', action='store_true', help='RViz を開かずに GT と比較し、bag 末尾で保存して終了')
    parser.add_argument('--tracking', choices=('vo', 'vio'), default='vo',
                        help='vo はステレオのみ（既定）、vio は統合 IMU も使用')
    parser.add_argument('--slam-max-map-size', type=int, default=0,
                        help='SLAM 地図の姿勢数上限（既定 0 は無制限）')
    args = parser.parse_args(argv)
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
            run(bag, args.rate, slam_max_map_size=args.slam_max_map_size, tracking=args.tracking, headless=args.headless)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'atd replay: {exc}', file=sys.stderr)
        return 1

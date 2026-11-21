"""Notebook で記録済みステレオ bag の地図作成を RViz で見る。"""
import argparse
from contextlib import ExitStack
import fcntl
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

from .replay_gt_geometry import Trace
from .replay_gt_source import find_ground_truth
from .replay_inputs import GROUND_TRUTH_ERROR, GROUND_TRUTH_PATH, STEREO_TOPICS, select_bag, validate_bag


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


def run_session(node, bag, rate, rviz_config, qos, ground_truth=None):
    from .replay_readiness import wait_and_resume

    with ExitStack() as children:
        def start(command):
            process = subprocess.Popen(command, start_new_session=True, stdin=subprocess.DEVNULL)
            children.callback(stop, process)
            return process

        launch = start(['ros2', 'launch', 'atd_bringup', 'bag_vslam.launch.py'])
        rviz = start(['rviz2', '-d', str(rviz_config), '--ros-args',
                      '-p', 'use_sim_time:=true', '-r', '__node:=rviz2'])
        overlay = None
        if ground_truth is not None:
            print(f'Gazebo 正解軌跡: {ground_truth}', flush=True)
            overlay = start(['ros2', 'run', 'atd_bringup', 'replay_ground_truth', str(ground_truth)])
        player_name = f'atd_replay_player_{os.getpid()}'
        player = start(['ros2', 'bag', 'play', str(bag), '--start-paused', '--disable-keyboard-controls',
                        '--clock', '--rate', str(rate), '--qos-profile-overrides-path', str(qos),
                        '--remap', f'__node:={player_name}',
                        '--topics', *STEREO_TOPICS])

        def check_viewer():
            if launch.poll() is not None:
                raise RuntimeError(f'cuVSLAM が終了しました（exit {launch.returncode}）。起動ログを確認してください')
            if rviz.poll() is not None:
                if rviz.returncode:
                    raise RuntimeError(f'RViz が終了しました（exit {rviz.returncode}）。画面接続と起動ログを確認してください')
                raise ViewerClosed()
            if overlay is not None and overlay.poll() is not None:
                raise RuntimeError(f'Gazebo 正解軌跡の表示が終了しました（exit {overlay.returncode}）。起動ログを確認してください')

        def check_startup():
            check_viewer()
            if player.poll() is not None:
                raise RuntimeError(f'再生開始前に rosbag2 が終了しました（exit {player.returncode}）')

        print(f'{bag} ({rate:g} 倍速)\ncuVSLAM と RViz の購読準備を待っています。', flush=True)
        try:
            wait_and_resume(node, player_name=f'/{player_name}', check_running=check_startup,
                            ground_truth=overlay is not None)
            print('bag を再生しています。終了するには RViz を閉じるか Ctrl+C を押してください。', flush=True)
            finished = False
            while True:
                check_viewer()
                if not finished and player.poll() is not None:
                    if player.returncode:
                        raise RuntimeError(f'rosbag2 の再生に失敗しました（exit {player.returncode}）')
                    print('bag の末尾に到達しました。RViz で最終表示を確認できます。', flush=True)
                    finished = True
                time.sleep(.15)
        except ViewerClosed:
            return


def run(bag, rate):
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise ValueError('RViz の画面出力先がありません。Notebook のデスクトップ端末で atd dev を起動してください')
    if not shutil.which('rviz2'):
        raise ValueError('コンテナに rviz2 がありません。Notebook の RViz 依存を準備してください')
    config = Path(get_package_share_directory('atd_bringup')) / 'config/bag_vslam.rviz'
    qos = Path(get_package_share_directory('atd_mapping')) / 'config/replay_qos.yaml'
    for path in (config, qos):
        if not path.is_file():
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
        validate_bag(bag)
        try:
            ground_truth = find_ground_truth(bag)
            if ground_truth is not None:
                Trace.from_file(ground_truth)
        except (OSError, ValueError) as exc:
            ground_truth = None
            print(f'Gazebo 正解軌跡を照合できません。cuVSLAM の推定軌跡のみ表示します。{exc}',
                  file=sys.stderr, flush=True)
        else:
            if ground_truth is None:
                print('対応する Gazebo 正解軌跡が見つからないため、cuVSLAM の推定軌跡のみ表示します。', flush=True)
        if ground_truth is None:
            config = estimated_only_config(config, stack.enter_context(TemporaryDirectory(prefix='atd-replay-rviz-')))
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        stack.callback(rclpy.try_shutdown)
        node = rclpy.create_node('atd_bag_replay')
        stack.callback(node.destroy_node)
        run_session(node, bag, rate, config, qos, ground_truth)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?', help='/data/record/rosbags からの相対パス、または絶対パス')
    parser.add_argument('--rate', type=float, default=0.5, help='再生倍率（既定 0.5）')
    args = parser.parse_args(argv)
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('--rate には正の有限値を指定してください')
    try:
        bag = select_bag(args.bag)
        if bag is not None:
            run(bag, args.rate)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'atd replay: {exc}', file=sys.stderr)
        return 1

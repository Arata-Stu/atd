"""Notebook 上の Gazebo と保存地図を読む Isaac ROS スタックを1試行として管理する。"""
import argparse
import json
import math
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time

from map_job import INSIDE, ISAAC, PROJECT, check_isaac_mounts, read_json, wait_exit, worker
from sensor_profile import RESOLUTIONS

sys.path.insert(0, str(PROJECT / 'ros2_ws/src/atd_planning'))
from atd_planning.core import LINE_KEYS, ReferenceLine


def choose(label, choices):
    if not sys.stdin.isatty():
        raise ValueError(f'{label}を引数で指定してください（候補: {", ".join(choices)}）')
    if shutil.which('fzf'):
        pick = subprocess.run(['fzf', '--prompt', label + ' > '], text=True,
                              input='\n'.join(choices) + '\n', stdout=subprocess.PIPE)
        if pick.returncode in (1, 130):
            return None
        pick.check_returncode()
        return pick.stdout.strip()
    for number, name in enumerate(choices, 1):
        print(f'{number:2d}) {name}', file=sys.stderr)
    print(label + 'の番号（空 Enter で戻る）> ', file=sys.stderr, end='', flush=True)
    selection = sys.stdin.readline().strip()
    if not selection:
        return None
    if not selection.isdecimal() or not 1 <= int(selection) <= len(choices):
        raise ValueError('一覧の番号を指定してください')
    return choices[int(selection) - 1]


def map_options(folder):
    """候補と描画解像度。地図・VGL engine の内容は ROS launch が再検査する。"""
    doc = read_json(folder / 'hdmap/map.json')
    vgl = read_json(folder / 'vgl.json')
    if (folder.is_symlink() or doc.get('id') != folder.name or doc.get('closed') is not True
            or doc.get('source', {}).get('kind') != 'nvidia_offline' or vgl.get('status') != 'complete'):
        raise ValueError('閉路の HDMap と VGL 準備済みの NVIDIA 地図が必要です')
    shape = vgl.get('input_shape', [])
    if (not isinstance(shape, list) or len(shape) != 4 or shape[:2] != [1, 3]
            or any(type(size) != int for size in shape[2:])):
        raise ValueError('VGL 地図の入力解像度が不正です')
    resolution = f'{shape[3]}x{shape[2]}'
    if resolution not in RESOLUTIONS:
        raise ValueError(f'地図の解像度 {resolution} は Gazebo で配信できません')
    lines = []
    for line in LINE_KEYS:
        try:
            ReferenceLine.from_document(doc, line)
        except ValueError:
            continue
        lines.append(line)
    if not lines:
        raise ValueError('地図の速度付き Centerline・Raceline・Customline のいずれかを保存してください')
    return lines, resolution


def select(args):
    maps = args.results / 'maps'
    valid_id = r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}'
    if args.map and not re.fullmatch(valid_id, args.map):
        raise ValueError('地図 ID が不正です')
    # プレビューは保存領域や Docker が未準備のマシンからも使える。
    if args.dry_run and ((args.map and not (maps / args.map).exists())
                         or (not args.map and (not maps.is_dir() or not sys.stdin.isatty()))):
        return (args.map or '<選択した地図>', args.line or 'raceline', args.resolution or '<地図の解像度>')
    selected = args.map
    if not selected:
        options = []
        for folder in sorted(maps.iterdir()) if maps.is_dir() else ():
            if not folder.is_dir() or not re.fullmatch(valid_id, folder.name):
                continue
            try:
                map_options(folder)
            except (OSError, ValueError, AttributeError, TypeError) as error:
                print(f'{folder.name}: {error}', file=sys.stderr)
            else:
                options.append(folder.name)
        if not options:
            raise ValueError('Notebook の「地図とライン」で bag から作成した NVIDIA 地図の VGL と走行ラインを準備してください')
        selected = choose('自己位置推定の地図 (--map)', options)
        if selected is None:
            return None
    lines, resolution = map_options(maps / selected)
    if args.resolution and args.resolution != resolution:
        raise ValueError(f'地図 {selected} の VGL 解像度は {resolution} です')
    line = args.line
    if line is None:
        line = choose('走行ライン (--line)', lines) if sys.stdin.isatty() and len(lines) > 1 else lines[0]
        if line is None:
            return None
    if line not in lines:
        raise ValueError(f'{selected} の速度付きラインを選んでください: {", ".join(lines)}')
    return selected, line, resolution


def finish_children(gazebo, ros, container):
    try:
        if gazebo.poll() is None:
            # 終了理由と最終状態を runner が保存する時間を確保する。
            try:
                gazebo.wait(timeout=4)
            except subprocess.TimeoutExpired:
                try:
                    subprocess.run(['docker', 'stop', '--time', '8', container], timeout=15,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                finally:
                    wait_exit(gazebo, 10)
    finally:
        if ros is not None:
            wait_exit(ros, 25)


def supervise(args, gazebo_command, ros_command):
    out = args.output
    gazebo = subprocess.Popen(gazebo_command, start_new_session=True)
    ros = None
    reason = '試行を終了しました'
    try:
        deadline = time.monotonic() + 180
        while not (out / 'gazebo.ready').is_file():
            if gazebo.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('/clock とセンサーの配信開始を確認できません。gazebo.log・run.json を確認してください')
            time.sleep(.25)
        print('Gazebo の時計・センサー配信開始。保存地図への位置合わせを待ちます。', flush=True)
        with (out / 'closed_loop.ros.log').open('x') as log:
            ros = subprocess.Popen(ros_command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            startup_deadline = time.monotonic() + 300
            active = False
            while True:
                result = out / 'closed_loop.result.json'
                if result.is_file():
                    report = read_json(result)
                    reason = f"走行結果 {report['status']}"
                    if report.get('reason'):
                        reason += f"（{report['reason']}）"
                    print(reason, result, flush=True)
                    return 0 if report['status'] == 'completed' else 1
                if gazebo.poll() is not None or ros.poll() is not None:
                    if result.is_file():
                        continue
                    raise RuntimeError(f'評価完了前にプロセスが終了しました (Gazebo={gazebo.returncode}, '
                                       f'Isaac ROS={ros.returncode})。closed_loop.ros.log・run.json を確認してください')
                if not active:
                    if (out / 'scene/closed_loop.active').is_file():
                        active = True
                        print('保存地図での追跡と最初の有効な制御指令を確認。走行を評価中です。', flush=True)
                    elif time.monotonic() >= startup_deadline:
                        raise RuntimeError('300実時間秒以内に自己位置推定から有効な走行指令まで到達しませんでした。closed_loop.ros.log を確認してください')
                time.sleep(.25)
    except KeyboardInterrupt:
        reason = 'ユーザーが走行を中断しました'
        raise
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        reason = str(error)
        raise
    finally:
        (out / 'closed_loop.cancel').write_text(reason + '\n')
        finish_children(gazebo, ros, args.container)


def host(args):
    if not math.isfinite(args.speed) or not .2 <= args.speed <= 3.2:
        raise ValueError('シミュレーションの速度上限は 0.2〜3.2 m/s で指定してください')
    if args.engine_dir and (Path(args.engine_dir).is_absolute() or '..' in Path(args.engine_dir).parts):
        raise ValueError('--engine-dir は results からの相対パスを指定してください')
    results = args.results.resolve()
    out = args.output.resolve()
    remote = Path('/data/results') / out.relative_to(results)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        raise ValueError('Gazebo 起動コマンドを指定してください')
    launch = ['ros2', 'launch', 'atd_bringup', 'gazebo_closed_loop.launch.py',
              f'map_id:={args.map}', f'line:={args.line}', f'max_speed_mps:={args.speed}']
    if args.engine_dir:
        launch.append('engine_dir:=' + args.engine_dir)
    if args.min_lookahead is not None:
        if not 0 < args.min_lookahead <= 1.5:
            raise ValueError('最短先読み距離は 0 より大きく、最大先読み距離 1.5 m 以下を指定してください')
        launch.append(f'min_lookahead_m:={args.min_lookahead}')
    ros = ['docker', 'exec', '-u', 'admin', ISAAC, '/usr/local/bin/atd-shell', 'env',
           f'ROS_DOMAIN_ID={args.domain}', f'FASTDDS_DEFAULT_PROFILES_FILE={args.profile}',
           'python3', str(INSIDE / 'tools/gazebo_monza/closed_loop_job.py'),
           'worker', '--output', str(remote), '--', *launch]
    if args.dry_run:
        print(shlex.join(command))
        print(shlex.join(ros))
        return 0
    session = dict(map_id=args.map, line=args.line, circuit=args.circuit, ros_domain_id=int(args.domain),
                   max_speed_mps=args.speed, engine_dir=args.engine_dir, min_lookahead_override_m=args.min_lookahead,
                   started=time.time())
    try:
        check_isaac_mounts(results)
        signal.signal(signal.SIGTERM, signal.default_int_handler)
        code = supervise(args, command, ros)
        session['status'] = read_json(out / 'closed_loop.result.json')['status']
        return code
    except KeyboardInterrupt:
        session['status'] = 'interrupted'
        raise
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        session.update(status='error', reason=str(error))
        raise
    finally:
        session['ended'] = time.time()
        (out / 'closed_loop.host.json').write_text(json.dumps(session, ensure_ascii=False, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    roles = parser.add_subparsers(dest='role', required=True)
    selection = roles.add_parser('select')
    selection.add_argument('--results', required=True, type=Path)
    selection.add_argument('--map')
    selection.add_argument('--line', choices=LINE_KEYS)
    selection.add_argument('--resolution', choices=RESOLUTIONS)
    selection.add_argument('--dry-run', action='store_true')
    child = roles.add_parser('worker')
    child.add_argument('--output', required=True, type=Path)
    child.add_argument('command', nargs=argparse.REMAINDER)
    launch = roles.add_parser('host')
    for key in ('output', 'results'):
        launch.add_argument('--' + key, type=Path, required=True)
    for key in ('domain', 'circuit', 'container', 'profile', 'map', 'line'):
        launch.add_argument('--' + key, required=True)
    launch.add_argument('--engine-dir')
    launch.add_argument('--min-lookahead', type=float)
    launch.add_argument('--speed', required=True, type=float)
    launch.add_argument('--dry-run', action='store_true')
    launch.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.role == 'select':
        choice = select(args)
        if choice:
            print('\t'.join(choice))
        return 0
    if args.role == 'worker':
        return worker(args.output, args.command[1:], cancel_name='closed_loop.cancel')
    return host(args)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (OSError, ValueError, RuntimeError, KeyError, subprocess.SubprocessError) as error:
        sys.exit('Gazebo closed-loop: ' + str(error))

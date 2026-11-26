"""Notebook 上の Gazebo と保存地図を読む Isaac ROS スタックを1試行として管理する。"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time

from map_job import INSIDE, ISAAC, PROJECT, check_isaac_mounts, read_json, wait_exit, worker
from sensor_profile import IMU_MODES, RESOLUTIONS

sys.path.insert(0, str(PROJECT / 'ros2_ws/src/atd_planning'))
sys.path.insert(0, str(PROJECT / 'ros2_ws/src/atd_control'))
from atd_control.bicycle import resolve_full_throttle_speed
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


def map_options(folder, requested_line, requested_resolution):
    """候補と描画解像度。地図・VGL engine の内容は ROS launch が再検査する。"""
    if not folder.is_dir():
        raise ValueError(f'保存地図のディレクトリが見つかりません（{folder}）')
    if not (folder / 'hdmap/map.json').is_file():
        raise ValueError('hdmap/map.json がありません。Web UI「地図とライン」でこの地図の HDMap と走行ラインを保存してください')
    if not (folder / 'vgl.json').is_file():
        raise ValueError('vgl.json がありません。Web UI「地図とライン」でこの地図の VGL 地図を準備してください')
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
    if requested_resolution and requested_resolution != resolution:
        raise ValueError(f'VGL 地図の解像度は {resolution} です（指定 {requested_resolution}）')
    lines = []
    for line in LINE_KEYS:
        try:
            ReferenceLine.from_document(doc, line)
        except ValueError:
            continue
        lines.append(line)
    if not lines:
        raise ValueError('地図の速度付き Centerline・Raceline・Customline のいずれかを保存してください')
    if requested_line and requested_line not in lines:
        raise ValueError(f'速度付き {requested_line} がありません（利用可能: {", ".join(lines)}）')
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
    if selected:
        folders = [maps / selected]
    else:
        folders = [p for p in sorted(maps.glob('*')) if p.is_dir() and re.fullmatch(valid_id, p.name)]
    options = {}
    for folder in folders:
        try:
            options[folder.name] = map_options(folder, args.line, args.resolution)
        except (OSError, ValueError, AttributeError, TypeError) as error:
            if selected:
                raise ValueError(f'地図 ID「{selected}」: {error}。別の保存地図を選ぶには --map を省略して再実行してください') from error
            print(f'{folder.name}: {error}', file=sys.stderr)
    if not selected:
        if not options:
            raise ValueError(f'{maps} に条件に合う保存地図がありません。Notebook の「地図とライン」で '
                             'bag から作成した NVIDIA 地図の VGL と走行ラインを準備してください')
        selected = choose('自己位置推定の地図 (--map)', list(options))
        if selected is None:
            return None
    lines, resolution = options[selected]
    line = args.line
    if line is None:
        line = choose('走行ライン (--line)', lines) if sys.stdin.isatty() and len(lines) > 1 else lines[0]
        if line is None:
            return None
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


def ros_exit_detail(path):
    """ROS launch の終了時に、該当するログ行を試行の停止理由に添える。"""
    try:
        with path.open('rb') as stream:
            end = stream.seek(0, 2)
            stream.seek(max(0, end - 16_384))
            lines = stream.read().decode(errors='replace').strip().splitlines()
    except OSError:
        return ''
    if not lines:
        return ''
    line = next((line for line in reversed(lines) if '[ERROR] [launch]' in line), lines[-1])
    return line[:2048]


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
        ros_log = out / 'closed_loop.ros.log'
        with ros_log.open('x') as log:
            ros = subprocess.Popen(ros_command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            previous_check = time.monotonic()
            startup_wait_s = 0.
            operator_paused = False
            active = False
            while True:
                result = out / 'closed_loop.result.json'
                if result.is_file():
                    report = read_json(result)
                    reason = f"走行結果 {report['status']}"
                    if report.get('reason'):
                        reason += f"（{report['reason']}）"
                    print(reason, result, flush=True)
                    return 0 if report['status'] in ('completed', 'stopped') else 1
                gazebo_code, ros_code = gazebo.poll(), ros.poll()
                if gazebo_code is not None or ros_code is not None:
                    if result.is_file():
                        continue
                    detail = ros_exit_detail(ros_log) if ros_code is not None else ''
                    gazebo_state = '稼働中' if gazebo_code is None else gazebo_code
                    ros_state = '稼働中' if ros_code is None else ros_code
                    raise RuntimeError(f'評価完了前にプロセスが終了しました (Gazebo={gazebo_state}, '
                                       f'Isaac ROS={ros_state})。{detail or "closed_loop.ros.log・run.json を確認してください"}')
                if not active:
                    now = time.monotonic()
                    if not operator_paused:
                        startup_wait_s += now - previous_check
                    previous_check = now
                    operator_paused = (out / 'scene/closed_loop.paused').is_file()
                    if (out / 'scene/closed_loop.active').is_file():
                        active = True
                        print('保存地図での追跡と最初の有効な制御指令を確認。走行を評価中です。', flush=True)
                    elif startup_wait_s >= 300:
                        raise RuntimeError('操作による一時停止を除く300実時間秒以内に自己位置推定から有効な走行指令まで到達しませんでした。closed_loop.ros.log を確認してください')
                time.sleep(.25)
    except KeyboardInterrupt:
        reason = 'ユーザーが走行を中断しました'
        raise
    except (OSError, RuntimeError, ValueError, KeyError, subprocess.SubprocessError) as error:
        # The worker uses the file's presence for shutdown. The evaluator also
        # needs to distinguish an infrastructure failure from an operator stop.
        reason = f'error\n{error}'
        raise
    finally:
        try:
            pending = out / 'closed_loop.cancel.tmp'
            pending.write_text(reason + '\n')
            pending.replace(out / 'closed_loop.cancel')
        finally:
            finish_children(gazebo, ros, args.container)


def host(args):
    full_speed = resolve_full_throttle_speed(args.speed, args.full_throttle_speed)
    if args.vgl_frequency is not None:
        if args.map_correction_source != 'vgl' or not math.isfinite(args.vgl_frequency) or args.vgl_frequency < 0:
            raise ValueError('--vgl-frequency は補正元 vgl で 0 以上の有限値を指定してください')
    imu = args.imu or ('400' if args.tracking == 'vio' else 'off')
    if args.tracking == 'vio' and imu == 'off':
        raise ValueError('VIO には --imu 200 または 400 が必要です')
    if args.engine_dir and (Path(args.engine_dir).is_absolute() or '..' in Path(args.engine_dir).parts):
        raise ValueError('--engine-dir は results からの相対パスを指定してください')
    results = args.results.resolve()
    out = args.output.resolve()
    remote = Path('/data/results') / out.relative_to(results)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        raise ValueError('Gazebo 起動コマンドを指定してください')
    command = [*command, '--full-throttle-speed', str(full_speed), '--imu', imu]
    launch = ['ros2', 'launch', 'atd_bringup', 'gazebo_closed_loop.launch.py',
              f'map_id:={args.map}', f'line:={args.line}', f'max_speed_mps:={args.speed}',
              f'full_throttle_speed_mps:={full_speed}', f'tracking:={args.tracking}',
              f'map_correction_source:={args.map_correction_source}',
              f'rviz:={str(args.rviz).lower()}', f'run_dir:={remote}']
    if args.vgl_frequency is not None:
        launch.append(f'vgl_frequency:={args.vgl_frequency}')
    if args.tracking == 'vio':
        launch.append('imu_frequency:=' + imu)
    if args.engine_dir:
        launch.append('engine_dir:=' + args.engine_dir)
    if args.web:
        launch.append('web:=true')
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
                   tracking=args.tracking, imu_hz=IMU_MODES[imu],
                   max_speed_mps=args.speed, full_throttle_speed_mps=full_speed, engine_dir=args.engine_dir,
                   min_lookahead_override_m=args.min_lookahead,
                   rviz=args.rviz, web=args.web, started=time.time())
    session['map_correction_source'] = args.map_correction_source
    if args.map_correction_source == 'vgl':
        session['vgl_frequency_hz'] = args.vgl_frequency if args.vgl_frequency is not None else 1.0
    try:
        check_isaac_mounts(results)
        signal.signal(signal.SIGTERM, signal.default_int_handler)
        code = supervise(args, command, ros)
        session['status'] = read_json(out / 'closed_loop.result.json')['status']
        return code
    except KeyboardInterrupt:
        session['status'] = 'interrupted'
        result = out / 'closed_loop.result.json'
        if result.is_file() and read_json(result)['status'] == 'stopped':
            session['status'] = 'stopped'
            print('耐久走行の結果を保存しました。', result, flush=True)
            return 0
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
    launch.add_argument('--rviz', action='store_true')
    launch.add_argument('--web', action='store_true')
    launch.add_argument('--tracking', choices=('vo', 'vio'), default='vo')
    launch.add_argument('--map-correction-source', choices=('cuvslam', 'vgl'), default='cuvslam')
    launch.add_argument('--vgl-frequency', type=float)
    launch.add_argument('--imu', choices=IMU_MODES)
    launch.add_argument('--speed', required=True, type=float)
    launch.add_argument('--full-throttle-speed', type=float)
    launch.add_argument('--dry-run', action='store_true')
    launch.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.role == 'select':
        choice = select(args)
        if choice:
            print('\t'.join(choice))
        return 0
    if args.role == 'worker':
        from ament_index_python.packages import PackageNotFoundError, get_package_prefix
        message = 'Notebook の Isaac ROS コンテナ内で atd build --packages-up-to atd_bringup を実行してください'
        try:
            prefix = Path(get_package_prefix('atd_bringup'))
        except PackageNotFoundError as error:
            raise RuntimeError(message) from error
        recorder = prefix / 'lib/atd_bringup/gazebo_closed_loop_log'
        if not recorder.is_file() or not os.access(recorder, os.X_OK):
            raise RuntimeError('解析ログ記録ノードが未ビルドです。' + message)
        return worker(args.output, args.command[1:], cancel_name='closed_loop.cancel')
    return host(args)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (OSError, ValueError, RuntimeError, KeyError, subprocess.SubprocessError) as error:
        sys.exit('Gazebo closed-loop: ' + str(error))

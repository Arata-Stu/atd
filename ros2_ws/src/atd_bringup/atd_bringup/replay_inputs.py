"""Notebook の VSLAM 再生に使う記録の選択と、ステレオ入力の事前検査。"""

from datetime import datetime, timedelta, timezone
import json
import math
import os
from pathlib import Path
import shutil
from statistics import median
import subprocess
import sys
from typing import NamedTuple

from atd_mapping.bag_inputs import inspect_bag, load_metadata
from .replay_gt_source import BAG_NAME, LEGACY_NAME, matching_manifest, matching_run


STEREO_WORKFLOW = {
    'left_image': '/camera/camera/infra1/image_rect_raw',
    'right_image': '/camera/camera/infra2/image_rect_raw',
    'left_info': '/camera/camera/infra1/camera_info',
    'right_info': '/camera/camera/infra2/camera_info',
    'tf_static': '/tf_static',
}
STEREO_TOPICS = tuple(STEREO_WORKFLOW.values())
IMU_TOPIC = '/camera/camera/imu'
GYRO_TOPIC = '/camera/camera/gyro/sample'
SLAM_ODOMETRY = '/visual_slam/vis/slam_odometry'
LOOP_CLOSURE_CLOUD = '/visual_slam/vis/loop_closure_cloud'
GROUND_TRUTH_PATH = '/atd/replay/ground_truth_path'
GROUND_TRUTH_ERROR = '/atd/replay/position_error'
GROUND_TRUTH_NODE = 'atd_replay_ground_truth'


def _legacy_sensor_label(relative, match, run):
    try:
        started = datetime.strptime(match['run'].split('-', 1)[0], '%Y%m%dT%H%M%SZ')
    except ValueError:
        return relative.as_posix()
    jst = started.replace(tzinfo=timezone.utc).astimezone(timezone(timedelta(hours=9), 'JST'))
    label = relative.with_name(f"{match['circuit']}  {jst:%Y-%m-%d %H:%M:%S %Z}").as_posix()
    if run is None:
        return f'{label}  センサー設定不明'

    directory, report = run
    audit = report['bag_audit']
    geometry = report.get('geometry') or audit.get('geometry')
    hz = report.get('camera_hz') or audit.get('nominal_sim_hz')
    imu = report.get('imu_hz')
    if geometry is None and isinstance(audit.get('stereo_calibration'), dict):
        left = audit['stereo_calibration'].get('left')
        if isinstance(left, dict) and left.get('width') and left.get('height'):
            geometry = [left['width'], left['height']]
    if geometry is None or hz is None or imu is None:
        try:
            scene = json.loads((directory / 'scene/scene.json').read_bytes())
            camera = scene.get('camera', {})
            geometry = geometry or [camera.get('width'), camera.get('height')]
            hz = hz or camera.get('rate_hz')
            if imu is None:
                imu = scene.get('imu', {}).get('rate_hz')
        except (OSError, ValueError, AttributeError):
            pass
    size = '解像度不明'
    if isinstance(geometry, list) and len(geometry) == 2 and all(type(v) is int and v > 0 for v in geometry):
        size = f'{geometry[0]}x{geometry[1]}'
    rate = f'{hz} Hz' if type(hz) is int and hz > 0 else '撮影Hz不明'
    inertial = 'IMU 不明'
    if type(imu) is int and imu >= 0:
        inertial = 'IMU OFF' if imu == 0 else f'IMU {imu} Hz'
    return f'{label}  {size}  {rate}  {inertial}'


def _positive_number(value):
    return type(value) in (float, int) and math.isfinite(value) and value > 0


def _legacy_bag_label(relative, root, results_root):
    legacy = LEGACY_NAME.fullmatch(relative.name)
    match = legacy or BAG_NAME.fullmatch(relative.name)
    if not match or match.groupdict().get('laps'):
        return relative.as_posix()
    bag = root / relative
    try:
        run = matching_run(bag, results_root)
    except (OSError, ValueError):
        run = None
    label = _legacy_sensor_label(relative, legacy, run) if legacy else relative.as_posix()
    report = run[1] if run else {}
    laps, speed = report.get('laps_requested'), report.get('speed_mps')
    needs_laps = type(laps) is not int or laps <= 0
    needs_speed = not _positive_number(speed)
    if needs_laps or needs_speed:
        try:
            course = matching_manifest(bag).get('course')
        except (OSError, ValueError):
            course = None
        if isinstance(course, dict) and course.get('circuit') == match['circuit']:
            if needs_laps:
                laps = course.get('laps')
            length, duration = course.get('lap_length_m'), course.get('lap_duration_s')
            if needs_speed and _positive_number(length) and _positive_number(duration):
                speed = length / duration
    lap_label = f'{laps}周' if type(laps) is int and laps > 0 else '周回数不明'
    speed_label = f'{speed:.6g} m/s' if _positive_number(speed) else '速度不明'
    return f'{label}  {lap_label}  {speed_label}'


def select_bag(value, root=Path('/data/record/rosbags'), *, results_root=Path('/data/results')):
    """絶対パス・記録ルートからの相対パスを指定、または対話形式で選択する。"""
    if value is None:
        if not sys.stdin.isatty():
            raise ValueError('非対話端末では atd replay BAG で bag のパスを指定してください')
        choices = sorted(p.parent.relative_to(root) for p in root.rglob('metadata.yaml') if p.is_file())
        if not choices:
            raise ValueError(f'{root} に metadata.yaml を含む bag がありません')
        rows = [f'{i:2d}) {_legacy_bag_label(path, root, results_root)}' for i, path in enumerate(choices, 1)]
        try:
            if shutil.which('fzf'):
                result = subprocess.run(
                    ['fzf', '--read0', '--print0', '--prompt=再生する bag > '],
                    input=os.fsencode('\0'.join(rows)), stdout=subprocess.PIPE)
                if result.returncode in (1, 130):
                    return None
                if result.returncode:
                    raise RuntimeError(f'fzf が終了コード {result.returncode} で失敗しました')
                row = os.fsdecode(result.stdout.removesuffix(b'\0'))
                if row not in rows:
                    raise ValueError('一覧から bag を選んでください')
                value = choices[rows.index(row)]
            else:
                for row in rows:
                    print(row)
                number = input('再生する bag の番号（空欄で終了）: ').strip()
                if not number:
                    return None
                if not number.isdecimal() or not 1 <= int(number) <= len(choices):
                    raise ValueError('一覧にある bag の番号を指定してください')
                value = choices[int(number) - 1]
        except (EOFError, KeyboardInterrupt):
            return None

    bag = (root / Path(value).expanduser()).resolve()
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError(f'bag の metadata.yaml が見つかりません: {bag / "metadata.yaml"}')
    return bag


class ImuProfile(NamedTuple):
    frame: str
    frequency: float


def gap_gyro_topic(path, *, required):
    """固定地図の補完には統合 IMU または D455 の単独ジャイロを使う。"""
    info = load_metadata(path)
    rows = {r['topic_metadata']['name']: r for r in info['topics_with_message_count']}
    for name in (IMU_TOPIC, GYRO_TOPIC):
        row = rows.get(name)
        if row and row['message_count'] >= 2 and row['topic_metadata']['type'] == 'sensor_msgs/msg/Imu':
            return name
    if required:
        raise ValueError(f'ジャイロ補完には {IMU_TOPIC} または {GYRO_TOPIC} の記録が必要です')
    return None


def recorded_imu(path, topic=IMU_TOPIC):
    """統合 IMU のヘッダーから、収録時の座標系とシミュレーション時刻での周期を読む。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    metadata = load_metadata(path)
    if not any(entry['topic_metadata']['name'] == topic and
               entry['topic_metadata']['type'] == 'sensor_msgs/msg/Imu' and entry['message_count'] >= 2
               for entry in metadata['topics_with_message_count']):
        kind = '統合 IMU' if topic == IMU_TOPIC else 'ジャイロ'
        raise ValueError(f'{kind} {topic} の記録がありません。'
                         'Gazebo では --imu 400 で記録してください')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    stamps = []
    frame = None
    while len(stamps) < 65 and reader.has_next():
        header = deserialize_message(reader.read_next()[1], Imu).header
        name = header.frame_id.lstrip('/')
        if not name or (frame is not None and frame != name):
            raise ValueError(f'{topic} の座標系を確認できません')
        frame = name
        stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        if stamps and stamp <= stamps[-1]:
            raise ValueError(f'{topic} の時刻が単調増加していません')
        stamps.append(stamp)
    if len(stamps) < 2:
        raise ValueError(f'{topic} の記録から周期を計算できません')
    period_ns = median(b - a for a, b in zip(stamps, stamps[1:]))
    return ImuProfile(frame, 1e9 / period_ns)


def validate_bag(path, *, vio=False):
    """再生に使うセンサーと車体の固定 TF を検査する。"""
    imu = recorded_imu(path) if vio else None
    inspect_bag(path, dict(STEREO_WORKFLOW, imu=IMU_TOPIC), {
        'tracking_mode': int(vio), 'imu_frame': imu.frame if imu else None,
        'base_frame': 'base_link',
        'camera_optical_frames': ['camera_infra1_optical_frame', 'camera_infra2_optical_frame'],
    })
    return imu

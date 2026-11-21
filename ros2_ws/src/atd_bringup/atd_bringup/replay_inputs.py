"""Notebook の VSLAM 再生に使う記録の選択と、ステレオ入力の事前検査。"""

from pathlib import Path
import shutil
from statistics import median
import subprocess
import sys
from typing import NamedTuple

from atd_mapping.bag_inputs import inspect_bag, load_metadata


STEREO_WORKFLOW = {
    'left_image': '/camera/camera/infra1/image_rect_raw',
    'right_image': '/camera/camera/infra2/image_rect_raw',
    'left_info': '/camera/camera/infra1/camera_info',
    'right_info': '/camera/camera/infra2/camera_info',
    'tf_static': '/tf_static',
}
STEREO_TOPICS = tuple(STEREO_WORKFLOW.values())
IMU_TOPIC = '/camera/camera/imu'
SLAM_ODOMETRY = '/visual_slam/vis/slam_odometry'
GROUND_TRUTH_PATH = '/atd/replay/ground_truth_path'
GROUND_TRUTH_ERROR = '/atd/replay/position_error'
GROUND_TRUTH_NODE = 'atd_replay_ground_truth'


def select_bag(value, root=Path('/data/record/rosbags')):
    """絶対パス・記録ルートからの相対パスを指定、または対話形式で選択する。"""
    if value is None:
        if not sys.stdin.isatty():
            raise ValueError('非対話端末では atd replay BAG で bag のパスを指定してください')
        choices = sorted(p.parent.relative_to(root).as_posix() for p in root.rglob('metadata.yaml')
                         if p.is_file())
        if not choices:
            raise ValueError(f'{root} に metadata.yaml を含む bag がありません')
        try:
            if shutil.which('fzf'):
                result = subprocess.run(
                    ['fzf', '--read0', '--print0', '--prompt=再生する bag > '],
                    input='\0'.join(choices), stdout=subprocess.PIPE, text=True)
                if result.returncode in (1, 130):
                    return None
                if result.returncode:
                    raise RuntimeError(f'fzf が終了コード {result.returncode} で失敗しました')
                value = result.stdout.removesuffix('\0')
                if value not in choices:
                    raise ValueError('一覧から bag を選んでください')
            else:
                for index, name in enumerate(choices, 1):
                    print(f'{index:2d}) {name}')
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


def recorded_imu(path):
    """統合 IMU のヘッダーから、収録時の座標系とシミュレーション時刻での周期を読む。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    metadata = load_metadata(path)
    if not any(entry['topic_metadata']['name'] == IMU_TOPIC and
               entry['topic_metadata']['type'] == 'sensor_msgs/msg/Imu' and entry['message_count'] >= 2
               for entry in metadata['topics_with_message_count']):
        raise ValueError(f'VIO に必要な統合 IMU {IMU_TOPIC} の記録がありません。'
                         'Gazebo では --imu 400 で記録してください')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[IMU_TOPIC]))
    stamps = []
    frame = None
    while len(stamps) < 65 and reader.has_next():
        header = deserialize_message(reader.read_next()[1], Imu).header
        name = header.frame_id.lstrip('/')
        if not name or (frame is not None and frame != name):
            raise ValueError(f'{IMU_TOPIC} の座標系を確認できません')
        frame = name
        stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        if stamps and stamp <= stamps[-1]:
            raise ValueError(f'{IMU_TOPIC} の時刻が単調増加していません')
        stamps.append(stamp)
    if len(stamps) < 2:
        raise ValueError(f'{IMU_TOPIC} の記録から周期を計算できません')
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

"""固定カメラの校正を復元し、実際に再生するステレオ画像の時間範囲を調べる。"""
from array import array
from dataclasses import dataclass
import math
from pathlib import Path
from statistics import median

from atd_mapping.bag_inputs import load_metadata
from atd_mapping.frames import stamp_ns
from atd_localization.replay_inputs import stereo_window
from .replay_inputs import IMU_TOPIC, STEREO_WORKFLOW


@dataclass
class OrbInputs:
    camera_fps: float
    first_left_ns: int
    last_left_ns: int
    pairs: int


def inputs_from_bag(bag, offset, *, vio=False):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, Imu

    metadata = load_metadata(bag)
    start = metadata['starting_time']['nanoseconds_since_epoch']
    duration = metadata['duration']['nanoseconds']
    cut = start + round(offset * 1e9)
    if cut >= start + duration:
        raise ValueError('開始位置が bag の末尾を超えています')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    left_image, right_image = (STEREO_WORKFLOW[k] for k in ('left_image', 'right_image'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[left_image, right_image, *([IMU_TOPIC] if vio else [])]))
    timestamps = {left_image: array('q'), right_image: array('q')}
    imu_first = imu_last = None
    while reader.has_next():
        topic, raw, receive_ns = reader.read_next()
        if topic == IMU_TOPIC:
            stamp = stamp_ns(deserialize_message(raw, Imu).header.stamp)
            if stamp <= 0 or (imu_last is not None and stamp <= imu_last):
                raise ValueError('統合 IMU の観測時刻が不正です')
            imu_last = stamp
            if imu_first is None:
                imu_first = stamp
        elif receive_ns >= cut:
            stamps = timestamps[topic]
            stamp = stamp_ns(deserialize_message(raw, Image).header.stamp)
            if stamp <= 0 or (stamps and stamp <= stamps[-1]):
                raise ValueError(f'bag の画像時刻は正かつ単調増加である必要があります: {topic}')
            stamps.append(stamp)
    left = timestamps[left_image]
    right = timestamps[right_image]
    if vio:
        if imu_last is None:
            raise ValueError('統合 IMU の観測がありません')
        # The node waits for IMU coverage through the left image before processing a pair.
        left = array('q', (stamp for stamp in left if imu_first <= stamp <= imu_last))
    window = stereo_window(left, right, 1_000_000, left_stamp=True)
    hz = 1e9 / median(b - a for a, b in zip(left[:256], left[1:256]))
    # bagの録画時刻の丸めのため 89.999... Hz となる場合を同じ公称レートとして扱う。
    fps = float(round(hz)) if abs(hz - round(hz)) < .05 else round(hz, 3)
    return OrbInputs(fps, window['first_stamp_ns'], window['last_stamp_ns'], window['stereo_pairs'])


def noise_settings(path):
    import yaml
    with path.open(encoding='utf-8') as stream:
        try:
            value = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            raise ValueError(f'IMU 校正 YAML が不正です: {path}') from exc
    keys = {'gyro_noise_density', 'accel_noise_density', 'gyro_random_walk', 'accel_random_walk'}
    if not isinstance(value, dict) or set(value) != keys or not all(
            type(v) in (float, int) and math.isfinite(v) and v > 0 for v in value.values()):
        raise ValueError('IMU 校正 YAML に正の有限値で指定してください: ' + ', '.join(sorted(keys)))
    return {k: float(v) for k, v in value.items()}


def resolve_imu_noise(path, *, gazebo):
    """明示校正を優先し、ATD Gazebo と照合済みの録画にだけ基準の重みを使う。"""
    if path is not None:
        path = Path(path).expanduser().resolve()
        return str(path), noise_settings(path)
    if not gazebo:
        raise ValueError('この bag の ORB-SLAM3 VIO には --imu-calibration で IMU 校正 YAML を指定してください')
    from ament_index_python.packages import get_package_share_directory
    profile = Path(get_package_share_directory('atd_bringup')) / 'config/orb_imu_gazebo.yaml'
    return 'gazebo_euroc_reference', noise_settings(profile)

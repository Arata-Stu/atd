"""固定 TF から車体上向き軸を求め、記録したジャイロをその軸に射影する。"""
import math
from statistics import median

from atd_localization.state import RigidPose
from atd_mapping.bag_inputs import load_metadata
from atd_mapping.frames import rotate, stamp_ns, transform_record

from .replay_inputs import IMU_TOPIC, STEREO_WORKFLOW


def _add_transform(parents, message):
    tf = transform_record(message)
    parent, child = tf['parent'], tf['child']
    if not parent or not child or parent == child:
        raise ValueError(f'不正な固定 TF: {parent} → {child}')
    pose = RigidPose.checked(tf['translation'], tf['rotation'])
    if child in parents:
        old_parent, old = parents[child]
        # q と -q は同じ回転。再送された同じ静的変換は1本として扱う。
        same_rotation = min(math.dist(old.xyzw, pose.xyzw),
                            math.dist(old.xyzw, [-v for v in pose.xyzw])) <= 1e-9
        if old_parent != parent or math.dist(old.xyz, pose.xyz) > 1e-9 or not same_rotation:
            raise ValueError(f'bag の固定 TF が途中で変化しています: {child}')
    else:
        parents[child] = parent, pose


def _ancestors(frame, parents):
    chain = [frame]
    while frame in parents:
        frame = parents[frame][0]
        if frame in chain:
            raise ValueError(f'固定 TF が循環しています: {frame}')
        chain.append(frame)
    return chain


def _body_axis(frame, parents):
    source = _ancestors(frame, parents)
    base = _ancestors('base_link', parents)
    common = next((f for f in source if f in base), None)
    if common is None:
        raise ValueError(f'base_link と {frame} を結ぶ固定 TF がありません')
    source_leg, base_leg = source[:source.index(common)], base[:base.index(common)]
    axis = (0., 0., 1.)
    for child in base_leg:
        axis = rotate(axis, parents[child][1].xyzw)
    for child in reversed(source_leg):
        x, y, z, w = parents[child][1].xyzw
        axis = rotate(axis, (-x, -y, -z, w))
    return axis, source_leg + [common] + list(reversed(base_leg))


def read_bag_gyro(bag):
    """画像時刻と比較できる計測時刻、車体の Z 軸角速度 [rad/s]、集計情報を返す。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage

    info = load_metadata(bag)
    types = {row['topic_metadata']['name']: row['topic_metadata']['type']
             for row in info['topics_with_message_count']}
    tf_topic = STEREO_WORKFLOW['tf_static']
    if types.get(IMU_TOPIC) != 'sensor_msgs/msg/Imu':
        raise ValueError(f'統合 IMU {IMU_TOPIC} (sensor_msgs/msg/Imu) がありません')
    if tf_topic in types and types[tf_topic] != 'tf2_msgs/msg/TFMessage':
        raise ValueError(f'{tf_topic} の ROS メッセージ型が不正です')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=info['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[IMU_TOPIC, tf_topic]))
    parents, stamps, velocities = {}, [], []
    frame = None
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == tf_topic:
            for tf in deserialize_message(raw, TFMessage).transforms:
                _add_transform(parents, tf)
        else:
            imu = deserialize_message(raw, Imu)
            name = imu.header.frame_id.lstrip('/')
            if not name or (frame is not None and frame != name):
                raise ValueError(f'{IMU_TOPIC} の座標系を一意に確認できません')
            frame = name
            stamp = stamp_ns(imu.header.stamp)
            if stamp < 0 or (stamps and stamp <= stamps[-1]):
                raise ValueError(f'{IMU_TOPIC} の計測時刻が不正です: {stamp}')
            w = imu.angular_velocity
            variance = imu.angular_velocity_covariance[0]
            if variance < 0 or not all(map(math.isfinite, (variance, w.x, w.y, w.z))):
                raise ValueError(f'{IMU_TOPIC} の角速度を確認できません: {stamp}')
            stamps.append(stamp)
            velocities.append((w.x, w.y, w.z))
    if len(stamps) < 2:
        raise ValueError(f'{IMU_TOPIC} のジャイロ計測が2件未満です')
    (ax, ay, az), path = _body_axis(frame, parents)
    rates = [ax*x + ay*y + az*z for x, y, z in velocities]
    metadata = dict(topic=IMU_TOPIC, imu_frame=frame, base_frame='base_link', tf_path=path,
                    sample_count=len(stamps), first_stamp_ns=stamps[0], last_stamp_ns=stamps[-1],
                    median_period_ms=median(b - a for a, b in zip(stamps, stamps[1:])) * 1e-6)
    return stamps, rates, metadata

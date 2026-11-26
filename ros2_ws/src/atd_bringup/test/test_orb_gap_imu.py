"""bag の収録時刻と IMU 軸を使い、車体座標のヨー角速度を復元する。"""
import json
import math
import sys
from types import SimpleNamespace as NS

import pytest

from atd_bringup.orb_gap_imu import read_bag_gyro
from atd_bringup.replay_inputs import IMU_TOPIC


def imu(t, wx, frame='imu'):
    return NS(header=NS(frame_id=frame, stamp=NS(sec=0, nanosec=t)),
              angular_velocity=NS(x=wx, y=0., z=0.), angular_velocity_covariance=[0.] * 9)


def tf(parent, child, q):
    return NS(header=NS(frame_id=parent, stamp=NS(sec=0, nanosec=0)), child_frame_id=child,
              transform=NS(translation=NS(x=8., y=4., z=2.), rotation=NS(**dict(zip('xyzw', q)))))


def read_fixture(monkeypatch, bag, records):
    """The fake storage applies the topic filter exactly as set by the real reader."""
    types = {IMU_TOPIC: 'sensor_msgs/msg/Imu', '/tf_static': 'tf2_msgs/msg/TFMessage', '/image': 'sensor_msgs/msg/Image'}
    topics = set(name for name, _ in records)
    entries = [dict(topic_metadata=dict(name=name, type=types[name]), message_count=sum(n == name for n, _ in records))
               for name in topics]
    (bag / 'metadata.yaml').write_text(json.dumps(dict(rosbag2_bagfile_information=dict(
        storage_identifier='mcap', topics_with_message_count=entries))))

    class Reader:
        def open(self, storage, _):
            assert storage.uri == str(bag) and storage.storage_id == 'mcap'

        def set_filter(self, filtering):
            assert set(filtering.topics) == {IMU_TOPIC, '/tf_static'}
            self.messages = [(topic, message, i) for i, (topic, message) in enumerate(records) if topic in filtering.topics]

        def has_next(self):
            return bool(self.messages)

        def read_next(self):
            return self.messages.pop(0)

    monkeypatch.setitem(sys.modules, 'rosbag2_py', NS(SequentialReader=Reader, StorageOptions=NS, StorageFilter=NS,
                                                      ConverterOptions=lambda *_: None))
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', NS(deserialize_message=lambda data, _: data))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(Imu=object))
    monkeypatch.setitem(sys.modules, 'tf2_msgs.msg', NS(TFMessage=object))
    return read_bag_gyro(bag)


def test_child_and_parent_rotations_use_gyro_stamp_and_ignore_translation(monkeypatch, tmp_path):
    h = math.sqrt(.5)
    transforms = [tf('world', 'base_link', (h, 0., 0., h)), tf('world', 'rig', (0., 0., h, h)),
                  tf('rig', 'imu', (0., 0., 0., 1.)), tf('world', 'rig', (0., 0., -h, -h))]
    records = [('/image', object()), (IMU_TOPIC, imu(100, -2.)), (IMU_TOPIC, imu(2500100, 1.)),
               ('/tf_static', NS(transforms=transforms))]
    stamps, rates, info = read_fixture(monkeypatch, tmp_path, records)
    assert stamps == [100, 2500100]
    assert rates == pytest.approx([2., -1.])
    assert info['tf_path'] == ['imu', 'rig', 'world', 'base_link']
    assert info['median_period_ms'] == pytest.approx(2.5)


def test_sensor_already_in_body_frame(monkeypatch, tmp_path):
    records = [(IMU_TOPIC, imu(t, 6., '/base_link')) for t in (0, 1000000)]
    records[0][1].angular_velocity.z = 3.
    _, rates, info = read_fixture(monkeypatch, tmp_path, records)
    assert rates == [3., 0.] and info['tf_path'] == ['base_link']


@pytest.mark.parametrize('issue', ['parent', 'rotation', 'cyclic', 'disconnected', 'quaternion',
                                   'frame', 'stamp', 'nonfinite', 'unavailable'])
def test_invalid_sensor_or_transform_is_rejected(monkeypatch, tmp_path, issue):
    first, second = imu(100, -2.), imu(200, -1.)
    identity = (0., 0., 0., 1.)
    transforms = [tf('base_link', 'imu', identity)]
    if issue == 'parent':
        transforms.append(tf('another', 'imu', identity))
    elif issue == 'rotation':
        transforms.append(tf('base_link', 'imu', (0., 1., 0., 0.)))
    elif issue == 'cyclic':
        transforms.append(tf('imu', 'base_link', identity))
    elif issue == 'disconnected':
        transforms.clear()
    elif issue == 'quaternion':
        transforms[0].transform.rotation.w = 0.
    elif issue == 'frame':
        second.header.frame_id = 'other'
    elif issue == 'stamp':
        second.header.stamp = first.header.stamp
    elif issue == 'nonfinite':
        first.angular_velocity.z = math.nan
    elif issue == 'unavailable':
        second.angular_velocity_covariance[0] = -1.
    with pytest.raises(ValueError):
        read_fixture(monkeypatch, tmp_path, [(IMU_TOPIC, first), ('/tf_static', NS(transforms=transforms)),
                                              (IMU_TOPIC, second)])

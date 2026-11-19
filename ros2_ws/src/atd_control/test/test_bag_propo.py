from contextlib import closing
import sqlite3
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from atd_control.bag_propo import JOY_TOPIC, JOY_TYPE, TOPIC, TYPE, extract, readings
from atd_control.manual_inputs import match_manual


def telemetry(ns, raw, manual=True):
    return NS(header=NS(frame_id='pico', stamp=NS(sec=10, nanosec=ns)), instance_id='session', serial='pico',
              channels=np.array([1024, raw] + [1024]*14, dtype=np.uint16), active_path=1 if manual else 2, power_ready=True,
              sbus_fresh=True, sbus_flags=0, sbus_age_ms=3, fault_bits=0, rc_valid=True)


def command(ns, throttle, valid=True):
    return NS(header=NS(stamp=NS(sec=10, nanosec=ns)), source='joy',
              valid=valid, throttle=throttle, enable_requested=True)


@pytest.mark.parametrize('topics', [(TOPIC,), (JOY_TOPIC,), (TOPIC, JOY_TOPIC), ()])
def test_extracts_each_recorded_input_and_preserves_invalid_barriers(tmp_path, monkeypatch, topics):
    start = 10_000_000_000
    data = [(topic, msg, start+offset) for topic, msg, offset in [
        (TOPIC, telemetry(1, 1200), 1), (TOPIC, telemetry(1, 1205), 2),
        (JOY_TOPIC, command(2, -.5), 3), (TOPIC, telemetry(4, 1300, manual=False), 4),
        (JOY_TOPIC, command(2, -.5), 5), (JOY_TOPIC, command(2, 0., False), 7),
        (JOY_TOPIC, command(2, -.5), 8), (JOY_TOPIC, command(9, 0.), 10),
    ] if topic in topics]
    (tmp_path / 'lap.mcap').write_bytes(b'fixture')
    kinds = {TOPIC: TYPE, JOY_TOPIC: JOY_TYPE}
    counts = {topic: sum(t == topic for t, _, _ in data) for topic in topics}
    meta = dict(relative_file_paths=['lap.mcap'], storage_identifier='mcap', topics_with_message_count=[
        dict(topic_metadata=dict(name=t, type=kinds[t]), message_count=n) for t, n in counts.items()])
    monkeypatch.setitem(sys.modules, 'atd_mapping.bag_inputs', NS(load_metadata=lambda _: meta))
    monkeypatch.setitem(sys.modules, 'atd_interfaces.msg', NS(PicoTelemetry=object, DriveCommand=object))
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', NS(deserialize_message=lambda msg, cls: msg))
    class Reader:
        def open(self, storage, converter):
            assert storage.storage_id == 'mcap' and storage.uri == str(tmp_path)
            self.index = 0
        def set_filter(self, condition):
            assert condition.topics == list(topics)
        def has_next(self):
            return self.index < len(data)
        def read_next(self):
            self.index += 1
            return data[self.index - 1]
    monkeypatch.setitem(sys.modules, 'rosbag2_py', NS(SequentialReader=Reader, StorageOptions=NS,
                        ConverterOptions=lambda *_:None, StorageFilter=NS))
    output = tmp_path/'samples.sqlite'
    extract(tmp_path, output)
    with closing(sqlite3.connect(output)) as db:
        assert dict(db.execute('SELECT topic,messages FROM topics')) == counts
        rows = list(db.execute('SELECT stamp,raw,reason,path FROM propo ORDER BY rowid'))
        joy = list(db.execute('SELECT effective,stamp,throttle,enabled,reason FROM joy ORDER BY effective,rowid'))
    if TOPIC in topics:
        assert rows[0] == (start+1, 1200, '', 'propo')
        assert rows[1][1] is None and '重複' in rows[1][2]
        assert rows[2][1] is None and rows[2][3] == 'host'
    else:
        assert not rows
    if JOY_TOPIC in topics:
        assert len(joy) == 3
        assert joy[0] == (start+2, start+2, -.5, 1, '')
        assert joy[1][:4] == (start+7, start+2, None, None) and joy[1][4]
        assert joy[2] == (start+9, start+9, 0., 1, '')
    else:
        assert not joy


def test_ambiguous_clock_rollback_is_rejected():
    with pytest.raises(ValueError, match='逆行'):
        list(readings([telemetry(200, 1100), telemetry(100, 1300)]))


def test_serialized_mcap_observes_propo_and_dualsense(tmp_path):
    rosbag = pytest.importorskip('rosbag2_py')
    from atd_interfaces.msg import DriveCommand, PicoTelemetry
    from rclpy.serialization import serialize_message

    bag = tmp_path/'manual'
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag.ConverterOptions('', ''))
    for i, (topic, kind) in enumerate(((TOPIC, TYPE), (JOY_TOPIC, JOY_TYPE))):
        writer.create_topic(rosbag.TopicMetadata(id=i, name=topic, type=kind, serialization_format='cdr'))
    start = 10_000_000_000
    pico = PicoTelemetry(instance_id='session', serial='fixture', active_path=1, power_ready=True,
                         rc_valid=True, sbus_fresh=True, channels=[1024, 1100]+[1024]*14)
    pico.header.frame_id, pico.header.stamp.sec = 'pico', 10
    writer.write(TOPIC, serialize_message(pico), start)
    pico.active_path, pico.header.stamp.nanosec = 2, 1_000_000
    writer.write(TOPIC, serialize_message(pico), start+1_000_000)
    joy = DriveCommand(source='joy', valid=True, throttle=-.375, enable_requested=True)
    joy.header.frame_id = 'atd_gamepad'
    joy.header.stamp.sec, joy.header.stamp.nanosec = 10, 2_000_000
    for offset in (3_000_000, 4_000_000):
        writer.write(JOY_TOPIC, serialize_message(joy), start+offset)
    del writer  # flush rosbag2 metadata

    output = tmp_path/'inputs.sqlite'
    extract(bag, output)
    with closing(sqlite3.connect(output)) as db:
        p0, p1 = db.execute('SELECT stamp,path,raw,reason FROM propo ORDER BY rowid')
        [(stamp, value, enabled, why)] = db.execute('SELECT stamp,throttle,enabled,reason FROM joy')
        assert dict(db.execute('SELECT topic,messages FROM topics')) == {TOPIC: 2, JOY_TOPIC: 2}
    assert match_manual(start, p0)['propo_raw'] == 1100
    sample = match_manual(start+5_000_000, p1, (stamp, value, bool(enabled), why))
    assert sample['propo_raw'] is None and sample['manual_source'] == 'host_joy'
    assert sample['joy_throttle'] == -.375 and sample['joy_age_s'] == .003

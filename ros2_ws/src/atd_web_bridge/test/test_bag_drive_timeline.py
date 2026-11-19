from collections import Counter
from contextlib import closing
import sqlite3
import sys
from types import SimpleNamespace as N

import pytest
import yaml

from atd_web_bridge.bag_drive_timeline import CDC0, CDC1, JOY, NANO, ROS_TYPES, build_index, extract, inspect
from atd_web_bridge.bag_motion import ACCEL, IMU, TF, LOCALIZATION, STEREO_TYPES


START = 1_800_000_000 * NANO
MS = 1_000_000


def header(offset, frame):
    sec, nsec = divmod(START + offset * MS, NANO)
    return N(stamp=N(sec=sec, nanosec=nsec), frame_id=frame)


def joy(ms, steering=0., throttle=0., valid=True):
    return N(header=header(ms, 'atd_gamepad'), source='joy', valid=valid,
             steering=steering, throttle=throttle, enable_requested=False)


def board(ms, ch3, path, *, driver=False, seq=None, faults=0, servo=1500, esc=1500):
    return N(header=header(ms, 'pico'), serial='PICO', session_id='test', has_report=True,
             sequence_number=ms if seq is None else seq, selector=int(path == 2 or ch3 >= 1700),
             rx_mode_us=ch3, active_path=path, joy_active=driver, fault_bits=faults,
             servo_us=servo, esc_us=esc, reason='')


def telemetry(ms, serial='pico', path=2, valid=True):
    return N(header=header(ms, 'pico'), serial=serial, instance_id='usb', sequence_number=ms,
             active_path=path, fault_bits=0, sbus_fresh=valid, sbus_flags=0, rc_valid=valid,
             channels=[992, 992, 1800] + [0]*13)


def rows(messages, table, fields, duration=1000):
    counts = Counter(topic for topic, msg, ms in messages)
    with closing(sqlite3.connect(':memory:')) as db:
        build_index(db, START, duration * MS, {topic: (n, '') for topic, n in counts.items()},
                    ((topic, msg, START + ms * MS) for topic, msg, ms in messages))
        return db.execute(f'SELECT {fields} FROM {table} ORDER BY t').fetchall()


def test_ch3_handoff_and_gate_change_on_same_board_frame():
    active = board(100, 1900, 2, driver=True, servo=1250, esc=1600)
    revoked = board(100, 1900, 2, servo=1250, esc=1600)
    readings = [(CDC0, board(0, 1100, 1, servo=1650), 3), (CDC0, board(25, 1900, 1), 26),
                (CDC0, board(75, 1900, 0), 76), (CDC0, active, 102), (CDC0, revoked, 112),
                (CDC0, revoked, 170), (CDC0, revoked, 310), (CDC0, board(400, 1500, 3, faults=1), 405)]
    assert rows(readings, 'board', 't,until_ns,ch3,control_source,servo_us,esc_us,pwm_enabled') == [
        (0, 25*MS, 'propo', 'propo', 1650, 1500, 1),
        (25*MS, 75*MS, 'jetson', 'propo', 1500, 1500, 1),
        (75*MS, 100*MS, 'jetson', 'standby', 1500, 1500, 1),
        (100*MS, 112*MS, 'jetson', 'dualsense', 1250, 1600, 1),
        (112*MS, 250*MS, 'jetson', 'host', 1250, 1600, 1),
        (400*MS, 550*MS, 'middle', 'fault', None, None, 0)]


def test_joy_independent_of_board_preserves_signed_values_zero_and_gaps():
    readings = [(JOY, joy(12, -.5, .25), 14), (CDC0, board(13, 1000, 1), 15),
                (JOY, joy(12, -.5, .25), 26), (JOY, joy(12, valid=False), 30),
                (JOY, joy(12, -.5, .25), 32), (JOY, joy(35), 39), (JOY, joy(35), 280),
                (JOY, joy(700), 299), (JOY, joy(505, 1., -1.), 510)]
    assert rows(readings, 'joy', 't,until_ns,steering,throttle') == [
        (12*MS, 30*MS, -.5, .25), (35*MS, 285*MS, 0., 0.), (505*MS, 755*MS, 1., -1.)]


def test_prebag_input_and_last_instant():
    readings = [(JOY, joy(-10, 0., .5), 3), (JOY, joy(20, 0., .25), 20)]
    assert rows(readings, 'joy', 't,until_ns,throttle', 20) == [(3*MS, 20*MS, .5), (20*MS, 20*MS+1, .25)]


def test_fallback_uses_matching_serial_and_reports_raw_only():
    readings = [(CDC1, telemetry(0, path=1), 1), (CDC0, board(10, 2000, 2), 11),
                (CDC1, telemetry(130), 131), (CDC1, telemetry(210, 'another'), 211),
                (CDC1, telemetry(450, valid=False), 451)]
    assert rows(readings, 'board', 't,until_ns,source,control_source,ch3,ch3_raw,esc_us') == [
        (0, 10*MS, 'cdc1', 'propo', 'unknown', 1800, None),
        (10*MS, 160*MS, 'cdc0', 'host', 'jetson', None, 1500),
        (160*MS, 210*MS, 'cdc1', 'host', 'unknown', 1800, None),
        (450*MS, 750*MS, 'cdc1', 'host', 'unknown', None, None)]


def test_old_cdc1_does_not_reappear_after_newer_cdc0_or_gate_revocation():
    active = board(20, 1900, 2, driver=True)
    revoked = board(20, 1900, 2, driver=False)
    readings = [(CDC1, telemetry(0, path=1), 2), (CDC0, active, 22),
                (CDC1, telemetry(30, path=1), 31), (CDC0, revoked, 40),
                (CDC1, telemetry(210, path=1), 211)]
    assert rows(readings, 'board', 't,until_ns,source,control_source') == [
        (0, 20*MS, 'cdc1', 'propo'), (20*MS, 40*MS, 'cdc0', 'dualsense'),
        (40*MS, 170*MS, 'cdc0', 'host'), (210*MS, 510*MS, 'cdc1', 'propo')]
    # 解除が届いていない場合も、CDC0 途絶後に古いプロポ状態を復元しない。
    assert rows(readings[:2], 'board', 't,until_ns,source,control_source') == [
        (0, 20*MS, 'cdc1', 'propo'), (20*MS, 170*MS, 'cdc0', 'dualsense')]


def write_metadata(path, topics, files=('part.mcap',)):
    meta = dict(storage_identifier='mcap', starting_time=dict(nanoseconds_since_epoch=START),
                duration=dict(nanoseconds=500 * MS), relative_file_paths=list(files),
                topics_with_message_count=[dict(topic_metadata=dict(name=t, type=kind, serialization_format='cdr'),
                                                message_count=n) for t, kind, n in topics])
    (path / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': meta}))


def test_filtered_reader_and_partial_bag(tmp_path, monkeypatch):
    type_joy = 'atd_interfaces/msg/DriveCommand'
    write_metadata(tmp_path, [(JOY, type_joy, 3), (CDC0, 'old/type', 2), (CDC1, 'atd_interfaces/msg/PicoTelemetry', 2),
                              ('/image', 'sensor_msgs/msg/Image', 400)])
    (tmp_path/'part.mcap').touch()
    messages = iter([(JOY, joy(5, .5, .3), START+10*MS), (JOY, 'corrupt', START+35*MS),
                     (JOY, joy(50, 1., .1), START+51*MS)])

    class Reader:
        def open(self, storage, converter):
            assert storage.storage_id == 'mcap' and storage.uri == str(tmp_path)
        def get_all_topics_and_types(self):
            return [N(name=JOY, type=type_joy, serialization_format='cdr'),
                    N(name=CDC1, type='other/msg/Telemetry', serialization_format='cdr')]
        def set_filter(self, spec):
            assert spec.topics == [JOY]
        def has_next(self):
            self.msg = next(messages, None)
            return self.msg is not None
        def read_next(self):
            return self.msg

    def deserialize(value, kind):
        if value == 'corrupt':
            raise RuntimeError('bad input')
        return value

    monkeypatch.setitem(sys.modules, 'rosbag2_py', N(SequentialReader=Reader, StorageOptions=N,
                        ConverterOptions=lambda *_: None, StorageFilter=N))
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', N(deserialize_message=deserialize))
    monkeypatch.setitem(sys.modules, 'atd_interfaces.msg', N(DriveCommand=object))
    path = tmp_path/'out.sqlite'
    extract(tmp_path, path)
    with closing(sqlite3.connect(path)) as db:
        assert db.execute('SELECT * FROM bag').fetchone() == (START, 500*MS, 0)
        assert db.execute('SELECT t,until_ns FROM joy ORDER BY t').fetchall() == [(5*MS, 35*MS), (50*MS, 300*MS)]
        assert db.execute('SELECT * FROM board').fetchall() == []
        reasons = dict(db.execute('SELECT topic,reason FROM topics'))
        assert '復号' in reasons[JOY] and '型' in reasons[CDC0] and 'ストレージ' in reasons[CDC1]


def test_blocks_split_path_outside_recording(tmp_path):
    bag = tmp_path/'bag'
    bag.mkdir()
    outside = tmp_path/'other.mcap'
    outside.touch()
    (bag/'part.mcap').symlink_to(outside)
    write_metadata(bag, [])
    with pytest.raises(ValueError, match='実ファイル'):
        extract(bag, tmp_path/'out')
    assert not (tmp_path/'out').exists()


def status(ms, speed, state='tracking', epoch=1):
    return N(header=header(ms, 'map'), map_id='lap', map_sha256='a'*64, hdmap_sha256='b'*64,
             session_id='run1', epoch=epoch, state=state, reason='', speed_mps=speed, yaw_rate_rps=0.,
             pose=N(position=N(x=0., y=0., z=0.), orientation=N(x=0., y=0., z=0., w=1.)))


def accel(ms, frame, vector=(0., -1., 9.8), available=True):
    return N(header=header(ms, frame), linear_acceleration=N(x=vector[0], y=vector[1], z=vector[2]),
             linear_acceleration_covariance=([0.] if available else [-1.]) + [0.] * 8)


def tf(child, parent, theta=0.):
    import math
    return N(header=N(frame_id=parent), child_frame_id=child, transform=N(
        translation=N(x=.1, y=.01, z=0.), rotation=N(x=0., y=0., z=math.sin(theta/2), w=math.cos(theta/2))))


def test_tracking_signed_speed_breaks_at_loss_epoch_and_gap():
    events = [(LOCALIZATION, status(t, v, state, epoch), r) for t, v, state, epoch, r in (
        (0, 1., 'tracking', 1, 10), (100, .4, 'tracking', 1, 105), (100, .4, 'tracking', 1, 110),
        (105, 0., 'lost', 2, 115), (200, 0., 'tracking', 2, 201), (250, 0., 'tracking', 2, 251),
        (300, -.4, 'tracking', 2, 310), (600, -.3, 'tracking', 2, 609), (610, 0., 'stale', 3, 611))]
    expected = [(0,100*MS,1.,1), (100*MS,100*MS+1,.4,1), (200*MS,250*MS,0.,2),
                (250*MS,300*MS,0.,2), (300*MS,300*MS+1,-.4,2), (600*MS,600*MS+1,-.3,3)]
    assert rows(events, 'speed', '*') == expected
    bad = status(620, 0.)
    bad.header.frame_id = 'camera'
    assert rows([(LOCALIZATION, bad, 621)], 'speed', '*') == []


def test_imu_uses_late_static_tf_signed_specific_force_and_source_time():
    import math
    frames = N(transforms=[tf('mount','base_link'), tf('optical','mount',math.pi/2)])
    samples = [(ACCEL, accel(20, 'optical'), 28), (IMU, accel(21,'base_link',(8.,0.,0.)), 29),
               (ACCEL, accel(25,'optical', (0.,2.,9.8)), 31), (ACCEL, accel(30,'optical',available=False), 35),
               (ACCEL, accel(80,'optical',(0.,0.,9.8)), 85), (TF,frames,100)]
    result = rows(samples, 'imu', '*')
    assert [(a,b) for a,b,_ in result] == [(20*MS,25*MS), (25*MS,25*MS+1), (80*MS,80*MS+1)]
    assert [v for _,_,v in result] == pytest.approx([1.,-2.,0.])
    # 不明・矛盾した取付姿勢を identity で補わない。ほかの入力は利用できる。
    assert rows([samples[0]], 'imu', '*') == []
    assert rows(samples[:1] + [(TF, frames, 90), (TF, N(transforms=[tf('optical','mount')]), 100)],
                'imu', '*') == []
    assert rows(samples[:2], 'imu', 'accel_mps2') == [(8.,)]


def test_manual_ch2_only_with_fresh_propo_path_and_matching_board():
    a = telemetry(10, path=1)
    a.power_ready, a.sbus_age_ms, a.channels[1] = True, 10, 1080
    events = [(CDC1,a,11), (CDC0,board(20,1000,1),21), (CDC0,board(30,2000,2),31)]
    assert rows(events + [(CDC0, board(35,1000,1), 36)], 'propo', '*') == [(10*MS,30*MS,1080)]
    assert rows(events[:1], 'propo', '*') == [(10*MS,50*MS,1080)]
    a.serial = 'different-pico'
    assert rows(events, 'propo', '*') == [(10*MS,20*MS,1080)]
    a.sbus_flags = 8
    assert rows(events, 'propo', '*') == []


def test_stereo_option_uses_metadata_without_deserializing_images(tmp_path):
    (tmp_path / 'part.mcap').touch()
    topics = [(t, kind, 2) for t,kind in STEREO_TYPES.items()]
    write_metadata(tmp_path, topics)
    _, _, _, selected, ready = inspect(tmp_path)
    assert ready and set(selected) == {TF}
    topic, kind, _ = topics[0]
    for entry in ((topic,kind,0), (topic,'other/msg/Image',2)):
        write_metadata(tmp_path, [entry, *topics[1:]])
        assert not inspect(tmp_path)[-1]


def test_gpu_option_requires_both_recorded_optical_mounts_even_when_metadata_is_complete():
    mount = tf('camera_link', 'base_link')
    left, right = (tf(f'camera_infra{i}_optical_frame', 'camera_link') for i in (1, 2))
    for frames, ready in (([], 0), ([left, right], 0), ([mount, left], 0), ([mount, left, right], 1),
                          ([mount, left, right, tf(right.child_frame_id, 'other_mount')], 0)):
        with closing(sqlite3.connect(':memory:')) as db:
            # The IMU and Joy remain readable even if the stereo transforms are incomplete.
            topics = {TF:(1,''), ACCEL:(1,''), JOY:(1,'')}
            readings = [(TF,N(transforms=frames),START), (JOY,joy(2),START+3*MS),
                        (ACCEL,accel(4, 'base_link'),START+5*MS)]
            build_index(db, START, 10*MS, topics, readings, stereo_ready=True)
            assert db.execute('SELECT stereo_ready FROM bag').fetchone() == (ready,)
            assert db.execute('SELECT count(*) FROM imu').fetchone() == (1,)
            assert db.execute('SELECT count(*) FROM joy').fetchone() == (1,)


def test_ros_mcap_roundtrip(tmp_path):
    rosbag = pytest.importorskip('rosbag2_py')
    from atd_interfaces.msg import DriveCommand, JoyDriveStatus, LocalizationStatus
    from geometry_msgs.msg import TransformStamped
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu
    from tf2_msgs.msg import TFMessage

    bag, output = tmp_path/'bag', tmp_path/'index.sqlite'
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag.ConverterOptions('', ''))
    for i, topic in enumerate((JOY, CDC0, TF, ACCEL, LOCALIZATION)):
        writer.create_topic(rosbag.TopicMetadata(id=i, name=topic, type=ROS_TYPES[topic], serialization_format='cdr'))
    j = DriveCommand(source='joy', valid=True, steering=-.5, throttle=.25)
    s = JoyDriveStatus(has_report=True, serial='pico', session_id='session', sequence_number=0,
                       rx_mode_us=1100, selector=0, active_path=1, servo_us=1400, esc_us=1500)
    v = LocalizationStatus(map_id='track', map_sha256='a'*64, hdmap_sha256='b'*64, state='tracking',
                           session_id='run', epoch=1, speed_mps=-.5)
    v.pose.orientation.w = 1.
    a = Imu()
    a.linear_acceleration.x = -1.25
    for msg, topic, frame, offset in [(j, JOY, 'atd_gamepad', 0), (s, CDC0, 'pico', 10),
                                       (a, ACCEL, 'camera_accel', 15), (v, LOCALIZATION, 'map', 17)]:
        msg.header.frame_id = frame
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(START+offset*MS, NANO)
        writer.write(topic, serialize_message(msg), START+offset*MS)
    mount = TransformStamped(child_frame_id='camera_accel')
    mount.header.frame_id = 'base_link'
    mount.transform.rotation.w = 1.
    writer.write(TF, serialize_message(TFMessage(transforms=[mount])), START + 20*MS)
    del writer
    extract(bag, output)
    with closing(sqlite3.connect(output)) as db:
        assert db.execute('SELECT t,steering,throttle FROM joy').fetchall() == [(0, -.5, .25)]
        assert db.execute('SELECT t,control_source,servo_us,esc_us FROM board').fetchall() == [(10*MS, 'propo', 1400, 1500)]
        assert db.execute('SELECT t,accel_mps2 FROM imu').fetchall() == [(15*MS, -1.25)]
        assert db.execute('SELECT t,speed_mps FROM speed').fetchall() == [(17*MS, -.5)]

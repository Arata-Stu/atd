"""Pico の基板経路・CH2 と DualSense の操作要求を元時刻付きで抽出する。"""
import argparse
from contextlib import closing
from pathlib import Path
import sqlite3

from .manual_inputs import JoyStream, pico_reading


TOPIC = '/vehicle/pico/telemetry'
TYPE = 'atd_interfaces/msg/PicoTelemetry'
JOY_TOPIC = '/teleop/command'
JOY_TYPE = 'atd_interfaces/msg/DriveCommand'


class PicoStream:
    def __init__(self):
        self.previous = self.session = None

    def event(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if stamp <= 0 or (self.previous is not None and stamp < self.previous):
            raise ValueError('Pico の観測時刻が不正または逆行しています')
        session = (msg.instance_id, msg.serial)
        if self.previous == stamp and self.session == session:
            path, raw, reason = 'unknown', None, 'Pico の元時刻が重複しています'
        else:
            path, raw, reason = pico_reading(msg)
        self.previous, self.session = stamp, session
        return stamp, path, raw, reason


def readings(messages):
    stream = PicoStream()
    for msg in messages:
        stamp, _, raw, reason = stream.event(msg)
        yield stamp, raw, reason


def extract(bag, output):
    from atd_mapping.bag_inputs import load_metadata
    from atd_interfaces.msg import DriveCommand, PicoTelemetry
    from rclpy.serialization import deserialize_message
    import rosbag2_py

    bag = bag.resolve(strict=True)
    meta = load_metadata(bag)
    files = meta.get('relative_file_paths', [])
    if not isinstance(files, list) or not files or any(
            not isinstance(f, str) or not f or Path(f).is_absolute() or '..' in Path(f).parts
            or not (bag / f).is_file() or not (bag / f).resolve(strict=True).is_relative_to(bag) for f in files):
        raise ValueError('記録ディレクトリ内の bag 分割ファイルを確認してください')
    expected = {TOPIC: TYPE, JOY_TOPIC: JOY_TYPE}
    counts = {}
    for entry in meta['topics_with_message_count']:
        info = entry['topic_metadata']
        topic = info['name']
        if topic in expected:
            if topic in counts or info['type'] != expected[topic]:
                raise ValueError(f'{topic} の型または記録情報が不正です')
            counts[topic] = entry['message_count']
    with closing(sqlite3.connect(output)) as db:
        db.execute('CREATE TABLE propo (stamp INTEGER, path TEXT, raw INTEGER, reason TEXT NOT NULL)')
        db.execute('CREATE TABLE joy (effective INTEGER, stamp INTEGER, throttle REAL, enabled INTEGER, reason TEXT NOT NULL)')
        db.execute('CREATE TABLE topics (topic TEXT PRIMARY KEY, messages INTEGER NOT NULL)')
        db.executemany('INSERT INTO topics VALUES (?,?)', counts.items())
        if any(counts.values()):
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=meta['storage_identifier']),
                        rosbag2_py.ConverterOptions('', ''))
            reader.set_filter(rosbag2_py.StorageFilter(topics=list(counts)))
            received = dict.fromkeys(counts, 0)
            pico, joy = PicoStream(), JoyStream()
            while reader.has_next():
                topic, raw, receipt = reader.read_next()
                if topic not in received:
                    raise ValueError('手動入力の topic filter が一致しません')
                received[topic] += 1
                if topic == TOPIC:
                    db.execute('INSERT INTO propo VALUES (?,?,?,?)', pico.event(deserialize_message(raw, PicoTelemetry)))
                else:
                    event = joy.event(deserialize_message(raw, DriveCommand), receipt)
                    if event is not None:
                        db.execute('INSERT INTO joy VALUES (?,?,?,?,?)', event)
            if received != counts:
                raise ValueError('手動入力の記録件数が metadata と一致しません')
        db.execute('CREATE INDEX joy_time ON joy(effective)')
        db.commit()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('新規出力先を指定してください')
    extract(args.bag, args.output)

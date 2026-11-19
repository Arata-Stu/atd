"""手動走行の rosbag を読み、元の観測時刻と有効期限を持つ区間に変換する。"""
import argparse
from contextlib import closing
from importlib import import_module
from numbers import Integral
from pathlib import Path
import sqlite3
from tempfile import TemporaryDirectory

from .drive_monitor import CDC0_TIMEOUT_S, CDC1_DEFAULT_TIMEOUT_S, JOY_TIMEOUT_S, feedback_sample, joy_sample, _path
from .bag_motion import BagMotion, MOTION_TYPES, STEREO_TYPES


JOY = '/teleop/command'
CDC0 = '/vehicle/joy_drive/status'
CDC1 = '/vehicle/pico/telemetry'
TYPES = {JOY: 'DriveCommand', CDC0: 'JoyDriveStatus', CDC1: 'PicoTelemetry'}
ROS_TYPES = {**{topic: 'atd_interfaces/msg/' + name for topic, name in TYPES.items()}, **MOTION_TYPES}
NANO = 1_000_000_000
TTL = {JOY: round(NANO * JOY_TIMEOUT_S), CDC0: round(NANO * CDC0_TIMEOUT_S),
       CDC1: round(NANO * CDC1_DEFAULT_TIMEOUT_S)}
BOARD_FIELDS = ('ch3', 'control_source', 'rx_mode_us', 'ch3_raw', 'servo_us', 'esc_us', 'source', 'pwm_enabled')


def inspect(bag):
    import yaml

    metadata = bag / 'metadata.yaml'
    if metadata.is_symlink():
        raise ValueError('metadata.yaml の symlink は読み込めません')
    doc = yaml.safe_load(metadata.read_text())
    meta = doc.get('rosbag2_bagfile_information') if isinstance(doc, dict) else None
    if not isinstance(meta, dict):
        raise ValueError('rosbag の metadata.yaml が不正です')
    try:
        start = meta['starting_time']['nanoseconds_since_epoch']
        duration = meta['duration']['nanoseconds']
        storage = meta['storage_identifier']
        files = meta['relative_file_paths']
        entries = meta['topics_with_message_count']
    except (KeyError, TypeError) as exc:
        raise ValueError('bag の開始時刻・長さ・分割ファイル・topic 情報を確認してください') from exc
    if (type(start) is not int or start <= 0 or type(duration) is not int or duration < 0
            or start + duration >= 2**63 - 1 or not isinstance(entries, list)
            or not isinstance(storage, str) or not storage or not isinstance(files, list) or not files):
        raise ValueError('bag の時刻または形式情報が不正です')
    for name in files:
        if not isinstance(name, str) or not name or Path(name).is_absolute() or '..' in Path(name).parts:
            raise ValueError('bag の分割ファイル指定が不正です')
        path = bag / name
        if path != path.resolve(strict=True) or not path.is_file():
            raise ValueError('bag 内の実ファイルを指定してください')
    topics, stereo = {}, set()
    for entry in entries:
        info = entry.get('topic_metadata') if isinstance(entry, dict) else None
        if not isinstance(info, dict):
            raise ValueError('bag の topic 情報が不正です')
        name = info.get('name')
        count = entry.get('message_count')
        if (name in STEREO_TYPES and info.get('type') == STEREO_TYPES[name] and type(count) is int and count > 0
                and info.get('serialization_format', 'cdr') == 'cdr'):
            stereo.add(name)
        if name not in ROS_TYPES:
            continue
        if name in topics or type(count) is not int or count < 0:
            raise ValueError(f'{name} の記録件数または重複を確認してください')
        reason = ''
        if info.get('type') != ROS_TYPES[name]:
            reason = f'{name} の記録型が一致しません'
        elif info.get('serialization_format', 'cdr') != 'cdr':
            reason = f'{name} のシリアライズ形式に対応していません'
        topics[name] = (count, reason)
    return start, duration, storage, topics, stereo == set(STEREO_TYPES)


def decode(topic, msg):
    """値、同一フレームの識別に用いる情報、CDC の個体識別子を返す。"""
    if topic == JOY:
        value = joy_sample(msg, {})
        return None if value['reason'] else (msg.steering, msg.throttle), (), ''
    serial = msg.serial.casefold()
    if topic == CDC0:
        value = feedback_sample(msg, {})
        if value['reason']:
            return None, (), serial
        # 同じ S で変わり得るのは Joy 送信側の状態。物理値と連番は固定。
        identity = (msg.session_id, serial, msg.sequence_number, msg.selector, msg.active_path,
                    msg.fault_bits, msg.rx_mode_us, msg.servo_us, msg.esc_us)
        return tuple(value[k] for k in BOARD_FIELDS), identity, serial

    if (msg.header.frame_id != 'pico' or not serial or not msg.instance_id
            or msg.active_path not in (0, 1, 2, 3) or len(msg.channels) != 16):
        return None, (), serial
    raw = int(msg.channels[2])
    rc_valid = (msg.rc_valid and msg.sbus_fresh and not msg.sbus_flags & 12 and 0 <= raw <= 2047)
    value = ('unknown', _path(msg.active_path, msg.fault_bits), None, raw if rc_valid else None,
             None, None, 'cdc1', None)
    return value, (serial, msg.instance_id, msg.sequence_number), serial


class Stream:
    """再配信で期限を延長せず、同じ S に対するゲート変更は通知時点で反映する。"""
    def __init__(self, topic, start):
        self.topic = topic
        self.start = start
        self.stamp = self.blocked = self.effective = 0
        self.value = self.identity = None
        self.serial = ''

    def event(self, msg, receipt):
        value, identity, serial = decode(self.topic, msg) if msg is not None else (None, (), '')
        if serial:
            self.serial = serial
        stamp = msg.header.stamp if msg is not None else None
        source = 0 if stamp is None or not 0 <= stamp.nanosec < NANO else stamp.sec * NANO + stamp.nanosec
        deadline = source + TTL[self.topic]
        if value is not None and 0 < source <= receipt < deadline and source > self.blocked:
            if source == self.stamp and identity == self.identity:
                if value == self.value:
                    return None
                if self.topic == CDC0:
                    # PWM とその失効時刻は同じまま、操作元だけを変更する。
                    self.value, self.effective = value, receipt
                    return receipt, deadline, self.serial, value
            if source > self.stamp:
                # 記録開始直前のサンプル、遅延した切替通知との順序競合は受信時から使う。
                effective = source if source >= max(self.start, self.effective) else receipt
                self.stamp, self.effective = source, effective
                self.value, self.identity = value, identity
                return effective, deadline, self.serial, value
        # 無効な要求・切断は元時刻が 0 または古い場合がある。
        self.blocked = self.effective = receipt
        self.value, self.identity = None, None
        return receipt, receipt, self.serial, None


class Intervals:
    """同じ観測値が連続した場合はまとめ、空白や切替点は残す。"""
    def __init__(self, db, table):
        self.db, self.table, self.pending = db, table, None

    def add(self, start, end, values):
        if end <= start or values is None:
            return
        if self.pending and self.pending[1] == start and self.pending[2] == values:
            self.pending = (self.pending[0], end, values)
            return
        self.flush()
        self.pending = (start, end, values)

    def flush(self):
        if self.pending is not None:
            start, end, values = self.pending
            row = (start, end, *values)
            self.db.execute(f'INSERT INTO {self.table} VALUES ({",".join("?" for _ in row)})', row)
            self.pending = None


def write_boards(rows, output, propo, end):
    # 新鮮な CDC0 はプロポ走行中・切替確認中も優先する。
    # CDC1 には経路と CH3 raw のみがあり、別個体の値は引き継がない。
    active = {}
    previous = 0
    last_feedback = None
    rc_raw = None
    rc_expiry = 0

    def emit(until):
        expiry_times = [rc_expiry, *(entry[0] for entry in active.values())]
        edges = sorted({previous, until, *(expiry for expiry in expiry_times if previous < expiry < until)})
        for t, limit in zip(edges, edges[1:]):
            c0, c1 = active.get(CDC0), active.get(CDC1)
            current = None
            if c0 and t < c0[0]:
                current = c0
            elif (c1 and t < c1[0] and (not c0 or not c0[1] or c0[1] == c1[1])
                  and (last_feedback is None or c1[0] - TTL[CDC1] >= last_feedback)):
                current = c1
            if current:
                output.add(t, limit, current[2])
                if (rc_raw is not None and t < rc_expiry and current[2][1] == 'propo'
                        and current[1] == c1[1]):
                    propo.add(t, limit, (rc_raw,))

    for t, topic, expiry, serial, *values in rows:
        emit(t)
        *fields, raw, raw_expiry = values
        active[topic] = (expiry, serial, tuple(fields))
        if topic == CDC1:
            rc_raw, rc_expiry = raw, raw_expiry
        if topic == CDC0 and expiry > t:
            # CDC0 の報告・Joy 解除より前の CDC1 を、その後の操作元として復活させない。
            last_feedback = t
            if fields[1] != 'propo':
                rc_raw = None
        previous = t
    emit(end)
    output.flush()
    propo.flush()


def build_index(db, start, duration, topics, messages, stereo_ready=False):
    """messages は (topic, デコード済み msg または復号失敗時 None, bag 受信 ns)。"""
    db.executescript('''
        CREATE TABLE bag(start_ns INTEGER NOT NULL, duration_ns INTEGER NOT NULL, stereo_ready INTEGER NOT NULL DEFAULT 0);
        CREATE TABLE topics(topic TEXT PRIMARY KEY, messages INTEGER NOT NULL, reason TEXT NOT NULL);
        CREATE TABLE joy(t INTEGER PRIMARY KEY, until_ns INTEGER NOT NULL, steering REAL NOT NULL, throttle REAL NOT NULL);
        CREATE TABLE propo(t INTEGER PRIMARY KEY, until_ns INTEGER NOT NULL, throttle_raw INTEGER NOT NULL);
        CREATE TABLE board(t INTEGER PRIMARY KEY, until_ns INTEGER NOT NULL, ch3 TEXT, control_source TEXT,
                           rx_mode_us INTEGER, ch3_raw INTEGER, servo_us INTEGER, esc_us INTEGER, source TEXT,
                           pwm_enabled INTEGER);
        CREATE TEMP TABLE inputs(t INTEGER PRIMARY KEY, until_ns INTEGER, steering REAL, throttle REAL);
        CREATE TEMP TABLE signals(t INTEGER, topic TEXT, until_ns INTEGER, serial TEXT, ch3 TEXT,
                           control_source TEXT, rx_mode_us INTEGER, ch3_raw INTEGER, servo_us INTEGER,
                           esc_us INTEGER, source TEXT, pwm_enabled INTEGER, raw INTEGER, raw_expiry INTEGER,
                           PRIMARY KEY (t,topic)) WITHOUT ROWID;
    ''')
    streams = {topic: Stream(topic, start) for topic in topics if topic in TYPES}
    seen, errors = dict.fromkeys(topics, 0), dict.fromkeys(topics, 0)
    end = duration + 1
    motion = BagMotion(db, start, end)
    last_receipt = {}
    for topic, msg, receipt in messages:
        seen[topic] += 1
        if msg is None:
            errors[topic] += 1
        if type(receipt) is not int or not start <= receipt <= start + duration or receipt < last_receipt.get(topic, 0):
            raise ValueError(f'{topic} の記録時刻が区間外か逆行しています')
        last_receipt[topic] = receipt
        if topic in MOTION_TYPES:
            motion.read(topic, msg, receipt)
            continue
        event = streams[topic].event(msg, receipt)
        if event is None:
            continue
        effective, deadline, serial, value = event
        t, expiry = effective - start, deadline - start
        if topic == JOY:
            db.execute('INSERT OR REPLACE INTO inputs VALUES (?,?,?,?)', (t, expiry, *(value or (None, None))))
        else:
            raw, raw_expiry = None, 0
            if topic == CDC1 and value and value[1] == 'propo' and value[3] is not None:
                age = getattr(msg, 'sbus_age_ms', None)
                ch2 = msg.channels[1]
                if (getattr(msg, 'power_ready', False) and type(age) is int and 0 <= age < 50
                        and isinstance(ch2, Integral) and 0 <= ch2 <= 2047):
                    raw = int(ch2)
                    raw_expiry = expiry - TTL[CDC1] + (50 - age) * 1_000_000
            db.execute('INSERT OR REPLACE INTO signals VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
                       (t, topic, expiry, serial, *(value or (None,) * len(BOARD_FIELDS)), raw, raw_expiry))
    motion.finish(topics)
    db.execute('INSERT INTO bag VALUES (?,?,?)', (start, duration, int(stereo_ready and motion.has_stereo_transforms())))
    for topic, (count, reason) in topics.items():
        if not reason and seen[topic] != count:
            raise ValueError(f'{topic} の読取件数が metadata と一致しません')
        if errors[topic]:
            reason = ' '.join(filter(None, (reason, f'{errors[topic]} 件を復号できませんでした')))
        db.execute('INSERT INTO topics VALUES (?,?,?)', (topic, count, reason))
    joy = Intervals(db, 'joy')
    rows = db.execute('SELECT t,until_ns,steering,throttle,lead(t) OVER (ORDER BY t) FROM inputs ORDER BY t')
    for t, expiry, steering, throttle, following in rows:
        if steering is not None:
            joy.add(t, min(expiry, following if following is not None else end), (steering, throttle))
    joy.flush()
    write_boards(db.execute('SELECT * FROM signals ORDER BY t,topic'), Intervals(db, 'board'), Intervals(db, 'propo'), end)
    db.execute('CREATE UNIQUE INDEX joy_end ON joy(until_ns)')
    db.execute('CREATE UNIQUE INDEX board_end ON board(until_ns)')
    db.commit()


def read_messages(bag, storage, topics):
    selected = [topic for topic, (count, reason) in topics.items() if count and not reason]
    if not selected:
        return
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage), rosbag2_py.ConverterOptions('', ''))
    stored = {info.name: info for info in reader.get_all_topics_and_types()}
    classes = {}
    for topic in selected:
        info = stored.get(topic)
        if not info or info.type != ROS_TYPES[topic] or info.serialization_format != 'cdr':
            topics[topic] = (topics[topic][0], f'{topic} のストレージ内の型または形式が一致しません')
        else:
            package, _, name = ROS_TYPES[topic].split('/')
            classes[topic] = getattr(import_module(package + '.msg'), name)
    if not classes:
        return
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(classes)))
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        if topic not in classes:
            raise ValueError('対象外の topic が読み出されました')
        try:
            msg = deserialize_message(data, classes[topic])
        except (RuntimeError, ValueError, TypeError):
            msg = None
        yield topic, msg, receipt


def extract(bag, output):
    bag = bag.resolve(strict=True)
    start, duration, storage, topics, stereo_ready = inspect(bag)
    # 同時要求・中断時も不完全な成果物を公開しない。
    with TemporaryDirectory(dir=output.parent) as temp:
        path = Path(temp) / 'timeline.sqlite'
        with closing(sqlite3.connect(path)) as db:
            build_index(db, start, duration, topics, read_messages(bag, storage, topics), stereo_ready)
        output.hardlink_to(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('新規の出力先を指定してください')
    extract(args.bag, args.output)

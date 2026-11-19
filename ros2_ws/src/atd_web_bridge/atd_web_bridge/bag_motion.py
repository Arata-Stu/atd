"""bag の元時刻で、追跡中の車体速度とセンサ位置の車体前後比力を抽出する。"""
import math

from .localization import observation


LOCALIZATION = '/localization/status'
ACCEL = '/camera/camera/accel/sample'
IMU = '/camera/camera/imu'
TF = '/tf_static'
NANO = 1_000_000_000
MOTION_TYPES = {LOCALIZATION: 'atd_interfaces/msg/LocalizationStatus',
                ACCEL: 'sensor_msgs/msg/Imu', IMU: 'sensor_msgs/msg/Imu', TF: 'tf2_msgs/msg/TFMessage'}
STEREO_TYPES = {f'/camera/camera/infra{n}/{suffix}': kind for n in (1, 2) for suffix, kind in (
    ('image_rect_raw', 'sensor_msgs/msg/Image'), ('camera_info', 'sensor_msgs/msg/CameraInfo'))}
STEREO_TYPES[TF] = MOTION_TYPES[TF]


def relative_stamp(msg, start, end, receipt):
    if msg is not None:
        stamp = msg.header.stamp
        source = stamp.sec * NANO + stamp.nanosec
        if 0 <= stamp.nanosec < NANO and start <= source < start + end and source <= receipt + 50_000_000:
            return source - start
    return None


class Measurements:
    """同じ区間の次の実観測まで表示する。単発値・末尾はその元時刻だけに残す。"""
    def __init__(self, db, table, max_gap):
        self.db = db
        self.table = table
        self.max_gap = max_gap
        self.previous = None
        self.high = -1
        self.segment = 0

    def flush(self, stop=None):
        if self.previous is not None:
            t, _, value = self.previous
            row = (t, stop if stop is not None else t + 1, value)
            if self.table == 'speed':
                self.db.execute('INSERT INTO speed VALUES (?,?,?,?)', (*row, self.segment))
            else:
                self.db.execute('INSERT INTO imu VALUES (?,?,?)', row)
            self.previous = None

    def sample(self, t, identity, value):
        if t is None or value is None or not math.isfinite(value):
            self.flush()
            if t is not None:
                self.high = max(self.high, t)
            return
        if t == self.high and self.previous == (t, identity, value):
            return  # 既出の追跡状態の再配信。
        if t <= self.high:
            self.flush()
            return
        same = self.previous and self.previous[1] == identity and t - self.previous[0] <= self.max_gap
        self.flush(t if same else None)
        if not same:
            self.segment += 1
        self.previous = (t, identity, value)
        self.high = t


class BagMotion:
    def __init__(self, db, start, end):
        self.db, self.start, self.end = db, start, end
        self.speed = Measurements(db, 'speed', 250_000_000)  # 10 Hz の観測。
        self.graph = None
        self.bad_children = set()
        self.tf_unreadable = False
        db.executescript('''
            CREATE TABLE speed(t INTEGER PRIMARY KEY, until_ns INTEGER, speed_mps REAL, segment INTEGER);
            CREATE TABLE imu(t INTEGER PRIMARY KEY, until_ns INTEGER, accel_mps2 REAL);
            CREATE TEMP TABLE accel_samples(topic TEXT,t INTEGER,frame TEXT,x REAL,y REAL,z REAL);
        ''')

    def transform(self, msg):
        from atd_vehicle_calibration.inputs import StaticTransformGraph

        if msg is None:
            self.tf_unreadable = True
            return
        if self.graph is None:
            self.graph = StaticTransformGraph()
        for tf in msg.transforms:
            p, q = tf.transform.translation, tf.transform.rotation
            try:
                self.graph.add(dict(parent=tf.header.frame_id, child=tf.child_frame_id,
                                    translation=(p.x, p.y, p.z), rotation=(q.x, q.y, q.z, q.w)))
            except ValueError:
                self.bad_children.add(tf.child_frame_id)

    def read(self, topic, msg, receipt):
        if topic == TF:
            self.transform(msg)
            return
        t = relative_stamp(msg, self.start, self.end, receipt)
        if topic == LOCALIZATION:
            status = observation(msg) if msg is not None else {}
            if status.get('state') != 'tracking' or t is None:
                self.speed.sample(t if t is not None else receipt - self.start, None, None)
            else:
                identity = (msg.session_id, msg.epoch, msg.map_id, msg.map_sha256, msg.hdmap_sha256)
                self.speed.sample(t, identity, msg.speed_mps)
            return
        frame = msg.header.frame_id if msg is not None else ''
        xyz = (None, None, None)
        if t is not None and frame and len(msg.linear_acceleration_covariance) == 9:
            cov, a = msg.linear_acceleration_covariance, msg.linear_acceleration
            if cov[0] >= 0 and all(map(math.isfinite, (*cov, a.x, a.y, a.z))):
                xyz = (a.x, a.y, a.z)
        self.db.execute('INSERT INTO accel_samples VALUES (?,?,?,?,?,?)',
                        (topic, t if t is not None else receipt - self.start, frame, *xyz))

    def rotation(self, frame):
        if frame == 'base_link':
            return [0., 0., 0., 1.]
        if self.tf_unreadable or self.graph is None:
            raise ValueError('記録した /tf_static で IMU の取付姿勢を確認できません')
        rotation = self.graph.transform('base_link', frame)['rotation']
        child = frame
        while child != 'base_link':
            if child in self.bad_children:
                raise ValueError(f'/tf_static の IMU 経路が一意ではありません（{child}）')
            child = self.graph.parents[child][0]
        return rotation

    def has_stereo_transforms(self):
        from atd_vehicle_calibration.inputs import CAMERA_FRAMES

        try:
            for frame in CAMERA_FRAMES:
                self.rotation(frame)
        except ValueError:
            return False
        return True

    def finish(self, topics):
        self.speed.flush()
        rotations = {}
        for topic in (ACCEL, IMU):
            if not topics.get(topic, (0,))[0]:
                continue
            from atd_vehicle_calibration.inputs import rotate

            accel = Measurements(self.db, 'imu', 50_000_000)
            missing_frames = set()
            samples = self.db.execute('SELECT t,frame,x,y,z FROM accel_samples WHERE topic=? ORDER BY rowid', (topic,))
            for t, frame, x, y, z in samples:
                value = None
                if x is not None:
                    if frame not in rotations:
                        try:
                            rotations[frame] = self.rotation(frame)
                        except ValueError:
                            rotations[frame] = None
                    rotation = rotations[frame]
                    if rotation is None:
                        missing_frames.add(frame)
                    else:
                        value = rotate((x, y, z), rotation)[0]
                accel.sample(t, frame, value)
            accel.flush()
            if missing_frames:
                count, reason = topics[topic]
                frames = ', '.join(sorted(missing_frames))
                topics[topic] = (count, ' '.join(filter(None, (reason, f'記録TFで base_link ← {frames} を確定できません'))))
            if self.db.execute('SELECT 1 FROM imu LIMIT 1').fetchone():
                break  # 加速度単独topicを優先し、同じ観測を統合IMUと二重計上しない。

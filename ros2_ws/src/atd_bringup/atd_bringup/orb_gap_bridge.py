"""固定 Atlas の画像観測と車体ジャイロから、地図上の姿勢を計測時刻順に配信する。"""
from collections import Counter, deque
from contextlib import ExitStack
import csv
import json
import math
from statistics import median

from atd_localization.state import RigidPose
from atd_mapping.frames import stamp_ns

from .orb_gap_imu import _add_transform, _body_axis
from .orb_gap_motion import MAX_GYRO_HOLD_NS
from .orb_online_motion import GyroGapMotion
from .replay_inputs import IMU_TOPIC
from .replay_result import GAP_NS

FRAME_STATUS = '/orbslam3/frame_status'
CONTINUOUS_ODOMETRY = '/orbslam3/continuous_odometry'
CONTINUOUS_STATUS = '/orbslam3/continuous_status'
TRAJECTORY_FIELDS = ('stamp_ns', 'source', 'age_s', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
RECOVERY_FIELDS = ('last_observed_ns', 'reacquired_ns', 'gap_s', 'previous_epoch', 'epoch', 'result',
                   'predicted_x_m', 'predicted_y_m', 'predicted_yaw_rad', 'observed_x_m', 'observed_y_m',
                   'observed_yaw_rad', 'position_jump_m', 'abs_yaw_jump_deg', 'previous_output_ns',
                   'previous_output_source', 'display_step_m', 'display_step_abs_yaw_deg', 'display_result')


def _yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def _with_yaw_delta(q, angle):
    # 地図の鉛直軸まわりの回転を左から掛け、最後に観測したロールとピッチを保持する。
    x, y, z, w = q
    angle = math.remainder(angle, math.tau)
    s, c = math.sin(angle / 2), math.cos(angle / 2)
    return (c*x - s*y, c*y + s*x, c*z + s*w, c*w - s*z)


def _errors(x, y, yaw, other_x, other_y, other_yaw):
    return (math.hypot(x - other_x, y - other_y),
            abs(math.degrees(math.remainder(yaw - other_yaw, math.tau))))


class OrbGapBridge:
    def __init__(self, node, directory, map_id, map_sha256, *, imu_topic=IMU_TOPIC):
        from atd_interfaces.msg import LocalizationStatus
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from nav_msgs.msg import Odometry
        from rclpy.clock import JumpThreshold
        from rclpy.duration import Duration
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Imu
        from tf2_msgs.msg import TFMessage

        self.motion = GyroGapMotion()
        self.map_id, self.map_sha256 = map_id, map_sha256
        self.directory, self.imu_topic = directory, imu_topic
        self.resources = ExitStack()
        self.parents = {}
        self.axis = self.imu_frame = None
        self.session = None
        self.retired_sessions = deque(maxlen=8)
        self.pending = deque()
        self.last_frame_ns = self.gyro_ns = self.last_fallback_ns = None
        self.anchor = self.last_output = None
        self.lost = False
        self.counts = Counter()
        self.unavailable = Counter()
        self.jumps = []
        self.summary = None
        self.trajectory = self.recoveries = None
        self._context_ok = node.context.ok
        self.types = Odometry, DiagnosticArray, DiagnosticStatus, KeyValue
        try:
            if directory is not None:
                self.trajectory = self._csv('gyro_trajectory.csv', TRAJECTORY_FIELDS)
                self.recoveries = self._csv('reacquisitions.csv', RECOVERY_FIELDS)
            self.odom = node.create_publisher(Odometry, CONTINUOUS_ODOMETRY, 32)
            self.resources.callback(node.destroy_publisher, self.odom)
            self.status = node.create_publisher(DiagnosticArray, CONTINUOUS_STATUS, 32)
            self.resources.callback(node.destroy_publisher, self.status)
            for typ, topic, cb, qos in (
                (TFMessage, '/tf_static', self._tf, QoSProfile(depth=64, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
                (Imu, imu_topic, self._imu, QoSProfile(depth=512, reliability=ReliabilityPolicy.BEST_EFFORT)),
                (LocalizationStatus, FRAME_STATUS, self._frame, QoSProfile(depth=256)),
            ):
                sub = node.create_subscription(typ, topic, cb, qos)
                self.resources.callback(node.destroy_subscription, sub)
            threshold = JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True)
            handle = node.get_clock().create_jump_callback(threshold, post_callback=lambda _: self._reset_session())
            self.resources.callback(handle.unregister)
        except BaseException:
            self.resources.close()
            raise

    def _csv(self, filename, fields):
        output = self.resources.enter_context((self.directory / filename).open('x', newline=''))
        writer = csv.writer(output)
        writer.writerow(fields)
        return writer

    def _tf(self, msg):
        for tf in msg.transforms:
            _add_transform(self.parents, tf)
        if self.imu_frame:
            self.axis, _ = _body_axis(self.imu_frame, self.parents)

    def _imu(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        frame = msg.header.frame_id.lstrip('/')
        w = msg.angular_velocity
        variance = msg.angular_velocity_covariance[0]
        if not frame or variance < 0 or not all(map(math.isfinite, (variance, w.x, w.y, w.z))):
            self.counts['invalid_imu'] += 1
            return
        if self.imu_frame is not None and frame != self.imu_frame:
            raise ValueError(f'補完中にジャイロ座標系が変わりました: {self.imu_frame} → {frame}')
        if self.axis is None:
            try:
                self.axis, _ = _body_axis(frame, self.parents)
            except ValueError:
                self.counts['imu_without_tf'] += 1
                return
            self.imu_frame = frame
        ax, ay, az = self.axis
        if not self.motion.add_gyro(stamp, ax*w.x + ay*w.y + az*w.z):
            return
        self.counts['gyro_samples'] += 1
        self.gyro_ns = stamp if self.gyro_ns is None else max(stamp, self.gyro_ns)
        self._drain()
        # When stereo delivery stops altogether, keep a 250ms reorder allowance for ORB's image
        # processing. Late visual decisions can still correct the anchor at their own image time.
        target = stamp - GAP_NS
        if (self.anchor and self.last_frame_ns is not None and target > self.last_frame_ns
                and (self.last_fallback_ns is None or target - self.last_fallback_ns >= 10_000_000)):
            self.last_fallback_ns = target
            self._predict_output(target)

    def _frame(self, msg):
        if (msg.map_id != self.map_id or msg.map_sha256 != self.map_sha256 or msg.header.frame_id != 'map'
                or not msg.session_id or msg.session_id in self.retired_sessions):
            return
        if self.session is not None and self.session != msg.session_id:
            self._reset_session()
        self.session = msg.session_id
        stamp = stamp_ns(msg.header.stamp)
        if stamp < 0 or (self.last_frame_ns is not None and stamp <= self.last_frame_ns):
            return
        self.counts['frame_decisions'] += 1
        self.last_frame_ns = stamp
        self.pending.append((stamp, msg))
        self._drain()

    def _reset_session(self):
        if self.session:
            self.retired_sessions.append(self.session)
        self.motion.reset()
        self.pending.clear()
        self.session = self.anchor = self.last_output = None
        self.last_frame_ns = self.gyro_ns = self.last_fallback_ns = None
        self.lost = False

    def _drain(self, *, finish=False):
        while self.pending:
            stamp, msg = self.pending[0]
            # DDS has no ordering between IMU and stereo topics. Wait at most one gyro hold
            # interval in image time; absent IMU must never block subsequent visual observations.
            needs_gyro = self.anchor and (msg.state != 'tracking' or self._reacquiring(stamp, msg.epoch))
            if (not finish and needs_gyro and (self.gyro_ns is None or self.gyro_ns < stamp)
                    and self.last_frame_ns - stamp < MAX_GYRO_HOLD_NS):
                break
            self.pending.popleft()
            if msg.state == 'tracking':
                self._observe(stamp, msg)
            else:
                self.lost = self.anchor is not None
                self._predict_output(stamp)

    def _reacquiring(self, stamp, epoch):
        return self.lost or epoch != self.anchor[1] or stamp - self.anchor[0] > GAP_NS

    def _observe(self, stamp, msg):
        p, q = msg.pose.position, msg.pose.orientation
        observed = RigidPose.checked((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        yaw = _yaw(observed.xyzw)
        if self.anchor and self._reacquiring(stamp, msg.epoch):
            self._record_recovery(stamp, msg.epoch, observed, yaw)
        self.motion.observe(stamp, p.x, p.y, yaw, msg.epoch)
        self.anchor = stamp, msg.epoch, observed, yaw
        self.lost = False
        self._emit(stamp, 'observed', 0., observed, yaw, msg.speed_mps, msg.yaw_rate_rps)

    def _predict_output(self, stamp):
        if self.last_output and stamp <= self.last_output[0]:
            return
        prediction, reason = self.motion.predict(stamp)
        if prediction is None:
            self.unavailable[reason] += 1
            age = (stamp - self.anchor[0]) * 1e-9 if self.anchor else None
            self._status(stamp, 'unavailable', age, reason)
            return
        _, _, previous, yaw = self.anchor
        predicted = RigidPose((prediction.x, prediction.y, previous.xyz[2]),
                              _with_yaw_delta(previous.xyzw, prediction.yaw - yaw))
        self._emit(stamp, 'predicted', prediction.age_s, predicted, prediction.yaw,
                   prediction.speed_mps, prediction.yaw_rate_radps)

    def _record_recovery(self, stamp, epoch, pose, yaw):
        x, y, _ = pose.xyz
        predicted, reason = self.motion.predict(stamp)
        endpoint = delta = (None, None)
        coords = (None, None, None)
        if predicted is not None:
            coords = predicted.x, predicted.y, predicted.yaw
            endpoint = _errors(x, y, yaw, *coords)
            self.counts['measured_jumps'] += 1
            if self.directory is not None:
                self.jumps.append(endpoint)
        self.counts['reacquisitions'] += 1
        prior_stamp = prior_source = None
        display_result = 'late_observation' if self.last_output else 'no_prior_output'
        if self.last_output and self.last_output[0] < stamp:
            prior_stamp, prior_source, prior_pose, prior_yaw = self.last_output
            delta = _errors(x, y, yaw, *prior_pose.xyz[:2], prior_yaw)
            display_result = 'published'
        if self.recoveries:
            start, old_epoch, _, _ = self.anchor
            self.recoveries.writerow((start, stamp, (stamp-start)*1e-9, old_epoch, epoch, reason,
                                      *coords, x, y, yaw, *endpoint, prior_stamp, prior_source, *delta, display_result))

    def _status(self, stamp, source, age, reason):
        _, Array, Status, KeyValue = self.types
        message = Array()
        message.header.frame_id = 'map'
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        detail = Status()
        detail.name = 'orbslam3/continuous_pose'
        detail.level = Status.OK if source == 'observed' else Status.WARN
        detail.message = source
        detail.values = [KeyValue(key='age_s', value=str(age) if age is not None else ''),
                         KeyValue(key='reason', value=reason)]
        message.status = [detail]
        self.status.publish(message)

    def _emit(self, stamp, source, age, pose, yaw, speed, rate):
        if self.last_output and stamp <= self.last_output[0]:
            self.counts['late_observed'] += 1
            return
        message = self.types[0]()
        message.header.frame_id = 'map'
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.child_frame_id = 'base_link'
        p, q = message.pose.pose.position, message.pose.pose.orientation
        p.x, p.y, p.z = pose.xyz
        q.x, q.y, q.z, q.w = pose.xyzw
        message.twist.twist.linear.x, message.twist.twist.angular.z = speed, rate
        self.odom.publish(message)
        self._status(stamp, source, age, '')
        self.last_output = stamp, source, pose, yaw
        self.counts[source] += 1
        if self.trajectory:
            self.trajectory.writerow((stamp, source, age, *pose.xyz, *pose.xyzw))

    def close(self):
        if self.summary is not None:
            return self.summary
        try:
            # A standalone node may receive SIGINT through the rclpy context before spin returns.
            if self._context_ok():
                self._drain(finish=True)
            else:
                self.pending.clear()
        finally:
            self.resources.close()
        self.summary = dict(imu_topic=self.imu_topic, imu_frame=self.imu_frame, max_prediction_s=self.motion.max_gap_ns*1e-9,
                            camera_stall_delay_s=GAP_NS*1e-9, observed=self.counts['observed'],
                            predicted=self.counts['predicted'], counts=dict(self.counts),
                            unavailable=dict(self.unavailable), reacquisitions=self.counts['reacquisitions'],
                            measured_jumps=self.counts['measured_jumps'])
        for index, key in enumerate(('position_jump_m', 'abs_yaw_jump_deg')):
            values = [row[index] for row in self.jumps]
            self.summary[key] = {'median': median(values) if values else None, 'max': max(values, default=None)}
        if self.directory is not None:
            temp = self.directory / 'gyro_bridge_summary.json.tmp'
            temp.write_text(json.dumps(self.summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
            temp.replace(self.directory / 'gyro_bridge_summary.json')
        return self.summary

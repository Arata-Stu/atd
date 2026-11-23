"""Jetson の planner を購読して横方向制御の観測値を配信する。"""
from dataclasses import asdict
import math
import time

import rclpy
from atd_interfaces.msg import ControlStatus, PlanningStatus
from atd_planning.core import LINE_KEYS, MAX_AGE_S, yaw_from_quaternion
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from .core import CONTROLLER, LookaheadConfig, pure_pursuit, unavailable
from .speed import limited_target
from .vehicle import load_measured_wheelbase


IDENTITY_FIELDS = ('map_id', 'map_sha256', 'hdmap_sha256', 'session_id', 'epoch', 'line')


def pursuit_status(source, data, config, wheelbase, *, pose=None, validity_limit=MAX_AGE_S):
    """操舵を計算した観測の地図・時刻・姿勢と、その計算が選んだ目標を組にする。"""
    msg = ControlStatus(controller=CONTROLLER, valid=data['valid'], reason=data['reason'], **asdict(config))
    if source is None:
        msg.header.frame_id = 'map'
    else:
        msg.header = source.header
        for key in IDENTITY_FIELDS:
            setattr(msg, key, getattr(source, key))
        if math.isfinite(source.validity_s) and source.validity_s > 0:
            msg.validity_s = min(validity_limit, source.validity_s)
    if msg.valid:
        msg.pose = source.pose if pose is None else pose
        msg.target.x, msg.target.y = data['target']
        for key in ('requested_lookahead_m', 'actual_lookahead_m', 'curvature_per_m', 'preview_limited'):
            setattr(msg, key, data[key])
        if data['steering_rad'] is not None:
            msg.has_steering = True
            msg.steering_rad, msg.wheelbase_m = data['steering_rad'], wheelbase
    return msg


class ControlMonitor(Node):
    def __init__(self, **kwargs):
        super().__init__('atd_control', **kwargs)
        descriptor = ParameterDescriptor(read_only=True)
        self.config = LookaheadConfig(**{k: self.declare_parameter(k, v, descriptor).value
                                         for k, v in asdict(LookaheadConfig()).items()})
        limit = self.declare_parameter('max_target_speed_mps', 0., descriptor).value
        limited_target(0., limit)
        self.low_speed_drive = limit > 0
        path = self.declare_parameter('profile', '/data/results/runs/vehicle/layout.yaml', descriptor).value
        try:
            self.wheelbase = load_measured_wheelbase(path)
        except ValueError as error:
            self.get_logger().error(str(error))
            self.wheelbase = None
        if self.wheelbase is None:
            self.get_logger().warn('実測ホイールベースが未設定です。目標と曲率のみ計算します')
        self.source = None
        self.last_identity = None
        self.last_stamp_ns = None
        self.changed_at = None
        self.publisher = self.create_publisher(ControlStatus, '/control/status', 10)
        self.subscription = self.create_subscription(PlanningStatus, '/planning/status', self.observe, 10)
        self.create_timer(.1, self.watch, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish(self, data):
        self.publisher.publish(pursuit_status(self.source, data, self.config, self.wheelbase))

    def issue(self):
        source = self.source
        if self.count_publishers('/planning/status') > 1:
            return 'planner の配信元が複数あります'
        if source is None:
            return 'planner の入力待ちです'
        if not source.valid:
            return source.reason or 'planner の参照軌道が無効です'
        if (source.header.frame_id != 'map' or source.line not in LINE_KEYS
                or not all((source.map_id, source.map_sha256, source.hdmap_sha256, source.session_id))):
            return 'planner の地図・ライン・追跡セッションまたは座標系が不明です'
        if not math.isfinite(source.validity_s) or source.validity_s <= 0:
            return 'planner の有効期限が不正です'
        stamp_ns = source.header.stamp.sec * 10**9 + source.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        validity_s = min(source.validity_s, MAX_AGE_S)
        if stamp_ns <= 0 or not -.05 <= age <= validity_s:
            return 'planner の元観測時刻が有効期間外です'
        if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
            return 'planner の元観測時刻が逆行しました'
        if stamp_ns == self.last_stamp_ns and time.monotonic() - self.changed_at > validity_s:
            return 'planner の新しい元観測が届いていません'
        return None

    def watch(self):
        issue = self.issue()
        if issue:
            self.publish(unavailable(issue))

    def observe(self, source):
        self.source = source
        identity = tuple(getattr(source, field) for field in IDENTITY_FIELDS)
        if identity != self.last_identity:
            self.last_identity, self.last_stamp_ns, self.changed_at = identity, None, None
        issue = self.issue()
        if issue:
            return self.publish(unavailable(issue))
        stamp_ns = source.header.stamp.sec * 10**9 + source.header.stamp.nanosec
        if stamp_ns == self.last_stamp_ns:
            return
        self.last_stamp_ns, self.changed_at = stamp_ns, time.monotonic()
        position, q = source.pose.position, source.pose.orientation
        if not math.isfinite(position.z):
            return self.publish(unavailable('後輪軸の高さが不正です'))
        try:
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        except ValueError as error:
            return self.publish(unavailable(str(error)))
        data = pure_pursuit([(p.x, p.y) for p in source.trajectory], x=position.x, y=position.y, yaw=yaw,
                            speed_mps=source.estimated_speed_mps, wheelbase_m=self.wheelbase,
                            config=self.config, low_speed_drive=self.low_speed_drive)
        self.publish(data)


def main():
    rclpy.init()
    try:
        node = ControlMonitor()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()

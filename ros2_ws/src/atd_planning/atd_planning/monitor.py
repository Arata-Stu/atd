"""Jetson の追跡状態から観測専用の参照軌道を配信する。"""
import json
import math
import time

import rclpy
from atd_interfaces.msg import LocalizationStatus, PlanningStatus
from atd_localization.assets import map_metadata, verified_hdmap
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from .core import DEFAULT_HORIZON_M, MAX_AGE_S, Planner, ReferenceLine, invalid, yaw_from_quaternion


class PlanningMonitor(Node):
    def __init__(self, **kwargs):
        super().__init__('atd_planning', **kwargs)
        descriptor = ParameterDescriptor(read_only=True)
        defaults = dict(results_root='/data/results', map_id='', line='raceline',
                        horizon_m=DEFAULT_HORIZON_M, max_offset_m=.75)
        cfg = {key: self.declare_parameter(key, value, descriptor).value for key, value in defaults.items()}
        sim_freshness = self.declare_parameter('simulation_freshness', False, descriptor).value
        self.allow_sparse_tf = self.declare_parameter('allow_sparse_tf', False, descriptor).value
        if sim_freshness and not self.get_parameter('use_sim_time').value:
            raise ValueError('模擬時間での鮮度検査には use_sim_time が必要です')
        if self.allow_sparse_tf and not sim_freshness:
            raise ValueError('低頻度 TF を待つには simulation_freshness が必要です')
        self.arrival_time = (lambda: self.get_clock().now().nanoseconds * 1e-9) if sim_freshness else time.monotonic
        self.line = cfg['line']
        self.identity = dict(map_id=cfg['map_id'], map_sha256='', hdmap_sha256='')
        self.asset_issue = ''
        self.planner = None
        try:
            meta = map_metadata(cfg['results_root'], cfg['map_id'])
            self.identity = {key: meta[key] for key in self.identity}
            doc = json.loads(verified_hdmap(cfg['results_root'], meta))
            self.planner = Planner(ReferenceLine.from_document(doc, self.line), horizon_m=cfg['horizon_m'],
                                   max_offset_m=cfg['max_offset_m'])
        except (ValueError, OSError, KeyError, TypeError) as error:
            self.asset_issue = str(error)
            self.get_logger().error(self.asset_issue)
        self.last_observation = None
        self.changed_at = None
        self.source = None
        self.publisher = self.create_publisher(PlanningStatus, '/planning/status', 10)
        self.subscription = self.create_subscription(LocalizationStatus, '/localization/status', self.observe, 10)
        self.create_timer(.1, self.watch, clock=self.get_clock() if sim_freshness else Clock(clock_type=ClockType.STEADY_TIME))

    def publish(self, data):
        msg = PlanningStatus()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.validity_s = MAX_AGE_S
        msg.line, msg.valid, msg.reason = self.line, data['valid'], data['reason']
        for key, value in self.identity.items():
            setattr(msg, key, value)
        if self.source is not None:
            msg.session_id, msg.epoch = self.source.session_id, self.source.epoch
        if msg.valid:
            msg.header.stamp = self.source.header.stamp
            msg.pose = self.source.pose
            for key in ('progress_m', 'lateral_error_m', 'target_speed_mps', 'estimated_speed_mps'):
                setattr(msg, key, float(data[key]))
            msg.trajectory = [Point(x=float(x), y=float(y), z=0.) for x, y in data['trajectory']]
            msg.trajectory_speeds_mps = data['trajectory_speeds_mps']
        self.publisher.publish(msg)

    def invalidate(self, reason):
        if self.planner:
            self.planner.reset()
        self.publish(invalid(reason))

    def watch(self):
        if self.asset_issue:
            self.invalidate(self.asset_issue)
        elif self.count_publishers('/localization/status') > 1:
            self.invalidate('自己位置の配信元が複数あります')
        elif self.last_observation is None:
            self.invalidate('自己位置の入力待ちです')
        elif self.allow_sparse_tf:
            # The Gazebo controller coasts and waits for the next observation. A plan
            # is published only when a new pose arrives or an explicit failure is seen.
            return
        elif self.arrival_time() - self.changed_at > MAX_AGE_S:
            self.invalidate('自己位置の新しい観測が届いていません')
        elif self.source.state == 'tracking' and not self.fresh_stamp(self.last_observation[-1]):
            self.invalidate('自己位置の観測時刻が現在時刻と整合しません')

    def fresh_stamp(self, stamp_ns):
        age = self.get_clock().now().nanoseconds - stamp_ns
        return stamp_ns > 0 and -50_000_000 <= age <= MAX_AGE_S * 1e9

    def observe(self, source):
        self.source = source
        if self.asset_issue:
            return self.invalidate(self.asset_issue)
        stamp_ns = source.header.stamp.sec * 10**9 + source.header.stamp.nanosec
        key = (source.session_id, source.epoch, source.state, stamp_ns)
        repeated = key == self.last_observation
        if not repeated:
            self.last_observation = key
            self.changed_at = self.arrival_time()
        if self.count_publishers('/localization/status') > 1:
            return self.invalidate('自己位置の配信元が複数あります')
        if source.header.frame_id != 'map' or any(getattr(source, k) != v for k, v in self.identity.items()):
            return self.invalidate('自己位置と計画ラインの地図 ID・地図版・座標系が一致しません')
        if self.allow_sparse_tf and source.state == 'stale':
            return
        if source.state != 'tracking':
            return self.invalidate(f'{source.state}: {source.reason}')
        if not self.fresh_stamp(stamp_ns) or self.arrival_time() - self.changed_at > MAX_AGE_S:
            # Past samples can arrive late or repeat during a Gazebo observation gap.
            # A malformed or future stamp is still reported as a planning failure.
            if self.allow_sparse_tf and 0 < stamp_ns <= self.get_clock().now().nanoseconds:
                return
            return self.invalidate('自己位置の元観測が古いか、時刻が不正です')
        if repeated:
            # Localization periodically republishes its last pose; replay CSV stores each stamp once.
            return
        position, q = source.pose.position, source.pose.orientation
        try:
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            if not math.isfinite(position.z):
                raise ValueError('車体高さに非有限値があります')
        except ValueError as error:
            return self.invalidate(str(error))
        data = self.planner.update(stamp_ns=stamp_ns, x=position.x, y=position.y, yaw=yaw,
                                   speed_mps=source.speed_mps, session_id=source.session_id, epoch=source.epoch)
        self.publish(data)


def main():
    rclpy.init()
    try:
        node = PlanningMonitor()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()

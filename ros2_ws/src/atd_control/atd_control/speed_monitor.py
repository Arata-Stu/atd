"""planner・Pico・Joy を元時刻で照合し、速度制御の計算結果を監視 topic へ送る。"""
from collections import deque
from dataclasses import asdict
import json
import math
import time

import rclpy
from atd_interfaces.msg import DriveCommand, PicoTelemetry, PlanningStatus, SpeedControlStatus
from atd_interfaces.srv import SetSpeedTable
from atd_planning.core import LINE_KEYS, MAX_AGE_S
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String

from .manual_inputs import JoyStream, match_manual, pico_reading
from .monitor import IDENTITY_FIELDS
from .speed import DEFAULT_FEEDFORWARD_PATH, DriveFeedback, PIDConfig, SpeedController, limited_target, speed_unavailable
from .speed_table_file import SpeedTableFile


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class SpeedMonitor(Node):
    def __init__(self, **kwargs):
        super().__init__('atd_speed_control', **kwargs)
        fixed = ParameterDescriptor(read_only=True)
        self.config = PIDConfig(**{k: self.declare_parameter('speed_' + k, v, fixed).value
                                   for k, v in asdict(PIDConfig()).items()})
        path = self.declare_parameter('feedforward_path', DEFAULT_FEEDFORWARD_PATH, fixed).value
        self.speed_limit = self.declare_parameter('max_target_speed_mps', 0., fixed).value
        limited_target(0., self.speed_limit)  # 起動時に設定値を検査する。
        self.tables = SpeedTableFile(path)
        self.controller = None
        self.next_check = 0.
        self.plan = None
        self.identity = None
        self.watermark = None
        self.arrived = None
        self.pico = deque(maxlen=512)
        self.pico_session = None
        self.joy = deque(maxlen=512)
        self.joy_stream = JoyStream()
        self.joy_gid = None
        self.joy_received_ns = 0
        self.drive_feedback = DriveFeedback() if self.speed_limit else None
        self.drive_gid = None
        self.publisher = self.create_publisher(SpeedControlStatus, '/control/speed_status', 10)
        self.pico_sub = self.create_subscription(PicoTelemetry, '/vehicle/pico/telemetry', self.receive_pico, 20)
        self.joy_sub = self.create_subscription(DriveCommand, '/teleop/command', self.receive_joy, 20)
        self.create_subscription(PlanningStatus, '/planning/status', self.observe, 10)
        if self.speed_limit:
            self.drive_sub = self.create_subscription(String, '/vehicle/auto_drive/status', self.receive_drive, 10)
        else:
            self.create_service(SetSpeedTable, '/control/set_speed_table', self.set_speed_table)
        self.refresh_table()
        self.create_timer(.1, self.watch, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def refresh_table(self):
        self.next_check = time.monotonic() + 1.
        if not self.tables.refresh():
            return False
        self.controller = (SpeedController(self.tables.table, self.config, max_target_speed_mps=self.speed_limit)
                           if self.tables.table else None)
        return True

    def set_speed_table(self, request, response):
        try:
            selected = self.tables.save(json.loads(request.table_json))
            changed = self.refresh_table()
            if changed:
                self.after_table_change()
            if self.controller is None or self.controller.table != selected:
                raise ValueError(self.tables.issue or 'FF 表の保存結果が他の変更と競合しました')
            response.saved, response.run_id, response.proposal_sha256 = True, selected.run_id, selected.proposal_sha256
            response.message = ('速度プレビュー用の FF 表を保存し、PID を初期化しました' if changed
                                else '選択した FF 表は既に速度プレビューに適用されています')
        except (OSError, ValueError, RecursionError) as exc:
            response.message = str(exc)
        return response

    def receive_pico(self, msg):
        stamp = stamp_ns(msg)
        session = (msg.instance_id, msg.serial)
        if session != self.pico_session or self.pico and stamp < self.pico[-1][0]:
            self.pico.clear()
        self.pico_session = session
        if stamp <= 0 or self.pico_sub.get_publisher_count() != 1:
            self.pico.clear()
            return
        if self.pico and stamp == self.pico[-1][0]:
            self.pico[-1] = (stamp, 'unknown', None, 'Pico の元時刻が重複しています')
        else:
            self.pico.append((stamp, *pico_reading(msg)))

    def receive_joy(self, msg, info):
        received_ns = self.get_clock().now().nanoseconds
        gid = (info.get('publisher_gid') or {}).get('data', b'')
        single = self.joy_sub.get_publisher_count() == 1
        if not single or gid != self.joy_gid or received_ns < self.joy_received_ns:
            self.joy.clear()
            self.joy_stream = JoyStream()
        self.joy_gid, self.joy_received_ns = gid, received_ns
        if single:
            event = self.joy_stream.event(msg, received_ns)
            if event is not None:
                self.joy.append(event)

    def manual_at(self, stamp):
        pico_missing = joy_missing = ''
        if self.pico_sub.get_publisher_count() != 1:
            self.pico.clear()
            pico_missing = 'Pico の配信元が一つに定まりません'
        if self.joy_sub.get_publisher_count() != 1:
            self.joy.clear()
            self.joy_gid = None
            joy_missing = 'DualSense の運転要求の配信元が一つに定まりません'
        pico = next((entry for entry in reversed(self.pico) if entry[0] <= stamp), None)
        joy = next((entry[1:] for entry in reversed(self.joy) if entry[0] <= stamp), None)
        return match_manual(stamp, pico, joy, pico_missing=pico_missing, joy_missing=joy_missing,
                            joy_drives=not self.speed_limit)

    def publish(self, data, manual=None):
        msg = SpeedControlStatus(valid=data['valid'], reason=data['reason'], **asdict(self.config))
        if self.plan is not None:
            msg.header = self.plan.header
            for key in IDENTITY_FIELDS:
                setattr(msg, key, getattr(self.plan, key))
            if math.isfinite(self.plan.validity_s) and self.plan.validity_s > 0:
                msg.validity_s = min(MAX_AGE_S, self.plan.validity_s)
        else:
            msg.header.frame_id = 'map'
        if self.controller:
            msg.calibration_run_id = self.controller.table.run_id
            msg.proposal_sha256 = self.controller.table.proposal_sha256
        if data['speed_error_mps'] is not None:
            msg.has_speeds = True
            for key in ('target_speed_mps', 'estimated_speed_mps', 'speed_error_mps'):
                setattr(msg, key, data[key])
        if data['valid']:
            for key in ('feedforward_raw', 'p_raw', 'i_raw', 'd_raw', 'calculated_raw', 'saturated'):
                setattr(msg, key, data[key])
        if manual is None:
            waiting = '対応する速度観測を待っています'
            manual = match_manual(0, pico_missing=waiting, joy_missing=waiting)
        msg.manual_source = manual['manual_source']
        msg.propo_reason, msg.joy_reason = manual['propo_reason'], manual['joy_reason']
        if manual['propo_raw'] is not None:
            msg.has_propo = True
            msg.propo_raw, msg.propo_age_s = manual['propo_raw'], manual['propo_age_s']
        if manual['joy_throttle'] is not None:
            msg.has_joy = True
            msg.joy_throttle, msg.joy_age_s = manual['joy_throttle'], manual['joy_age_s']
            msg.joy_enable_requested = manual['joy_enable_requested']
        self.publisher.publish(msg)

    def invalid(self, reason):
        if self.controller:
            self.controller.reset()
        self.publish(speed_unavailable(reason))

    def receive_drive(self, msg, info):
        gid = bytes((info.get('publisher_gid') or {}).get('data', b''))
        single = self.drive_sub.get_publisher_count() == 1
        if not single or gid != self.drive_gid:
            self.drive_gid = None
            if single and gid:
                endpoints = self.get_publishers_info_by_topic('/vehicle/auto_drive/status')
                if (len(endpoints) == 1 and endpoints[0].node_name == 'atd_auto_drive'
                        and endpoints[0].node_namespace == '/' and bytes(endpoints[0].endpoint_gid) == gid):
                    self.drive_gid = gid
            self.drive_feedback = DriveFeedback()
            if self.controller:
                self.controller.reset()
        try:
            data = json.loads(msg.data) if self.drive_gid else None
        except (RecursionError, ValueError):
            data = None
        if not self.drive_feedback.update(data, time.time_ns(), time.monotonic_ns()) and self.controller:
            self.controller.reset()

    def source_issue(self, now_ns):
        if self.plan is None:
            return 'planner の速度観測を待っています'
        if self.count_publishers('/planning/status') != 1:
            return 'planner の配信元が一つに定まりません'
        plan = self.plan
        if not plan.valid:
            return plan.reason or 'planner の速度観測が無効です'
        if (plan.header.frame_id != 'map' or plan.line not in LINE_KEYS
                or not all((plan.map_id, plan.map_sha256, plan.hdmap_sha256, plan.session_id))):
            return '速度観測の座標系・地図・ラインまたは追跡区間が不正です'
        stamp = stamp_ns(plan)
        if not math.isfinite(plan.validity_s) or plan.validity_s <= 0:
            return '速度観測の有効期限が不正です'
        ttl = min(plan.validity_s, MAX_AGE_S)
        if (stamp <= 0 or not -50_000_000 <= now_ns - stamp <= ttl * 1e9
                or stamp == self.watermark and time.monotonic() - self.arrived > ttl):
            return '速度観測が古いか、元時刻が不正です'
        if self.watermark is not None and stamp < self.watermark:
            return '速度観測の元時刻が逆行しています'
        return ''

    def observe(self, plan):
        self.plan = plan
        identity = tuple(getattr(plan, k) for k in IDENTITY_FIELDS)
        if self.identity != identity:
            self.identity, self.watermark, self.arrived = identity, None, None
            if self.controller:
                self.controller.reset()
        issue = self.source_issue(self.get_clock().now().nanoseconds)
        if issue:
            return self.invalid(issue)
        stamp = stamp_ns(plan)
        if self.watermark == stamp:
            return
        self.watermark, self.arrived = stamp, time.monotonic()
        target, measured = plan.target_speed_mps, plan.estimated_speed_mps
        if self.controller:
            if self.drive_feedback and (self.drive_sub.get_publisher_count() != 1 or not self.drive_feedback.matches(
                    (plan.map_id, plan.line), time.time_ns(), time.monotonic_ns())):
                self.controller.reset()
            result = self.controller.update(stamp, target, measured, identity)
        else:
            result = speed_unavailable(self.tables.issue, limited_target(target, self.speed_limit), measured)
        self.publish(result, self.manual_at(stamp))

    def after_table_change(self):
        issue = self.source_issue(self.get_clock().now().nanoseconds)
        if issue:
            self.invalid(issue)
        else:
            target, measured = limited_target(self.plan.target_speed_mps, self.speed_limit), self.plan.estimated_speed_mps
            self.publish(speed_unavailable(self.tables.issue or 'FF 表を更新しました。次の速度観測を待っています',
                                           target, measured), self.manual_at(stamp_ns(self.plan)))

    def watch(self):
        if not self.speed_limit and time.monotonic() >= self.next_check and self.refresh_table():
            return self.after_table_change()
        issue = self.source_issue(self.get_clock().now().nanoseconds)
        if issue:
            self.invalid(issue)


def main():
    rclpy.init()
    try:
        node = SpeedMonitor()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()

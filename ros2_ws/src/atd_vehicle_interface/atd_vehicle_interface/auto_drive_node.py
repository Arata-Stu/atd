"""Jetson と Pico CDC0 の低速・前進限定の自動走行ノード。"""
from functools import partial
import json
import time
from uuid import uuid4

import rclpy
from atd_control.drive_tuning import DEFAULT_DRIVE_TUNING, DriveTuning
from atd_control.speed import DEFAULT_FEEDFORWARD_PATH
from atd_interfaces.msg import ControlStatus, LocalizationStatus, SpeedControlStatus
from atd_localization.assets import map_metadata, verified_hdmap
from atd_planning.core import DEFAULT_HORIZON_M, LINE_KEYS, ReferenceLine
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from .auto_board import AutoBoard
from .auto_calibration import load_calibration
from .auto_drive import AutoDriveGate
from .auto_status import snapshot

# rosbag2/player や別セッションが同じ topic 名で配信しても実車の指令にしない。
SOURCES = {
    'localization': ('/localization/status', 'atd_localization_monitor', LocalizationStatus),
    'steering': ('/control/status', 'atd_control', ControlStatus),
    'speed': ('/control/speed_status', 'atd_speed_control', SpeedControlStatus),
}
PLANNER = ('/planning/status', 'atd_planning')


class AutoDriveNode(Node):
    def __init__(self, **kwargs):
        super().__init__('atd_auto_drive', **kwargs)

        def setting(name, value):
            return self.declare_parameter(name, value, ParameterDescriptor(read_only=True)).value

        self.port = setting('port', '')
        self.serial = setting('serial', '')
        self.calibration = load_calibration(setting('auto_calibration_path', '/data/results/runs/vehicle/auto_drive.yaml'),
                                            setting('feedforward_path', DEFAULT_FEEDFORWARD_PATH))
        self.drive_tuning = setting('drive_tuning', False)
        map_id = setting('map_id', '')
        line = self.declare_parameter('line', 'raceline', ParameterDescriptor(read_only=not self.drive_tuning)).value
        cap = self.declare_parameter('max_speed_mps', .5, ParameterDescriptor(read_only=not self.drive_tuning)).value
        self.gate_args = (map_id, line, cap, self.calibration)
        self.gate = AutoDriveGate(*self.gate_args)
        self.input_sources = SOURCES.copy()
        if self.drive_tuning:
            from atd_interfaces.msg import TunedControlStatus, TunedSpeedControlStatus
            self.input_sources.update(steering=('/control/tuned_steering', 'atd_control', TunedControlStatus),
                                      speed=('/control/tuned_speed', 'atd_speed_control', TunedSpeedControlStatus))
            self.configure_tuning(setting('results_root', '/data/results'), map_id, line, cap)
        self.sources = {}
        self.next_graph = self.next_open = self.last_wall = self.next_heartbeat = 0
        self.board = None
        self.session_id = uuid4().hex
        self.reported = None
        self.last_telemetry = None
        self.publisher = self.create_publisher(String, '/vehicle/auto_drive/status', 10)
        for kind, (topic, _, message_type) in self.input_sources.items():
            self.create_subscription(message_type, topic, partial(self.receive, kind), 10)
        self.board_timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def configure_tuning(self, root, map_id, line, cap):
        values = {k: self.declare_parameter(k, v).value for k, v in DEFAULT_DRIVE_TUNING.values().items()
                  if k not in ('line', 'max_speed_mps')}
        self.tuning = DriveTuning.from_dict(dict(values, line=line, max_speed_mps=cap))
        self.horizon = max(DEFAULT_HORIZON_M, self.tuning.lookahead.max_lookahead_m)
        meta = map_metadata(root, map_id)
        self.map_identity = {k: meta[k] for k in ('map_id', 'map_sha256', 'hdmap_sha256')}
        doc = json.loads(verified_hdmap(root, meta))
        self.available = []
        for name in LINE_KEYS:
            try:
                ref = ReferenceLine.from_document(doc, name)
            except ValueError:
                continue
            if min(ref.speeds) >= self.calibration.table.speeds[0]:
                self.available.append(name)
        self.validate_tuning(self.tuning)
        limits = self.describe_parameter('max_speed_mps')
        limits.floating_point_range = [FloatingPointRange(from_value=self.calibration.table.speeds[0], to_value=cap)]
        self.set_descriptor('max_speed_mps', limits)
        # These values describe the verified startup assets, even with ROS parameter overrides.
        self.declare_parameter('available_lines', self.available, ParameterDescriptor(read_only=True), ignore_override=True)
        self.declare_parameter('launch_speed_cap_mps', cap, ParameterDescriptor(read_only=True), ignore_override=True)
        self.tuning_pub = self.create_publisher(String, '/vehicle/drive_tuning',
                                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tuning_stamp_ns = 0
        self.publish_tuning(self.tuning)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.add_post_set_parameters_callback(self.apply_parameters)

    def validate_tuning(self, tuning):
        if tuning.line not in self.available:
            raise ValueError('選択した地図に前進校正域で走行できるラインがありません')
        if tuning.max_speed_mps > self.gate.speed_ceiling:
            raise ValueError(f'起動時の実車速度上限 {self.gate.speed_ceiling:g} m/s を超えています')
        if self.calibration.table.interpolate(tuning.max_speed_mps) is None:
            raise ValueError('速度上限を前進 FF 表の校正速度域に設定してください')
        if tuning.lookahead.max_lookahead_m > self.horizon:
            raise ValueError(f'最大先読み距離が前方軌道長 {self.horizon:g} m を超えています')

    def candidate(self, params):
        values = self.tuning.values()
        for p in params:
            if p.name in values:
                values[p.name] = p.value
        return DriveTuning.from_dict(values)

    def validate_parameters(self, params):
        try:
            self.validate_tuning(self.candidate(params))
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def apply_parameters(self, params):
        candidate = self.candidate(params)
        if candidate != self.tuning:
            self.publish_tuning(candidate)

    def publish_tuning(self, tuning):
        self.tuning_stamp_ns = max(self.tuning_stamp_ns + 1, time.time_ns())
        self.gate.retune(tuning.line, tuning.max_speed_mps, self.tuning_stamp_ns)
        self.tuning = tuning
        data = dict(self.map_identity, **tuning.values(), stamp_ns=self.tuning_stamp_ns)
        self.tuning_pub.publish(String(data=json.dumps(data)))
        self.get_logger().info(f'走行設定を適用 {data}')

    def graph(self, steady):
        expected = {kind: (topic, node) for kind, (topic, node, _) in self.input_sources.items()}
        expected['planner'] = PLANNER
        if any(self.count_publishers(topic) != 1 for topic, _ in expected.values()):
            self.next_graph = 0
            return {}
        if steady < self.next_graph:
            return self.sources
        ids = {}
        for kind, (topic, name) in expected.items():
            endpoints = self.get_publishers_info_by_topic(topic)
            if len(endpoints) != 1 or endpoints[0].node_name != name or endpoints[0].node_namespace != '/':
                return {}
            gid = bytes(endpoints[0].endpoint_gid)
            if not any(gid):
                return {}
            ids[kind] = gid
        self.next_graph = steady + 100_000_000
        return ids

    def receive(self, kind, msg, info):
        gid = bytes((info.get('publisher_gid') or {}).get('data', b''))
        if not gid or gid != self.sources.get(kind):
            self.gate.invalidate('制御または自己位置の配信元を確認できません')
            self.gate.speeds.clear(time.time_ns())
            return
        if self.drive_tuning:
            if kind != 'localization':
                if msg.tuning_stamp_ns != self.tuning_stamp_ns:
                    return
                msg = msg.status
            if any(getattr(msg, key) != value for key, value in self.map_identity.items()):
                self.gate.invalidate('ROS 観測の地図と起動時の地図が一致しません')
                return
        matched = self.gate.receive(kind, msg, time.time_ns(), time.monotonic_ns())
        if matched and self.gate.host_since is not None:
            # 通常走行はカメラ観測ごとに照合・送信。観測が途切れても CH3・期限・制動を監視する。
            self.tick()
            self.board_timer.reset()

    def close(self):
        if self.board is not None:
            self.board.close()
            self.board = None
        last_stop = self.gate.last_stop
        self.gate = AutoDriveGate(*self.gate_args)
        if self.drive_tuning:
            self.gate.retune(self.tuning.line, self.tuning.max_speed_mps, self.tuning_stamp_ns)
        self.gate.last_stop = last_stop
        self.sources = {}
        self.next_graph = 0

    def update(self, wall, steady):
        if self.get_parameter('use_sim_time').value or wall < self.last_wall:
            self.close()
            return 'シミュレーション時刻または時刻逆行のため停止しました'
        if self.board is None:
            if steady < self.next_open:
                return self.reported[0]
            self.board = AutoBoard(self.port, self.serial, brake_level=self.calibration.brake_level,
                                   brake_hold_s=self.calibration.brake_hold_s)
            self.board.open()
        status = self.board.poll()
        sources = self.graph(steady)
        if sources != self.sources or not sources:
            self.gate.invalidate('Jetson の自己位置・planner・controller の配信元を確認できません')
            self.gate.speeds.clear(wall)
            self.sources = sources
        return self.gate.step(self.board, status, time.time_ns(), time.monotonic_ns())

    def tick(self):
        wall, steady = time.time_ns(), time.monotonic_ns()
        try:
            reason = self.update(wall, steady)
        except (OSError, ValueError, RuntimeError) as exc:
            self.close()
            self.next_open = steady + 1_000_000_000
            reason = f'Pico CDC0 再接続待ち（{exc}）'
        self.last_wall = wall
        payload = snapshot(self.gate, self.board, self.session_id, reason, time.time_ns(), time.monotonic_ns())
        current = (reason, payload['active'], payload['braking'])
        telemetry = (payload['pico'], payload['command'], payload['speed'], payload['last_stop'])
        if current != self.reported:
            self.get_logger().info(reason)
        elif telemetry == self.last_telemetry and steady < self.next_heartbeat:
            return
        self.reported, self.last_telemetry = current, telemetry
        self.next_heartbeat = steady + 250_000_000
        self.publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main():
    rclpy.init()
    try:
        node = AutoDriveNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.close()
            node.destroy_node()
    finally:
        rclpy.try_shutdown()

"""Jetson と Pico CDC0 の低速・前進限定の自動走行ノード。"""
from functools import partial
import json
import time
from uuid import uuid4

import rclpy
from atd_control.speed import DEFAULT_FEEDFORWARD_PATH
from atd_interfaces.msg import ControlStatus, LocalizationStatus, SpeedControlStatus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
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
        fixed = ParameterDescriptor(read_only=True)

        def setting(name, value):
            return self.declare_parameter(name, value, fixed).value

        self.port = setting('port', '')
        self.serial = setting('serial', '')
        self.calibration = load_calibration(setting('auto_calibration_path', '/data/results/runs/vehicle/auto_drive.yaml'),
                                            setting('feedforward_path', DEFAULT_FEEDFORWARD_PATH))
        self.gate_args = (setting('map_id', ''), setting('line', 'raceline'), setting('max_speed_mps', .5),
                          self.calibration)
        self.gate = AutoDriveGate(*self.gate_args)
        self.sources = {}
        self.next_graph = self.next_open = self.last_wall = self.next_heartbeat = 0
        self.board = None
        self.session_id = uuid4().hex
        self.reported = None
        self.last_telemetry = None
        self.publisher = self.create_publisher(String, '/vehicle/auto_drive/status', 10)
        for kind, (topic, _, message_type) in SOURCES.items():
            self.create_subscription(message_type, topic, partial(self.receive, kind), 10)
        self.board_timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def graph(self, steady):
        expected = {kind: (topic, node) for kind, (topic, node, _) in SOURCES.items()}
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

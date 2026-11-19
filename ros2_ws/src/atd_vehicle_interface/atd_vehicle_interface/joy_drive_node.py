"""DualSense の運転要求を Jetson の Pico CDC0 に接続する ROS ノード。"""
import time
from uuid import uuid4

import rclpy
from atd_esc_trials.board import ManualBoard
from atd_interfaces.msg import DriveCommand, JoyDriveStatus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .joy_drive import DriveGate, drive_feedback


# bag 再生も /joy 系 topic を配信しうる。実機入力と役割処理の配信元を照合する。
SOURCES = (('teleop/drive_command', 'atd_teleop_node'), ('joy', 'atd_joy_node'),
           ('joy/ready', 'atd_joy_node'), ('joy/status', 'atd_joy_node'))


class JoyDriveNode(Node):
    def __init__(self, **kwargs):
        super().__init__('atd_joy_drive', **kwargs)
        fixed = ParameterDescriptor(read_only=True)
        self.port = self.declare_parameter('port', '', fixed).value
        self.serial = self.declare_parameter('serial', '', fixed).value
        self.board = None
        self.session_id = ''
        self.gate = DriveGate()
        self.sources = ()
        self.next_graph = 0
        self.next_open = 0
        self.last_wall = 0
        self.message = None
        self.last_feedback = None
        self.next_heartbeat = 0
        self.feedback_pub = self.create_publisher(JoyDriveStatus, 'vehicle/joy_drive/status',
                                                  QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        # 短い enable 解放や無効化もカーネル I/O の合間に受け取る。鮮度は元時刻で検査する。
        self.create_subscription(DriveCommand, SOURCES[0][0], self.receive, 10)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def report(self, message):
        if self.message != message:
            self.message = message
            self.get_logger().info(message)

    def live_sources(self, now):
        # 名前・GID の照合は 100 ms ごと。発行者の欠落・重複は毎回検査する。
        if any(self.count_publishers(topic) != 1 for topic, _ in SOURCES):
            self.next_graph = 0
            return ()
        if now < self.next_graph:
            return self.sources
        result = []
        for topic, name in SOURCES:
            endpoints = self.get_publishers_info_by_topic(topic)
            if (len(endpoints) != 1 or endpoints[0].node_name != name
                    or endpoints[0].node_namespace != self.get_namespace()):
                return ()
            gid = bytes(endpoints[0].endpoint_gid)
            if not any(gid):
                return ()
            result.append(gid)
        self.next_graph = now + 100_000_000
        return tuple(result)

    def receive(self, msg, info):
        gid = (info.get('publisher_gid') or {}).get('data', b'')
        if not self.sources or bytes(gid) != self.sources[0]:
            self.gate.invalidate()
            return
        self.gate.receive(msg, time.time_ns(), time.monotonic_ns())

    def close(self):
        if self.board is not None:
            self.board.close()
            self.board = None
        self.session_id = ''
        self.sources = ()
        self.next_graph = 0
        self.gate = DriveGate()

    def tick(self):
        self.update_drive()
        # 制御の後で、受信済み S のスナップショットを best-effort で公開する。
        fields = drive_feedback(self.board, self.gate, self.session_id, self.message or '')
        now = time.monotonic_ns()
        if fields == self.last_feedback and now < self.next_heartbeat:
            return
        self.last_feedback = fields.copy()
        stamp = fields.pop('stamp_ns')
        msg = JoyDriveStatus(**fields)
        msg.header.frame_id = 'pico'
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        self.feedback_pub.publish(msg)
        self.next_heartbeat = now + 1_000_000_000

    def update_drive(self):
        now = time.monotonic_ns()
        wall = time.time_ns()
        if self.get_parameter('use_sim_time').value or wall < self.last_wall:
            self.close()
            self.last_wall = wall
            self.report('実時間の Joy 入力を待機中')
            return
        self.last_wall = wall
        try:
            if self.board is None:
                if now < self.next_open:
                    return
                self.board = ManualBoard(self.port, self.serial)
                self.board.open()
                self.session_id = uuid4().hex
            status = self.board.poll()
            if status and status['host_selected']:
                sources = self.live_sources(now)
                if self.sources != sources:
                    self.gate.stop(self.board)
                    self.gate = DriveGate()
                    self.sources = sources
                if not sources:
                    self.report('Jetson の Joy・teleop 配信元を待機中')
                    return
            self.report(self.gate.step(self.board, status, time.time_ns(), time.monotonic_ns()))
        except (OSError, ValueError, RuntimeError) as exc:
            self.close()
            self.next_open = now + 1_000_000_000
            self.report(f'Pico CDC0 再接続待ち: {exc}')


def main():
    rclpy.init()
    try:
        node = JoyDriveNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.close()
            node.destroy_node()
    finally:
        rclpy.try_shutdown()

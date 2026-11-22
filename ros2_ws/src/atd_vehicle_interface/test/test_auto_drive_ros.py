"""ROS 実行環境では実 topic と配信元の切替から模擬 CDC0 への出力まで検査する。"""
import os
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from atd_interfaces.msg import ControlStatus, LocalizationStatus, PlanningStatus, SpeedControlStatus
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from atd_vehicle_interface import auto_drive_node


def test_live_ros_tracking_loss_stops_and_recovery_requires_rc(monkeypatch):
    writes = []

    class Board:
        def __init__(self, *args, **kwargs):
            self.selector = 0
            self.armed = self.braking = False
            self.serial = '1234567890abcdef'

        def open(self):
            pass

        def latest_report(self):
            return None

        def latest_command(self, wall_ns, steady_ns):
            return None

        def poll(self):
            return dict(host_selected=self.selector == 1, selector=self.selector, fault_bits=0,
                        rx_mode_us=1900 if self.selector else 1100,
                        active_path=(2 if self.armed else 0) if self.selector else 1,
                        esc_us=1500, servo_us=1500)

        def arm(self):
            self.armed = True
            writes.append('arm')
            return True

        def drive(self, steering, throttle):
            writes.append((steering, throttle))
            return True

        def stop_forward(self):
            self.braking = True
            writes.append('brake')

        def disarm(self):
            self.armed = self.braking = False
            writes.append('disable')

        def close(self):
            self.disarm()

    monkeypatch.setattr(auto_drive_node, 'AutoBoard', Board)
    calibration = SimpleNamespace(brake_level=.1, brake_hold_s=.2,
                                  commands=lambda steering, speed, **_: (steering, .1))
    monkeypatch.setattr(auto_drive_node, 'load_calibration', lambda *_: calibration)
    ctx = Context()
    rclpy.init(context=ctx, domain_id=65 + os.getpid() % 20)
    executor = SingleThreadedExecutor(context=ctx)
    driver = auto_drive_node.AutoDriveNode(context=ctx, parameter_overrides=[Parameter('map_id', value='test')])
    nodes = [driver]
    pubs = {}
    for kind, (topic, name, message) in auto_drive_node.SOURCES.items():
        source = Node(name, context=ctx)
        pubs[kind] = source.create_publisher(message, topic, 10)
        nodes.append(source)
    planner = Node('atd_planning', context=ctx)
    planner.create_publisher(PlanningStatus, '/planning/status', 10)
    nodes.append(planner)
    for node in nodes:
        executor.add_node(node)

    identity = dict(map_id='test', map_sha256='1'*64, hdmap_sha256='2'*64, session_id='live', epoch=1)
    ctl = dict(**identity, valid=True, line='raceline', validity_s=.25)
    loc = LocalizationStatus(**identity, state='tracking', speed_mps=.01)
    steering = ControlStatus(**ctl, controller='pure_pursuit', has_steering=True, steering_rad=.15)
    speed = SpeedControlStatus(**ctl, has_speeds=True, target_speed_mps=.3, estimated_speed_mps=.01)

    def emit():
        now = driver.get_clock().now().to_msg()
        for kind, msg in (('localization', loc), ('steering', steering), ('speed', speed)):
            msg.header.frame_id = 'map'
            msg.header.stamp = now
            pubs[kind].publish(msg)

    def spin_until(condition, emit_samples=True):
        deadline, next_emit = time.monotonic() + 6, 0
        while not condition() and time.monotonic() < deadline:
            if time.monotonic() >= next_emit and emit_samples:
                emit()
                next_emit = time.monotonic() + .05
            executor.spin_once(timeout_sec=.005)
        assert condition()

    try:
        spin_until(lambda: driver.gate.rc_seen and bool(driver.sources))
        driver.board.selector = 1
        spin_until(lambda: (.15, .1) in writes)
        # タイマーを待たず、同時刻の3入力がそろった callback だけで送る。
        driver.board_timer.cancel()
        steering.steering_rad = .25
        emit()
        spin_until(lambda: (.25, .1) in writes, emit_samples=False)
        assert not driver.board_timer.is_canceled()
        loc.speed_mps = speed.estimated_speed_mps = .2
        spin_until(lambda: driver.gate.velocity and driver.gate.velocity[2] == .2)
        loc.state = 'lost'
        spin_until(lambda: 'brake' in writes)
        count = len(writes)
        loc.state = 'tracking'
        until = time.monotonic() + .25
        spin_until(lambda: time.monotonic() >= until)
        assert len(writes) == count and not driver.gate.active
        driver.board.selector = 0
        driver.board.armed = driver.board.braking = False  # Pico took back the HOST path.
        loc.speed_mps = speed.estimated_speed_mps = .01
        spin_until(lambda: driver.gate.rc_seen)
        driver.board.selector = 1
        spin_until(lambda: driver.gate.active)
        replay = Node('rosbag2_player', context=ctx)
        nodes.append(replay)
        executor.add_node(replay)
        replay.create_publisher(PlanningStatus, '/planning/status', 10)
        spin_until(lambda: not driver.gate.active and writes[-1] == 'disable')
    finally:
        driver.close()
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown(context=ctx)

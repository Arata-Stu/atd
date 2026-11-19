"""実 ROS graph / subscription と模擬 CDC0 を使う。"""
import os
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from atd_interfaces.msg import DriveCommand, JoyDriveStatus
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from atd_vehicle_interface import joy_drive_node


def test_drive_topic_requires_live_teleop_and_joy_publishers(monkeypatch):
    writes = []

    class Board:
        def __init__(self, *_):
            self.path = 0
            self.serial = '1234567890abcdef'
            self.report = None
            self.sequence = 0

        def open(self):
            pass

        def close(self):
            self.disarm()

        def poll(self):
            self.sequence += 1
            sample = dict(host_selected=True, active_path=self.path, servo_us=1500, esc_us=1500,
                          rx_mode_us=1900, selector=1, fault_bits=0, sequence=self.sequence)
            self.report = time.time_ns(), sample
            return sample

        def latest_report(self):
            return self.report

        def arm(self):
            writes.append('arm')
            self.path = 2
            return True

        def disarm(self):
            writes.append('disable')
            self.path = 0

        def drive(self, steering, throttle):
            writes.append((steering, throttle))
            return True

    monkeypatch.setattr(joy_drive_node, 'ManualBoard', Board)
    ctx = Context()
    rclpy.init(context=ctx, domain_id=70 + os.getpid() % 25)
    executor = SingleThreadedExecutor(context=ctx)
    driver = joy_drive_node.JoyDriveNode(context=ctx)
    joy = Node('atd_joy_node', context=ctx)
    teleop = Node('atd_teleop_node', context=ctx)
    bag = Node('rosbag2_player', context=ctx)
    nodes = (driver, joy, teleop, bag)
    for node in nodes:
        executor.add_node(node)

    def spin_until(condition, emit=None):
        deadline = time.monotonic() + 5
        while not condition() and time.monotonic() < deadline:
            if emit:
                emit()
            executor.spin_once(timeout_sec=.01)
        assert condition()

    try:
        joy.create_publisher(Joy, 'joy', 1)
        joy.create_publisher(Bool, 'joy/ready', 1)
        joy.create_publisher(String, 'joy/status', 1)
        drive = teleop.create_publisher(DriveCommand, 'teleop/drive_command', 1)
        cmd = DriveCommand(source='joy', valid=True)
        cmd.header.frame_id = 'atd_gamepad'
        observations = []
        joy.create_subscription(JoyDriveStatus, '/vehicle/joy_drive/status', observations.append, qos_profile_sensor_data)

        def emit():
            cmd.header.stamp = joy.get_clock().now().to_msg()
            drive.publish(cmd)

        spin_until(lambda: bool(driver.sources) and drive.get_subscription_count() == 1)
        spin_until(lambda: (0., 0.) in writes, emit)
        cmd.steering, cmd.throttle = -.5, .25
        spin_until(lambda: (-.5, .25) in writes, emit)
        spin_until(lambda: observations and observations[-1].joy_active, emit)
        assert observations[-1].serial == '1234567890abcdef' and observations[-1].has_report
        assert observations[-1].session_id and observations[-1].active_path == 2
        bag.create_publisher(Joy, 'joy', 1)
        spin_until(lambda: writes[-1] == 'disable' and not driver.sources, emit)
        spin_until(lambda: observations and not observations[-1].joy_active, emit)
    finally:
        driver.close()
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown(context=ctx)

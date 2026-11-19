"""ROS 環境では型生成済みの topic を隔離 domain 内で往復し、失効・復帰まで確認する。"""
import json
import os
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from atd_interfaces.msg import LocalizationStatus, PlanningStatus
from atd_planning import monitor
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


def test_reference_messages_and_invalidation(monkeypatch):
    identity = dict(map_id='course', map_sha256='a' * 64, hdmap_sha256='b' * 64)
    doc = dict(id='course', frame='map', closed=False, lines=dict(custom=dict(points=[[0, 0], [3, 0]],
        profile=[[0, 0, 0, 0, 0, 1, 0], [3, 3, 0, 0, 0, 0, 0]])))
    monkeypatch.setattr(monitor, 'map_metadata', lambda *_: identity)
    monkeypatch.setattr(monitor, 'verified_hdmap', lambda *_: json.dumps(doc).encode())
    context = Context()
    rclpy.init(context=context, domain_id=110 + os.getpid() % 45)
    node = monitor.PlanningMonitor(context=context, parameter_overrides=[
        Parameter('map_id', value='course'), Parameter('line', value='custom')])
    driver = Node('validation_inputs', context=context)
    executor = SingleThreadedExecutor(context=context)
    for item in (driver, node):
        executor.add_node(item)
    try:
        received = []
        driver.create_subscription(PlanningStatus, '/planning/status', received.append, 10)
        publisher = driver.create_publisher(LocalizationStatus, '/localization/status', 10)

        def wait(predicate):
            end = time.monotonic() + 5.
            while time.monotonic() < end and not predicate():
                executor.spin_once(timeout_sec=.02)
            assert predicate()

        wait(lambda: publisher.get_subscription_count() == 1 and node.publisher.get_subscription_count() == 1)
        source = LocalizationStatus(**identity, session_id='manual', epoch=1, state='tracking', speed_mps=-.2)
        source.header.frame_id = 'map'
        source.pose.position.x, source.pose.position.y, source.pose.orientation.w = .5, -.15, 1.

        def send():
            source.header.stamp = driver.get_clock().now().to_msg()
            publisher.publish(source)

        send()
        wait(lambda: received and received[-1].valid)
        plan = received[-1]
        assert plan.header.stamp == source.header.stamp and plan.pose == source.pose and plan.line == 'custom'
        assert plan.lateral_error_m == pytest.approx(-.15) and plan.estimated_speed_mps == -.2
        assert plan.trajectory[0].x == .5 and plan.trajectory[-1].x == 2.5
        assert plan.session_id == 'manual' and plan.epoch == 1
        source.hdmap_sha256 = 'c' * 64
        send()
        wait(lambda: not received[-1].valid and '地図版' in received[-1].reason)
        assert not received[-1].trajectory and not received[-1].trajectory_speeds_mps
        source.hdmap_sha256 = identity['hdmap_sha256']
        source.state = 'lost'
        source.epoch = 2
        send()
        wait(lambda: not received[-1].valid and 'lost' in received[-1].reason)
        source.state = 'tracking'
        send()
        wait(lambda: received[-1].valid and received[-1].epoch == 2)
        wait(lambda: not received[-1].valid)
        # Repeating a frozen localization input must not revive it.
        publisher.publish(source)
        wait(lambda: not received[-1].valid and '元観測' in received[-1].reason)
    finally:
        executor.shutdown()
        driver.destroy_node()
        node.destroy_node()
        rclpy.shutdown(context=context)

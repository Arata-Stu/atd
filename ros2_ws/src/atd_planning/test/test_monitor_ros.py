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


def test_sparse_tf_planner_waits_through_temporal_gaps_and_forwards_failures():
    from types import SimpleNamespace

    identity = dict(map_id='course', map_sha256='a' * 64, hdmap_sha256='b' * 64)
    source = LocalizationStatus(**identity, session_id='sim', state='tracking')
    source.header.frame_id = 'map'
    source.header.stamp.sec = 4
    source.pose.orientation.w = 1.
    now = SimpleNamespace(nanoseconds=4_000_000_000)
    node = SimpleNamespace(allow_sparse_tf=True, asset_issue='', last_observation=None, changed_at=None,
                           pending_selection=None, switch_stamp=-1,
                           identity=identity, count_publishers=lambda _: 1, arrival_time=lambda: now.nanoseconds * 1e-9,
                           get_clock=lambda: SimpleNamespace(now=lambda: now))
    node.fresh_stamp = lambda stamp: monitor.PlanningMonitor.fresh_stamp(node, stamp)
    calls = []
    node.publish = lambda value: calls.append('plan')
    node.invalidate = calls.append
    node.planner = SimpleNamespace(update=lambda **_: {})
    monitor.PlanningMonitor.observe(node, source)
    now.nanoseconds += 2_000_000_000
    monitor.PlanningMonitor.watch(node)
    monitor.PlanningMonitor.observe(node, source)
    source.state = 'stale'
    monitor.PlanningMonitor.observe(node, source)
    assert calls == ['plan']
    source.state = 'tracking'
    source.header.stamp.sec = 6
    monitor.PlanningMonitor.observe(node, source)
    source.state, source.reason = 'lost', 'VO lost'
    monitor.PlanningMonitor.observe(node, source)
    assert calls == ['plan', 'plan', 'lost: VO lost']


def test_selected_line_waits_for_its_writer_and_a_fresh_pose(monkeypatch):
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    identity = dict(map_id='course', map_sha256='a' * 64, hdmap_sha256='b' * 64)
    def lane(y):
        return dict(points=[[0, y], [3, y]], profile=[[0, 0, y, 0, 0, .5, 0], [3, 3, y, 0, 0, .5, 0]])
    doc = dict(frame='map', closed=False, lines=dict(centerline=lane(0.), raceline=lane(.15)))
    monkeypatch.setattr(monitor, 'map_metadata', lambda *_: identity)
    monkeypatch.setattr(monitor, 'verified_hdmap', lambda *_: json.dumps(doc).encode())
    context = Context()
    rclpy.init(context=context, domain_id=110 + os.getpid() % 45)
    node = monitor.PlanningMonitor(context=context, parameter_overrides=[
        Parameter('map_id', value='course'), Parameter('reference_line_topic', value='/vehicle/drive_tuning')])
    driver = Node('atd_auto_drive', context=context)
    executor = SingleThreadedExecutor(context=context)
    for item in (driver, node):
        executor.add_node(item)

    def spin_until(predicate):
        end = time.monotonic() + 5.
        while time.monotonic() < end and not predicate():
            executor.spin_once(timeout_sec=.02)
        assert predicate()

    try:
        route = driver.create_publisher(String, '/vehicle/drive_tuning',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        poses = driver.create_publisher(LocalizationStatus, '/localization/status', 10)
        received = []
        driver.create_subscription(PlanningStatus, '/planning/status', received.append, 10)
        spin_until(lambda: poses.get_subscription_count() and node.publisher.get_subscription_count())
        source = LocalizationStatus(**identity, session_id='test', state='tracking', speed_mps=.2)
        source.header.frame_id, source.pose.orientation.w = 'map', 1.
        source.header.stamp = driver.get_clock().now().to_msg()
        poses.publish(source)
        spin_until(lambda: any(p.valid and p.line == 'raceline' for p in received))
        old_pose = driver.get_clock().now()
        route.publish(String(data=json.dumps(dict(identity, line='centerline', stamp_ns=old_pose.nanoseconds))))
        spin_until(lambda: node.line == 'centerline')
        source.header.stamp = old_pose.to_msg()
        poses.publish(source)
        spin_until(lambda: node.last_observation[-1] == old_pose.nanoseconds)
        source.header.stamp = driver.get_clock().now().to_msg()
        poses.publish(source)
        spin_until(lambda: any(p.valid and p.line == 'centerline' for p in received))
        new_plans = [p for p in received if p.valid and p.line == 'centerline']
        assert all(p.header.stamp == source.header.stamp for p in new_plans)
        assert new_plans[-1].trajectory[0].y == 0.
    finally:
        executor.shutdown()
        driver.destroy_node()
        node.destroy_node()
        rclpy.shutdown(context=context)

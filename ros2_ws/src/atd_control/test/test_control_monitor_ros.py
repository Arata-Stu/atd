"""ROS を使用できる環境で実メッセージの購読・期限切れ・復帰を確認する。"""
import os
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from atd_control.monitor import ControlMonitor
from atd_interfaces.msg import ControlStatus, PlanningStatus
from atd_vehicle_layout.profile import default_profile, save
from geometry_msgs.msg import Point
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


def test_pure_pursuit_observation_invalidation_and_recovery(tmp_path):
    profile = default_profile()
    profile['dimensions']['wheelbase'] = dict(value_m=.25, source='manual')
    path = tmp_path / 'layout.yaml'
    save(path, profile)
    context = Context()
    rclpy.init(context=context, domain_id=156 + os.getpid() % 40)
    observer = ControlMonitor(context=context, parameter_overrides=[Parameter('profile', value=str(path))])
    inputs = Node('control_validation', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    executor.add_node(inputs)
    try:
        received = []
        inputs.create_subscription(ControlStatus, '/control/status', received.append, 10)
        plans = inputs.create_publisher(PlanningStatus, '/planning/status', 10)

        def wait(predicate):
            deadline = time.monotonic() + 5
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            assert predicate()

        wait(lambda: plans.get_subscription_count() and observer.publisher.get_subscription_count())
        plan = PlanningStatus(map_id='map-a', map_sha256='a' * 64, hdmap_sha256='b' * 64,
                              session_id='manual', epoch=1, line='custom', valid=True, validity_s=.5,
                              estimated_speed_mps=1., trajectory=[Point(x=1.), Point(x=4.)])
        plan.header.frame_id = 'map'
        plan.pose.position.x, plan.pose.position.y, plan.pose.orientation.w = 1., .1, 1.

        def send():
            plan.header.stamp = inputs.get_clock().now().to_msg()
            plans.publish(plan)

        send()
        wait(lambda: received and received[-1].valid)
        msg = received[-1]
        assert msg.header == plan.header and msg.pose == plan.pose and msg.line == plan.line
        assert msg.epoch == plan.epoch and msg.session_id == plan.session_id
        assert msg.has_steering and msg.wheelbase_m == .25 and msg.steering_rad < 0
        assert msg.actual_lookahead_m == pytest.approx(.75) and msg.target.x > 1.

        wait(lambda: not received[-1].valid)
        assert not received[-1].has_steering
        expired_stamp = plan.header.stamp
        plans.publish(plan)
        send()
        wait(lambda: received[-1].valid and received[-1].header.stamp != expired_stamp)
        plan.valid, plan.reason = False, '追跡喪失'
        send()
        wait(lambda: received[-1].reason == plan.reason)
        assert not received[-1].valid
    finally:
        executor.shutdown()
        inputs.destroy_node()
        observer.destroy_node()
        rclpy.shutdown(context=context)

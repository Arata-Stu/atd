"""Jetson の2つの制御器をソフトウェアのみで起動して設定の切替を検証する。"""
import json
import os
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from atd_control.drive_tuning import DEFAULT_DRIVE_TUNING
from atd_control.monitor import ControlMonitor
from atd_control.speed_monitor import SpeedMonitor
from atd_interfaces.msg import PlanningStatus, TunedControlStatus, TunedSpeedControlStatus
from geometry_msgs.msg import Point
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def test_tuned_outputs_share_a_revision_and_stop_on_invalid_plan(tmp_path):
    ctx = Context()
    rclpy.init(context=ctx, domain_id=101 + os.getpid() % 40)
    executor = SingleThreadedExecutor(context=ctx)
    owner = Node('atd_auto_drive', context=ctx)
    limits = [Parameter('drive_tuning', value=True), Parameter('max_target_speed_mps', value=.5)]
    table = tmp_path / 'table.json'
    table.write_text(json.dumps(dict(schema='atd.throttle_feedforward.v1', input_unit='m/s', output_unit='sbus_raw_ch2',
        usable=True, direction='increasing', interpolation='linear', extrapolation='none', run_id='test',
        proposal_sha256='a'*64, speed_min_mps=.2, speed_max_mps=1.,
        points=[dict(speed_mps=.2, throttle_raw=1100), dict(speed_mps=1., throttle_raw=1200)])))
    steering = ControlMonitor(context=ctx, parameter_overrides=limits + [Parameter('profile', value=str(tmp_path/'layout.yaml'))])
    speed = SpeedMonitor(context=ctx, parameter_overrides=limits + [Parameter('feedforward_path', value=str(table))])
    for node in (owner, steering, speed):
        executor.add_node(node)

    def wait(condition):
        deadline = time.monotonic() + 5.
        while not condition():
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=.01)

    try:
        identity = dict(map_id='odaiba', map_sha256='a'*64, hdmap_sha256='b'*64)
        config = owner.create_publisher(String, '/vehicle/drive_tuning',
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        plans = owner.create_publisher(PlanningStatus, '/planning/status', 10)
        lateral, longitudinal = [], []
        owner.create_subscription(TunedControlStatus, '/control/tuned_steering', lateral.append, 10)
        owner.create_subscription(TunedSpeedControlStatus, '/control/tuned_speed', longitudinal.append, 10)
        wait(lambda: plans.get_subscription_count() == 2 and config.get_subscription_count() == 2
             and steering.tuned.publisher.get_subscription_count() and speed.tuned.publisher.get_subscription_count())
        plan = PlanningStatus(**identity, session_id='track', line='raceline', valid=True, validity_s=.5,
                              target_speed_mps=.8, estimated_speed_mps=.25, trajectory=[Point(), Point(x=2.)])
        plan.header.frame_id, plan.pose.orientation.w = 'map', 1.

        def select(line, cap, ahead):
            cut = owner.get_clock().now().nanoseconds
            config.publish(String(data=json.dumps(DEFAULT_DRIVE_TUNING.values() | identity | dict(
                stamp_ns=cut, line=line, max_speed_mps=cap, min_lookahead_m=ahead, lookahead_time_s=0.))))
            wait(lambda: steering.tuned.stamp_ns == speed.tuned.stamp_ns == cut)
            return cut

        def observe(line):
            plan.line = line
            plan.header.stamp = owner.get_clock().now().to_msg()
            plans.publish(plan)
            wait(lambda: all(rows and rows[-1].status.header == plan.header for rows in (lateral, longitudinal)))

        first = select('raceline', .5, .4)
        observe('raceline')
        assert lateral[-1].tuning_stamp_ns == longitudinal[-1].tuning_stamp_ns == first
        assert lateral[-1].status.requested_lookahead_m == .4 and longitudinal[-1].status.target_speed_mps == .5
        second = select('centerline', .3, .22)
        plans.publish(plan)  # 変更前の観測を再送する。
        observe('centerline')
        assert lateral[-1].status.requested_lookahead_m == .22 and longitudinal[-1].status.target_speed_mps == .3
        for rows in (lateral, longitudinal):
            assert rows[-1].tuning_stamp_ns == second and rows[-1].status.valid
            assert not any(m.tuning_stamp_ns == second and m.status.line == 'raceline' for m in rows)
        plan.valid, plan.reason = False, '追跡喪失'
        plans.publish(plan)  # 有効な計画と同じ画像時刻の無効化。
        wait(lambda: all(rows[-1].status.reason == '追跡喪失' for rows in (lateral, longitudinal)))
        assert not lateral[-1].status.valid and not longitudinal[-1].status.valid
    finally:
        executor.shutdown()
        for node in (speed, steering, owner):
            node.destroy_node()
        rclpy.shutdown(context=ctx)

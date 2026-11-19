import json
import os
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from atd_control.speed_monitor import SpeedMonitor
from atd_interfaces.msg import DriveCommand, PicoTelemetry, PlanningStatus, SpeedControlStatus
from atd_interfaces.srv import SetSpeedTable
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


def test_autonomous_speed_cap_and_startup_table(tmp_path):
    path = tmp_path / 'forward.json'
    table = dict(schema='atd.throttle_feedforward.v1', input_unit='m/s', output_unit='sbus_raw_ch2',
                 usable=True, direction='decreasing', interpolation='linear', extrapolation='none',
                 run_id='low-speed', proposal_sha256='a' * 64, speed_min_mps=.2, speed_max_mps=.8,
                 points=[dict(speed_mps=.2, throttle_raw=950), dict(speed_mps=.8, throttle_raw=830)])
    path.write_text(json.dumps(table))
    ctx = Context()
    rclpy.init(context=ctx, domain_id=101 + os.getpid() % 40)
    node = SpeedMonitor(context=ctx, parameter_overrides=[Parameter('feedforward_path', value=str(path)),
                        Parameter('max_target_speed_mps', value=.5)])
    source = Node('atd_auto_drive', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    executor.add_node(source)
    try:
        results = []
        source.create_subscription(SpeedControlStatus, '/control/speed_status', results.append, 10)
        pub = source.create_publisher(PlanningStatus, '/planning/status', 10)
        authority = source.create_publisher(String, '/vehicle/auto_drive/status', 10)
        assert '/control/set_speed_table' not in [svc for svc, _ in node.get_service_names_and_types_by_node(
            node.get_name(), '/')]
        # 実行中にファイルを置換しても自動走行セッションへは採用しない。
        path.write_text('{}')
        node.next_check = 0
        node.watch()
        deadline = time.monotonic() + 5
        while not (pub.get_subscription_count() and node.publisher.get_subscription_count()
                   and authority.get_subscription_count()):
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=.01)
        plan = PlanningStatus(map_id='course', map_sha256='b'*64, hdmap_sha256='c'*64, session_id='track',
                              epoch=1, line='raceline', validity_s=.4, valid=True,
                              target_speed_mps=1.5, estimated_speed_mps=.25)
        plan.header.frame_id = 'map'
        plan.header.stamp = source.get_clock().now().to_msg()
        pub.publish(plan)
        while not any(r.valid for r in results):
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=.01)
        result = next(r for r in results if r.valid)
        assert result.target_speed_mps == .5 and result.speed_error_mps == pytest.approx(.25)
        assert result.p_raw == pytest.approx(-10.) and result.feedforward_raw == pytest.approx(890.)
        assert result.calculated_raw == pytest.approx(890.) and result.saturated

        def observe():
            plan.header.stamp = source.get_clock().now().to_msg()
            pub.publish(plan)
            deadline = time.monotonic() + 2
            while not (results and results[-1].header == plan.header and results[-1].valid):
                assert time.monotonic() < deadline
                executor.spin_once(timeout_sec=.01)
            return results[-1]

        def active(value):
            stamp = time.time_ns()
            authority.publish(String(data=json.dumps(dict(session_id='auto-node', stamp_ns=stamp, map_id='course',
                                                          line='raceline', active=value, braking=False))))
            deadline = time.monotonic() + 2
            while node.drive_feedback.stamp != stamp:
                assert time.monotonic() < deadline
                executor.spin_once(timeout_sec=.01)

        plan.target_speed_mps, plan.estimated_speed_mps = .3, .29
        assert observe().i_raw == observe().i_raw == 0.
        active(True)
        assert observe().i_raw == 0. and observe().i_raw < 0
        active(False)
        assert observe().i_raw == observe().i_raw == 0.
        plan.estimated_speed_mps = -.005
        stopped = observe()
        assert stopped.estimated_speed_mps == -.005 and stopped.p_raw == pytest.approx(-12.)
    finally:
        executor.shutdown()
        source.destroy_node()
        node.destroy_node()
        rclpy.shutdown(context=ctx)


def test_matched_manual_and_ff_selection_without_a_vehicle_publisher(tmp_path):
    path = tmp_path / 'vehicle' / 'throttle.json'
    ctx = Context()
    rclpy.init(context=ctx, domain_id=101 + os.getpid() % 40)
    node = SpeedMonitor(context=ctx, parameter_overrides=[Parameter('feedforward_path', value=str(path))])
    source = Node('speed_test_input', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    executor.add_node(source)

    def wait(condition):
        limit = time.monotonic() + 5
        while not condition() and time.monotonic() < limit:
            executor.spin_once(timeout_sec=.01)
        assert condition()

    try:
        results = []
        source.create_subscription(SpeedControlStatus, '/control/speed_status', results.append, 10)
        plans = source.create_publisher(PlanningStatus, '/planning/status', 10)
        telemetry = source.create_publisher(PicoTelemetry, '/vehicle/pico/telemetry', 10)
        teleop = source.create_publisher(DriveCommand, '/teleop/command', 10)
        client = source.create_client(SetSpeedTable, '/control/set_speed_table')
        wait(lambda: node.publisher.get_subscription_count() and plans.get_subscription_count()
             and telemetry.get_subscription_count() and teleop.get_subscription_count() and client.service_is_ready())
        plan = PlanningStatus(map_id='a', map_sha256='a' * 64, hdmap_sha256='b' * 64, session_id='manual',
                              epoch=2, line='custom', validity_s=.5, valid=True,
                              target_speed_mps=1., estimated_speed_mps=.8)
        plan.header.frame_id = 'map'
        ch2 = PicoTelemetry(instance_id='one', serial='rev6', active_path=1, rc_valid=True, power_ready=True,
                            sbus_fresh=True, channels=[992, 1321] + [992] * 14)
        ch2.header.frame_id = 'pico'

        def send(joy=None):
            ch2.header.stamp = source.get_clock().now().to_msg()
            if joy is not None:
                joy.header.stamp = ch2.header.stamp
                prior = len(node.joy)
                teleop.publish(joy)
                wait(lambda: len(node.joy) > prior)
            prior = len(node.pico)
            telemetry.publish(ch2)
            wait(lambda: len(node.pico) > prior)
            plan.header.stamp = ch2.header.stamp
            plans.publish(plan)
            wait(lambda: results and results[-1].header == plan.header and results[-1].has_speeds)
            return results[-1]

        awaiting = send()
        assert not awaiting.valid and awaiting.has_propo and awaiting.propo_raw == 1321
        assert not awaiting.has_joy and awaiting.manual_source == 'propo'
        doc = dict(schema='atd.throttle_feedforward.v1', input_unit='m/s', output_unit='sbus_raw_ch2',
                   usable=True, direction='increasing', interpolation='linear', extrapolation='none',
                   run_id='cal-a', proposal_sha256='c' * 64, speed_min_mps=.5, speed_max_mps=1.5,
                   points=[dict(speed_mps=.5, throttle_raw=1100), dict(speed_mps=1.5, throttle_raw=1300)])
        future = client.call_async(SetSpeedTable.Request(table_json=json.dumps(doc)))
        wait(future.done)
        assert future.result().saved and json.loads(path.read_text()) == doc
        joy = DriveCommand(source='joy', throttle=-.75, valid=True, enable_requested=False)
        computed = send(joy)
        assert computed.valid and (computed.calibration_run_id, computed.proposal_sha256) == ('cal-a', 'c' * 64)
        assert computed.feedforward_raw == 1200 and computed.calculated_raw == pytest.approx(1208)
        assert computed.has_propo and computed.propo_age_s == 0. and computed.propo_raw == 1321
        assert computed.manual_source == 'propo' and computed.has_joy and computed.joy_throttle == -.75
        ch2.active_path = 2
        joy.throttle = 0.
        host = send(joy)
        assert not host.has_propo and host.manual_source == 'host_joy' and host.has_joy
        assert host.joy_age_s == 0. and host.joy_throttle == 0. and not host.joy_enable_requested
        # Configuration invalidates the held source stamp at the time of the notification.
        joy.valid = False
        prior = len(node.joy)
        teleop.publish(joy)
        wait(lambda: len(node.joy) > prior)
        unavailable = send()
        assert not unavailable.has_joy and unavailable.joy_reason and unavailable.manual_source == 'host'
        plan.estimated_speed_mps = -.1
        reverse = send()
        assert not reverse.valid and reverse.has_speeds and reverse.estimated_speed_mps == -.1
        wait(lambda: results and not results[-1].has_speeds and not results[-1].valid)
        pubs = [name for name, _ in source.get_publisher_names_and_types_by_node(node.get_name(), '/')]
        assert '/control/speed_status' in pubs
        assert all(name in ('/control/speed_status', '/rosout', '/parameter_events') for name in pubs)
    finally:
        executor.shutdown()
        source.destroy_node()
        node.destroy_node()
        rclpy.shutdown(context=ctx)

"""Gazebo 制御での同一画像時刻の照合・速度制御・停止ラッチ。"""
import math
import os
from pathlib import Path
import runpy
import sys
import time
from types import SimpleNamespace as Message

import pytest

for package in ('atd_bringup', 'atd_control', 'atd_mapping', 'atd_planning'):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / package))

from atd_bringup.gazebo_control import COMMAND_TTL_NS, GazeboController, default_min_lookahead, simulation_table


ROUTE = ('odaiba', 'a' * 64, 'b' * 64)
START = 15_000_000_000


def inputs(stamp=START, y=0., speed=0., target=1.5):
    sec, ns = divmod(stamp, 10**9)
    shared = dict(map_id=ROUTE[0], map_sha256=ROUTE[1], hdmap_sha256=ROUTE[2], session_id='localize', epoch=1,
                  header=Message(frame_id='map', stamp=Message(sec=sec, nanosec=ns)), reason='')
    loc = Message(**shared, state='tracking', speed_mps=speed, pose=Message(
        position=Message(x=0., y=y, z=0.), orientation=Message(x=0., y=0., z=0., w=1.)))
    plan = Message(**shared, valid=True, line='raceline', validity_s=.5, lateral_error_m=y, estimated_speed_mps=speed,
                   target_speed_mps=target, trajectory=[Message(x=0., y=0.), Message(x=3., y=0.)])
    return loc, plan


def gate():
    return GazeboController(ROUTE, 'raceline', 1.5, simulation_table())


def test_gazebo_log_keeps_source_stamps_and_distinguishes_unavailable_pose_and_timing():
    recorder = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts/gazebo_closed_loop_log'))
    loc, _ = inputs(y=2.)
    ref = dict(frame='map', map_id=ROUTE[0], map_sha256=ROUTE[1], hdmap_sha256=ROUTE[2])
    row = recorder['localization_row'](loc, START + 50_000_000, ref)
    assert (row['image_stamp_ns'], row['received_sim_ns'], row['y_m']) == (START, START + 50_000_000, 2.)
    assert row['orientation_xyzw'] == [0., 0., 0., 1.]
    assert recorder['localization_row'](loc, START, dict(ref, hdmap_sha256='wrong'))['state'] == 'invalid'
    loc.state, loc.pose = 'lost', None
    lost = recorder['localization_row'](loc, START + 50_000_000, ref)
    assert lost['image_stamp_ns'] is None and 'x_m' not in lost and lost['header_stamp_ns'] == START
    status = Message(header=loc.header, vo_state=2, track_execution_time=.002, node_callback_execution_time=math.nan)
    assert recorder['vslam_row'](status) == dict(image_stamp_ns=START, vo_state=2,
                                                track_execution_time_s=.002, node_callback_execution_time_s=None)


def run(c, stamp=START, **kwargs):
    loc, plan = inputs(stamp, **kwargs)
    assert c.observe('planning', plan, stamp) is None
    return c.observe('localization', loc, stamp)


def test_stamp_matching_ackermann_and_real_pid():
    c = gate()
    stamp, angle, throttle = run(c, y=.06, speed=-.02, target=3.)
    assert c.state == 'driving' and stamp == START and angle < 0
    assert throttle == pytest.approx(((992 + 1.5 * 250 + 40 * 1.5) - 992) / 800.)
    assert c.ready(START) is None
    # Integral and derivative receive consecutive estimates, despite forward-only deadband at standstill.
    assert run(c, START + 100_000_000, y=-.06, speed=.3)[1] > 0
    assert c.pid.last['i_raw'] > 0 and c.pid.last['d_raw'] < 0


def test_clock_trailing_frame_buffers_and_never_drives_on_misaligned_stamps():
    c = gate()
    loc, plan = inputs()
    c.observe('localization', loc, START - 1)
    assert c.observe('planning', plan, START - 1) is None
    assert c.ready(START)[0] == START
    newer, _ = inputs(START + 1)
    assert c.observe('localization', newer, START + 1) is None


def test_delayed_plan_can_match_a_valid_90hz_pose():
    c = gate()
    for frame in range(21):
        now = START + frame * 10**9 // 90
        loc, _ = inputs(now)
        assert c.observe('localization', loc, now) is None
    _, plan = inputs(START)
    assert c.observe('planning', plan, now)[0] == START


@pytest.mark.parametrize('condition', ('loss', 'invalid_plan', 'wrong_map', 'wrong_line', 'new_session',
                                      'invalid_speed', 'invalid_offset', 'rewind'))
def test_invalid_inputs_latch_stop_until_new_run(condition):
    c = gate()
    run(c)
    now = START + 100_000_000
    loc, plan = inputs(now)
    if condition == 'loss':
        loc.state = 'lost'
        c.observe('localization', loc, now)
    elif condition == 'invalid_plan':
        plan.valid = False
        c.observe('planning', plan, now)
    elif condition == 'wrong_map':
        loc.hdmap_sha256 = '0' * 64
        c.observe('localization', loc, now)
    elif condition == 'wrong_line':
        plan.line = 'centerline'
        c.observe('planning', plan, now)
    elif condition == 'new_session':
        loc.epoch += 1
        c.observe('localization', loc, now)
    elif condition == 'invalid_speed':
        loc.speed_mps = math.nan
        c.observe('planning', plan, now)
        c.observe('localization', loc, now)
    elif condition == 'invalid_offset':
        plan.lateral_error_m = math.nan
        c.observe('planning', plan, now)
        c.observe('localization', loc, now)
    elif condition == 'rewind':
        c.tick(START - 1)
    assert c.state == 'stopped' and c.reason
    assert run(c, now + 1) is None


def test_missing_tf_update_waits_then_resumes_with_new_matched_plan_and_pose():
    c = gate()
    run(c)
    c.tick(START + COMMAND_TTL_NS)
    assert c.state == 'waiting_observation' and c.observation is None
    now = START + 2_000_000_000
    for stamp in (START, START + 1):
        loc, plan = inputs(stamp)
        assert c.observe('localization', loc, now) is None
        assert c.observe('planning', plan, now) is None
    assert c.state == 'waiting_observation'
    loc, plan = inputs(now, speed=.2)
    c.observe('planning', plan, now)
    assert c.observe('localization', loc, now)[0] == now
    assert c.state == 'driving' and c.pid.last['i_raw'] == 0.


def test_explicit_loss_while_waiting_is_terminal():
    c = gate()
    run(c)
    loc, plan = inputs(START + 10)
    c.observe('planning', plan, START + 10)
    loc.state = 'stale'
    c.observe('localization', loc, START + 11)
    assert c.state == 'waiting_observation' and not c.plans
    loc.state, loc.reason = 'lost', '追跡喪失'
    c.observe('localization', loc, START + 12)
    assert c.state == 'stopped' and c.reason == '追跡喪失'
    assert run(c, START + 20) is None


def test_requires_standstill_and_forward_table():
    c = gate()
    assert run(c, speed=.23) is None and c.state == 'waiting'
    assert run(c, START + 10_000_000) is not None
    with pytest.raises(ValueError):
        GazeboController(ROUTE, 'raceline', 4., simulation_table())


@pytest.mark.parametrize('counts', [(0, 1), (1, 0), (2, 1), (1, 2)])
def test_missing_or_ambiguous_publisher_clears_buffer_and_inhibits_driving(counts):
    c = gate()
    loc, plan = inputs()
    c.observe('localization', loc, START)
    assert not c.check_sources(*counts) and c.state == 'waiting'
    assert c.check_sources(1, 1)
    assert c.observe('planning', plan, START) is None
    assert c.observe('localization', loc, START) is not None
    assert not c.check_sources(*counts) and c.state == 'stopped'
    assert c.ready(START) is None and c.observation is None
    assert c.check_sources(1, 1) and run(c, START + 1) is None


def test_raceline_acquisition_lookahead_and_unreachable_start_reason():
    assert default_min_lookahead('centerline') == .35
    c = gate()
    assert c.lookahead.min_lookahead_m == .65
    assert run(c, y=1.) is None and c.state == 'waiting' and '0.65 m' in c.reason
    assert run(c, y=.49) is not None


def test_ros_actuator_goes_neutral_when_the_planner_disconnects():
    rclpy = pytest.importorskip('rclpy')
    from atd_interfaces.msg import LocalizationStatus, PlanningStatus
    from geometry_msgs.msg import Point
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState

    executable = Path(__file__).resolve().parents[1] / 'scripts/gazebo_control'
    control_node = runpy.run_path(str(executable))['GazeboControlNode']
    keys = dict(zip(('map_id', 'map_sha256', 'hdmap_sha256'), ROUTE))
    parameters = [Parameter(k, value=v) for k, v in dict(keys, use_sim_time=True).items()]
    context = Context()
    rclpy.init(context=context, domain_id=160 + os.getpid() % 30)
    controller = control_node(context=context, parameter_overrides=parameters)
    driver = Node('gazebo_control_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    for node in (controller, driver):
        executor.add_node(node)

    def wait(predicate):
        end = time.monotonic() + 5.
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
        assert predicate()

    try:
        commands = []
        subscription = driver.create_subscription(JointState, '/atd/gazebo/actuators', commands.append, 10)
        poses = driver.create_publisher(LocalizationStatus, '/localization/status', 10)
        plans = driver.create_publisher(PlanningStatus, '/planning/status', 10)
        clock = driver.create_publisher(Clock, '/clock', 10)
        wait(lambda: subscription.get_publisher_count() == 1 and all(
            p.get_subscription_count() for p in (clock, poses, plans)))
        tick = Clock()
        tick.clock.sec = 15
        clock.publish(tick)
        wait(lambda: controller.get_clock().now().nanoseconds == START)
        pose = LocalizationStatus(**keys, session_id='run', state='tracking')
        pose.header.frame_id, pose.header.stamp = 'map', tick.clock
        pose.pose.orientation.w = 1.
        plan = PlanningStatus(**keys, session_id='run', valid=True, line='raceline',
                              trajectory=[Point(), Point(x=3.)], target_speed_mps=1.5, validity_s=.5)
        plan.header = pose.header
        poses.publish(pose)
        plans.publish(plan)
        wait(lambda: commands and commands[-1].position[-1] == 1.)
        assert commands[-1].name == ['steering', 'throttle', 'enable']
        assert commands[-1].header.stamp == tick.clock and commands[-1].position[1] > 0.
        driver.destroy_publisher(plans)
        wait(lambda: controller.observers[1].get_publisher_count() == 0)
        tick.clock.nanosec = 100_000_000
        clock.publish(tick)
        wait(lambda: list(commands[-1].position) == [0., 0., 0.])
        assert controller.controller.state == 'stopped'
    finally:
        executor.shutdown()
        driver.destroy_node()
        controller.destroy_node()
        rclpy.shutdown(context=context)

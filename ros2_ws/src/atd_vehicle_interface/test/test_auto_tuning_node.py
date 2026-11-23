"""ROS の通信だけを置き換え、実車ノードの設定・受信コールバックを検証する。"""
import json
import os
import runpy
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from atd_control.drive_tuning import DEFAULT_DRIVE_TUNING
from atd_control.speed import FeedforwardTable


@pytest.fixture
def driver():
    modules = {
        'rclpy': NS(), 'rclpy.node': NS(Node=object), 'rclpy.clock': NS(Clock=object, ClockType=object),
        'rclpy.executors': NS(ExternalShutdownException=Exception),
        'rclpy.qos': NS(DurabilityPolicy=object, QoSProfile=object), 'std_msgs.msg': NS(String=NS),
        'rcl_interfaces.msg': NS(FloatingPointRange=NS, ParameterDescriptor=NS, SetParametersResult=NS),
        'atd_interfaces.msg': NS(ControlStatus=NS, LocalizationStatus=NS, SpeedControlStatus=NS),
    }
    with patch.dict(sys.modules, modules):
        path = Path(__file__).resolve().parents[1] / 'atd_vehicle_interface' / 'auto_drive_node.py'
        cls = runpy.run_path(str(path), run_name='atd_vehicle_interface._test_node')['AutoDriveNode']
    node = cls.__new__(cls)
    node.drive_tuning = True
    node.tuning = DEFAULT_DRIVE_TUNING
    node.tuning_stamp_ns = 10
    node.gate = Mock(speed_ceiling=.5, host_since=None)
    node.horizon = 2.
    node.available = ('raceline', 'centerline')
    node.map_identity = dict(map_id='odaiba', map_sha256='1'*64, hdmap_sha256='2'*64)
    node.calibration = NS(table=FeedforwardTable('run', 'a'*64, (.2, .7), (1100., 1300.), 1))
    node.tuning_pub = Mock()
    node.get_logger = Mock()
    return node


def parameters(**updates):
    return [NS(name=name, value=value) for name, value in updates.items()]


def test_atomic_validation_changes_no_active_values_until_commit(driver):
    for changes in ({'line': 'custom'}, {'max_speed_mps': .51}, {'max_speed_mps': .19},
                    {'max_lookahead_m': 2.1}, {'min_lookahead_m': 1.6}, {'speed_kp': -1.}):
        batch = parameters(**({'lookahead_time_s': .2} | changes))
        result = driver.validate_parameters(batch)
        assert not result.successful and result.reason

    batch = parameters(line='centerline', min_lookahead_m=1.6, max_lookahead_m=1.7, speed_kd=0., max_speed_mps=.4)
    assert driver.validate_parameters(batch).successful
    assert driver.tuning == DEFAULT_DRIVE_TUNING
    driver.tuning_pub.publish.assert_not_called()
    driver.gate.retune.assert_not_called()
    driver.apply_parameters(batch)
    stamp = driver.tuning_stamp_ns
    assert stamp > 10
    expected = DEFAULT_DRIVE_TUNING.values() | {p.name: p.value for p in batch}
    assert driver.tuning.values() == expected
    driver.gate.retune.assert_called_once_with('centerline', .4, stamp)
    driver.tuning_pub.publish.assert_called_once()
    payload = json.loads(driver.tuning_pub.publish.call_args.args[0].data)
    assert payload == driver.map_identity | expected | {'stamp_ns': stamp}
    # 同じ設定を再送しても新しい版を発行しない。
    driver.apply_parameters(batch)
    driver.tuning_pub.publish.assert_called_once()
    assert driver.tuning_stamp_ns == stamp


@pytest.mark.parametrize('kind', ['speed', 'steering'])
def test_accepts_only_current_controller_and_tuning_revision_from_matching_map(driver, kind):
    driver.sources = {kind: b'current'}
    source = {'publisher_gid': {'data': b'current'}}
    good = NS(**driver.map_identity)
    stamp = driver.tuning_stamp_ns
    for previous_or_future in (stamp - 1, stamp + 1):
        driver.receive(kind, NS(tuning_stamp_ns=previous_or_future, status=good), source)
    driver.gate.receive.assert_not_called()
    driver.gate.invalidate.assert_not_called()
    wrong_map = NS(**(driver.map_identity | {'hdmap_sha256': '3'*64}))
    driver.receive(kind, NS(tuning_stamp_ns=stamp, status=wrong_map), source)
    driver.gate.invalidate.assert_called_once()
    driver.gate.receive.assert_not_called()
    msg = NS(tuning_stamp_ns=stamp, status=good)
    driver.receive(kind, msg, {'publisher_gid': {'data': b'old'}})
    driver.gate.speeds.clear.assert_called_once()
    driver.gate.receive.assert_not_called()
    driver.receive(kind, msg, source)
    driver.gate.receive.assert_called_once()
    assert driver.gate.receive.call_args.args[:2] == (kind, good)


def test_ros_atomic_service_keeps_invalid_batch_and_startup_limits_out_of_effect(monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from atd_vehicle_interface import auto_drive_node

    identity = dict(map_id='odaiba', map_sha256='1'*64, hdmap_sha256='2'*64)
    points = [[0., 0.], [1., 0.]]
    reference = dict(points=points, profile=[[x, x, y, 0., 0., .6, 0.] for x, y in points])
    hdmap = dict(frame='map', closed=False, lines=dict(raceline=reference, centerline=reference, custom=dict(points=points)))
    monkeypatch.setattr(auto_drive_node, 'map_metadata', lambda *_: identity)
    monkeypatch.setattr(auto_drive_node, 'verified_hdmap', lambda *_: json.dumps(hdmap).encode())
    table = FeedforwardTable('run', 'a'*64, (.2, .7), (1100., 1300.), 1)
    monkeypatch.setattr(auto_drive_node, 'load_calibration', lambda *_: NS(table=table))
    ctx = Context()
    rclpy.init(context=ctx, domain_id=65 + os.getpid() % 20)
    node = None
    try:
        node = auto_drive_node.AutoDriveNode(context=ctx, parameter_overrides=[
            Parameter('map_id', value='odaiba'), Parameter('drive_tuning', value=True)])
        assert set(node.get_parameter('available_lines').value) == {'centerline', 'raceline'}
        assert node.get_parameter('launch_speed_cap_mps').value == .5
        assert node.describe_parameter('launch_speed_cap_mps').read_only
        first = node.tuning_stamp_ns
        line = Parameter('line', value='centerline')
        result = node.set_parameters_atomically([line, Parameter('max_lookahead_m', value=20.)])
        assert not result.successful
        assert node.get_parameter('line').value == node.gate.line == 'raceline' and node.tuning_stamp_ns == first
        assert node.set_parameters_atomically([line, Parameter('max_speed_mps', value=.4)]).successful
        assert node.get_parameter('line').value == node.gate.line == 'centerline'
        assert node.get_parameter('max_speed_mps').value == node.gate.max_speed == .4
        assert node.tuning_stamp_ns > first
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown(context=ctx)

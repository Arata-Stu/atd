import json
from types import SimpleNamespace
from threading import RLock
from unittest.mock import Mock, patch

import pytest

from atd_web_bridge.core import Cache
from atd_web_bridge.gazebo import GazeboDriveWindow
from test_propo_routes import load_bridge_module


def report(t, state='driving', **updates):
    return dict(dict(stamp_ns=t, state=state, map_id='odaiba', line='centerline', reason='',
                     steering_rad=-.15, throttle=.34, estimated_speed_mps=.8, target_speed_mps=1.2), **updates)


def test_history_uses_sim_time_and_separates_command_values_from_missing_readings():
    window = GazeboDriveWindow(2.5)
    with patch('atd_web_bridge.gazebo.time.monotonic', return_value=100.):
        for sec in range(20):
            window.receive(report(sec * 10**9), 'sim')
        snapshot = window.snapshot(19., 'sim')
        assert len(snapshot['history']) == 16
        row = snapshot['current']
        assert row['stamp_s'] == 19. and row['steering_rad'] == -.15 and row['target_mps'] == 1.2
        assert window.snapshot(19., 'other')['current'] is None
        window.receive(report(20 * 10**9, 'stopped', reason='追跡を失いました'), 'sim')
        row = window.snapshot(20., 'sim')['current']
        assert row['reason'] == '追跡を失いました' and row['throttle'] is None
    with patch('atd_web_bridge.gazebo.time.monotonic', return_value=104.):
        assert window.snapshot(20., 'sim')['current'] is None


def test_simulation_rejects_hardware_operations_and_tuning_for_a_previous_run():
    bridge = load_bridge_module().BridgeNode
    node = object.__new__(bridge)
    node.identity = dict(environment='gazebo', instance_id='sim-1')
    node.operation_lock = RLock()
    node.apply_parameters = Mock(return_value={'applied': True})
    with pytest.raises(ValueError, match='Gazebo bridge'):
        node.dispatch('/vehicle/profile', 'PUT', {})
    values = dict(node='/atd_gazebo_control', values={'max_speed_mps': .5})
    with pytest.raises(ValueError, match='試行'):
        node.dispatch('/live/parameters', 'POST', dict(values, instance_id='old'))
    node.apply_parameters.assert_not_called()
    assert node.dispatch('/live/parameters', 'POST', dict(values, instance_id='sim-1'))['applied']
    node.settings = {'parameter_node_prefix': '/atd_'}
    node.get_node_names_and_namespaces = Mock(return_value=[('atd_gazebo_control', '/'), ('atd_other', '/')])
    assert node.nodes() == ['/atd_gazebo_control']
    node.get_node_names_and_namespaces.return_value.append(('atd_gazebo_control', '/'))
    assert node.nodes() == []


def test_live_dispatch_serves_actual_control_state_and_rejects_an_ambiguous_publisher():
    node = object.__new__(load_bridge_module().BridgeNode)
    node.identity = dict(environment='gazebo', instance_id='one-run')
    node.operation_lock = RLock()
    node.cache = Cache()
    node.gazebo_window = GazeboDriveWindow(2.5)
    node.settings = dict(localization_topic='/localization/status', planning_topic='/planning/status',
                         gazebo_control_topic='/atd/gazebo/control_state', parameter_node_prefix='/atd_gazebo_control')
    node.get_clock = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 2 * 10**9
    node.count_publishers = Mock(return_value=0)
    node.get_node_names_and_namespaces = Mock(return_value=[('atd_gazebo_control', '/')])
    endpoint = SimpleNamespace(node_namespace='/', node_name='atd_gazebo_control', endpoint_gid=b'writer')
    node.get_publishers_info_by_topic = Mock(return_value=[endpoint])
    node.receive_gazebo(SimpleNamespace(data=json.dumps(report(2 * 10**9))), {'publisher_gid': {'data': b'writer'}})
    result = node.dispatch('/live/status', 'GET', {})
    assert result['gazebo']['current']['steering_rad'] == -.15
    assert result['parameter_nodes'] == ['/atd_gazebo_control']
    assert result['pose'] is None and 'auto_drive' not in result
    node.get_publishers_info_by_topic.return_value.append(endpoint)
    assert node.dispatch('/live/status', 'GET', {})['gazebo']['current'] is None

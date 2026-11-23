import json
from types import SimpleNamespace
from threading import RLock
from unittest.mock import Mock, patch

import pytest

from atd_web_bridge.core import Cache
from atd_web_bridge.gazebo import GazeboDriveWindow, ground_truth_sample, live_gazebo_sample, vehicle_hold_sample
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


def test_vehicle_tuning_requires_an_autonomous_bridge_unique_writer_and_current_connection():
    node = object.__new__(load_bridge_module().BridgeNode)
    node.identity = dict(environment='jetson', instance_id='jetson-1')
    node.settings = {'parameter_node_prefix': '/atd_', 'drive_tuning': False}
    node.operation_lock = RLock()
    node.get_node_names_and_namespaces = Mock(return_value=[('atd_auto_drive', '/'), ('atd_joy', '/')])
    assert node.nodes() == ['/atd_joy']
    node.settings['drive_tuning'] = True
    assert node.nodes() == ['/atd_auto_drive', '/atd_joy']
    node.apply_parameters = Mock(return_value={'applied': True})
    update = dict(node='/atd_auto_drive', values={'line': 'centerline'})
    for request in (update, dict(update, instance_id='jetson-2')):
        with pytest.raises(ValueError, match='接続先'):
            node.dispatch('/live/parameters', 'POST', request)
    node.apply_parameters.assert_not_called()
    assert node.dispatch('/live/parameters', 'POST', dict(update, instance_id='jetson-1'))['applied']
    node.get_node_names_and_namespaces.return_value.append(('atd_auto_drive', '/'))
    with pytest.raises(ValueError, match='稼働中'):
        node.client('/atd_auto_drive')


@pytest.fixture
def live_bridge():
    node = object.__new__(load_bridge_module().BridgeNode)
    node.identity = dict(environment='gazebo', instance_id='one-run')
    node.operation_lock = RLock()
    node.cache = Cache()
    node.gazebo_window = GazeboDriveWindow(2.5)
    node.allowed = {'/atd_gazebo_control': ['operator_paused']}
    node.settings = dict(localization_topic='/localization/status', planning_topic='/planning/status',
                         gazebo_control_topic='/atd/gazebo/control_state', control_topic='/atd/gazebo/pursuit_status',
                         gazebo_hold_topic='/atd/gazebo/vehicle_hold',
                         gazebo_gt_stream_id='run', gazebo_gt_topic='/atd/gazebo/aligned_ground_truth',
                         parameter_node_prefix='/atd_gazebo_control')
    node.get_clock = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 2 * 10**9
    node.count_publishers = Mock(return_value=0)
    node.get_node_names_and_namespaces = Mock(return_value=[('atd_gazebo_control', '/')])
    endpoint = SimpleNamespace(node_namespace='/', node_name='atd_gazebo_control', endpoint_gid=b'writer')
    node.get_publishers_info_by_topic = Mock(return_value=[endpoint])
    return node, endpoint


def test_live_dispatch_serves_actual_control_state_and_rejects_an_ambiguous_publisher(live_bridge):
    node, endpoint = live_bridge
    node.receive_gazebo(SimpleNamespace(data=json.dumps(report(2 * 10**9))), {'publisher_gid': {'data': b'writer'}})
    result = node.dispatch('/live/status', 'GET', {})
    assert result['gazebo']['current']['steering_rad'] == -.15
    assert result['control'] is None
    assert result['gazebo']['ground_truth'] is None
    assert result['parameter_nodes'] == ['/atd_gazebo_control']
    assert result['pose'] is None and 'auto_drive' not in result
    source = dict(map_id='odaiba', map_sha256='a'*64, hdmap_sha256='b'*64, session_id='run', epoch=1, stamp=1.95)
    node.count_publishers.return_value = 1
    node.cache.put('localization', dict(source, state='tracking', pose={'x': 1., 'y': 2.}, velocity={}))
    plan = dict(source, valid=True, validity_s=.5, line='centerline')
    node.cache.put('planning', plan)
    node.cache.put('control', dict(plan, publisher_gid=b'writer'.hex(), target=[1.3, 2.4], validity_s=.25))
    assert node.dispatch('/live/status', 'GET', {})['control']['target'] == [1.3, 2.4]
    stopped = json.dumps(report(2 * 10**9, 'stopped'))
    node.receive_gazebo(SimpleNamespace(data=stopped), {'publisher_gid': {'data': b'writer'}})
    assert node.dispatch('/live/status', 'GET', {})['control'] is None
    node.get_publishers_info_by_topic.return_value.append(endpoint)
    assert node.dispatch('/live/status', 'GET', {})['gazebo']['current'] is None
    gt_source = SimpleNamespace(node_namespace='/', node_name='atd_gazebo_closed_loop_paths', endpoint_gid=b'viewer')
    node.get_publishers_info_by_topic.side_effect = lambda topic: (
        [gt_source] if topic == node.settings['gazebo_gt_topic'] else [])
    row = dict(source, frame='map', stamp_s=1.99, x=.5, y=1., yaw=0., stream_id='run',
               footprint=dict(vehicle_length=.35, vehicle_width=.19, rear_axle_to_rear=.1065, margin=0.))
    node.receive_gazebo_gt(SimpleNamespace(data=json.dumps(row)), {'publisher_gid': {'data': b'viewer'}})
    node.cache.put('localization', dict(source, state='lost'))
    live = node.dispatch('/live/status', 'GET', {})
    assert live['pose'] is None and live['control'] is None
    assert live['gazebo']['ground_truth']['x'] == .5


def test_operator_pause_distinguishes_controller_acceptance_and_fresh_vehicle_feedback(live_bridge):
    node, endpoint = live_bridge
    plant = SimpleNamespace(node_namespace='/', node_name='atd_gazebo_actuators', endpoint_gid=b'plant')
    node.get_publishers_info_by_topic.side_effect = lambda topic: (
        [plant] if topic == node.settings['gazebo_hold_topic'] else [endpoint])
    node.receive_gazebo(SimpleNamespace(data=json.dumps(report(2 * 10**9, 'paused'))),
                        {'publisher_gid': {'data': b'writer'}})
    paused = node.dispatch('/live/status', 'GET', {})['gazebo']
    assert paused['current']['state'] == 'paused' and paused['pause_supported']
    assert paused['vehicle_hold'] is None
    request = dict(node='/atd_gazebo_control', instance_id='one-run')
    node.apply_parameters = Mock(return_value={'applied': True})

    def set_hold(value):
        return node.dispatch('/live/parameters', 'POST', dict(request, values={'operator_paused': value}))

    assert set_hold(True)['applied']
    with pytest.raises(ValueError, match='停車確認'):
        set_hold(False)
    sample = dict(held=True, speed_mps=0., stamp_ns=2 * 10**9)
    for update in (dict(held=1), dict(speed_mps=.1), dict(stamp_ns=-1), dict(speed_mps=float('nan'))):
        assert vehicle_hold_sample(dict(sample, **update), 'plant') is None
    node.receive_gazebo_hold(SimpleNamespace(data=json.dumps(sample)), {'publisher_gid': {'data': b'other'}})
    assert node.dispatch('/live/status', 'GET', {})['gazebo']['vehicle_hold'] is None
    node.receive_gazebo_hold(SimpleNamespace(data=json.dumps(sample)), {'publisher_gid': {'data': b'plant'}})
    feedback = node.dispatch('/live/status', 'GET', {})['gazebo']['vehicle_hold']
    assert feedback['held'] and feedback['speed_mps'] == 0 and feedback['stamp_s'] == 2.
    assert set_hold(False)['applied']
    node.get_clock.return_value.now.return_value.nanoseconds += 3 * 10**9
    with pytest.raises(ValueError, match='停車確認'):
        set_hold(False)
    node.get_publishers_info_by_topic.side_effect = lambda _: [plant, plant]
    assert not node.dispatch('/live/status', 'GET', {})['gazebo']['pause_supported']
    with pytest.raises(ValueError, match='車両を確認'):
        set_hold(True)
    node.identity = dict(environment='jetson', instance_id='one-run')
    with pytest.raises(ValueError, match='Gazebo 車両だけ'):
        set_hold(True)


def test_gt_requires_matching_run_writer_frame_dimensions_and_fresh_sim_and_wall_time():
    dims = dict(vehicle_length=.35, vehicle_width=.19, rear_axle_to_rear=.1065, margin=0.)
    row = dict(frame='map', stamp_s=20., x=1., y=2., yaw=.7, footprint=dims,
               map_id='odaiba', map_sha256='a'*64, hdmap_sha256='b'*64, stream_id='this-run')
    for update in (dict(frame='gazebo_world'), dict(stream_id='old-run'), dict(map_sha256=''), dict(yaw=float('nan')),
                   dict(footprint=dict(dims, rear_axle_to_rear=1.))):
        assert ground_truth_sample(dict(row, **update), 'this-run', 'writer') is None
    sample = ground_truth_sample(row, 'this-run', 'writer')
    cache = Cache(.5)
    with patch('time.monotonic', return_value=10.):
        cache.put('gazebo_gt', sample)
        gt = live_gazebo_sample(cache, 'gazebo_gt', 20.25, 'writer')
        assert gt['x'] == 1. and gt['footprint'] == dims and gt['remaining_s'] == .25
        assert 'publisher_gid' not in gt and 'stream_id' not in gt
        assert live_gazebo_sample(cache, 'gazebo_gt', 20., 'other') is None
        assert live_gazebo_sample(cache, 'gazebo_gt', 20., None) is None
        assert live_gazebo_sample(cache, 'gazebo_gt', 19., 'writer') is None
        assert live_gazebo_sample(cache, 'gazebo_gt', 21., 'writer') is None
    with patch('time.monotonic', return_value=11.):
        assert live_gazebo_sample(cache, 'gazebo_gt', 20., 'writer') is None

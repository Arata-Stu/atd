"""Bridge dispatch contract with in-process service fakes, without ROS I/O."""
import importlib
import json
import sys
import tempfile
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from atd_web_bridge.core import Cache, TeleopProfileStore


def load_bridge_module():
    names = {
        'rclpy': {}, 'rclpy.node': {'Node': object}, 'rclpy.parameter': {'Parameter': object},
        'rclpy.parameter_client': {'AsyncParameterClient': object},
        'rclpy.qos': {'qos_profile_sensor_data': object()},
        'rcl_interfaces': {}, 'rcl_interfaces.msg': {'ParameterDescriptor': object},
        'nav_msgs': {}, 'nav_msgs.msg': {'Odometry': object},
        'geometry_msgs': {}, 'geometry_msgs.msg': {'PoseStamped': object, 'TwistStamped': object},
        'sensor_msgs': {}, 'sensor_msgs.msg': {'Joy': object},
        'std_msgs': {}, 'std_msgs.msg': {'String': object},
        'std_srvs': {}, 'std_srvs.srv': {
            'SetBool': types.SimpleNamespace(Request=lambda **kw: types.SimpleNamespace(**kw)),
            'Trigger': types.SimpleNamespace(Request=lambda: types.SimpleNamespace())},
    }
    modules = {}
    for name, attributes in names.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        modules[name] = module
    with patch.dict(sys.modules, modules):
        return importlib.import_module('atd_web_bridge.node')


class TeleopRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = load_bridge_module()

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.node = object.__new__(self.bridge.BridgeNode)
        self.node.cache = Cache()
        self.node.operation_lock = threading.RLock()
        self.node.identity = {'instance_id': 'test_bridge'}
        self.node.teleop_profiles = TeleopProfileStore(Path(self.folder.name) / 'teleop.yaml')
        self.clients = {name: Mock() for name in ('configure', 'reload')}
        for client in self.clients.values():
            client.wait_for_service.return_value = True
            client.call_async.return_value = types.SimpleNamespace(success=True, message='accepted')
        self.node.teleop_clients = self.clients
        self.node.wait = lambda future: future
        self.profile = self.node.teleop_profiles.read()['profile']
        self.profile['axes']['steering']['axis'] = 'left_x'
        self.profile['axes']['throttle']['axis'] = 'r2'

    def dispatch(self, path, method, body=None):
        return self.node.dispatch(path, method, {'instance_id': 'test_bridge', **(body or {})})

    def test_speed_table_goes_to_one_monitor_bound_to_the_current_bridge(self):
        route = '/live/speed-table'
        table = dict(run_id='run', proposal_sha256='d'*64)
        client = self.node.speed_table_client = Mock()
        client.srv_name = '/control/set_speed_table'
        client.srv_type.Request = types.SimpleNamespace
        client.wait_for_service.return_value = True
        client.call_async.return_value = types.SimpleNamespace(saved=True, **table)
        self.node.count_services = Mock(return_value=1)
        with self.assertRaises(ValueError):
            self.dispatch(route, 'PUT', dict(instance_id='other', table=table))
        self.node.count_services.return_value = 2
        with self.assertRaises(ValueError):
            self.dispatch(route, 'PUT', dict(table=table))
        client.call_async.assert_not_called()
        self.node.count_services.return_value = 1
        self.assertEqual(self.dispatch(route, 'PUT', dict(table=table)), dict(saved=True, **table))
        self.assertEqual(json.loads(client.call_async.call_args.args[0].table_json), table)
        client.call_async.return_value.proposal_sha256 = 'f'*64
        with self.assertRaises(ValueError):
            self.dispatch(route, 'PUT', dict(table=table))

    def test_live_auto_history_requires_one_matching_driver_and_fresh_payload(self):
        node = self.node
        node.auto_drive = self.bridge.AutoDriveWindow()
        node.speed_window = self.bridge.SpeedWindow(node.cache)
        node.settings = {key + '_topic': '/' + key for key in
                         ('localization', 'planning', 'control', 'speed_control', 'auto_drive', 'odometry', 'pose', 'velocity')}
        node.count_publishers = lambda _: 0
        node.get_clock = Mock()
        node.get_clock().now().nanoseconds = 100_000_000_000
        node.nodes = lambda: []
        endpoint = types.SimpleNamespace(node_name='atd_auto_drive', node_namespace='/', endpoint_gid=b'writer')
        node.get_publishers_info_by_topic = Mock(return_value=[endpoint])
        info = {'publisher_gid': {'data': b'writer'}}
        data = dict(state='propo', active=False, braking=False, reason='', map_id='map', line='centerline',
                    max_speed_mps=.5, session_id='driver', stamp_ns=100_000_000_000)
        msg = types.SimpleNamespace(data=json.dumps(data))
        read = lambda: node.dispatch('/live/status', 'GET', {})['auto_drive']
        with patch.object(self.bridge.time, 'time_ns', return_value=data['stamp_ns']), \
                patch.object(self.bridge.time, 'monotonic_ns', return_value=100):
            node.receive_auto_drive(msg, info)
            self.assertEqual(read()['current']['state'], 'propo')
            node.get_publishers_info_by_topic.return_value = [endpoint, endpoint]
            self.assertEqual((read()['publishers'], read()['current']), (2, None))
            node.get_publishers_info_by_topic.return_value = [endpoint]
            self.assertFalse(read()['history'])
            endpoint.node_name = 'rosbag2_player'
            node.receive_auto_drive(msg, info)
            self.assertIsNone(read()['current'])
            endpoint.node_name = 'atd_auto_drive'
            node.receive_auto_drive(msg, {'publisher_gid': {'data': b'old'}})
            self.assertIsNone(read()['current'])
            node.receive_auto_drive(msg, info)
            node.receive_auto_drive(types.SimpleNamespace(data='{'), info)
            self.assertIsNone(read()['current'])
            node.get_publishers_info_by_topic.return_value = []
            self.assertEqual(read()['publishers'], 0)

    def status(self, **values):
        self.node.cache.put('teleop_status', {
            'configuring': True, 'profile_valid': True, 'ready': False,
            'profile_path': str(self.node.teleop_profiles.path), 'profile': self.profile, **values})

    def save(self):
        self.status()
        self.node.teleop_profiles.write(self.profile, self.node.teleop_profiles.read()['revision'],
                                       self.node.cache.teleop_status())

    def test_profile_can_be_read_without_running_node_but_cannot_be_saved(self):
        result = self.dispatch('/teleop/profile', 'GET')
        self.assertFalse(result['saved'])
        self.assertFalse(result['applied'])
        with self.assertRaises(ValueError):
            self.dispatch('/teleop/profile', 'PUT', {'profile': self.profile, 'revision': result['revision']})

    def test_reload_is_single_request_and_never_resumes_output(self):
        self.save()
        result = self.dispatch('/teleop/reload', 'POST')
        self.assertFalse(result['output_resumed'])
        self.clients['reload'].call_async.assert_called_once()
        self.clients['configure'].call_async.assert_not_called()
        self.assertIsNone(self.node.cache.teleop_status()['status'])

    def test_resume_requires_saved_mapping_to_match_active_mapping(self):
        self.save()
        active = self.node.teleop_profiles.read()['profile']
        active['axes']['steering']['axis'] = 'right_x'
        self.status(profile=active)
        with self.assertRaises(ValueError):
            self.dispatch('/teleop/configure', 'POST', {'enabled': False})
        self.clients['configure'].call_async.assert_not_called()
        self.status()
        result = self.dispatch('/teleop/configure', 'POST', {'enabled': False})
        self.assertTrue(result['output_resumed'])
        self.assertIs(self.clients['configure'].call_async.call_args.args[0].data, False)

    def test_missing_status_and_non_boolean_config_request_are_rejected(self):
        with self.assertRaises(ValueError):
            self.dispatch('/teleop/configure', 'POST', {'enabled': True})
        self.status()
        with self.assertRaises(ValueError):
            self.dispatch('/teleop/configure', 'POST', {'enabled': 1})
        self.clients['configure'].call_async.assert_not_called()

    def test_recording_only_profile_can_resume_but_empty_or_enable_only_cannot(self):
        from atd_teleop_tools import profile as profiles
        for role in ('record_start', 'record_stop'):
            with self.subTest(role=role):
                self.profile = profiles.default_profile()
                self.profile['buttons'][role] = 'cross'
                self.save()
                self.status(ready=False, events_ready=False)
                result = self.dispatch('/teleop/configure', 'POST', {'enabled': False})
                self.assertTrue(result['output_resumed'])
        calls = self.clients['configure'].call_async.call_count
        for enable in ('', 'r1'):
            with self.subTest(enable=enable):
                self.profile = profiles.default_profile()
                self.profile['buttons']['enable'] = enable
                self.save()
                self.status()
                with self.assertRaisesRegex(ValueError, '割り当て'):
                    self.dispatch('/teleop/configure', 'POST', {'enabled': False})
        self.assertEqual(self.clients['configure'].call_async.call_count, calls)

    def test_timeout_invalidates_observation_and_does_not_retry(self):
        self.save()
        self.node.wait = Mock(side_effect=ValueError('ROS service timeout'))
        with self.assertRaisesRegex(ValueError, 'timeout'):
            self.dispatch('/teleop/reload', 'POST')
        self.clients['reload'].call_async.assert_called_once()
        self.assertIsNone(self.node.cache.teleop_status()['status'])

    def test_unknown_or_drive_route_is_not_forwarded(self):
        for path in ('/teleop/drive', '/teleop/command', '/unknown'):
            with self.subTest(path=path), self.assertRaisesRegex(ValueError, 'Unsupported'):
                self.dispatch(path, 'POST', {'throttle': 1})

    def test_mutations_bind_to_the_observed_bridge_instance(self):
        self.save()
        current = self.dispatch('/teleop/profile', 'GET')
        old_revision = current['revision']
        self.node.identity['instance_id'] = 'replacement_bridge'
        # Identical saved settings (and hence revision) do not identify a Jetson.
        self.assertEqual(self.node.teleop_profiles.read()['revision'], old_revision)
        for identity in ({}, {'instance_id': 'test_bridge'}, {'instance_id': 1}):
            for path, method, body in (
                    ('/teleop/configure', 'POST', {'enabled': True}),
                    ('/teleop/reload', 'POST', {}),
                    ('/teleop/profile', 'PUT', {'profile': self.profile, 'revision': old_revision})):
                with self.subTest(identity=identity, path=path), self.assertRaisesRegex(ValueError, '接続先'):
                    self.node.dispatch(path, method, {**body, **identity})
        self.clients['configure'].call_async.assert_not_called()
        self.clients['reload'].call_async.assert_not_called()


class ManualMonitorTests(unittest.TestCase):
    def setUp(self):
        bridge = load_bridge_module()
        self.node = object.__new__(bridge.BridgeNode)
        self.node.cache = Cache()
        self.node.operation_lock = threading.RLock()
        self.node.settings = dict(teleop_node='/atd_teleop_node', teleop_command_topic='/teleop/command',
                                  joy_drive_status_topic='/vehicle/joy_drive/status', pico_status_topic='/vehicle/pico/status')
        self.info = {'publisher_gid': {'data': b'writer'}}
        self.endpoints = {
            topic: [types.SimpleNamespace(node_namespace='/', node_name=name, endpoint_gid=list(b'writer'))]
            for topic, name in [('/teleop/command', 'atd_teleop_node'), ('/vehicle/joy_drive/status', 'atd_joy_drive'),
                                ('/vehicle/pico/status', 'atd_vehicle_interface')]}
        self.node.get_publishers_info_by_topic = self.endpoints.__getitem__
        stamp = types.SimpleNamespace(sec=100, nanosec=0)
        self.joy = types.SimpleNamespace(header=types.SimpleNamespace(frame_id='atd_gamepad', stamp=stamp),
            source='joy', valid=True, steering=-.4, throttle=0., enable_requested=False)
        self.status = types.SimpleNamespace(header=types.SimpleNamespace(frame_id='pico', stamp=stamp),
            has_report=True, serial='abc', session_id='link', sequence_number=1, selector=1, rx_mode_us=1900,
            active_path=2, fault_bits=0, servo_us=1400, esc_us=1750, joy_active=True, reason='')
        self.bridge = bridge

    def query(self, now=100.01):
        with patch('atd_web_bridge.node.time.time', return_value=now):
            return self.node.dispatch('/input/status', 'GET', {})['drive_monitor']

    def publish(self):
        self.node.cache.put('manual_joy', self.bridge.joy_sample(self.joy, self.info))
        self.bridge.receive_feedback(self.node.cache, self.status, self.info)

    def test_same_report_can_revoke_attribution_but_cannot_refresh_pwm(self):
        self.publish()
        data = self.query()
        self.assertEqual((data['joy']['steering'], data['joy']['throttle']), (-.4, 0.))
        self.assertEqual((data['board']['ch3'], data['board']['control_source']), ('jetson', 'dualsense'))
        self.assertEqual((data['board']['servo_us'], data['board']['esc_us']), (1400, 1750))
        self.status.joy_active = False
        self.publish()
        self.assertEqual(self.query()['board']['control_source'], 'host')
        self.assertIsNone(self.query(100.16)['board'])
        self.assertIsNotNone(self.query(100.16)['joy'])
        self.assertIsNone(self.query(100.26)['joy'])

    def test_ch3_switch_and_output_path_are_separate(self):
        for path, pulse, route, position in [(1, 1900, 'propo', 'jetson'), (0, 1700, 'standby', 'jetson'),
                                             (0, 1550, 'standby', 'middle'), (1, 1300, 'propo', 'propo'),
                                             (0, 0, 'standby', 'unknown')]:
            with self.subTest(path=path, pulse=pulse):
                self.status.active_path, self.status.rx_mode_us = path, pulse
                self.publish()
                board = self.query()['board']
                self.assertEqual((board['ch3'], board['control_source']), (position, route))
                self.assertEqual(board['rx_mode_us'], pulse or None)
        self.status.active_path, self.status.fault_bits = 3, 1
        self.publish()
        board = self.query()['board']
        self.assertEqual((board['control_source'], board['pwm_enabled'], board['esc_us']), ('fault', False, None))

    def test_missing_publishers_invalid_input_and_usb_disconnect_clear_values_independently(self):
        self.publish()
        self.endpoints['/teleop/command'][0].node_name = 'rosbag2_player'
        self.assertIsNone(self.query()['joy'])
        self.assertIsNotNone(self.query()['board'])
        self.endpoints['/teleop/command'][0].node_name = 'atd_teleop_node'
        self.joy.valid = False
        self.publish()
        self.assertIsNone(self.query()['joy'])
        self.endpoints['/vehicle/joy_drive/status'][0].endpoint_gid = list(b'restarted')
        self.assertIsNone(self.query()['board'])
        self.endpoints['/vehicle/joy_drive/status'][0].endpoint_gid = list(b'writer')
        self.status.has_report, self.status.reason = False, 'Pico 切断'
        self.publish()
        self.assertEqual(self.query()['board_reason'], 'Pico 切断')
        self.assertIsNone(self.query()['board'])

    def test_monitor_only_uses_cdc1_without_inventing_ch3_position_or_output_pulses(self):
        self.status.has_report = False
        self.publish()
        telemetry = dict(stamp=100., age_s=.01, stale=False, serial='ABC', instance_id='usb', rc_valid=True,
                         sbus_fresh=True, sbus_flags=0, channels=[992]*16, active_path=1, fault_bits=0)
        status = dict(connected=True, streaming=True, serial='ABC', instance_id='usb', telemetry=telemetry)
        def send():
            self.node.pico_status(types.SimpleNamespace(data=json.dumps(status)), self.info)
        send()
        board = self.query()['board']
        self.assertEqual((board['source'], board['ch3_raw'], board['control_source']), ('cdc1', 992, 'propo'))
        self.assertEqual((board['ch3'], board['rx_mode_us'], board['servo_us'], board['esc_us']), ('unknown', None, None, None))
        telemetry['active_path'] = 2
        send()
        self.assertEqual(self.query()['board']['control_source'], 'host')
        self.assertIsNone(self.query(100.4)['board'])
        status['serial'] = telemetry['serial'] = 'different'
        send()
        self.assertIsNone(self.query()['board'])

    def test_missing_cdc0_does_not_restore_older_cdc1_route(self):
        def receiver(stamp):
            telemetry = dict(stamp=stamp, age_s=0., stale=False, serial='ABC', instance_id='usb', rc_valid=True,
                             sbus_fresh=True, sbus_flags=0, channels=[992]*16, active_path=1, fault_bits=0)
            status = dict(connected=True, streaming=True, serial='ABC', instance_id='usb', telemetry=telemetry)
            self.node.pico_status(types.SimpleNamespace(data=json.dumps(status)), self.info)

        receiver(99.98)
        self.publish()  # CDC0 DualSense @ 100.0
        self.assertEqual(self.query()['board']['control_source'], 'dualsense')
        self.assertIsNone(self.query(100.16)['board'])
        self.status.has_report = False
        self.status.header.stamp = types.SimpleNamespace(sec=0, nanosec=0)
        self.publish()
        self.assertIsNone(self.query(100.16)['board'])
        self.endpoints['/vehicle/joy_drive/status'].clear()
        self.assertIsNone(self.query(100.16)['board'])
        receiver(100.14)
        self.assertEqual(self.query(100.16)['board']['control_source'], 'propo')


if __name__ == '__main__':
    unittest.main()

"""Pico/propo の観測鮮度と設定仲介。ROS/USB/SSHは実行しない。"""
import copy
import importlib
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import Mock, patch

from atd_web_bridge.core import Cache, PropoProfileStore


def load_bridge_module():
    definitions = {
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
    for name, attributes in definitions.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        modules[name] = module
    with patch.dict(sys.modules, modules):
        return importlib.import_module('atd_web_bridge.node')


def board_sample(**changes):
    value = {'connected': True, 'streaming': True, 'instance_id': 'usb-session', 'serial': 'PICO1',
             'telemetry_timeout_s': .3,
             'telemetry': {'instance_id': 'usb-session', 'serial': 'PICO1', 'stamp': 500.,
                           'age_s': .02, 'stale': False, 'rc_valid': True, 'sbus_fresh': True,
                           'sbus_flags': 0, 'channels': [992] * 16}}
    value.update(changes)
    return value


class PropoCacheTests(unittest.TestCase):
    def input_sample(self, **changes):
        value = {'input_instance_id': 'usb-session', 'serial': 'PICO1', 'input_age_s': .02,
                 'profile_valid': True, 'configuring': False, 'ready': True}
        value.update(changes)
        return value

    def test_missing_data_stays_missing_not_zero_input(self):
        status = Cache().propo_status()
        self.assertIsNone(status['board'])
        self.assertIsNone(status['input'])
        self.assertFalse(status['ready'])
        self.assertFalse(status['rc_valid'])

    def test_embedded_source_age_is_added_to_bridge_age_and_stamp_preserved(self):
        cache = Cache()
        with patch('atd_web_bridge.core.time.monotonic', return_value=100.):
            cache.put('pico_status', board_sample())
            cache.put('rc_status', self.input_sample())
        with patch('atd_web_bridge.core.time.monotonic', return_value=100.1):
            result = cache.propo_status()
        self.assertAlmostEqual(result['board']['telemetry']['age_s'], .12)
        self.assertEqual(result['board']['telemetry']['stamp'], 500.)
        self.assertAlmostEqual(result['input']['input_age_s'], .12)
        self.assertTrue(result['ready'])
        with patch('atd_web_bridge.core.time.monotonic', return_value=100.31):
            stale = cache.propo_status()
        self.assertFalse(stale['ready'])
        self.assertFalse(stale['rc_valid'])
        self.assertTrue(stale['board']['telemetry']['stale'])
        self.assertFalse(stale['board']['streaming'])

    def test_periodic_status_cannot_refresh_old_source_sample(self):
        cache = Cache()
        value = board_sample()
        value['telemetry']['age_s'] = .4
        cache.put('pico_status', value)
        cache.put('rc_status', self.input_sample())
        result = cache.propo_status()
        self.assertFalse(result['rc_valid'])
        self.assertFalse(result['ready'])
        self.assertEqual(result['board']['telemetry']['channels'], [992] * 16)

    def test_mismatched_usb_session_or_serial_does_not_enable_roles(self):
        for change in ({'input_instance_id': 'previous'}, {'serial': 'OTHER'}):
            with self.subTest(change=change):
                cache = Cache()
                cache.put('pico_status', board_sample())
                cache.put('rc_status', self.input_sample(**change))
                self.assertFalse(cache.propo_status()['input_current'])
                self.assertFalse(cache.propo_status()['ready'])

    def test_lost_failsafe_malformed_shape_or_source_age_is_invalid(self):
        variations = [
            {'sbus_flags': 4}, {'sbus_flags': 8}, {'sbus_fresh': False},
            {'channels': [992] * 6}, {'channels': [True] * 16},
            {'age_s': float('nan')}, {'age_s': -1}, {'age_s': '0'},
            {'instance_id': 'old'}, {'serial': 'OTHER'},
        ]
        for change in variations:
            with self.subTest(change=change):
                cache = Cache()
                value = board_sample()
                value['telemetry'].update(change)
                cache.put('pico_status', value)
                cache.put('rc_status', self.input_sample())
                self.assertFalse(cache.propo_status()['rc_valid'])
                self.assertFalse(cache.propo_status()['ready'])

    def test_disconnect_and_stale_rc_role_data_are_not_ready(self):
        cache = Cache()
        cache.put('pico_status', board_sample(connected=False))
        cache.put('rc_status', self.input_sample())
        self.assertFalse(cache.propo_status()['ready'])
        cache.put('pico_status', board_sample())
        cache.put('rc_status', self.input_sample(input_age_s=.4))
        self.assertFalse(cache.propo_status()['ready'])


class PropoRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = load_bridge_module()

    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.node = object.__new__(self.bridge.BridgeNode)
        self.node.cache = Cache()
        self.node.operation_lock = threading.RLock()
        self.node.identity = {'instance_id': 'bridge-instance'}
        self.node.propo_profiles = PropoProfileStore(Path(folder.name) / 'propo.yaml')
        self.clients = {name: Mock() for name in ('configure', 'reload')}
        for client in self.clients.values():
            client.wait_for_service.return_value = True
            client.call_async.return_value = types.SimpleNamespace(success=True, message='accepted')
        self.node.propo_clients = self.clients
        self.node.wait = lambda future: future
        self.profile = self.node.propo_profiles.read()['profile']
        self.profile['roles']['recording']['channel'] = 4

    def dispatch(self, path, method='GET', body=None):
        return self.node.dispatch(path, method, {'instance_id': 'bridge-instance', **(body or {})})

    def status(self, **changes):
        value = {'configuring': True, 'profile_valid': True, 'ready': False,
                 'profile_path': str(self.node.propo_profiles.path), 'profile': self.profile,
                 'input_instance_id': 'usb-session', 'serial': 'PICO1', 'input_age_s': .02}
        value.update(changes)
        self.node.cache.put('rc_status', value)

    def save(self):
        self.status()
        result = self.dispatch('/propo/profile')
        return self.dispatch('/propo/profile', 'PUT', {'profile': self.profile, 'revision': result['revision']})

    def test_profile_read_without_nodes_does_not_claim_saved_or_applied(self):
        result = self.dispatch('/propo/profile')
        self.assertFalse(result['saved'])
        self.assertFalse(result['applied'])
        self.assertEqual(result['channels'], list(range(4, 17)))
        self.assertEqual(result['fixed_roles'], {'steering': 1, 'throttle': 2, 'authority': 3})
        with self.assertRaises(ValueError):
            self.dispatch('/propo/profile', 'PUT', {'profile': self.profile, 'revision': result['revision']})

    def test_save_is_possible_in_config_mode_without_board_but_not_resume(self):
        result = self.save()
        self.assertTrue(result['saved'])
        self.assertFalse(result['applied'])
        with self.assertRaisesRegex(ValueError, '新鮮'):
            self.dispatch('/propo/configure', 'POST', {'enabled': False})
        self.clients['configure'].call_async.assert_not_called()

    def test_extended_channel_save_reload_apply_and_resume(self):
        for channel in (10, 16):
            with self.subTest(channel=channel):
                previous = self.node.propo_profiles.read()['profile']
                self.profile['roles']['recording']['channel'] = channel
                self.status(profile=previous)
                current = self.dispatch('/propo/profile')
                saved = self.dispatch('/propo/profile', 'PUT', {
                    'profile': self.profile, 'revision': current['revision']})
                self.assertEqual(saved['profile']['roles']['recording']['channel'], channel)
                self.assertFalse(saved['applied'])
                self.assertFalse(self.dispatch('/propo/profile')['applied'])
                self.dispatch('/propo/reload', 'POST')
                self.status()
                self.node.cache.put('pico_status', board_sample())
                self.assertTrue(self.dispatch('/propo/profile')['applied'])
                self.assertTrue(self.dispatch('/propo/configure', 'POST', {'enabled': False})['output_resumed'])

    def test_save_checks_revision_path_config_mode_and_fixed_channels(self):
        result = self.save()
        original = copy.deepcopy(result['profile'])
        bad = copy.deepcopy(original)
        bad['roles']['recording']['channel'] = 1
        with self.assertRaises(ValueError):
            self.dispatch('/propo/profile', 'PUT', {'profile': bad, 'revision': result['revision']})
        with self.assertRaises(ValueError):
            self.dispatch('/propo/profile', 'PUT', {'profile': original, 'revision': 'old'})
        self.status(profile_path='/other/propo.yaml')
        with self.assertRaises(ValueError):
            self.dispatch('/propo/profile', 'PUT', {'profile': original, 'revision': result['revision']})
        self.assertEqual(self.node.propo_profiles.read()['profile'], original)

    def test_resume_checks_active_profile_signal_session_and_does_not_touch_pico(self):
        self.save()
        self.node.cache.put('pico_status', board_sample())
        bad = copy.deepcopy(self.profile)
        bad['roles']['recording']['channel'] = 5
        self.status(profile=bad)
        with self.assertRaisesRegex(ValueError, '再読込'):
            self.dispatch('/propo/configure', 'POST', {'enabled': False})
        self.status(input_instance_id='old')
        with self.assertRaisesRegex(ValueError, 'Pico'):
            self.dispatch('/propo/configure', 'POST', {'enabled': False})
        self.status()
        result = self.dispatch('/propo/configure', 'POST', {'enabled': False})
        self.assertTrue(result['output_resumed'])
        self.assertIs(self.clients['configure'].call_async.call_args.args[0].data, False)
        self.assertIsNone(self.node.cache.propo_status()['input'])

    def test_reload_is_single_request_keeps_output_suspended_and_invalidates_timeout(self):
        self.save()
        result = self.dispatch('/propo/reload', 'POST')
        self.assertFalse(result['output_resumed'])
        self.clients['reload'].call_async.assert_called_once()
        self.clients['configure'].call_async.assert_not_called()
        self.status()
        self.node.wait = Mock(side_effect=ValueError('ROS service timeout'))
        with self.assertRaisesRegex(ValueError, 'timeout'):
            self.dispatch('/propo/reload', 'POST')
        self.assertEqual(self.clients['reload'].call_async.call_count, 2)
        self.assertIsNone(self.node.cache.propo_status()['input'])

    def test_mutations_bind_to_bridge_instance_and_unknown_output_routes_reject(self):
        self.save()
        for path, method, body in (
                ('/propo/configure', 'POST', {'enabled': True}), ('/propo/reload', 'POST', {}),
                ('/propo/profile', 'PUT', {'profile': self.profile, 'revision': 'x'})):
            with self.subTest(path=path), self.assertRaisesRegex(ValueError, '接続先'):
                self.node.dispatch(path, method, body)
        with self.assertRaisesRegex(ValueError, 'Unsupported'):
            self.dispatch('/propo/drive', 'POST', {'throttle': 1})
        for client in self.clients.values():
            client.call_async.assert_not_called()


if __name__ == '__main__':
    unittest.main()

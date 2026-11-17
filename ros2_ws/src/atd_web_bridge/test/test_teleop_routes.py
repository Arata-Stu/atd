"""Bridge dispatch contract with in-process service fakes, without ROS I/O."""
import importlib
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


if __name__ == '__main__':
    unittest.main()

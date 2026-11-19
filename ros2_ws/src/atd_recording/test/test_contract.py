"""Portable configuration/launch contract checks; these do not run ROS."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml

PACKAGE = Path(__file__).resolve().parents[1]


class RecordingContractTest(unittest.TestCase):
    def config(self, profile):
        return yaml.safe_load((PACKAGE / 'config' / f'{profile}.yaml').read_text())['/atd_recording_node']['ros__parameters']

    def test_all_profiles_are_explicit_and_exclude_own_status(self):
        for profile in ('mapping', 'calibration', 'inputs'):
            with self.subTest(profile=profile):
                config = self.config(profile)
                topics = config['topics']
                self.assertEqual(config['profile_name'], profile)
                self.assertEqual(config['record_root'], '/data/record/rosbags')
                self.assertEqual(len(topics), len(set(topics)))
                self.assertTrue(all(value.startswith('/') for value in topics))
                self.assertNotIn('/recording/status', topics)
                self.assertNotIn('all_topics', config)
                required = config.get('required_topics', [])
                self.assertEqual(len(required), len(config.get('required_types', [])))
                self.assertTrue(set(required) <= set(topics))
                best_effort = set(config.get('best_effort_topics', []))
                transient = set(config.get('transient_local_topics', []))
                self.assertTrue(best_effort <= set(topics))
                self.assertTrue(transient <= set(topics))
                self.assertFalse(best_effort & transient)
                self.assertEqual(transient, {'/tf_static'})
                self.assertTrue({'/teleop/events', '/rc/events', '/vehicle/pico/telemetry'} <= set(topics))
                # CDC0 の報告と Joy 操作は、使った機器に応じて片側だけでも記録する。
                manual_drive = {'/vehicle/joy_drive/status', '/teleop/command'}
                self.assertTrue(manual_drive <= set(topics))
                self.assertTrue(manual_drive.isdisjoint(required))
                self.assertIn('/vehicle/joy_drive/status', best_effort)
                self.assertIn('/localization/status', topics)
                self.assertNotIn('/localization/status', required)
                autonomous = {'/planning/status', '/control/status', '/control/speed_status', '/vehicle/auto_drive/status'}
                self.assertTrue(autonomous <= set(topics))
                self.assertTrue(autonomous.isdisjoint(required))
                self.assertTrue({'/camera/camera/accel/sample', '/camera/camera/imu'} <= best_effort)

    def test_mapping_requires_stereo_and_static_tf(self):
        config = self.config('mapping')
        required = dict(zip(config['required_topics'], config['required_types']))
        self.assertEqual(required['/tf_static'], 'tf2_msgs/msg/TFMessage')
        for eye in ('infra1', 'infra2'):
            for suffix, typename in (('image_rect_raw', 'Image'), ('camera_info', 'CameraInfo')):
                topic = f'/camera/camera/{eye}/{suffix}'
                self.assertEqual(required[topic], f'sensor_msgs/msg/{typename}')
                self.assertIn(topic, config['best_effort_topics'])
        self.assertEqual(len(required), 5)

    def test_calibration_adds_imu_and_raw_board_telemetry(self):
        config = self.config('calibration')
        required = dict(zip(config['required_topics'], config['required_types']))
        self.assertEqual(required['/camera/camera/accel/sample'], 'sensor_msgs/msg/Imu')
        self.assertEqual(required['/camera/camera/gyro/sample'], 'sensor_msgs/msg/Imu')
        self.assertEqual(required['/vehicle/pico/telemetry'], 'atd_interfaces/msg/PicoTelemetry')
        self.assertTrue(set(self.config('mapping')['required_topics']) <= set(required))

    def test_inputs_does_not_require_camera_or_manufacture_own_messages(self):
        config = self.config('inputs')
        self.assertFalse(config.get('required_topics', []))
        self.assertEqual({topic for topic in config['topics'] if topic.startswith('/camera/')},
                         {'/camera/camera/accel/sample', '/camera/camera/imu'})
        self.assertNotIn('/recording/status', config['topics'])

    def launch_module(self):
        class Action:
            def __init__(self, **kwargs):
                self.kwargs = kwargs

        class Argument:
            def __init__(self, name, **kwargs):
                self.name, self.kwargs = name, kwargs

        class Configuration:
            def __init__(self, name):
                self.name = name

            def perform(self, context):
                return context[self.name]

        modules = {
            'ament_index_python': types.ModuleType('ament_index_python'),
            'ament_index_python.packages': types.SimpleNamespace(get_package_share_directory=lambda name: str(PACKAGE)),
            'launch': types.SimpleNamespace(LaunchDescription=lambda actions: actions),
            'launch.actions': types.SimpleNamespace(DeclareLaunchArgument=Argument, OpaqueFunction=Action),
            'launch.substitutions': types.SimpleNamespace(LaunchConfiguration=Configuration),
            'launch_ros': types.ModuleType('launch_ros'),
            'launch_ros.actions': types.SimpleNamespace(Node=Action),
        }
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('recording_launch_test', PACKAGE / 'launch' / 'recording.launch.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        return module

    def test_launch_default_and_supported_profiles(self):
        module = self.launch_module()
        declarations = {action.name: action.kwargs for action in module.generate_launch_description() if hasattr(action, 'name')}
        self.assertEqual(declarations['profile']['default_value'], 'mapping')
        self.assertEqual(set(declarations['profile']['choices']), {'mapping', 'calibration', 'inputs'})
        self.assertEqual(declarations['config']['default_value'], '')
        for profile in declarations['profile']['choices']:
            node = module.start({'profile': profile, 'config': ''})[0]
            self.assertEqual(node.kwargs['package'], 'atd_recording')
            self.assertEqual(node.kwargs['executable'], 'atd_recording_node')
            self.assertEqual(node.kwargs['parameters'], [str(PACKAGE / 'config' / f'{profile}.yaml')])

    def test_explicit_config_absolute_existing_only(self):
        module = self.launch_module()
        with self.assertRaises(ValueError):
            module.start({'profile': 'mapping', 'config': 'relative.yaml'})
        with tempfile.TemporaryDirectory(prefix='atd-record-launch-') as temporary:
            path = Path(temporary) / 'config.yaml'
            with self.assertRaises(FileNotFoundError):
                module.start({'profile': 'mapping', 'config': str(path)})
            path.write_text('/atd_recording_node:\n  ros__parameters:\n    profile_name: inputs\n')
            self.assertEqual(module.start({'profile': 'mapping', 'config': str(path)})[0].kwargs['parameters'], [str(path)])


if __name__ == '__main__':
    unittest.main()

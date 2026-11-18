from contextlib import contextmanager
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


class Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.__dict__.update(kwargs)


class Description:
    def __init__(self, entities):
        self.entities = entities


class Configuration:
    def __init__(self, name):
        self.name = name


@contextmanager
def fake_launch():
    def module(name, **members):
        value = types.ModuleType(name)
        value.__dict__.update(members)
        return value
    modules = {
        'ament_index_python': module('ament_index_python'),
        'ament_index_python.packages': module('ament_index_python.packages',
            get_package_share_directory=lambda name: '/share/' + name),
        'launch': module('launch', LaunchDescription=Description),
        'launch.actions': module('launch.actions', DeclareLaunchArgument=Action,
                                 GroupAction=Action, IncludeLaunchDescription=Action),
        'launch.launch_description_sources': module('launch.launch_description_sources',
                                                    PythonLaunchDescriptionSource=Action),
        'launch.substitutions': module('launch.substitutions', LaunchConfiguration=Configuration),
    }
    with patch.dict(sys.modules, modules):
        yield


class CompositionTests(unittest.TestCase):
    def generate(self, manual):
        from atd_bringup.launching import description
        with fake_launch():
            return description(manual).entities

    def groups(self, manual):
        return [value for value in self.generate(manual) if hasattr(value, 'actions')]

    def test_d455_has_only_camera_and_vehicle_scoped_groups(self):
        groups = self.groups(False)
        self.assertEqual(len(groups), 2)
        self.assertTrue(all(group.scoped for group in groups))
        files = [group.actions[0].args[0].args[0] for group in groups]
        self.assertEqual(files, ['/share/atd_sensors/launch/d455.launch.py',
                                 '/share/atd_vehicle_description/launch/vehicle.launch.py'])
        self.assertFalse(any('bridge' in path for path in files))

    def test_manual_adds_one_input_include_and_does_not_start_a_second_bridge(self):
        groups = self.groups(True)
        self.assertEqual(len(groups), 3)
        self.assertTrue(all(group.scoped for group in groups))
        files = [group.actions[0].args[0].args[0] for group in groups]
        self.assertEqual(files.count('/share/atd_web_bridge/launch/input.launch.py'), 1)
        self.assertFalse(any(path.endswith('/bridge.launch.py') for path in files))

    def test_config_and_profile_values_are_deliberately_mapped_to_each_child(self):
        camera, vehicle, inputs = [dict(group.actions[0].launch_arguments) for group in self.groups(True)]
        self.assertEqual(camera['config'].name, 'camera_config')
        self.assertEqual(camera['serial'].name, 'camera_serial')
        self.assertNotIn('profile', camera)
        self.assertEqual(vehicle['config'].name, 'vehicle_config')
        self.assertEqual(vehicle['profile'].name, 'vehicle_profile')
        self.assertEqual(inputs['config'].name, 'config')
        self.assertEqual(inputs['profile'].name, 'profile')
        self.assertEqual(inputs['vehicle_profile'].name, 'vehicle_profile')
        self.assertIs(vehicle['profile'], inputs['vehicle_profile'])
        self.assertNotIn('camera_config', inputs)

    def test_persistent_camera_defaults_and_stream_contract_are_declared(self):
        from atd_bringup.launching import CAMERA_ARGUMENTS
        declarations = {value.args[0]: value.default_value for value in self.generate(True)
                        if hasattr(value, 'default_value')}
        self.assertEqual(declarations['camera_config'], '/data/results/runs/bringup/d455.yaml')
        self.assertEqual(declarations['vehicle_profile'], '/data/results/runs/vehicle/layout.yaml')
        self.assertTrue(all(key in declarations for key in CAMERA_ARGUMENTS))


if __name__ == '__main__':
    unittest.main()

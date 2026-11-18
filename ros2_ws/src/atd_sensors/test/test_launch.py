from contextlib import contextmanager
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

from atd_sensors.profile import DEFAULTS, OVERRIDE_ARGUMENTS


class Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.__dict__.update(kwargs)


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


@contextmanager
def loaded_launch():
    def module(name, **members):
        value = types.ModuleType(name)
        value.__dict__.update(members)
        return value
    modules = {
        'ament_index_python': module('ament_index_python'),
        'ament_index_python.packages': module('ament_index_python.packages',
            get_package_share_directory=lambda name: '/share/' + name),
        'launch': module('launch', LaunchDescription=Action),
        'launch.actions': module('launch.actions', DeclareLaunchArgument=Action, GroupAction=Action,
            IncludeLaunchDescription=Action, LogInfo=Action, OpaqueFunction=Action),
        'launch.launch_description_sources': module('launch.launch_description_sources',
                                                    PythonLaunchDescriptionSource=Action),
        'launch.substitutions': module('launch.substitutions', LaunchConfiguration=Configuration),
    }
    path = Path(__file__).resolve().parents[1] / 'launch/d455.launch.py'
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location('camera_launch_test', path)
        value = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(value)
        yield value


class CameraLaunchTests(unittest.TestCase):
    def context(self):
        return {'config': '/tmp/requested.yaml', 'serial': '', **{key: '' for key in OVERRIDE_ARGUMENTS}}

    def test_exact_sdk_validation_precedes_official_driver_with_requested_parameters(self):
        with loaded_launch() as launch, patch.object(launch, 'load', return_value=deepcopy(DEFAULTS)), \
                patch.object(launch.probe, 'validate', side_effect=lambda value: {**value, 'serial': '123456'}) as validate:
            context = self.context()
            context['infra_profile'] = '848x480x60'
            context.update(config_file='/tmp/unvalidated.yaml', json_file_path='/tmp/unvalidated.json',
                           rosbag_filename='/tmp/unvalidated.bag')
            actions = launch.start_camera(context)
        validate.assert_called_once()
        self.assertEqual(validate.call_args.args[0]['streams']['infra1']['width'], 848)
        self.assertEqual(len(actions), 1)
        self.assertTrue(actions[0].scoped)
        self.assertFalse(actions[0].forwarding)
        include = actions[0].actions[0]
        self.assertEqual(include.args[0].args[0], '/share/realsense2_camera/launch/rs_launch.py')
        parameters = dict(include.launch_arguments)
        self.assertEqual(parameters['depth_module.infra_profile'], '848,480,60')
        self.assertEqual(parameters['serial_no'], '_123456')
        self.assertEqual(parameters['camera_namespace'], 'camera')
        self.assertEqual(parameters['camera_name'], 'camera')
        self.assertEqual(parameters['publish_tf'], 'true')
        self.assertTrue({'config_file', 'json_file_path', 'rosbag_filename'}.isdisjoint(parameters))

    def test_all_off_requires_no_camera_sdk_and_starts_no_driver(self):
        with loaded_launch() as launch, patch.object(launch, 'load', return_value=deepcopy(DEFAULTS)), \
                patch.object(launch.probe, 'validate') as validate:
            context = self.context()
            context.update({key: 'false' for key in OVERRIDE_ARGUMENTS if key.startswith('enable_')})
            actions = launch.start_camera(context)
        validate.assert_not_called()
        self.assertEqual(len(actions), 1)
        self.assertIn('all streams OFF', actions[0].msg)
        self.assertFalse(hasattr(actions[0], 'actions'))

    def test_sdk_rejection_propagates_instead_of_returning_a_driver(self):
        with loaded_launch() as launch, patch.object(launch, 'load', return_value=deepcopy(DEFAULTS)), \
                patch.object(launch.probe, 'validate', side_effect=ValueError('unsupported requested stream')), \
                self.assertRaisesRegex(ValueError, 'unsupported'):
            launch.start_camera(self.context())


if __name__ == '__main__':
    unittest.main()

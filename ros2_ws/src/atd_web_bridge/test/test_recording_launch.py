"""Input と manual の合流点に記録managerが1つだけあることを確認する。"""
from contextlib import contextmanager
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


class Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.__dict__.update(kwargs)


@contextmanager
def input_launch():
    def module(name, **members):
        value = types.ModuleType(name)
        value.__dict__.update(members)
        return value

    modules = {
        'ament_index_python': module('ament_index_python'),
        'ament_index_python.packages': module('ament_index_python.packages',
            get_package_share_directory=lambda name: '/share/' + name),
        'launch': module('launch', LaunchDescription=Action),
        'launch.actions': module('launch.actions', DeclareLaunchArgument=Action,
            GroupAction=Action, IncludeLaunchDescription=Action),
        'launch.conditions': module('launch.conditions', IfCondition=Action),
        'launch.launch_description_sources': module('launch.launch_description_sources', PythonLaunchDescriptionSource=Action),
        'launch.substitutions': module('launch.substitutions', LaunchConfiguration=Action),
        'launch_ros': module('launch_ros'),
        'launch_ros.actions': module('launch_ros.actions', Node=Action),
        'launch_ros.parameter_descriptions': module('launch_ros.parameter_descriptions', ParameterValue=Action),
    }
    path = Path(__file__).resolve().parents[1] / 'launch/input.launch.py'
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location('recording_input_launch_test', path)
        launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launch)
        yield launch.generate_launch_description().args[0]


class RecordingLaunchTests(unittest.TestCase):
    def test_one_scoped_recorder_waits_for_events_with_an_input_only_default(self):
        with input_launch() as actions:
            declarations = {item.args[0]: item for item in actions if hasattr(item, 'default_value')}
            self.assertEqual(declarations['recording'].default_value, 'true')
            self.assertEqual(declarations['recording_profile'].default_value, 'inputs')
            self.assertIn('calibration', declarations['recording_profile'].choices)
            groups = [item for item in actions if hasattr(item, 'actions')]
            recording = [item for item in groups if item.actions[0].args[0].args[0]
                         == '/share/atd_recording/launch/recording.launch.py']
            self.assertEqual(len(recording), 1)
            self.assertTrue(recording[0].scoped)
            include = recording[0].actions[0]
            self.assertEqual(include.condition.args[0].args[0], 'recording')
            arguments = dict(include.launch_arguments)
            self.assertEqual(arguments['config'].args[0], 'recording_config')
            self.assertEqual(arguments['profile'].args[0], 'recording_profile')
            self.assertFalse(any(hasattr(item, 'package') and item.package == 'atd_recording'
                                 for item in actions))

    def test_recorder_config_does_not_replace_joy_or_bridge_config(self):
        with input_launch() as actions:
            joy = next(item.actions[0] for item in actions if hasattr(item, 'actions')
                       and item.actions[0].args[0].args[0].endswith('/atd_joy/launch/joy.launch.py'))
            self.assertEqual(dict(joy.launch_arguments)['config'].args[0], 'joy_config')
            bridge = next(item for item in actions if getattr(item, 'package', None) == 'atd_web_bridge')
            self.assertEqual(bridge.parameters[0].args[0], 'config')
            self.assertEqual(bridge.parameters[1]['drive_tuning'].value_type, bool)
            self.assertEqual(next(x.default_value for x in actions if getattr(x, 'args', ()) == ('drive_tuning',)),
                             'false')


if __name__ == '__main__':
    unittest.main()

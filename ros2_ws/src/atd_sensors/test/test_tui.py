from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from atd_sensors import tui
from atd_sensors.profile import DEFAULTS, load
from test_profile import device


class TuiTests(unittest.TestCase):
    def test_numeric_picker_and_cancel_without_fzf(self):
        with patch('shutil.which', return_value=None), patch('sys.stdin', io.StringIO('2\n')):
            self.assertEqual(tui.choose('mode', [('off', 'OFF'), ('on', 'ON')]), 'on')
        with patch('shutil.which', return_value=None), patch('sys.stdin', io.StringIO('\n')), self.assertRaises(tui.Cancelled):
            tui.choose('mode', [('off', 'OFF')])

    def test_fzf_output_is_whitelisted(self):
        class Result:
            returncode = 0
            stdout = '$(touch forbidden)\tlabel\n'
        with patch('shutil.which', return_value='/fake/fzf'), patch('subprocess.run', return_value=Result()), \
                self.assertRaises(ValueError):
            tui.choose('mode', [('off', 'OFF')])

    def test_modes_are_sdk_derived_and_off_is_always_possible(self):
        settings = deepcopy(DEFAULTS)
        hardware = device()
        hardware['profiles'] = [value for value in hardware['profiles'] if value['stream'] != 'depth']
        with patch('atd_sensors.tui.choose', return_value='off') as choose:
            tui.edit_stream(settings, hardware, 'depth')
        self.assertEqual(choose.call_args.args[1], [('off', 'OFF')])
        self.assertFalse(settings['streams']['depth']['enabled'])

    def test_infra_intersection_requires_both_enabled_sides(self):
        hardware = device()
        hardware['profiles'].append({'stream': 'infra1', 'format': 'Y8', 'width': 848, 'height': 480, 'fps': 60})
        settings = deepcopy(DEFAULTS)
        self.assertNotIn((848, 480, 60), tui.infrared_modes(settings, hardware))
        settings['streams']['infra2']['enabled'] = False
        self.assertIn((848, 480, 60), tui.infrared_modes(settings, hardware))

    def test_consumed_camera_overrides_do_not_undo_saved_tui_edits(self):
        literal = 'profile:=$(touch /must-not-run)'
        result = tui.launch_arguments(['enable_depth:=true', 'camera_serial:=123456', literal,
                                       'vehicle_profile:=/path with spaces/layout.yaml'], Path('/tmp/d455.yaml'))
        self.assertEqual(result, [literal, 'vehicle_profile:=/path with spaces/layout.yaml',
                                  'camera_config:=/tmp/d455.yaml'])

    def test_dry_run_uses_no_sdk_or_ros_and_writes_no_settings(self):
        output = io.StringIO()
        with patch('atd_sensors.tui.probe.devices') as query, patch('atd_sensors.tui.save') as save, \
                patch('sys.stdout', output):
            self.assertEqual(tui.main(['--dry-run', '--', 'enable_depth:=true']), 0)
        query.assert_not_called()
        save.assert_not_called()
        self.assertIn('未照会', output.getvalue())
        self.assertIn('"depth"', output.getvalue())

    def test_save_and_launch_runs_validation_first_and_preserves_literal_other_args(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'camera settings.yaml'
            choices = iter(['start'])
            with patch('sys.stdin.isatty', return_value=True), patch('sys.stdout.isatty', return_value=True), \
                    patch('atd_sensors.tui.probe.devices', return_value=[device()]), \
                    patch('atd_sensors.tui.choose', side_effect=lambda *args: next(choices)), \
                    patch('atd_sensors.tui.load', return_value=deepcopy(DEFAULTS)), \
                    patch('atd_sensors.tui.probe.validate', side_effect=lambda value, _: value) as validate, \
                    patch('os.execvp', side_effect=SystemExit(0)) as execute, self.assertRaises(SystemExit):
                tui.main(['--bringup-profile', 'manual', '--', f'camera_config:={path}',
                          'profile:=$(touch /never)', 'enable_depth:=true'])
            validate.assert_called_once()
            self.assertTrue(load(path)['streams']['depth']['enabled'])
            command = execute.call_args.args[1]
            self.assertEqual(command[:4], ['ros2', 'launch', 'atd_bringup', 'manual.launch.py'])
            self.assertIn('profile:=$(touch /never)', command)
            self.assertNotIn('enable_depth:=true', command)


if __name__ == '__main__':
    unittest.main()

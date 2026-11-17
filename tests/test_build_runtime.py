"""ROS build は専用処理が所有し、環境ライブラリだけを共有することを検証。"""

import json
import shlex
import unittest

from python_env_fixture import PROJECT, PythonEnvironmentFixture


class BuildRuntimeTests(PythonEnvironmentFixture, unittest.TestCase):
    script = PROJECT / 'docker/scripts/atd-build.sh'

    def setUp(self):
        super().setUp()
        self.env['ATD_TEST_FAKE_VENV_PYTHON'] = '1'
        self.python_helper_called = self.root / 'python-helper-was-called'
        helper = self.bin / 'atd-python'
        helper.write_text('#!/bin/sh\ntouch ' + shlex.quote(str(self.python_helper_called)) + '\nexit 91\n')
        helper.chmod(0o755)

    def build(self):
        return json.loads(self.build_log.read_text())

    def test_build_uses_ros_interpreter_and_preserves_user_cmake_arguments(self):
        result = self.invoke('--packages-select', 'atd_example',
                             '--cmake-args', '-DCMAKE_BUILD_TYPE=Release')
        self.assertEqual(result.returncode, 0, result.stderr)
        build = self.build()
        self.assertEqual(build['cwd'], str(self.workspace))
        self.assertEqual(build['args'], ['/usr/bin/colcon', 'build', '--base-paths', str(self.workspace / 'src'),
                                         '--packages-select', 'atd_example',
                                         '--cmake-args', '-DPython3_EXECUTABLE=' + str(self.venv('ros') / 'bin/python'),
                                         '-DCMAKE_BUILD_TYPE=Release'])
        self.assertEqual(build['venv'], str(self.venv('ros')))
        self.assertFalse(self.python_helper_called.exists())
        creation = next(call for call in self.calls() if call['args'][0] == 'venv')
        self.assertIn('--system-site-packages', creation['args'])
        self.assertIn('--no-managed-python', creation['args'])
        self.assertIn('--no-python-downloads', creation['args'])
        sync = next(call for call in self.calls() if call['args'][0] == 'sync')
        self.assertIn('--locked', sync['args'])
        self.assertEqual(sync['env']['UV_PROJECT_ENVIRONMENT'], str(self.venv('ros')))

    def test_inline_cmake_args_are_forwarded_with_the_ros_interpreter(self):
        result = self.invoke('--cmake-args=-DCMAKE_BUILD_TYPE=Release')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.build()['args'], [
            '/usr/bin/colcon', 'build', '--base-paths', str(self.workspace / 'src'), '--cmake-args',
            '-DPython3_EXECUTABLE=' + str(self.venv('ros') / 'bin/python'), '-DCMAKE_BUILD_TYPE=Release'])

    def test_python_cmake_override_and_repeated_cmake_groups_are_rejected(self):
        for arguments in (
                ('--cmake-args', '-DPython3_EXECUTABLE:FILEPATH=/usr/bin/python3'),
                ('--cmake-args=-DPython3_EXECUTABLE:FILEPATH=/usr/bin/python3',),
                ('--cmake-args', '-DPYTHON_EXECUTABLE=/other/python'),
                ('--cmake-args', '-DCMAKE_BUILD_TYPE=Release', '--cmake-args=-DFOO=1')):
            with self.subTest(arguments=arguments):
                result = self.invoke(*arguments)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.build_log.exists())
                self.assertFalse(self.python_helper_called.exists())

    def test_explicit_package_search_path_is_preserved(self):
        source = self.root / 'selected sources'
        result = self.invoke('--base-paths', str(source))
        self.assertEqual(result.returncode, 0, result.stderr)
        arguments = self.build()['args']
        self.assertEqual(arguments.count('--base-paths'), 1)
        self.assertEqual(arguments[arguments.index('--base-paths') + 1], str(source))

    def test_inline_package_search_path_does_not_add_a_default(self):
        source = '--base-paths=' + str(self.root / 'selected sources')
        result = self.invoke(source)
        self.assertEqual(result.returncode, 0, result.stderr)
        arguments = self.build()['args']
        self.assertIn(source, arguments)
        self.assertNotIn('--base-paths', arguments)

    def test_literal_arguments_and_colcon_exit_status_are_preserved(self):
        sentinel = self.root / 'must-not-exist'
        values = ['-DCONFIG=path with spaces', '$(touch ' + str(sentinel) + ');*']
        result = self.invoke('--cmake-args', *values, env={'ATD_TEST_BUILD_EXIT': '23'})
        self.assertEqual(result.returncode, 23, result.stderr)
        self.assertEqual(self.build()['args'][-2:], values)
        self.assertFalse(sentinel.exists())
        self.assertFalse(self.python_helper_called.exists())

    def test_ros_setup_and_overlay_preserve_the_selected_dds_environment(self):
        overlay = self.workspace / 'install/setup.bash'
        overlay.parent.mkdir()
        overlay.write_text('export ATD_TEST_OVERLAY=loaded\nexport RMW_IMPLEMENTATION=old-overlay\n')
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        environment = self.build()['env']
        self.assertEqual(environment['RMW_IMPLEMENTATION'], 'rmw_fastrtps_cpp')
        self.assertIsNone(environment['ROS_DISCOVERY_SERVER'])
        self.assertEqual(environment['ATD_TEST_OVERLAY'], 'loaded')
        self.assertEqual(environment['PYTHONPATH'], str(self.ros_lib))

    def test_sync_failure_prevents_colcon(self):
        result = self.invoke(env={'ATD_TEST_SYNC_EXIT': '17'})
        self.assertEqual(result.returncode, 17, result.stderr)
        self.assertFalse(self.build_log.exists())
        self.assertFalse(self.python_helper_called.exists())

    def test_existing_venv_is_checked_before_colcon_and_sync(self):
        self.assertEqual(self.invoke().returncode, 0)
        self.log.unlink()
        self.build_log.unlink()
        config = self.venv('ros') / 'pyvenv.cfg'
        config.write_text(config.read_text().replace('include-system-site-packages = true',
                                                    'include-system-site-packages = false'))
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('system-site-packages', result.stderr)
        self.assertEqual(self.calls(), [])
        self.assertFalse(self.build_log.exists())

    def test_help_does_not_prepare_the_environment(self):
        result = self.invoke('--help')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls(), [])
        self.assertFalse((self.workspace / '.atd-python').exists())
        self.assertFalse(self.build_log.exists())


if __name__ == '__main__':
    unittest.main()

from contextlib import contextmanager
import json
from pathlib import Path
import runpy
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


class Shutdown(Action):
    pass


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


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
                                 GroupAction=Action, IncludeLaunchDescription=Action, OpaqueFunction=Action, Shutdown=Shutdown),
        'launch.conditions': module('launch.conditions', IfCondition=Action),
        'launch.launch_description_sources': module('launch.launch_description_sources',
                                                    PythonLaunchDescriptionSource=Action),
        'launch.substitutions': module('launch.substitutions', AndSubstitution=Action, LaunchConfiguration=Configuration),
        'launch_ros.actions': module('launch_ros.actions', Node=Action, ComposableNodeContainer=Action),
        'launch_ros.descriptions': module('launch_ros.descriptions', ComposableNode=Action),
        'launch_ros.parameter_descriptions': module('launch_ros.parameter_descriptions', ParameterValue=Action),
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

    def test_gazebo_profile_runs_only_stereo_vo_with_the_simulation_clock(self):
        path = Path(__file__).resolve().parents[1] / 'launch/gazebo.launch.py'
        with fake_launch():
            clock, container = runpy.run_path(str(path))['generate_launch_description']().entities
        self.assertEqual((clock.args[0], clock.default_value, clock.choices), ('use_sim_time', 'true', ['true']))
        tracker, = container.composable_node_descriptions
        self.assertEqual(container.package, 'rclcpp_components')
        self.assertEqual(tracker.package, 'isaac_ros_cuvslam')
        params, = tracker.parameters
        sim_clock = params['use_sim_time']
        self.assertIs(container.parameters[0]['use_sim_time'], sim_clock)
        self.assertEqual((sim_clock.args[0].name, sim_clock.value_type), ('use_sim_time', bool))
        self.assertEqual((params['num_cameras'], params['min_num_images'], params['tracking_mode']), (2, 2, 0))
        self.assertEqual(params['image_qos'], 'SENSOR_DATA')
        self.assertFalse(params['override_publishing_stamp'])
        self.assertFalse(params['enable_localization_n_mapping'])
        self.assertEqual(params['base_frame'], 'base_link')
        self.assertEqual(params['camera_optical_frames'],
                         ['camera_infra1_optical_frame', 'camera_infra2_optical_frame'])
        remaps = dict(tracker.remappings)
        for index in (0, 1):
            prefix = f'/camera/camera/infra{index + 1}'
            self.assertEqual(remaps[f'visual_slam/image_{index}'], prefix + '/image_rect_raw')
            self.assertEqual(remaps[f'visual_slam/camera_info_{index}'], prefix + '/camera_info')

    def test_bag_vslam_loads_the_mapper_component_for_live_visualization(self):
        path = Path(__file__).resolve().parents[1] / 'launch/bag_vslam.launch.py'
        with fake_launch():
            container, = runpy.run_path(str(path))['generate_launch_description']().entities
        tracker, = container.composable_node_descriptions
        self.assertEqual((container.package, container.executable), ('rclcpp_components', 'component_container_mt'))
        self.assertEqual((tracker.package, tracker.plugin),
                         ('isaac_ros_cuvslam', 'nvidia::isaac_ros::visual_slam::VisualSlamNode'))
        self.assertEqual(tracker.extra_arguments, [{'use_intra_process_comms': True}])
        params, = tracker.parameters
        self.assertTrue(container.parameters[0]['use_sim_time'])
        for flag in ('use_sim_time', 'enable_localization_n_mapping', 'enable_slam_visualization',
                     'enable_landmarks_view', 'publish_map_to_odom_tf', 'publish_odom_to_base_tf'):
            self.assertIs(params[flag], True)
        self.assertEqual(params['path_max_size'], 50000)
        self.assertEqual(params['base_frame'], 'base_link')
        remaps = dict(tracker.remappings)
        for i in (0, 1):
            prefix = f'/camera/camera/infra{i + 1}'
            self.assertEqual(remaps[f'visual_slam/image_{i}'], prefix + '/image_rect_raw')
            self.assertEqual(remaps[f'visual_slam/camera_info_{i}'], prefix + '/camera_info')

    def test_gazebo_mapping_launch_uses_slam_and_shuts_down_with_the_supervisor(self):
        path = Path(__file__).resolve().parents[1] / 'launch/gazebo_mapping.launch.py'
        with fake_launch():
            run_dir, map_dir, container, monitor = runpy.run_path(str(path))['generate_launch_description']().entities
        self.assertEqual([arg.args[0] for arg in (run_dir, map_dir)], ['run_dir', 'map_dir'])
        params, = container.composable_node_descriptions[0].parameters
        self.assertTrue(params['enable_localization_n_mapping'])
        self.assertTrue(params['publish_map_to_odom_tf'])
        self.assertTrue(params['use_sim_time'])
        self.assertFalse(params['override_publishing_stamp'])
        self.assertEqual(monitor.executable, 'gazebo_lap_map')
        self.assertEqual([x.name for x in monitor.arguments[1::2]], ['run_dir', 'map_dir'])
        self.assertEqual(monitor.parameters, [{'use_sim_time': True}])
        self.assertIsInstance(monitor.on_exit[0], Shutdown)
        self.assertIsInstance(container.on_exit[0], Shutdown)

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
        self.assertEqual(inputs['recording'].name, 'recording')
        self.assertEqual(inputs['recording_profile'].name, 'recording_profile')
        self.assertEqual(inputs['recording_config'].name, 'recording_config')
        self.assertNotIn('recording_config', camera)
        self.assertNotIn('recording_config', vehicle)

    def test_persistent_camera_defaults_and_stream_contract_are_declared(self):
        from atd_bringup.launching import CAMERA_ARGUMENTS
        declarations = {value.args[0]: value.default_value for value in self.generate(True)
                        if hasattr(value, 'default_value')}
        self.assertEqual(declarations['camera_config'], '/data/results/runs/bringup/d455.yaml')
        self.assertEqual(declarations['vehicle_profile'], '/data/results/runs/vehicle/layout.yaml')
        self.assertTrue(all(key in declarations for key in CAMERA_ARGUMENTS))
        self.assertEqual(declarations['recording'], 'true')
        self.assertEqual(declarations['recording_profile'], 'mapping')
        self.assertEqual(declarations['recording_config'], '')

    def test_drive_profiles_select_joy_writer_and_matching_pico_with_or_without_camera(self):
        launch_dir = Path(__file__).resolve().parents[1] / 'launch'
        for profile, drive_default, recording_default, camera in (
                ('joy_drive', 'true', 'inputs', False), ('manual', 'true', 'mapping', True),
                ('localization', 'false', 'mapping', True)):
            with self.subTest(profile=profile), fake_launch():
                entry = runpy.run_path(str(launch_dir / f'{profile}.launch.py'))
                entities = entry['generate_launch_description']().entities
                arguments = {e.args[0]: e for e in entities if hasattr(e, 'default_value')}
                self.assertEqual(arguments['joy_drive'].default_value, drive_default)
                self.assertEqual(arguments['joy_drive'].choices, ['true', 'false'])
                self.assertEqual(arguments['joy_drive_port'].default_value, '')
                self.assertEqual(arguments['pico_serial'].default_value, '')
                self.assertEqual(arguments['recording_profile'].default_value, recording_default)
                writer, = [e for e in entities if getattr(e, 'executable', None) == 'joy_drive']
                self.assertEqual((writer.package, writer.name), ('atd_vehicle_interface', 'atd_joy_drive'))
                self.assertEqual([v.name for v in writer.condition.args[0].args], ['teleop', 'joy_drive'])
                params, = writer.parameters
                self.assertEqual({k: (v.args[0].name, v.value_type) for k, v in params.items()},
                                 {'port': ('joy_drive_port', str), 'serial': ('pico_serial', str)})
                included = [e.actions[0].args[0].args[0] for e in entities if hasattr(e, 'actions')]
                self.assertEqual(included.count('/share/atd_web_bridge/launch/input.launch.py'), 1)
                self.assertEqual('/share/atd_sensors/launch/d455.launch.py' in included, camera)
                self.assertEqual('camera_config' in arguments, camera)

    def test_localization_wires_planner_horizon_and_controller_to_the_vehicle_profile(self):
        from atd_bringup.launching import description
        with fake_launch():
            entities = description(localization=True).entities
            context = {e.args[0]: e.default_value for e in entities if hasattr(e, 'default_value')}
            planning = next(e.function for e in entities if hasattr(e, 'function'))
            plan_arguments = lambda ctx: dict(planning(ctx)[0].actions[0].launch_arguments)
            self.assertEqual(float(plan_arguments(context)['horizon_m']), 2.)
            context['max_lookahead_m'] = '3.5'
            self.assertEqual(float(plan_arguments(context)['horizon_m']), 3.5)
            groups = [g for e in entities for g in (e.function(context) if hasattr(e, 'function') else [e])
                      if hasattr(g, 'actions')]
        included = [group.actions[0] for group in groups]
        self.assertEqual([entry.args[0].args[0] for entry in included], [
            '/share/atd_sensors/launch/d455.launch.py', '/share/atd_vehicle_description/launch/vehicle.launch.py',
            '/share/atd_web_bridge/launch/input.launch.py', '/share/atd_localization/launch/localization.launch.py',
            '/share/atd_planning/launch/planning.launch.py', '/share/atd_control/launch/control.launch.py'])
        self.assertEqual({k: v.name for k, v in included[-3].launch_arguments},
                         {k: k for k in ('map_id', 'results_root', 'engine_dir')})
        self.assertEqual({k: v.name for k, v in included[-2].launch_arguments if k != 'horizon_m'},
                         {k: k for k in ('map_id', 'results_root', 'line')})
        self.assertEqual({k: v.name for k, v in included[-1].launch_arguments}, {
            'profile': 'vehicle_profile', 'min_lookahead_m': 'min_lookahead_m',
            'lookahead_time_s': 'lookahead_time_s', 'max_lookahead_m': 'max_lookahead_m',
            'feedforward_path': 'feedforward_path', 'speed_kp': 'speed_kp', 'speed_ki': 'speed_ki', 'speed_kd': 'speed_kd'})

    def test_sensor_only_profile_does_not_start_or_configure_a_recorder(self):
        entities = self.generate(False)
        declarations = {value.args[0] for value in entities if hasattr(value, 'default_value')}
        self.assertTrue({'recording', 'recording_profile', 'recording_config', 'joy_drive'}.isdisjoint(declarations))
        self.assertFalse(any(getattr(e, 'executable', None) == 'joy_drive' for e in entities))

    def test_autonomous_uses_one_pico_writer_and_one_speed_limit(self):
        launch = Path(__file__).resolve().parents[1] / 'launch/autonomous.launch.py'
        with fake_launch():
            entities = runpy.run_path(str(launch))['generate_launch_description']().entities
        arguments = {e.args[0]: e.default_value for e in entities if hasattr(e, 'default_value')}
        self.assertNotIn('joy_drive', arguments)
        self.assertEqual(arguments['max_speed_mps'], '0.5')
        self.assertEqual(arguments['auto_calibration_path'], '/data/results/runs/vehicle/auto_drive.yaml')
        writer, = [e for e in entities if getattr(e, 'package', None) == 'atd_vehicle_interface']
        self.assertEqual(writer.executable, 'auto_drive')
        shutdown, = writer.on_exit
        self.assertIsInstance(shutdown, Shutdown)
        self.assertIn('Pico 出力ノードが終了', shutdown.reason)
        params, = writer.parameters
        self.assertEqual(params['port'].args[0].name, 'auto_drive_port')
        self.assertEqual(params['serial'].args[0].name, 'pico_serial')
        self.assertEqual(params['map_id'].args[0].name, 'map_id')
        self.assertEqual(params['line'].args[0].name, 'line')
        groups = [e.actions[0] for e in entities if hasattr(e, 'actions')]
        included = {e.args[0].args[0]: dict(e.launch_arguments) for e in groups}
        self.assertIn('/share/atd_sensors/launch/d455.launch.py', included)
        self.assertIn('/share/atd_localization/launch/localization.launch.py', included)
        controller = included['/share/atd_control/launch/control.launch.py']
        self.assertIs(controller['max_target_speed_mps'], params['max_speed_mps'].args[0])
        self.assertIs(controller['feedforward_path'], params['feedforward_path'].args[0])
        # Explicit map_id/line on the CLI bypasses the chooser. Validate the saved line before the writer starts.
        context = {**arguments, 'map_id': 'course_a', 'line': 'custom'}
        planning = next(e.function for e in entities if hasattr(e, 'function'))
        coords = [[0., 0.], [1., 0.]]
        route = dict(points=coords, profile=[[i, x, y, 0, 0, .2, 0] for i, (x, y) in enumerate(coords)])
        doc = json.dumps(dict(frame='map', closed=False, lines={'custom': route})).encode()
        with fake_launch(), patch('atd_localization.assets.map_metadata', return_value={}) as meta, \
                patch('atd_localization.assets.verified_hdmap', return_value=doc):
            group, = planning(context)
            self.assertEqual(dict(group.actions[0].launch_arguments)['line'].perform(context), 'custom')
            meta.assert_called_once_with('/data/results', 'course_a')
            with self.assertRaisesRegex(ValueError, 'raceline'):
                planning({**context, 'line': 'raceline'})


if __name__ == '__main__':
    unittest.main()

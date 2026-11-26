"""D455/vehicle/input の launch 引数を scope ごとに分離する。"""
from pathlib import Path

CAMERA_ARGUMENTS = ('enable_infra1', 'enable_infra2', 'enable_color', 'enable_depth',
                    'enable_accel', 'enable_gyro', 'infra_profile', 'color_profile',
                    'depth_profile', 'accel_fps', 'gyro_fps', 'unite_imu_method')
INPUT_ARGUMENTS = ('config', 'profile', 'joy_config', 'teleop', 'teleop_profile', 'teleop_config',
                   'propo', 'pico_config', 'pico_serial', 'pico_sensor_path', 'rc_config', 'rc_profile',
                   'vehicle_profile', 'recording', 'recording_profile', 'recording_config')


def description(manual=False, localization=False, camera=True, autonomous=False):
    localization = localization or autonomous
    manual = manual or localization
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, Shutdown
    from launch.conditions import IfCondition
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import AndSubstitution, LaunchConfiguration
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue

    def share(package):
        return Path(get_package_share_directory(package))

    def include(package, launch, arguments):
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share(package) / 'launch' / launch)),
            launch_arguments=arguments.items())])

    defaults = {'vehicle_profile': '/data/results/runs/vehicle/layout.yaml'}
    if camera:
        defaults.update({
            'camera_config': '/data/results/runs/bringup/d455.yaml', 'camera_serial': '',
            'vehicle_config': str(share('atd_vehicle_description') / 'config/vehicle.yaml'),
            **{key: '' for key in CAMERA_ARGUMENTS},
        })
    if manual:
        defaults.update({
            'config': str(share('atd_web_bridge') / 'config/bridge.yaml'),
            'profile': '/data/results/runs/input/dualsense.yaml',
            'joy_config': str(share('atd_joy') / 'config/joy.yaml'), 'teleop': 'true',
            'teleop_profile': '/data/results/runs/input/teleop.yaml',
            'teleop_config': str(share('atd_teleop') / 'config/teleop.yaml'), 'propo': 'true',
            'pico_config': str(share('atd_vehicle_interface') / 'config/pico.yaml'),
            'pico_serial': '', 'pico_sensor_path': '',
            'rc_config': str(share('atd_rc_input') / 'config/rc.yaml'),
            'rc_profile': '/data/results/runs/input/propo.yaml',
            'recording': 'true', 'recording_profile': 'mapping' if camera else 'inputs', 'recording_config': '',
        })
        if not autonomous:
            defaults.update(joy_drive='false' if localization else 'true', joy_drive_port='')
    if localization:
        from atd_bringup.replay_inputs import GYRO_TOPIC
        from atd_control.core import LookaheadConfig
        from atd_control.speed import DEFAULT_FEEDFORWARD_PATH, PIDConfig
        from atd_planning.core import DEFAULT_HORIZON_M
        tuning = ('min_lookahead_m', 'lookahead_time_s', 'max_lookahead_m')
        controller = LookaheadConfig()
        defaults.update(map_id='', results_root='/data/results', engine_dir='', line='raceline')
        defaults.update(orb_gyro_bridge='false', orb_gyro_topic=GYRO_TOPIC)
        defaults.update({key: str(getattr(controller, key)) for key in tuning})
        pid = PIDConfig()
        speed_arguments = {'feedforward_path': DEFAULT_FEEDFORWARD_PATH,
                           **{'speed_' + k: str(getattr(pid, k)) for k in ('kp', 'ki', 'kd')}}
        defaults.update(speed_arguments)
    if autonomous:
        defaults.update(auto_drive_port='', max_speed_mps='0.5',
                        auto_calibration_path='/data/results/runs/vehicle/auto_drive.yaml')
    arguments = {key: LaunchConfiguration(key) for key in defaults}
    actions = [DeclareLaunchArgument(key, default_value=value,
                                     choices=['true', 'false'] if key in ('joy_drive', 'orb_gyro_bridge') else None)
               for key, value in defaults.items()]
    if camera:
        actions += [
            include('atd_sensors', 'd455.launch.py', {
                'config': arguments['camera_config'], 'serial': arguments['camera_serial'],
                **{key: arguments[key] for key in CAMERA_ARGUMENTS}}),
            include('atd_vehicle_description', 'vehicle.launch.py', {
                'config': arguments['vehicle_config'], 'profile': arguments['vehicle_profile']}),
        ]
    if manual:
        # input.launch.py already owns the only web_bridge instance.
        inputs = {key: arguments[key] for key in INPUT_ARGUMENTS}
        if autonomous:
            inputs['drive_tuning'] = 'true'
        actions.append(include('atd_web_bridge', 'input.launch.py', inputs))
    if manual and not autonomous:
        actions.append(Node(package='atd_vehicle_interface', executable='joy_drive', name='atd_joy_drive',
                            output='screen', condition=IfCondition(AndSubstitution(arguments['teleop'], arguments['joy_drive'])),
                            parameters=[{'port': ParameterValue(arguments['joy_drive_port'], value_type=str),
                                         'serial': ParameterValue(arguments['pico_serial'], value_type=str)}]))
    if localization:
        actions.append(include('atd_localization', 'localization.launch.py',
                               {key: arguments[key] for key in ('map_id', 'results_root', 'engine_dir')}))
        actions.append(Node(package='atd_bringup', executable='orb_gyro_bridge', output='screen',
                            condition=IfCondition(arguments['orb_gyro_bridge']),
                            parameters=[{key: ParameterValue(arguments[arg], value_type=str) for key, arg in
                                         [('map_id', 'map_id'), ('results_root', 'results_root'),
                                          ('imu_topic', 'orb_gyro_topic')]}]))
        def planning(context):
            if autonomous:
                import json
                from atd_localization.assets import map_metadata, verified_hdmap
                from atd_planning.core import ReferenceLine
                root = arguments['results_root'].perform(context)
                metadata = map_metadata(root, arguments['map_id'].perform(context))
                ReferenceLine.from_document(json.loads(verified_hdmap(root, metadata)), arguments['line'].perform(context))
            config = LookaheadConfig(**{key: float(arguments[key].perform(context)) for key in tuning})
            horizon = max(DEFAULT_HORIZON_M, config.max_lookahead_m)
            params = {key: arguments[key] for key in ('map_id', 'results_root', 'line')}
            params['horizon_m'] = str(horizon)
            if autonomous:
                params['reference_line_topic'] = '/vehicle/drive_tuning'
            return [include('atd_planning', 'planning.launch.py', params)]

        actions.append(OpaqueFunction(function=planning))
        control = {'profile': arguments['vehicle_profile'], **{key: arguments[key] for key in (*tuning, *speed_arguments)}}
        if autonomous:
            control['max_target_speed_mps'] = arguments['max_speed_mps']
            control['drive_tuning'] = 'true'
        actions.append(include('atd_control', 'control.launch.py', control))
    if autonomous:
        auto_params = {key: ParameterValue(arguments[key], value_type=str)
                       for key in ('map_id', 'results_root', 'line', 'auto_calibration_path', 'feedforward_path')}
        auto_params.update(port=ParameterValue(arguments['auto_drive_port'], value_type=str),
                           serial=ParameterValue(arguments['pico_serial'], value_type=str),
                           drive_tuning=True)
        for key in ('max_speed_mps', *tuning, 'speed_kp', 'speed_ki', 'speed_kd'):
            auto_params[key] = ParameterValue(arguments[key], value_type=float)
        actions.append(Node(package='atd_vehicle_interface', executable='auto_drive', name='atd_auto_drive',
                            output='screen', parameters=[auto_params],
                            on_exit=[Shutdown(reason='自動走行の Pico 出力ノードが終了しました')]))
    return LaunchDescription(actions)

"""D455/vehicle/input の launch 引数を scope ごとに分離する。"""
from pathlib import Path

CAMERA_ARGUMENTS = ('enable_infra1', 'enable_infra2', 'enable_color', 'enable_depth',
                    'enable_accel', 'enable_gyro', 'infra_profile', 'color_profile',
                    'depth_profile', 'accel_fps', 'gyro_fps', 'unite_imu_method')
INPUT_ARGUMENTS = ('config', 'profile', 'joy_config', 'teleop', 'teleop_profile', 'teleop_config',
                   'propo', 'pico_config', 'pico_serial', 'pico_sensor_path', 'rc_config', 'rc_profile',
                   'vehicle_profile')


def description(manual=False):
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration

    def share(package):
        return Path(get_package_share_directory(package))

    def include(package, launch, arguments):
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share(package) / 'launch' / launch)),
            launch_arguments=arguments.items())])

    defaults = {
        'camera_config': '/data/results/runs/bringup/d455.yaml', 'camera_serial': '',
        'vehicle_config': str(share('atd_vehicle_description') / 'config/vehicle.yaml'),
        'vehicle_profile': '/data/results/runs/vehicle/layout.yaml',
        **{key: '' for key in CAMERA_ARGUMENTS},
    }
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
        })
    arguments = {key: LaunchConfiguration(key) for key in defaults}
    actions = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    actions += [
        include('atd_sensors', 'd455.launch.py', {
            'config': arguments['camera_config'], 'serial': arguments['camera_serial'],
            **{key: arguments[key] for key in CAMERA_ARGUMENTS}}),
        include('atd_vehicle_description', 'vehicle.launch.py', {
            'config': arguments['vehicle_config'], 'profile': arguments['vehicle_profile']}),
    ]
    if manual:
        # input.launch.py already owns the only web_bridge instance.
        actions.append(include('atd_web_bridge', 'input.launch.py',
                               {key: arguments[key] for key in INPUT_ARGUMENTS}))
    return LaunchDescription(actions)

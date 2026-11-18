"""Jetson の入力・操作要求・記録待機・監視を起動する。録画は操作イベントで開始する。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(path, arguments, condition=None):
    # 子launchの config/profile/namespace 等を他の入力やbridgeへ漏らさない。
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)), condition=condition,
        launch_arguments=arguments.items())])


def generate_launch_description():
    share = Path(get_package_share_directory('atd_web_bridge'))
    joy = Path(get_package_share_directory('atd_joy'))
    teleop = Path(get_package_share_directory('atd_teleop'))
    pico = Path(get_package_share_directory('atd_vehicle_interface'))
    rc = Path(get_package_share_directory('atd_rc_input'))
    recording = Path(get_package_share_directory('atd_recording'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/bridge.yaml')),
        DeclareLaunchArgument('profile', default_value='/data/results/runs/input/dualsense.yaml'),
        DeclareLaunchArgument('joy_config', default_value=str(joy / 'config/joy.yaml')),
        DeclareLaunchArgument('teleop', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('teleop_profile', default_value='/data/results/runs/input/teleop.yaml'),
        DeclareLaunchArgument('teleop_config', default_value=str(teleop / 'config/teleop.yaml')),
        DeclareLaunchArgument('propo', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('pico_config', default_value=str(pico / 'config/pico.yaml')),
        DeclareLaunchArgument('pico_serial', default_value=''),
        DeclareLaunchArgument('pico_sensor_path', default_value=''),
        DeclareLaunchArgument('rc_config', default_value=str(rc / 'config/rc.yaml')),
        DeclareLaunchArgument('rc_profile', default_value='/data/results/runs/input/propo.yaml'),
        DeclareLaunchArgument('vehicle_profile', default_value='/data/results/runs/vehicle/layout.yaml'),
        DeclareLaunchArgument('recording', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('recording_profile', default_value='inputs', choices=['mapping', 'calibration', 'inputs']),
        DeclareLaunchArgument('recording_config', default_value=''),
        include(joy / 'launch/joy.launch.py',
                {'profile': LaunchConfiguration('profile'), 'config': LaunchConfiguration('joy_config')}),
        include(teleop / 'launch/teleop.launch.py',
                {'profile': LaunchConfiguration('teleop_profile'), 'config': LaunchConfiguration('teleop_config')},
                IfCondition(LaunchConfiguration('teleop'))),
        include(pico / 'launch/pico.launch.py',
                {'config': LaunchConfiguration('pico_config'), 'serial': LaunchConfiguration('pico_serial'),
                 'sensor_path': LaunchConfiguration('pico_sensor_path')}, IfCondition(LaunchConfiguration('propo'))),
        include(rc / 'launch/rc.launch.py',
                {'profile': LaunchConfiguration('rc_profile'), 'config': LaunchConfiguration('rc_config')},
                IfCondition(LaunchConfiguration('propo'))),
        include(recording / 'launch/recording.launch.py',
                {'profile': LaunchConfiguration('recording_profile'),
                 'config': LaunchConfiguration('recording_config')}, IfCondition(LaunchConfiguration('recording'))),
        Node(package='atd_web_bridge', executable='web_bridge', output='screen',
             parameters=[LaunchConfiguration('config'), {'profile_path': LaunchConfiguration('profile'),
                          'teleop_profile_path': LaunchConfiguration('teleop_profile'),
                          'rc_profile_path': LaunchConfiguration('rc_profile'),
                          'vehicle_profile_path': LaunchConfiguration('vehicle_profile')}]),
    ])

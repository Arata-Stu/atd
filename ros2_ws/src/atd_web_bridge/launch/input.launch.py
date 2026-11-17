"""Jetson の入力・操作要求・監視を起動する。基板出力や記録は開始しない。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('atd_web_bridge'))
    joy = Path(get_package_share_directory('atd_joy'))
    teleop = Path(get_package_share_directory('atd_teleop'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/bridge.yaml')),
        DeclareLaunchArgument('profile', default_value='/data/results/runs/input/dualsense.yaml'),
        DeclareLaunchArgument('teleop', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('teleop_profile', default_value='/data/results/runs/input/teleop.yaml'),
        DeclareLaunchArgument('teleop_config', default_value=str(teleop / 'config/teleop.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(joy / 'launch/joy.launch.py')),
                                 launch_arguments={'profile': LaunchConfiguration('profile')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(teleop / 'launch/teleop.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('teleop')),
                                 launch_arguments={'profile': LaunchConfiguration('teleop_profile'),
                                                   'config': LaunchConfiguration('teleop_config')}.items()),
        Node(package='atd_web_bridge', executable='web_bridge', output='screen',
             parameters=[LaunchConfiguration('config'), {'profile_path': LaunchConfiguration('profile'),
                          'teleop_profile_path': LaunchConfiguration('teleop_profile')}]),
    ])

"""Explicit Jetson-side launch; never starts vehicle control or bag recording."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('atd_web_bridge'))
    joy = Path(get_package_share_directory('atd_joy'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/bridge.yaml')),
        DeclareLaunchArgument('profile', default_value='/data/results/runs/input/dualsense.yaml'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(joy / 'launch/joy.launch.py')),
                                 launch_arguments={'profile': LaunchConfiguration('profile')}.items()),
        Node(package='atd_web_bridge', executable='web_bridge', output='screen',
             parameters=[LaunchConfiguration('config'), {'profile_path': LaunchConfiguration('profile')}]),
    ])

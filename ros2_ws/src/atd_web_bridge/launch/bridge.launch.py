from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('atd_web_bridge'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/bridge.yaml')),
        DeclareLaunchArgument('vehicle_profile', default_value='/data/results/runs/vehicle/layout.yaml'),
        Node(package='atd_web_bridge', executable='web_bridge', output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'vehicle_profile_path': LaunchConfiguration('vehicle_profile')}]),
    ])

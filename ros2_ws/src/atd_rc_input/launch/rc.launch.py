from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('atd_rc_input'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/rc.yaml')),
        DeclareLaunchArgument('profile', default_value='/data/results/runs/input/propo.yaml'),
        Node(package='atd_rc_input', executable='atd_rc_input_node', output='screen',
             parameters=[LaunchConfiguration('config'), {'profile_path': LaunchConfiguration('profile')}]),
    ])

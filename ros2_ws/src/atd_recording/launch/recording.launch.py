"""選択した記録設定で、操作イベントを待つ recorder を起動する。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    config = LaunchConfiguration('config').perform(context)
    if config:
        path = Path(config)
        if not path.is_absolute():
            raise ValueError('記録設定はコンテナ内の絶対パスで指定してください')
    else:
        profile = LaunchConfiguration('profile').perform(context)
        path = Path(get_package_share_directory('atd_recording')) / 'config' / f'{profile}.yaml'
    if not path.is_file():
        raise FileNotFoundError(f'記録設定を読み込めません: {path}')
    return [Node(package='atd_recording', executable='atd_recording_node',
                 name='atd_recording_node', parameters=[str(path)], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='mapping',
                              choices=['mapping', 'calibration', 'inputs']),
        DeclareLaunchArgument('config', default_value=''),
        OpaqueFunction(function=start),
    ])

"""Notebook で再生するステレオ bag から地図を作り、点群と軌跡を配信する。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from atd_bringup.gazebo_stereo import stereo_container


def generate_launch_description():
    limit = ParameterValue(LaunchConfiguration('slam_max_map_size'), value_type=int)
    return LaunchDescription([
        DeclareLaunchArgument('slam_max_map_size', default_value='0',
                              description='SLAM 地図に保持する姿勢数の上限。0 は無制限。'),
        stereo_container(True, mapping=True, visualize_landmarks=True, slam_max_map_size=limit),
    ])

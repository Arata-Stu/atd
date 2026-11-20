"""Notebook の Gazebo のステレオ画像をシミュレーション時刻で追跡する。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from atd_bringup.gazebo_stereo import stereo_container


def generate_launch_description():
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true'],
                              description='Gazebo の /clock を使用します。'),
        stereo_container(sim_time),
    ])

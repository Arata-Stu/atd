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
        DeclareLaunchArgument('tracking_mode', default_value='0', choices=['0', '1']),
        DeclareLaunchArgument('imu_frame', default_value='camera_gyro_optical_frame'),
        DeclareLaunchArgument('imu_frequency', default_value='200.0'),
        stereo_container(True, mapping=True, visualize_landmarks=True, slam_max_map_size=limit,
                         tracking_mode=ParameterValue(LaunchConfiguration('tracking_mode'), value_type=int),
                         imu_frame=ParameterValue(LaunchConfiguration('imu_frame'), value_type=str),
                         imu_frequency=ParameterValue(LaunchConfiguration('imu_frequency'), value_type=float)),
    ])

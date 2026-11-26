"""Notebook で再生するステレオ bag から地図を作り、点群と軌跡を配信する。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from atd_bringup.gazebo_stereo import stereo_container
from atd_bringup.replay_inputs import IMU_TOPIC, STEREO_WORKFLOW
from atd_localization.bag_source import stereo_bag_input


def _container(context):
    bag = LaunchConfiguration('bag_path').perform(context)
    source = None
    if bag:
        imu = IMU_TOPIC if LaunchConfiguration('tracking_mode').perform(context) == '1' else ''
        source = stereo_bag_input(
            bag, float(LaunchConfiguration('rate').perform(context)),
            LaunchConfiguration('player_name').perform(context), STEREO_WORKFLOW,
            required_image_readers=1, imu_topic=imu)
    limit = ParameterValue(LaunchConfiguration('slam_max_map_size'), value_type=int)
    return [stereo_container(True, mapping=True, visualize_landmarks=True, slam_max_map_size=limit, bag_input=source,
                             tracking_mode=ParameterValue(LaunchConfiguration('tracking_mode'), value_type=int),
                             imu_frame=ParameterValue(LaunchConfiguration('imu_frame'), value_type=str),
                             imu_frequency=ParameterValue(LaunchConfiguration('imu_frequency'), value_type=float))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('slam_max_map_size', default_value='0',
                              description='SLAM 地図に保持する姿勢数の上限。0 は無制限。'),
        DeclareLaunchArgument('tracking_mode', default_value='0', choices=['0', '1']),
        DeclareLaunchArgument('imu_frame', default_value='camera_gyro_optical_frame'),
        DeclareLaunchArgument('imu_frequency', default_value='200.0'),
        DeclareLaunchArgument('bag_path', default_value=''),
        DeclareLaunchArgument('rate', default_value='0.5'),
        DeclareLaunchArgument('player_name', default_value='atd_replay_player'),
        OpaqueFunction(function=_container),
    ])

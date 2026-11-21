"""カメラ台の指定周回に合わせて、Notebook でシミュレーションの cuVSLAM 地図を保存する。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from atd_bringup.gazebo_stereo import stereo_container


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_dir', description='Gazebo と共有する新しい試行ディレクトリ'),
        DeclareLaunchArgument('map_dir', description='この試行の cuVSLAM 地図出力先'),
        stereo_container(True, mapping=True),
        Node(package='atd_bringup', executable='gazebo_lap_map', output='screen',
             arguments=['--run-dir', LaunchConfiguration('run_dir'), '--map-dir', LaunchConfiguration('map_dir')],
             parameters=[{'use_sim_time': True}],
             on_exit=[Shutdown(reason='Gazebo の周回地図作成を終了しました')]),
    ])

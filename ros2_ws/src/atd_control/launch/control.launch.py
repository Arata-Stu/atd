"""選択済みラインの参照軌道・目標速度を使って操舵とスロットルの計算を観測する。"""
from dataclasses import asdict

from atd_control.core import LookaheadConfig
from atd_control.speed import DEFAULT_FEEDFORWARD_PATH, PIDConfig
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lateral = dict(profile='/data/results/runs/vehicle/layout.yaml', use_sim_time=False, max_target_speed_mps=0.,
                   drive_tuning=False, **asdict(LookaheadConfig()))
    speed = dict(feedforward_path=DEFAULT_FEEDFORWARD_PATH, max_target_speed_mps=0., use_sim_time=False,
                 drive_tuning=False, **{'speed_' + key: v for key, v in asdict(PIDConfig()).items()})

    def parameters(defaults):
        return [{key: ParameterValue(LaunchConfiguration(key), value_type=type(value))
                 for key, value in defaults.items()}]

    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=str(value).lower() if isinstance(value, bool) else str(value))
          for key, value in (lateral | speed).items()),
        Node(package='atd_control', executable='control_monitor', output='screen',
             parameters=parameters(lateral)),
        Node(package='atd_control', executable='speed_monitor', output='screen', parameters=parameters(speed)),
    ])

"""保存済みのラインを使う観測用 planner。車両出力は持たない。"""
from atd_planning.core import DEFAULT_HORIZON_M
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {'results_root': '/data/results', 'line': 'raceline', 'horizon_m': str(DEFAULT_HORIZON_M),
            'max_offset_m': '0.75', 'use_sim_time': 'false', 'simulation_freshness': 'false',
            'allow_sparse_tf': 'false', 'reference_line_topic': ''}
    parameters = {key: ParameterValue(LaunchConfiguration(key), value_type=str)
                  for key in ('map_id', 'results_root', 'line', 'reference_line_topic')}
    for key in ('horizon_m', 'max_offset_m'):
        parameters[key] = ParameterValue(LaunchConfiguration(key), value_type=float)
    for key in ('use_sim_time', 'simulation_freshness', 'allow_sparse_tf'):
        parameters[key] = ParameterValue(LaunchConfiguration(key), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('map_id'),
        *(DeclareLaunchArgument(key, default_value=value) for key, value in args.items()),
        Node(package='atd_planning', executable='planning_monitor', output='screen', parameters=[parameters])
    ])

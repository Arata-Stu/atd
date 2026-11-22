"""Notebook の保存地図・Sim Clock・ステレオ入力で Gazebo 内の車両を走らせる。"""
from dataclasses import asdict
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from atd_bringup.gazebo_control import default_min_lookahead, simulation_table
from atd_control.bicycle import FULL_SPEED_MPS
from atd_control.core import LookaheadConfig
from atd_control.speed import PIDConfig
from atd_localization.assets import map_metadata, verified_hdmap
from atd_planning.core import DEFAULT_HORIZON_M, ReferenceLine


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    root, key, line = arg('results_root'), arg('map_id'), arg('line')
    metadata = map_metadata(root, key)
    document = json.loads(verified_hdmap(root, metadata))
    ref = ReferenceLine.from_document(document, line)
    if not ref.closed:
        raise ValueError('周回評価には選択した保存地図上の閉じた走行ラインが必要です')
    table = simulation_table(arg('feedforward_path'))
    cap = float(arg('max_speed_mps'))
    if not table.speeds[0] <= cap <= min(FULL_SPEED_MPS, table.speeds[-1]):
        raise ValueError('速度上限が Gazebo の模擬速度域または FF 表の範囲外です')
    if min(ref.speeds) < table.speeds[0]:
        raise ValueError('走行ラインの最低速度が選択した前進 FF 表の範囲を下回っています')
    lateral_args = {k: arg(k) for k in asdict(LookaheadConfig())}
    if lateral_args['min_lookahead_m'] == '':
        lateral_args['min_lookahead_m'] = default_min_lookahead(line)
    lateral = LookaheadConfig(**{k: float(v) for k, v in lateral_args.items()})
    pid = PIDConfig(**{k: float(arg('speed_' + k)) for k in asdict(PIDConfig())})

    def include(package, name, settings):
        launch = Path(get_package_share_directory(package)) / 'launch' / name
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch)), launch_arguments=settings.items())])

    common = dict(map_id=key, results_root=root, use_sim_time='true', simulation_freshness='true')
    control = {k: ParameterValue(metadata[k], value_type=str) for k in ('map_id', 'map_sha256', 'hdmap_sha256')}
    control.update(line=ParameterValue(line, value_type=str), feedforward_path=ParameterValue(arg('feedforward_path'),
                   value_type=str), max_speed_mps=cap, use_sim_time=True, **asdict(lateral),
                   **{'speed_' + k: v for k, v in asdict(pid).items()})
    return [
        include('atd_localization', 'localization.launch.py', dict(common, engine_dir=arg('engine_dir'))),
        include('atd_planning', 'planning.launch.py', dict(common, line=line,
                horizon_m=str(max(DEFAULT_HORIZON_M, lateral.max_lookahead_m)))),
        Node(package='atd_bringup', executable='gazebo_control', output='screen', parameters=[control],
             on_exit=[Shutdown(reason='Gazebo の車両制御ノードが終了しました')]),
    ]


def generate_launch_description():
    defaults = dict(line='raceline', results_root='/data/results', engine_dir='', feedforward_path='',
                    max_speed_mps='1.5', **asdict(LookaheadConfig()),
                    **{'speed_' + k: v for k, v in asdict(PIDConfig()).items()})
    defaults['min_lookahead_m'] = ''
    return LaunchDescription([
        DeclareLaunchArgument('map_id'),
        *(DeclareLaunchArgument(k, default_value=str(v)) for k, v in defaults.items()),
        OpaqueFunction(function=setup),
    ])

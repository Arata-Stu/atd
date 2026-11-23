"""Notebook の保存地図・Sim Clock・ステレオ入力で Gazebo 内の車両を走らせる。"""
import atexit
from dataclasses import asdict
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from atd_bringup.gazebo_control import TUNABLE_PARAMETERS, default_min_lookahead, simulation_table, validate_speed_cap
from atd_control.bicycle import resolve_full_throttle_speed
from atd_control.core import LookaheadConfig
from atd_control.speed import PIDConfig
from atd_localization.assets import map_metadata, verified_hdmap
from atd_planning.core import DEFAULT_HORIZON_M, ReferenceLine


def rviz_nodes(root, key, line, reference):
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise ValueError('RViz の画面出力先がありません。Notebook のデスクトップ端末で atd dev を起動してください')
    preset = Path(get_package_share_directory('atd_bringup')) / 'config/gazebo_closed_loop.rviz'
    config = yaml.safe_load(preset.read_text())
    xs = [segment.x for segment in reference.segments]
    ys = [segment.y for segment in reference.segments]
    # Centre the top view on the saved course, including maps with a displaced origin.
    view = config['Visualization Manager']['Views']['Current']
    view.update(X=(min(xs) + max(xs)) / 2, Y=(min(ys) + max(ys)) / 2,
                Scale=min(720 / max(1., max(xs) - min(xs)), 520 / max(1., max(ys) - min(ys))))
    work = TemporaryDirectory(prefix='atd-gazebo-rviz-')
    atexit.register(work.cleanup)
    actual = Path(work.name) / preset.name
    actual.write_text(yaml.safe_dump(config, sort_keys=False))
    params = {k: ParameterValue(v, value_type=str) for k, v in
              [('results_root', root), ('map_id', key), ('line', line)]}
    return [
        Node(package='atd_bringup', executable='gazebo_closed_loop_paths', output='screen',
             parameters=[dict(params, use_sim_time=True)]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(actual)], output='screen',
             parameters=[{'use_sim_time': True}]),
    ]


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    root, key, line = arg('results_root'), arg('map_id'), arg('line')
    metadata = map_metadata(root, key)
    hdmap = verified_hdmap(root, metadata)
    ref = ReferenceLine.from_document(json.loads(hdmap), line)
    if not ref.closed:
        raise ValueError('周回評価には選択した保存地図上の閉じた走行ラインが必要です')
    cap = float(arg('max_speed_mps'))
    model_arg = arg('full_throttle_speed_mps')
    full_speed = resolve_full_throttle_speed(cap, float(model_arg) if model_arg else None)
    table = simulation_table(arg('feedforward_path'), full_speed)
    validate_speed_cap(cap, full_speed, table)
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

    common = dict(map_id=key, results_root=root, use_sim_time='true', simulation_freshness='true',
                  allow_sparse_tf='true')
    control = {k: ParameterValue(metadata[k], value_type=str) for k in ('map_id', 'map_sha256', 'hdmap_sha256')}
    control.update(line=ParameterValue(line, value_type=str), feedforward_path=ParameterValue(arg('feedforward_path'),
                   value_type=str), max_speed_mps=cap, full_throttle_speed_mps=full_speed,
                   use_sim_time=True, **asdict(lateral),
                   **{'speed_' + k: v for k, v in asdict(pid).items()})
    actions = [
        include('atd_localization', 'localization.launch.py',
                dict(common, engine_dir=arg('engine_dir'), visualize_landmarks=arg('rviz'))),
        include('atd_planning', 'planning.launch.py', dict(common, line=line,
                horizon_m=str(max(DEFAULT_HORIZON_M, lateral.max_lookahead_m)))),
        Node(package='atd_bringup', executable='gazebo_control', output='screen', parameters=[control],
             on_exit=[Shutdown(reason='Gazebo の車両制御ノードが終了しました')]),
    ]
    if arg('run_dir'):
        directory = Path(arg('run_dir'))
        reference = directory / 'closed_loop.reference.json'
        if reference.exists() or reference.is_symlink():
            raise FileExistsError(reference)
        # Freeze the same verified bytes from which the selected line was loaded.
        with (directory / 'hdmap.json').open('xb') as stream:
            stream.write(hdmap)
        identity = {k: metadata[k] for k in ('map_id', 'map_sha256', 'hdmap_sha256')}
        pending = reference.with_suffix('.tmp')
        with pending.open('x', encoding='utf-8') as stream:
            json.dump(dict(identity, frame='map', line=line), stream, ensure_ascii=False)
            stream.write('\n')
        pending.replace(reference)
        actions.append(Node(package='atd_bringup', executable='gazebo_closed_loop_log', output='screen',
                            arguments=[str(directory)], parameters=[{'use_sim_time': True}],
                            on_exit=[Shutdown(reason='Gazebo の解析ログ記録が終了しました')]))
    if arg('rviz') == 'true':
        actions.extend(rviz_nodes(root, key, line, ref))
    if arg('web') == 'true':
        allowed = json.dumps({'/atd_gazebo_control': TUNABLE_PARAMETERS})
        actions.append(Node(package='atd_web_bridge', executable='web_bridge', output='screen',
                            parameters=[dict(use_sim_time=True, environment='gazebo', port=8791,
                                             control_topic='/atd/gazebo/pursuit_status',
                                             parameter_node_prefix='/atd_gazebo_control',
                                             parameter_allowlist_json=ParameterValue(allowed, value_type=str))],
                            on_exit=[Shutdown(reason='Gazebo の Web bridge が終了しました')]))
    return actions


def generate_launch_description():
    defaults = dict(line='raceline', results_root='/data/results', engine_dir='', feedforward_path='', run_dir='',
                    max_speed_mps='1.5', full_throttle_speed_mps='', **asdict(LookaheadConfig()),
                    **{'speed_' + k: v for k, v in asdict(PIDConfig()).items()})
    defaults['min_lookahead_m'] = ''
    return LaunchDescription([
        DeclareLaunchArgument('map_id'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('web', default_value='false', choices=['true', 'false']),
        *(DeclareLaunchArgument(k, default_value=str(v)) for k, v in defaults.items()),
        OpaqueFunction(function=setup),
    ])

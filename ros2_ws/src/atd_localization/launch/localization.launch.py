"""Shared ATD stereo localization for a live camera or isolated bag clock."""
import atexit
import math
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from atd_mapping.capture_config import component_configuration
from atd_localization.assets import prepare_runtime
from atd_localization.bag_source import stereo_bag_input


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    clock = value('use_sim_time').lower()
    if clock not in ('true', 'false'):
        raise ValueError('use_sim_time に true / false を指定してください')
    sim_time = clock == 'true'
    bag = value('replay_bag')
    if bag and not sim_time:
        raise ValueError('bag のセンサー入力には use_sim_time:=true が必要です')
    gazebo_imu_frequency = value('gazebo_imu_frequency')
    if gazebo_imu_frequency and not sim_time:
        raise ValueError('Gazebo の統合 IMU にはシミュレーション時刻が必要です')
    source = value('map_correction_source')
    frequency = float(value('vgl_frequency'))
    work = tempfile.TemporaryDirectory(prefix='atd-localization-')
    try:
        assets = prepare_runtime(value('results_root'), value('map_id'), value('engine_dir'), work.name,
                                 map_correction_source=source)
    except BaseException:
        work.cleanup()
        raise
    # Keep this invocation's checked map copy for the lifetime of the launch process.
    atexit.register(work.cleanup)
    workflow = assets['workflow']
    tracking = component_configuration(Path(get_package_share_directory('atd_mapping')) / 'config', workflow,
        assets['map_dir'], assets['map_root'], assets['model_dir'], assets['config_dir'],
        namespace='/localization', use_sim_time=sim_time, capture_visualization=value('visualize_landmarks') == 'true',
        map_correction_source=source, vgl_frequency=frequency)
    if gazebo_imu_frequency:
        tracking['parameters']['cuvslam'].update(tracking_mode=1, imu_frame='camera_gyro_optical_frame',
                                                calibration_frequency=float(gazebo_imu_frequency))
        tracking['remaps']['cuvslam'].append(('visual_slam/imu', '/camera/camera/imu'))
    nodes = []
    for kind, package, plugin, name in (
        ('cuvslam', 'isaac_ros_cuvslam', 'nvidia::isaac_ros::visual_slam::VisualSlamNode', 'visual_slam'),
        ('vgl', 'isaac_ros_visual_global_localization',
         'nvidia::isaac_ros::visual_global_localization::VisualGlobalLocalizationNode', 'visual_global_localization'),
    ):
        nodes.append(ComposableNode(package=package, plugin=plugin, name=name,
                                    parameters=[tracking['parameters'][kind]], remappings=tracking['remaps'][kind],
                                    extra_arguments=[{'use_intra_process_comms': True}]))
    scale = value('rpm_pulses_per_meter')
    model_speed = value('gazebo_model_speed') == 'true'
    if model_speed and (scale or not sim_time):
        raise ValueError('Gazebo 車速は模擬時刻で使用し、実車 RPM と同時指定できません')
    if scale or model_speed:
        params = dict(gyro_topic=value('rpm_gyro_topic'), use_sim_time=sim_time)
    if scale:
        pulses = float(scale)
        if not math.isfinite(pulses) or pulses <= 0:
            raise ValueError('rpm_pulses_per_meter には実測した正のパルス数/mを指定してください')
        params['pulses_per_meter'] = pulses
    if model_speed:
        params['model_speed_topic'] = '/atd/gazebo/model_speed'
    if scale or model_speed:
        nodes.append(ComposableNode(package='atd_localization', plugin='atd_localization::RpmImuOdometry',
                                    name='rpm_imu_ekf', parameters=[params],
                                    extra_arguments=[{'use_intra_process_comms': True}]))
    if bag:
        nodes.append(stereo_bag_input(bag, value('replay_rate'), value('replay_player_name'), workflow,
                                     imu_topic='/camera/camera/imu' if gazebo_imu_frequency else '',
                                     trace_publications=value('trace_publications') == 'true'))
    # ROS launch parses scalar strings as YAML by default (IDs such as "001" must keep their type).
    monitor = {key: ParameterValue(assets[key], value_type=str) for key in ('map_id', 'map_sha256', 'hdmap_sha256')}
    monitor.update({key: ParameterValue(workflow[key], value_type=str) for key in (
        'base_frame', 'left_info', 'right_info', 'left_frame', 'right_frame')})
    monitor.update(map_correction_source=source,
                   simulation_freshness=ParameterValue(LaunchConfiguration('simulation_freshness'), value_type=bool),
                   allow_sparse_tf=ParameterValue(LaunchConfiguration('allow_sparse_tf'), value_type=bool),
                   use_sim_time=sim_time, image_height=assets['input_shape'][2], image_width=assets['input_shape'][3])
    tracking_label = f'VIO (IMU {gazebo_imu_frequency} Hz)' if gazebo_imu_frequency else 'VO'
    print(f"ATD localization map={assets['map_id']} engine={assets['model_dir']} "
          f"stereo={assets['input_shape'][2:]} tracking={tracking_label} map→odom={source}",
          flush=True)
    return [
        Node(package='atd_localization', executable='localization_monitor', name='atd_localization_monitor',
             output='screen', parameters=[monitor], on_exit=[Shutdown(reason='自己位置推定の監視が終了しました')]),
        ComposableNodeContainer(name='atd_localization_container', namespace='', package='rclcpp_components',
            executable='component_container_mt', output='screen', composable_node_descriptions=nodes,
            on_exit=[Shutdown(reason='自己位置推定 component container が終了しました')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_id'), DeclareLaunchArgument('results_root', default_value='/data/results'),
        DeclareLaunchArgument('engine_dir', default_value=''), DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_correction_source', default_value='cuvslam', choices=['cuvslam', 'vgl']),
        DeclareLaunchArgument('vgl_frequency', default_value='1.0'),
        DeclareLaunchArgument('rpm_pulses_per_meter', default_value=''),
        DeclareLaunchArgument('gazebo_model_speed', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('rpm_gyro_topic', default_value='/camera/camera/gyro/sample'),
        DeclareLaunchArgument('replay_bag', default_value=''),
        DeclareLaunchArgument('replay_rate', default_value='1.0'),
        DeclareLaunchArgument('replay_player_name', default_value='atd_validation_player'),
        DeclareLaunchArgument('trace_publications', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('visualize_landmarks', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gazebo_imu_frequency', default_value='', choices=['', '200', '400']),
        DeclareLaunchArgument('simulation_freshness', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('allow_sparse_tf', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])

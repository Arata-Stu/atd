"""Shared ATD stereo localization for a live camera or isolated bag clock."""
import atexit
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
from atd_localization.orb_assets import is_orb_map, orb_map_metadata


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    clock = value('use_sim_time').lower()
    if clock not in ('true', 'false'):
        raise ValueError('use_sim_time に true / false を指定してください')
    sim_time = clock == 'true'
    gazebo_imu_frequency = value('gazebo_imu_frequency')
    if gazebo_imu_frequency and not sim_time:
        raise ValueError('Gazebo の統合 IMU にはシミュレーション時刻が必要です')
    if is_orb_map(value('results_root'), value('map_id')):
        if value('engine_dir') or gazebo_imu_frequency:
            raise ValueError('ORB-SLAM3 の固定地図定位は stereo を使用します。engine_dir と IMU 指定を外してください')
        assets = orb_map_metadata(value('results_root'), value('map_id'))
        params = {key: ParameterValue(assets[key], value_type=str)
                  for key in ('map_id', 'map_sha256', 'hdmap_sha256', 'atlas_path')}
        params.update(mode='localization', camera_fps=float(assets['camera_fps']), use_sim_time=sim_time)
        return [Node(package='atd_orbslam3', executable='orbslam3_node', name='orbslam3_node',
                     parameters=[params], output='screen',
                     on_exit=[Shutdown(reason='ORB-SLAM3 固定地図の自己位置推定が終了しました')])]
    work = tempfile.TemporaryDirectory(prefix='atd-localization-')
    try:
        assets = prepare_runtime(value('results_root'), value('map_id'), value('engine_dir'), work.name)
    except BaseException:
        work.cleanup()
        raise
    # Keep this invocation's checked map copy for the lifetime of the launch process.
    atexit.register(work.cleanup)
    workflow = assets['workflow']
    tracking = component_configuration(Path(get_package_share_directory('atd_mapping')) / 'config', workflow,
        assets['map_dir'], assets['map_root'], assets['model_dir'], assets['config_dir'],
        namespace='/localization', use_sim_time=sim_time, capture_visualization=value('visualize_landmarks') == 'true')
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
                                    extra_arguments=[{'use_intra_process_comms': True}] if kind == 'cuvslam' else []))
    # ROS launch parses scalar strings as YAML by default (IDs such as "001" must keep their type).
    monitor = {key: ParameterValue(assets[key], value_type=str) for key in ('map_id', 'map_sha256', 'hdmap_sha256')}
    monitor.update({key: ParameterValue(workflow[key], value_type=str) for key in (
        'base_frame', 'left_info', 'right_info', 'left_frame', 'right_frame')})
    monitor.update(simulation_freshness=ParameterValue(LaunchConfiguration('simulation_freshness'), value_type=bool),
                   allow_sparse_tf=ParameterValue(LaunchConfiguration('allow_sparse_tf'), value_type=bool),
                   use_sim_time=sim_time, image_height=assets['input_shape'][2], image_width=assets['input_shape'][3])
    tracking_label = f'VIO (IMU {gazebo_imu_frequency} Hz)' if gazebo_imu_frequency else 'VO'
    print(f"ATD localization map={assets['map_id']} engine={assets['model_dir']} "
          f"stereo={assets['input_shape'][2:]} tracking={tracking_label}",
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
        DeclareLaunchArgument('visualize_landmarks', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gazebo_imu_frequency', default_value='', choices=['', '200', '400']),
        DeclareLaunchArgument('simulation_freshness', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('allow_sparse_tf', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])

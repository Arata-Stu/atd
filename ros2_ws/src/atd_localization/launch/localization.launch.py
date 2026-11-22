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


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    clock = value('use_sim_time').lower()
    if clock not in ('true', 'false'):
        raise ValueError('use_sim_time に true / false を指定してください')
    sim_time = clock == 'true'
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
        namespace='/localization', use_sim_time=sim_time, capture_visualization=False)
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
                   use_sim_time=sim_time, image_height=assets['input_shape'][2], image_width=assets['input_shape'][3])
    print(f"ATD localization map={assets['map_id']} engine={assets['model_dir']} stereo={assets['input_shape'][2:]}",
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
        DeclareLaunchArgument('simulation_freshness', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])

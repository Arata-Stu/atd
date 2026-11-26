"""Saved-map capture only: no sensors, vehicle control, bag recorder or parking."""
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def setup(context):
    from atd_mapping.capture_config import resolve_capture, inspect_engines
    config = Path(get_package_share_directory('atd_mapping')) / 'config'
    job = json.loads(Path(LaunchConfiguration('job_file').perform(context)).read_text())
    resolved = resolve_capture(config, job, inspect_engines)
    parameters, remaps = resolved['parameters'], resolved['remaps']
    print('ATD capture parameters: ' + repr(parameters) + '; remaps: ' + repr(remaps), flush=True)
    nodes = [ComposableNode(package='isaac_ros_cuvslam', plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam', parameters=[parameters['cuvslam']], remappings=remaps['cuvslam'],
        extra_arguments=[{'use_intra_process_comms': True}]),
        ComposableNode(package='isaac_ros_visual_global_localization',
        plugin='nvidia::isaac_ros::visual_global_localization::VisualGlobalLocalizationNode',
        name='visual_global_localization', parameters=[parameters['vgl']], remappings=remaps['vgl'],
        extra_arguments=[{'use_intra_process_comms': True}])]
    return [ComposableNodeContainer(name='atd_mapping_container', namespace='',
        package='rclcpp_components', executable='component_container_mt', output='screen',
        composable_node_descriptions=nodes)]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('job_file'), OpaqueFunction(function=setup)])

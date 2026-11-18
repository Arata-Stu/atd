from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def start(context):
    parameters = [LaunchConfiguration('config').perform(context)]
    overrides = {name: LaunchConfiguration(name).perform(context) for name in ('serial', 'sensor_path')}
    parameters.append({name: value for name, value in overrides.items() if value})
    namespace = LaunchConfiguration('namespace').perform(context)
    if LaunchConfiguration('composed').perform(context) == 'true':
        return [ComposableNodeContainer(
            name='pico_container', namespace=namespace, package='rclcpp_components',
            executable='component_container', output='screen',
            composable_node_descriptions=[ComposableNode(
                package='atd_vehicle_interface', plugin='atd_vehicle_interface::PicoNode',
                name='atd_vehicle_interface', namespace=namespace, parameters=parameters,
                extra_arguments=[{'use_intra_process_comms': True}])])]
    return [Node(package='atd_vehicle_interface', executable='atd_vehicle_interface_node',
                 name='atd_vehicle_interface', namespace=namespace, parameters=parameters, output='screen')]


def generate_launch_description():
    config = Path(get_package_share_directory('atd_vehicle_interface')) / 'config/pico.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('serial', default_value=''),
        DeclareLaunchArgument('sensor_path', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('composed', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=start),
    ])

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def start(context):
    parameters = [LaunchConfiguration("config").perform(context)]
    profile = LaunchConfiguration("profile").perform(context)
    if profile:
        parameters.append({"profile_path": profile})
    namespace = LaunchConfiguration("namespace").perform(context)
    if LaunchConfiguration("composed").perform(context).lower() == "true":
        return [ComposableNodeContainer(
            name="joy_container", namespace=namespace, package="rclcpp_components",
            executable="component_container", output="screen",
            composable_node_descriptions=[ComposableNode(
                package="atd_joy", plugin="atd_joy::JoyNode", name="atd_joy_node",
                namespace=namespace, parameters=parameters,
                extra_arguments=[{"use_intra_process_comms": True}])])]
    return [Node(package="atd_joy", executable="atd_joy_node", name="atd_joy_node",
                 namespace=namespace, output="screen", parameters=parameters)]


def generate_launch_description():
    config = Path(get_package_share_directory("atd_joy")) / "config/joy.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(config)),
        DeclareLaunchArgument("profile", default_value=""),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("composed", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=start),
    ])

"""Validate the exact requested profiles with SDK before starting the official driver."""
from pathlib import Path
from runpy import run_path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

from atd_sensors import probe
from atd_sensors.profile import DEFAULT_PATH, OVERRIDE_ARGUMENTS, driver_arguments, load, with_overrides


def start_camera(context):
    values = {key: LaunchConfiguration(key).perform(context) for key in ('serial', *OVERRIDE_ARGUMENTS)}
    settings = with_overrides(load(LaunchConfiguration('config').perform(context)), values)
    if not any(value['enabled'] for value in settings['streams'].values()):
        return [LogInfo(msg='D455: all streams OFF; camera driver is not started')]
    settings = probe.validate(settings)
    source = Path(get_package_share_directory('realsense2_camera')) / 'launch/rs_launch.py'
    checked = driver_arguments(settings)
    target = LaunchConfiguration('target_container').perform(context)
    if target:
        # rs_launch.py sets additional camera parameters, including exposure and HDR.
        # Keep the defaults from the installed driver for settings ATD does not override.
        defaults = run_path(str(source))['configurable_parameters']
        parameters = {p['name']: yaml.safe_load(checked.get(p['name'], p['default'])) for p in defaults}
        camera = ComposableNode(package='realsense2_camera', plugin='realsense2_camera::RealSenseNodeFactory',
                                name=checked['camera_name'], namespace=checked['camera_namespace'],
                                parameters=[parameters], extra_arguments=[{'use_intra_process_comms': True}])
        return [GroupAction(scoped=True, forwarding=False, actions=[
            LoadComposableNodes(target_container=target, composable_node_descriptions=[camera])])]
    # Do not inherit unpublished driver overrides such as config_file/json_file_path.
    # They could replace the exact stream configuration validated above.
    return [GroupAction(scoped=True, forwarding=False, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(source)), launch_arguments=checked.items())])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(DEFAULT_PATH)),
        DeclareLaunchArgument('serial', default_value=''),
        DeclareLaunchArgument('target_container', default_value=''),
        *(DeclareLaunchArgument(key, default_value='') for key in OVERRIDE_ARGUMENTS),
        OpaqueFunction(function=start_camera),
    ])

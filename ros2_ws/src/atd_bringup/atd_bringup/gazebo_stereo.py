"""Notebook の Gazebo のステレオ画像を、同じシミュレーション時刻で追跡する。"""
from launch.actions import Shutdown
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def sensor_ingress(sim_time):
    return ComposableNode(
        package='atd_sensors', plugin='atd_sensors::GazeboSensorIngress', name='gazebo_sensor_ingress',
        parameters=[{'use_sim_time': sim_time}], extra_arguments=[{'use_intra_process_comms': True}],
    )


def stereo_container(sim_time, *, live_sensors=False, mapping=False, visualize_landmarks=False, slam_max_map_size=0,
                     tracking_mode=0, imu_frame='camera_gyro_optical_frame', imu_frequency=200.0):
    mapping_params = {'slam_max_map_size': slam_max_map_size} if mapping else {}
    if visualize_landmarks:
        mapping_params.update(enable_slam_visualization=True, enable_landmarks_view=True, path_max_size=50000)
    tracker = ComposableNode(
        package='isaac_ros_cuvslam', plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        name='visual_slam',
        parameters=[{
            **mapping_params,
            'use_sim_time': sim_time,
            'num_cameras': 2, 'min_num_images': 2, 'tracking_mode': tracking_mode,
            'imu_frame': imu_frame, 'calibration_frequency': imu_frequency, 'imu_qos': 'SENSOR_DATA',
            'rectified_images': True, 'image_qos': 'SENSOR_DATA',
            'base_frame': 'base_link', 'odom_frame': 'odom',
            'camera_optical_frames': ['camera_infra1_optical_frame', 'camera_infra2_optical_frame'],
            'override_publishing_stamp': False,
            'enable_localization_n_mapping': mapping, 'enable_request_hint': False,
            'publish_odom_to_base_tf': True, 'publish_map_to_odom_tf': mapping,
        }],
        remappings=[
            ('visual_slam/imu', '/camera/camera/imu'),
            ('visual_slam/image_0', '/camera/camera/infra1/image_rect_raw'),
            ('visual_slam/image_1', '/camera/camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_0', '/camera/camera/infra1/camera_info'),
            ('visual_slam/camera_info_1', '/camera/camera/infra2/camera_info'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    return ComposableNodeContainer(
        name='atd_gazebo_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', output='screen', parameters=[{'use_sim_time': sim_time}],
        composable_node_descriptions=[tracker, sensor_ingress(sim_time)] if live_sensors else [tracker],
        on_exit=[Shutdown(reason='Gazebo 用ステレオ追跡が終了しました')],
    )

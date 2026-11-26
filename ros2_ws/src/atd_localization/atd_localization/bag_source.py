"""bag のセンサー入力を追跡ノードと同じコンテナにロードする。"""


def stereo_bag_input(bag, rate, player_name, workflow, *, required_image_readers=2,
                     imu_topic='', publish_calibration=True, trace_publications=False):
    from launch_ros.descriptions import ComposableNode
    from launch_ros.parameter_descriptions import ParameterValue

    params = {key: workflow[key] for key in ('left_image', 'right_image', 'left_info', 'right_info', 'tf_static')}
    params.update(bag_path=str(bag), player_name=player_name, imu_topic=imu_topic)
    return ComposableNode(
        package='atd_localization', plugin='atd_localization::StereoBagInput', name='atd_sensor_bag_input',
        parameters=[{**{k: ParameterValue(v, value_type=str) for k, v in params.items()},
                     'rate': float(rate), 'required_image_readers': required_image_readers,
                     'publish_calibration': publish_calibration, 'trace_publications': trace_publications}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

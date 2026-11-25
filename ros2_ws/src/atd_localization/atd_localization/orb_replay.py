"""再生前に記録済みの固定校正を発行し、ORB 語彙・Atlas の初期化完了を待つ。"""
from time import monotonic

from atd_mapping.bag_inputs import load_metadata

from .orb_assets import D455


def recorded_calibration(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo
    from tf2_msgs.msg import TFMessage

    left, right, tf_topic = (D455[key] for key in ('left_info', 'right_info', 'tf_static'))
    metadata = load_metadata(bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[left, right, tf_topic]))
    infos, transforms = {}, {}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == tf_topic:
            for tf in deserialize_message(raw, TFMessage).transforms:
                old = transforms.get(tf.child_frame_id)
                if old is not None and (old.header.frame_id != tf.header.frame_id or old.transform != tf.transform):
                    raise ValueError(f'bag の固定 TF が途中で変化しています: {tf.child_frame_id}')
                transforms[tf.child_frame_id] = tf
        else:
            msg = deserialize_message(raw, CameraInfo)
            if topic in infos:
                old = infos[topic]
                # 撮影時刻を除いた CameraInfo 全体で、途中でのレンズ設定変更も検出する。
                msg.header.stamp = old.header.stamp
                if old != msg:
                    raise ValueError(f'bag のカメラ校正が途中で変化しています: {topic}')
            else:
                infos[topic] = msg
    if set(infos) != {left, right} or not transforms:
        raise ValueError('左右の CameraInfo または /tf_static がありません')
    size = infos[left].width, infos[left].height
    if not all(size) or size != (infos[right].width, infos[right].height):
        raise ValueError('bag の左右カメラ解像度が一致しません')
    return {**infos, tf_topic: TFMessage(transforms=list(transforms.values()))}, size


class OrbBootstrap:
    def __init__(self, node, calibration):
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import Bool

        self.node = node
        self.ready = False
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.seeds = [(node.create_publisher(type(message), topic, retained), message)
                      for topic, message in calibration.items()]
        self.topics = set(calibration)

        def ready(msg):
            self.ready = msg.data

        node.create_subscription(Bool, '/orbslam3/ready', ready, retained)

    def wait(self, check_running, timeout=120.):
        import rclpy

        deadline = monotonic() + timeout
        next_send = 0.
        while not self.ready:
            check_running()
            now = monotonic()
            if now >= deadline:
                raise RuntimeError('ORB-SLAM3 のカメラ校正・TF・語彙・地図の準備がタイムアウトしました')
            # CameraInfo consumers use volatile SensorDataQoS. Send only after discovery, and retry until acknowledged.
            if now >= next_send and all(p.get_subscription_count() for p, _ in self.seeds):
                for publisher, message in self.seeds:
                    publisher.publish(message)
                next_send = now + 1.
            rclpy.spin_once(self.node, timeout_sec=.05)

"""Jetson の走行設定と元観測が一致する制御結果だけを実車側へ渡す。"""
import json
import re

from .drive_tuning import DriveTuning
from .speed import limited_target


TOPIC = '/vehicle/drive_tuning'
MAP_KEYS = ('map_id', 'map_sha256', 'hdmap_sha256')


class TunedOutput:
    def __init__(self, node, message_type, topic, apply):
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self.node = node
        self.message_type = message_type
        self.apply = apply
        self.stamp_ns = 0
        self.route = None
        self.pending = None
        self.publisher = node.create_publisher(message_type, topic, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = node.create_subscription(String, TOPIC, self.receive, qos)

    def receive(self, msg, info):
        gid = bytes((info.get('publisher_gid') or {}).get('data', b''))
        if any(gid):
            self.pending = (gid, msg.data)
            self.resolve()

    def resolve(self):
        # 起動直後は transient-local の設定がノード名の DDS 探索より先に届くことがある。
        if self.pending is None:
            return
        gid, raw = self.pending
        endpoints = self.node.get_publishers_info_by_topic(TOPIC)
        if len(endpoints) != 1 or bytes(endpoints[0].endpoint_gid) != gid:
            return
        endpoint = endpoints[0]
        if endpoint.node_name == '_NODE_NAME_UNKNOWN_':
            return
        self.pending = None
        if endpoint.node_namespace != '/' or endpoint.node_name != 'atd_auto_drive':
            return
        try:
            data = json.loads(raw)
            config = DriveTuning.from_dict(data)
            limited_target(0., config.max_speed_mps)
            stamp = data.get('stamp_ns')
            if type(stamp) is not int or not 0 < stamp < 2**63:
                raise ValueError('走行設定の時刻が不正です')
            if (not isinstance(data.get('map_id'), str) or not data['map_id']
                    or not all(isinstance(data.get(k), str) and re.fullmatch('[0-9a-f]{64}', data[k])
                               for k in MAP_KEYS[1:])):
                raise ValueError('走行設定の地図 ID または版が不正です')
            if stamp <= self.stamp_ns:
                return
            self.apply(config)
            self.stamp_ns = stamp
            self.route = tuple(data[key] for key in MAP_KEYS) + (config.line,)
        except (KeyError, TypeError, ValueError, OverflowError, RecursionError) as exc:
            self.node.get_logger().warning(str(exc))

    def matches(self, source):
        if source is None or self.route is None:
            return False
        stamp = source.header.stamp
        return (stamp.sec * 10**9 + stamp.nanosec > self.stamp_ns
                and (source.map_id, source.map_sha256, source.hdmap_sha256, source.line) == self.route)

    def publish(self, status):
        if self.matches(status):
            self.publisher.publish(self.message_type(tuning_stamp_ns=self.stamp_ns, status=status))

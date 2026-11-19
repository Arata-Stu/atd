"""Publish one coherent, read-only localization observation for the car and replay."""
from functools import partial
import math
import re
import time
import uuid

import rclpy
from atd_interfaces.msg import LocalizationStatus
from atd_mapping.frames import stamp_ns
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from .state import InitialHintRequest, RigidPose, TrackingState


def rigid(position, rotation):
    return RigidPose.checked((position.x, position.y, position.z),
                             (rotation.x, rotation.y, rotation.z, rotation.w))


class LocalizationMonitor(Node):
    def __init__(self, **kwargs):
        super().__init__('atd_localization_monitor', **kwargs)
        defaults = dict(map_id='', map_sha256='', hdmap_sha256='', map_frame='map', odom_frame='odom',
                        base_frame='base_link', max_age_s=1.0, left_info='', right_info='', left_frame='',
                        right_frame='', image_width=0, image_height=0)
        self.config = {key: self.declare_parameter(key, value, ParameterDescriptor(read_only=True)).value
                       for key, value in defaults.items()}
        c = self.config
        if not c['map_id'] or any(not re.fullmatch('[0-9a-f]{64}', c[k]) for k in ('map_sha256', 'hdmap_sha256')):
            raise ValueError('地図 ID、cuVSLAM と HDMap の SHA256 が必要です')
        if (any(not c[k] for k in ('left_info', 'right_info', 'left_frame', 'right_frame'))
                or c['left_info'] == c['right_info'] or c['left_frame'] == c['right_frame']
                or min(c['image_width'], c['image_height']) <= 0):
            raise ValueError('左右それぞれの CameraInfo・光学フレームと解像度が必要です')
        self.session_id = str(uuid.uuid4())
        self.state = TrackingState(c['max_age_s'])
        self.startup_hint = InitialHintRequest()
        self.hint_client = self.create_client(Trigger, '/localization/request_hint')
        self.publisher = self.create_publisher(LocalizationStatus, '/localization/status', 10)
        self.hint_subscription = self.create_subscription(PoseWithCovarianceStamped, '/localization/pose_hint', self.on_hint, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostic, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
        for side in ('left', 'right'):
            self.create_subscription(CameraInfo, c[side + '_info'], partial(self.on_camera, side), qos_profile_sensor_data)
        self.create_timer(0.1, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def source(self, stream, info):
        self.state.clock(self.get_clock().now().nanoseconds)
        gid = info.get('publisher_gid') or {}
        return self.state.source(stream, bytes(gid.get('data', b'')))

    def on_camera(self, side, msg, info):
        if self.source(side, info):
            matches = (msg.header.frame_id == self.config[side + '_frame']
                       and msg.width == self.config['image_width'] and msg.height == self.config['image_height'])
            self.state.camera(side, stamp_ns(msg.header.stamp), time.monotonic(), matches)

    def on_hint(self, msg, info):
        if msg.header.frame_id != self.config['map_frame'] or not self.source('VGL', info):
            return
        try:
            rigid(msg.pose.pose.position, msg.pose.pose.orientation)
        except ValueError:
            return
        self.state.hint(stamp_ns(msg.header.stamp))

    def on_diagnostic(self, msg, info):
        for status in msg.status:
            if status.hardware_id == 'visual_slam' and self.source('cuVSLAM diagnostics', info):
                self.state.diagnostic(stamp_ns(msg.header.stamp), time.monotonic(),
                                      {value.key: value.value for value in status.values})

    def on_tf(self, msg, info):
        for tf in msg.transforms:
            if (tf.header.frame_id != self.config['map_frame'] or tf.child_frame_id != self.config['odom_frame']
                    or not self.source('map→odom', info)):
                continue
            try:
                pose = rigid(tf.transform.translation, tf.transform.rotation)
            except ValueError:
                continue
            self.state.transform(stamp_ns(tf.header.stamp), time.monotonic(), pose)

    def on_odometry(self, msg, info):
        if (msg.header.frame_id != self.config['odom_frame'] or msg.child_frame_id != self.config['base_frame']
                or not self.source('VO', info)):
            return
        try:
            pose = rigid(msg.pose.pose.position, msg.pose.pose.orientation)
        except ValueError:
            return
        self.state.odometry(stamp_ns(msg.header.stamp), time.monotonic(), pose,
                            msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def hint_ready(self, wall):
        if not (self.state.initialized_stereo(wall) and self.hint_client.service_is_ready()
                and self.hint_subscription.get_publisher_count() == 1):
            return False
        endpoints = self.get_subscriptions_info_by_topic('/localization/pose_hint')
        names = [item.node_name for item in endpoints if item.node_namespace == '/'
                 and item.topic_type == 'geometry_msgs/msg/PoseWithCovarianceStamped']
        return names.count('visual_slam') == 1 and names.count(self.get_name()) == 1

    def publish(self):
        now = self.get_clock().now()
        self.state.clock(now.nanoseconds)
        wall = time.monotonic()
        self.startup_hint.tick(wall, stop=(self.state.registered or self.state.lost
                                          or self.state.conflict or self.state.camera_mismatch),
                               ready=lambda: self.hint_ready(wall),
                               request=lambda: self.hint_client.call_async(Trigger.Request()))
        state, reason, sample = self.state.status(wall)
        if self.startup_hint.error and state in ('waiting_hint', 'localizing'):
            reason = self.startup_hint.error
        msg = LocalizationStatus()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.config['map_frame']
        msg.map_id = self.config['map_id']
        msg.map_sha256 = self.config['map_sha256']
        msg.hdmap_sha256 = self.config['hdmap_sha256']
        msg.session_id, msg.epoch = self.session_id, self.state.epoch
        msg.state, msg.reason = state, reason
        if sample is not None:
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(sample.stamp, 1_000_000_000)
            p, q = msg.pose.position, msg.pose.orientation
            p.x, p.y, p.z = sample.pose.xyz
            q.x, q.y, q.z, q.w = sample.pose.xyzw
            msg.speed_mps, msg.yaw_rate_rps = sample.speed, sample.yaw_rate
        else:
            p, q = msg.pose.position, msg.pose.orientation
            p.x = p.y = p.z = math.nan
            q.x = q.y = q.z = q.w = math.nan
            msg.speed_mps = msg.yaw_rate_rps = math.nan
        self.publisher.publish(msg)


def main():
    rclpy.init()
    try:
        node = LocalizationMonitor()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()

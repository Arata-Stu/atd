"""Gazebo native Depth を atd_ftg の ROS 入力へ渡す。"""
from array import array
import math
from pathlib import Path
import sys
from threading import Lock

# The Gazebo image mounts the checkout without a ROS workspace overlay.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2_ws/src/atd_ftg"))
from atd_ftg.planner import BODY_HEIGHT_M, COMMAND_AGE_NS, FAR_M, NEAR_M, DepthProjection, GapPlanner
from sensor_profile import REAR_AXLE_HEIGHT_M

OPTICAL_FRAME = "camera_depth_optical_frame"
FTG_NAMESPACE = "/depth_ftg"


def topic(course):
    return f"/{course}/depth/image"


def optical_rotation(pitch):
    """Quaternion for the fixed optical axes followed by camera pitch about base Y."""
    c, s = math.cos(pitch / 2), math.sin(pitch / 2)
    return (-(c + s) / 2, (c + s) / 2, (s - c) / 2, (c - s) / 2)


def actuator_values(speed, yaw_rate, current_steering, wheelbase, steering_limit, full_speed):
    if not (math.isfinite(speed) and math.isfinite(yaw_rate) and 0. <= speed <= full_speed):
        return None
    if speed == 0.:
        return (current_steering, 0., 1.) if yaw_rate == 0. else None
    angle = math.atan(wheelbase * yaw_rate / speed)
    if abs(angle) > steering_limit + 1e-7:
        return None
    return angle, speed / full_speed, 1.


class DepthPublisher:
    def __init__(self, course, camera, ros_node, wheelbase):
        from gz.msgs.image_pb2 import Image, R_FLOAT32
        from gz.transport import Node
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image as RosImage
        from tf2_ros import StaticTransformBroadcaster

        pitch = math.radians(camera["pitch_down_deg"])
        self.projection = DepthProjection.simulated(
            camera["width"], camera["height"], math.radians(camera["horizontal_fov_deg"]),
            camera["midpoint_forward_from_rear_axle_m"], camera["height_above_floor_m"], pitch)
        p = self.projection
        self.float32 = R_FLOAT32
        self.lock = Lock()
        self.pending = None
        self.last_stamp = 0
        self.wheelbase = wheelbase
        self.node = Node()
        if not self.node.subscribe(Image, topic(course), self.receive):
            raise RuntimeError("Gazebo Depth の購読を開始できません")
        self.image = RosImage(width=p.width, height=p.height, encoding="32FC1", step=p.width * 4,
                              is_bigendian=int(sys.byteorder == "big"))
        self.image.header.frame_id = OPTICAL_FRAME
        self.info = CameraInfo(width=p.width, height=p.height)
        self.info.header.frame_id = OPTICAL_FRAME
        self.info.k = [p.fx, 0., p.cx, 0., p.fy, p.cy, 0., 0., 1.]
        self.info.p = [p.fx, 0., p.cx, 0., 0., p.fy, p.cy, 0., 0., 0., 1., 0.]
        self.info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        self.odom = Odometry(child_frame_id="base_link")
        self.odom.header.frame_id = "gazebo_world"
        self.odom.pose.pose.orientation.w = 1.
        self.image_pub = ros_node.create_publisher(RosImage, f"{FTG_NAMESPACE}/depth", qos_profile_sensor_data)
        self.info_pub = ros_node.create_publisher(CameraInfo, f"{FTG_NAMESPACE}/camera_info", qos_profile_sensor_data)
        self.odom_pub = ros_node.create_publisher(Odometry, f"{FTG_NAMESPACE}/odometry", qos_profile_sensor_data)
        transform = TransformStamped(child_frame_id=OPTICAL_FRAME)
        transform.header.frame_id = "base_link"
        translation = transform.transform.translation
        translation.x, translation.y, translation.z = p.origin
        translation.z -= REAR_AXLE_HEIGHT_M
        rotation = transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = optical_rotation(pitch)
        self.tf = StaticTransformBroadcaster(ros_node)
        self.tf.sendTransform(transform)

    def receive(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nsec
        valid = (msg.pixel_format_type == self.float32 and msg.width == self.projection.width
                 and msg.height == self.projection.height and msg.step == msg.width * 4)
        data = msg.data if valid else b""
        with self.lock:
            if self.pending is None or stamp >= self.pending[0]:
                self.pending = stamp, data

    def take(self, now_ns):
        with self.lock:
            frame = self.pending
        if frame is None or frame[0] > now_ns or frame[0] <= self.last_stamp:
            return None
        stamp, data = frame
        self.last_stamp = stamp
        if now_ns - stamp > COMMAND_AGE_NS or len(data) != self.projection.width * self.projection.height * 4:
            return None
        return frame

    def publish(self, now_ns, vehicle):
        frame = self.take(now_ns)
        if frame is None:
            return None
        stamp, data = frame
        for message in (self.info, self.odom, self.image):
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
        pose = self.odom.pose.pose
        pose.position.x, pose.position.y, pose.position.z = vehicle.x, vehicle.y, REAR_AXLE_HEIGHT_M
        pose.orientation.z, pose.orientation.w = math.sin(vehicle.yaw / 2), math.cos(vehicle.yaw / 2)
        self.odom.twist.twist.linear.x = vehicle.speed
        self.odom.twist.twist.angular.z = vehicle.speed * math.tan(vehicle.steering) / self.wheelbase
        self.image.data = array("B", data)
        self.info_pub.publish(self.info)
        self.odom_pub.publish(self.odom)
        self.image_pub.publish(self.image)
        return frame

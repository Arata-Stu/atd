"""D455 または ROS Depth 入力の FTG 目標速度を配信するノード。"""
import json
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .planner import BODY_HEIGHT_M, COMMAND_AGE_NS, GapPlanner, RecentCorridorWidth
from .ros_input import depth_metres, projection


def stamp_ns(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec


class DepthFTGNode(Node):
    def __init__(self):
        super().__init__("depth_ftg")
        fixed = ParameterDescriptor(read_only=True)
        self.base = self.declare_parameter("base_frame", "base_link", fixed).value
        self.allow_far_clip = self.declare_parameter("allow_far_clip", False, fixed).value
        self.show_obstacles = self.declare_parameter("publish_obstacles", False, fixed).value
        frequency = self.declare_parameter("planning_hz", 20., fixed).value
        if not math.isfinite(frequency) or frequency <= 0.:
            raise ValueError("planning_hz は有限の正数が必要です")
        settings = {}
        for name, default in (
            ("wheelbase", .257), ("steering_limit", math.radians(28.)), ("speed_limit", .5),
            ("speed_tau", .5), ("deceleration", 1.8), ("vehicle_length", .35),
            ("vehicle_width", .19), ("rear_axle_to_rear", .1065), ("body_height", BODY_HEIGHT_M),
            ("rear_axle_height", .0325),
        ):
            settings[name] = self.declare_parameter(name, default, fixed).value
        if (not self.base or any(not math.isfinite(value) or value <= 0 for value in settings.values())
                or settings["steering_limit"] >= math.pi / 2
                or settings["rear_axle_to_rear"] >= settings["vehicle_length"]
                or settings["rear_axle_height"] >= settings["body_height"]):
            raise ValueError("FTG の車体寸法・操舵・速度設定が不正です")
        self.rear_axle_height = settings.pop("rear_axle_height")
        self.planner = GapPlanner(**settings)
        self.corridor_width = RecentCorridorWidth()
        self.transforms = Buffer(node=self)
        self.listener = TransformListener(self.transforms, self)
        self.depth = self.calibration = self.odometry = None
        self.projection_key = self.camera = None
        self.last_planned = None
        self.steering = 0.
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "~/depth", self.on_depth, sensor)
        self.create_subscription(CameraInfo, "~/camera_info", self.on_calibration, sensor)
        self.create_subscription(Odometry, "~/odometry", self.on_odometry, sensor)
        self.target = self.create_publisher(TwistStamped, "~/target_twist", 1)
        self.status = self.create_publisher(String, "~/status", 1)
        self.create_timer(1 / frequency, self.update)

    def on_depth(self, message):
        self.depth = message

    def on_calibration(self, message):
        self.calibration = message

    def on_odometry(self, message):
        self.odometry = message

    def publish(self, now, steering=0., speed=0., report=None, obstacles=()):
        message = TwistStamped()
        message.header.frame_id = self.base
        message.header.stamp = now.to_msg()
        message.twist.linear.x = speed
        message.twist.angular.z = speed * math.tan(steering) / self.planner.wheelbase
        self.target.publish(message)
        if self.status.get_subscription_count():
            status = dict(report or {"state": "waiting_input"}, stamp_ns=now.nanoseconds,
                          steering_rad=steering, target_speed_mps=speed, obstacle_voxels=len(obstacles))
            if self.show_obstacles:
                status["obstacles"] = obstacles
            self.status.publish(String(data=json.dumps(status, separators=(",", ":"))))

    def update(self):
        now = self.get_clock().now()
        depth, info, odom = self.depth, self.calibration, self.odometry
        if (depth is None or info is None or odom is None
                or not depth.header.frame_id or depth.header.frame_id != info.header.frame_id
                or (depth.width, depth.height) != (info.width, info.height)
                or odom.child_frame_id != self.base
                or any(not 0 <= now.nanoseconds - stamp_ns(header) <= COMMAND_AGE_NS or stamp_ns(header) == 0
                       for header in (depth.header, odom.header))):
            self.last_planned = None
            self.steering = 0.
            self.publish(now)
            return
        twist = odom.twist.twist
        speed, yaw_rate = twist.linear.x, twist.angular.z
        if not math.isfinite(speed) or not math.isfinite(yaw_rate) or speed < -.01:
            self.last_planned = None
            self.publish(now)
            return
        if depth is self.last_planned:
            return
        try:
            pose = self.transforms.lookup_transform(self.base, depth.header.frame_id,
                                                    Time.from_msg(depth.header.stamp)).transform
            translation = (pose.translation.x, pose.translation.y, pose.translation.z)
            rotation = (pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)
            intrinsics = (info.p[0], info.p[5], info.p[2], info.p[6])
            key = (depth.width, depth.height, intrinsics, translation, rotation)
            if key != self.projection_key:
                self.camera = projection(*key, self.rear_axle_height, self.planner.body_height,
                                         allow_far_clip=self.allow_far_clip)
                self.projection_key = key
            pixels = depth_metres(depth)
        except (TransformException, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.)
            self.publish(now)
            return

        if speed > .05:
            limit = self.planner.steering_limit
            self.steering = max(-limit, min(limit, math.atan(self.planner.wheelbase * yaw_rate / speed)))
        obstacles = self.camera.obstacles(pixels)
        depth_stamp = stamp_ns(depth.header)
        self.steering, target, report = self.planner.plan(
            obstacles, lambda x, y, z: self.camera.visible(pixels, x, y, z), max(speed, 0.), self.steering,
            recent_width=self.corridor_width.get(depth_stamp))
        if "observed_corridor_width_m" in report:
            self.corridor_width.observe(depth_stamp, report["observed_corridor_width_m"])
        self.last_planned = depth
        self.publish(Time.from_msg(depth.header.stamp), self.steering, target, report, obstacles)


def main():
    with rclpy.init():
        node = DepthFTGNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

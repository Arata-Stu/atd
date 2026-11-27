"""RViz に FTG の入力と車体・軌道を配信する。"""
from array import array
from collections import deque
import math
import sys

from depth_ftg import BODY_HEIGHT_M, COMMAND_AGE_NS, OPTICAL_FRAME
from sensor_profile import REAR_AXLE_HEIGHT_M


DEPTH_RED_M = .2
DEPTH_WHITE_M = 2.5
DEPTH_COLORS = tuple(bytes((255, level, level)) for level in range(256))
INVALID_DEPTH_COLOR = bytes((70, 74, 80))


def depth_heatmap(data):
    """Convert native float32 metres to RGB using a fixed colour range."""
    colors = []
    scale = 255 / (DEPTH_WHITE_M - DEPTH_RED_M)
    for depth in memoryview(data).cast("f"):
        if not depth > 0:
            colors.append(INVALID_DEPTH_COLOR)
        elif depth >= DEPTH_WHITE_M:
            colors.append(DEPTH_COLORS[-1])
        else:
            level = max(0, round((depth - DEPTH_RED_M) * scale))
            colors.append(DEPTH_COLORS[level])
    return b"".join(colors)


class FTGVisualization:
    def __init__(self, node, projection, planner):
        from geometry_msgs.msg import Point, TransformStamped
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from tf2_ros import TransformBroadcaster
        from visualization_msgs.msg import Marker, MarkerArray

        self.point_type, self.transform_type = Point, TransformStamped
        self.image_type, self.marker_type, self.array_type = Image, Marker, MarkerArray
        self.projection, self.planner = projection, planner
        self.markers = node.create_publisher(MarkerArray, "/atd/gazebo/ftg/markers", 1)
        self.depth = node.create_publisher(Image, "/atd/gazebo/ftg/depth", qos_profile_sensor_data)
        self.heatmap = node.create_publisher(Image, "/atd/gazebo/ftg/depth_heatmap", qos_profile_sensor_data)
        self.tf = TransformBroadcaster(node)
        self.trail = deque(maxlen=2000)
        self.snapshot = None
        self.image_stamp = self.heatmap_stamp = None

    def observe(self, stamp, data, obstacles, angle, target, report):
        self.snapshot = stamp, data, obstacles, angle, target, report

    def publish(self, stamp, pose, speed, state):
        transform = self.transform_type()
        transform.header.frame_id = "gazebo_world"
        transform.child_frame_id = "base_link"
        transform.header.stamp.sec, transform.header.stamp.nanosec = divmod(stamp, 10**9)
        translation = transform.transform.translation
        translation.x, translation.y = pose.x(), pose.y()
        translation.z = pose.z() + REAR_AXLE_HEIGHT_M
        transform.transform.rotation.z = math.sin(pose.yaw() / 2)
        transform.transform.rotation.w = math.cos(pose.yaw() / 2)
        self.tf.sendTransform(transform)
        show_markers = self.markers.get_subscription_count()
        show_depth = self.depth.get_subscription_count()
        show_heatmap = self.heatmap.get_subscription_count()
        if not (show_markers or show_depth or show_heatmap):
            return

        snapshot = self.snapshot
        if snapshot is None or not 0 <= stamp - snapshot[0] <= COMMAND_AGE_NS or state.get("state") in (
            "invalid_or_stale_depth", "stale_depth"
        ):
            snapshot = None
        if snapshot:
            image_stamp, data = snapshot[:2]
            if show_depth and image_stamp != self.image_stamp:
                self.publish_image(self.depth, image_stamp, data, "32FC1", 4, int(sys.byteorder == "big"))
                self.image_stamp = image_stamp
            if show_heatmap and image_stamp != self.heatmap_stamp:
                self.publish_image(self.heatmap, image_stamp, depth_heatmap(data), "rgb8", 3, 0)
                self.heatmap_stamp = image_stamp
        if not show_markers:
            return

        marker = self.marker_type

        def make(name, kind, color, frame="base_link"):
            item = marker(ns=name, type=kind)
            item.header.frame_id = frame
            item.header.stamp = transform.header.stamp
            item.pose.orientation.w = 1.
            item.color.r, item.color.g, item.color.b, item.color.a = color
            return item

        body = make("Vehicle", marker.CUBE, (.22, .58, 1., .55))
        body.pose.position.x = (self.planner.front - self.planner.rear) / 2
        body.pose.position.z = BODY_HEIGHT_M / 2 - REAR_AXLE_HEIGHT_M
        body.scale.x = self.planner.front + self.planner.rear
        body.scale.y = 2 * self.planner.half_width
        body.scale.z = BODY_HEIGHT_M

        point = self.point_type
        obstacles = make("Obstacle voxels used by FTG", marker.POINTS, (1., .3, .2, 1.))
        obstacles.scale.x = obstacles.scale.y = .019
        arc = make("Selected arc", marker.LINE_STRIP, (.28, 1., .5, 1.))
        arc.scale.x = .018
        if snapshot:
            _, _, points, angle, target, report = snapshot
            obstacles.points = [point(x=x, y=y, z=z - REAR_AXLE_HEIGHT_M) for x, y, z in points]
            if target > 0.:
                arc.points = [point(z=.012 - REAR_AXLE_HEIGHT_M)]
                arc.points.extend(point(x=x, y=y, z=.012 - REAR_AXLE_HEIGHT_M)
                                  for distance, x, y, _, _ in self.planner.arc(angle)
                                  if distance <= report["clearance_m"])
        if not obstacles.points:
            obstacles.action = marker.DELETE
        if len(arc.points) < 2:
            arc.action = marker.DELETE

        trail = make("Gazebo driven path", marker.LINE_STRIP, (.35, .65, 1., .8), "gazebo_world")
        trail.scale.x = .012
        previous = self.trail[-1] if self.trail else None
        if previous is None or math.hypot(translation.x - previous.x, translation.y - previous.y) >= .02:
            self.trail.append(point(x=translation.x, y=translation.y, z=pose.z() + .008))
        trail.points = list(self.trail)
        if len(trail.points) < 2:
            trail.action = marker.DELETE

        status = make("FTG status", marker.TEXT_VIEW_FACING, (1., 1., 1., 1.))
        status.pose.position.z = .30
        status.scale.z = .055
        status.text = f"{state.get('state', 'waiting_depth')}  {speed:.2f} m/s"
        if "clearance_m" in state:
            status.text += f"  clear {state['clearance_m']:.2f} m"
        self.markers.publish(self.array_type(markers=[body, obstacles, arc, trail, status]))

    def publish_image(self, publisher, stamp, data, encoding, bytes_per_pixel, bigendian):
        image = self.image_type(width=self.projection.width, height=self.projection.height,
                                encoding=encoding, step=self.projection.width * bytes_per_pixel,
                                is_bigendian=bigendian, data=array("B", data))
        image.header.frame_id = OPTICAL_FRAME
        image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 10**9)
        publisher.publish(image)

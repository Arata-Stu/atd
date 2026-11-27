"""RViz に FTG の入力と車体・軌道を配信する。"""
from array import array
from collections import deque
import math
import sys

from depth_ftg import BODY_HEIGHT_M, COMMAND_AGE_NS, OPTICAL_FRAME
from sensor_profile import REAR_AXLE_HEIGHT_M


DEPTH_NEAR_M = .2
DEPTH_FAR_M = 2.5
# Turbo RGB samples, converted to uint8 in the same way as the Depth Pro 2 CLI.
# Reference: https://matplotlib.org/stable/users/explain/colors/colormaps.html
TURBO_RGB = bytes.fromhex(
    "30123b31154232184a341b51351e5836215f37236538266c3929723a2c793b2f7f3c32853c358b3d37913e3a963f3d9c"
    "4040a14043a64145ab4148b0424bb5434eba4350be4353c24456c74458cb455bce455ed24560d64563d94666dd4668e0"
    "466be3466de64670e84673eb4675ed4678f0467af2467df4467ff64682f84584f94587fb4589fc448cfd438efd4291fe"
    "4193fe4096fe3f98fe3e9bfe3c9dfd3ba0fc39a2fc38a5fb36a8f934aaf833acf631aff52fb1f32db4f12bb6ef2ab9ed"
    "28bbeb26bde925c0e623c2e421c4e120c6df1ec9dc1dcbda1ccdd71bcfd41ad1d219d3cf18d5cc18d7ca17d9c717dac4"
    "17dcc217debf18e0bd18e1ba19e3b81ae4b61be5b41de7b11ee8af20e9ac22eba924eca627eda329eea02cef9d2ff09a"
    "32f19735f39438f4913bf48d3ff58a42f68746f7834af8804df97c51f97955fa7659fb725dfb6f61fc6c65fc6869fd65"
    "6dfd6271fd5f74fe5c78fe597cfe5680fe5384fe5087fe4d8bfe4b8efe4892fe4695fe4498fe429bfd409efd3ea1fc3d"
    "a4fc3ba6fb3aa9fb39acfa37aef937b1f836b3f835b6f735b9f534bbf434bef334c0f233c3f133c5ef33c8ee33caed33"
    "cdeb34cfea34d1e834d4e735d6e535d8e335dae236dde036dfde36e1dc37e3da37e5d838e7d738e8d538ead339ecd139"
    "edcf39efcd39f0cb3af2c83af3c63af4c43af6c23af7c039f8be39f9bc39f9ba38fab737fbb537fbb336fcb035fcae34"
    "fdab33fda932fda631fda330fea12ffe9e2efe9b2dfe982cfd952bfd9229fd8f28fd8c27fc8926fc8624fb8323fb8022"
    "fa7d20fa7a1ff9771ef8741cf7711bf76e1af66b18f56817f46516f36315f26014f15d13ef5a11ee5810ed550fec520e"
    "ea500de94d0de84b0ce6490be5460ae3440ae24209e04008de3e08dd3c07db3a07d93806d73606d63405d43205d23005"
    "d02f04ce2d04cb2b03c92903c72803c52602c32402c02302be2102bb1f01b91e01b61c01b41b01b11901ae1801ac1601"
    "a91501a61401a31201a011019d10019a0e01970d01940c01910b018e0a018b09018708018407018106027d05027a0402"
)
DEPTH_COLORS = tuple(TURBO_RGB[offset:offset + 3] for offset in range(0, len(TURBO_RGB), 3))
INVALID_DEPTH_COLOR = bytes((70, 74, 80))


def depth_heatmap(data):
    """Map inverse depth in a fixed range to the Turbo palette."""
    colors = []
    far_inverse = 1 / DEPTH_FAR_M
    scale = 255 / (1 / DEPTH_NEAR_M - far_inverse)
    for depth in memoryview(data).cast("f"):
        if not depth > 0:
            colors.append(INVALID_DEPTH_COLOR)
        elif depth >= DEPTH_FAR_M:
            colors.append(DEPTH_COLORS[0])
        elif depth <= DEPTH_NEAR_M:
            colors.append(DEPTH_COLORS[-1])
        else:
            index = int((1 / depth - far_inverse) * scale)
            colors.append(DEPTH_COLORS[index])
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

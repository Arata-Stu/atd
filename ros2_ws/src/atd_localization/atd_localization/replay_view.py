"""保存した HDMap と検証済みの map 座標の姿勢を、RViz のマーカーで表示する。"""
from collections import deque
import json
import math

from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def marker(name, kind, color):
    item = Marker(ns=name, type=kind)
    item.header.frame_id = 'map'
    item.pose.orientation.w = 1.
    item.color.r, item.color.g, item.color.b, item.color.a = (*color, 1.)
    return item


def hdmap_markers(document):
    result = MarkerArray()

    def line(name, points, closed, color):
        if len(points) < 2:
            return
        item = marker(name, Marker.LINE_STRIP, color)
        item.scale.x = .045
        item.points = [Point(x=float(x), y=float(y)) for x, y in points]
        if closed and points[0] != points[-1]:
            item.points.append(item.points[0])
        result.markers.append(item)

    for side in ('left', 'right'):
        line(side, document.get(side, []), document.get('closed'), (.65, .71, .75))
    for name, color in [('centerline', (.3, .65, .7)), ('raceline', (.4, .85, .47)),
                        ('custom', (.7, .54, .88))]:
        reference = document.get('lines', {}).get(name)
        if reference:
            line(name, reference['points'], reference.get('closed', document.get('closed')), color)
    return result


class ReplayView:
    """同じ追跡区間の観測だけを結び、画面用の履歴を間引いて配信する。"""

    def __init__(self, node, hdmap_path):
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.hdmap = node.create_publisher(MarkerArray, '/localization/replay/hdmap', qos)
        self.trajectory = node.create_publisher(MarkerArray, '/localization/replay/trajectory', qos)
        self.hdmap.publish(hdmap_markers(json.loads(hdmap_path.read_bytes())))
        self.interval = None
        self.last_point = None
        self.segments = deque(maxlen=4096)  # LINE_LIST の2頂点を組で追加・破棄する。
        self.trail = marker('Trajectory', Marker.LINE_LIST, (1., .63, .20))
        self.trail.scale.x = .075
        self.arrow = marker('Estimated pose', Marker.ARROW, (1., .47, .10))
        self.arrow.scale.x, self.arrow.scale.y, self.arrow.scale.z = .65, .16, .16
        self.arrow.action = Marker.DELETE
        self.dirty = False
        node.create_timer(.1, self.flush)

    def ready(self):
        return self.hdmap.get_subscription_count() > 0 and self.trajectory.get_subscription_count() > 0

    def observe(self, msg, interval):
        if interval is None:
            self.interval = None
            if self.arrow.action == Marker.ADD:
                self.arrow.action = Marker.DELETE
                self.dirty = True
            return
        self.arrow.action = Marker.ADD
        self.arrow.header, self.arrow.pose = msg.header, msg.pose
        point = msg.pose.position
        if self.interval is not interval:
            self.interval, self.last_point = interval, point
        elif math.dist((self.last_point.x, self.last_point.y, self.last_point.z),
                       (point.x, point.y, point.z)) >= .08:
            self.segments.extend((self.last_point, point))
            self.last_point = point
        self.dirty = True

    def flush(self, *, wait_for_ack=False):
        if self.dirty:
            self.trail.points = list(self.segments)
            self.trajectory.publish(MarkerArray(markers=[self.trail, self.arrow]))
            self.dirty = False
        if wait_for_ack:
            self.trajectory.wait_for_all_acked(Duration(seconds=1))

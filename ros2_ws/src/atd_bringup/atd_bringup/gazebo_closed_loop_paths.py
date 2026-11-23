"""表示専用の後輪軸軌跡。最初の同時刻の追跡姿勢で Gazebo world を保存地図に合わせる。"""
from bisect import bisect_left
from collections import deque
import math

from atd_control.core import CONTROLLER
from atd_mapping.frames import stamp_ns
from atd_planning.core import yaw_from_quaternion

from .gazebo_control import COMMAND_TTL_NS
from .replay_gt_geometry import Alignment, PlanarPose, Trace


def lookahead_remaining_ns(msg, now_ns, identity, line, segment, *, selection_stamp_ns=0):
    """現在の走行地図・追跡区間で、この操舵計算を表示できる残りシミュレーション時間。"""
    if (msg is None or not msg.valid or msg.controller != CONTROLLER or msg.header.frame_id != 'map'
            or (msg.map_id, msg.map_sha256, msg.hdmap_sha256) != identity or msg.line != line
            or not msg.session_id or (msg.session_id, msg.epoch) != segment):
        return 0
    stamp = stamp_ns(msg.header.stamp)
    p, t = msg.pose.position, msg.target
    if (not all(map(math.isfinite, (p.x, p.y, t.x, t.y, msg.validity_s)))
            or not selection_stamp_ns < stamp <= now_ns or msg.validity_s <= 0):
        return 0
    ttl = round(min(msg.validity_s, COMMAND_TTL_NS * 1e-9) * 1e9)
    return max(0, stamp + ttl - now_ns)


class PathHistory:
    def __init__(self, make_pose, history_size):
        self.make_pose = make_pose
        self.points = deque(maxlen=history_size)
        self.current = None
        self.last_position = None
        self.changed = False

    def add(self, stamp, position, orientation):
        self.current = self.make_pose(stamp, position, orientation)
        if not self.points.maxlen:
            return
        if self.last_position is None or math.dist(position, self.last_position) >= .04:
            self.points.append(self.current)
            self.last_position = position
            self.changed = True

    def new_segment(self):
        self.points.clear()
        self.last_position = None
        self.changed = True


class ClosedLoopPaths:
    def __init__(self, map_identity, make_pose, history_size=8192):
        self.identity = map_identity
        self.gt = PathHistory(make_pose, history_size)
        self.estimate = PathHistory(make_pose, history_size)
        self.alignment = None
        self.gt_stamp = None
        self.estimate_stamp = None
        self.segment = None
        self.gt_buffer = deque(maxlen=128)
        self.waiting = deque(maxlen=32)

    def world_pose(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if (msg.header.frame_id != 'gazebo_world' or stamp < 0
                or (self.gt_stamp is not None and stamp <= self.gt_stamp)):
            return
        p, q = msg.pose.position, msg.pose.orientation
        if not all(map(math.isfinite, (p.x, p.y, p.z))):
            return
        world = PlanarPose(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.gt_stamp = stamp
        if self.alignment:
            self.gt.add(stamp, *self.alignment.apply(world))
        else:
            self.gt_buffer.append((stamp, world))
            self.align()

    def localization(self, msg):
        if (msg.header.frame_id != 'map' or not msg.session_id or msg.state != 'tracking'
                or (msg.map_id, msg.map_sha256, msg.hdmap_sha256) != self.identity):
            self.segment = None
            self.estimate.current = None
            self.waiting.clear()
            return
        stamp = stamp_ns(msg.header.stamp)
        if stamp <= 0 or (self.estimate_stamp is not None and stamp <= self.estimate_stamp):
            return
        p, q = msg.pose.position, msg.pose.orientation
        position, orientation = (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
        if not all(map(math.isfinite, position)) or not math.isclose(math.hypot(*orientation), 1., abs_tol=.01):
            return
        segment = (msg.session_id, msg.epoch)
        if self.segment != segment:
            self.estimate.new_segment()
            self.waiting.clear()
            self.segment = segment
        self.estimate_stamp = stamp
        self.estimate.add(stamp, position, orientation)
        if self.alignment is None:
            self.waiting.append((stamp, position, orientation))
            self.align()

    def align(self):
        if not self.waiting or not self.gt_buffer:
            return
        stamps, poses = zip(*self.gt_buffer)
        while self.waiting and self.waiting[0][0] <= stamps[-1]:
            stamp, position, orientation = self.waiting.popleft()
            index = bisect_left(stamps, stamp)
            if stamps[index] != stamp and (index == 0 or stamps[index] - stamps[index-1] > 250_000_000):
                continue
            anchor = Trace(stamps, poses).at(stamp)
            self.alignment = Alignment.from_anchor(anchor, position, orientation)
            self.gt.add(stamp, *self.alignment.apply(anchor))
            for ns, world in self.gt_buffer:
                if ns > stamp:
                    self.gt.add(ns, *self.alignment.apply(world))
            self.waiting.clear()
            self.gt_buffer.clear()
            break

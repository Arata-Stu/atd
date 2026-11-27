"""画像時刻で EKF と地図姿勢を比較し、局所 VO を用いて地図補正量を分離する。"""
import argparse
import csv
import math
import sys

from atd_mapping.frames import rotate, stamp_ns
from .state import RigidPose

EKF = '/localization/rpm_imu/odom'
VO = '/visual_slam/tracking/odometry'
STATUS = '/localization/status'
FIELDS = ('stamp_ns', 'ekf_frame', 'vslam_session', 'epoch', 'map_id', 'map_sha256', 'map_x_m', 'map_y_m', 'map_yaw_rad',
          'aligned_ekf_x_m', 'aligned_ekf_y_m', 'aligned_ekf_yaw_rad',
          'aligned_vo_x_m', 'aligned_vo_y_m', 'aligned_vo_yaw_rad',
          'correction_forward_m', 'correction_left_m', 'correction_yaw_deg')


def pose(msg):
    p, q = msg.position, msg.orientation
    return RigidPose.checked((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def inverse(p):
    x, y, z, w = p.xyzw
    q = (-x, -y, -z, w)
    return RigidPose(tuple(rotate(tuple(-v for v in p.xyz), q)), q)


def planar(p):
    x, y, z, w = p.xyzw
    return p.xyz[0], p.xyz[1], math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_xy(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c*x - s*y, s*x + c*y


class StampCursor:
    """Two adjacent records of an ordered stream, with no extrapolation."""
    def __init__(self, records):
        self.records = iter(records)
        self.left = None
        self.right = next(self.records, None)

    def advance(self, stamp):
        while self.right is not None and self.right[0] < stamp:
            self.left, self.right = self.right, next(self.records, None)

    def exact(self, stamp):
        self.advance(stamp)
        return self.right[1] if self.right and self.right[0] == stamp else None

    def bracket(self, stamp):
        self.advance(stamp)
        if self.right and self.right[0] == stamp:
            return self.right, self.right
        if self.left and self.right and self.left[0] < stamp < self.right[0]:
            return self.left, self.right
        return None


def aligned_rows(samples, visual, statuses):
    return aligned_stream(sorted(samples.items()), sorted(visual.items()), statuses)


def aligned_stream(samples, visual, statuses, *, on_anchor=None):
    wheels, poses = StampCursor(samples), StampCursor(visual)
    previous = ekf_key = vo_key = None
    for t, tracking, map_pose in statuses:
        if map_pose is None:
            previous = vo_key = None
            continue
        vo = poses.exact(t)
        bracket = wheels.bracket(t)
        if vo is None or bracket is None:
            previous = None
            continue
        (left, (frame, a)), (right, (next_frame, b)) = bracket
        if right-left > 200_000_000 or frame != next_frame:
            previous = None
            continue
        f = (t-left)/(right-left) if right != left else 0.
        ex, ey = (a[j] + f*(b[j]-a[j]) for j in range(2))
        yaw = a[2] + f*wrapped(b[2]-a[2])
        mx, my, heading = planar(map_pose)
        key = (frame, *tracking)
        # Preserve the wheel trajectory through a visual loss/relocalization on
        # the same map. Re-anchoring here would hide exactly that discrepancy.
        wheel_map = (frame, tracking[2], tracking[3])
        if ekf_key != wheel_map:
            ekf_key = wheel_map
            angle = heading-yaw
            ox, oy = rotate_xy(ex, ey, angle)
            tx, ty = mx-ox, my-oy
            if on_anchor is not None:
                on_anchor(frame, t, tx, ty, angle)
        if vo_key != key:
            vo_key = key
            vo_alignment = map_pose.compose(inverse(vo))
            previous = None
        ax, ay = rotate_xy(ex, ey, angle)
        delta = ('', '', '')
        if previous and 0 < t-previous[0] <= 200_000_000:
            # Apply the previous map alignment to the CURRENT raw VO pose.
            predicted = previous[1].compose(vo)
            difference = inverse(predicted).compose(map_pose)
            dx, dy, dyaw = planar(difference)
            delta = (dx, dy, math.degrees(dyaw))
        previous = (t, map_pose.compose(inverse(vo)))
        aligned_vo = planar(vo_alignment.compose(vo))
        yield (t, *key, mx, my, heading, tx+ax, ty+ay, wrapped(angle+yaw), *aligned_vo, *delta)


def read_bag(path):
    from atd_interfaces.msg import ControlStatus, LocalizationStatus
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    control = '/control/status'
    reader.set_filter(rosbag2_py.StorageFilter(topics=[EKF, VO, STATUS, control]))
    steering = {}
    samples, visual, statuses = {}, {}, []
    while reader.has_next():
        name, raw, _ = reader.read_next()
        if name == control:
            msg = deserialize_message(raw, ControlStatus)
            if msg.valid and msg.has_steering and math.isfinite(msg.steering_rad):
                steering[(stamp_ns(msg.header.stamp), msg.session_id, msg.epoch,
                          msg.map_id, msg.map_sha256)] = msg.steering_rad
            continue
        msg = deserialize_message(raw, LocalizationStatus if name == STATUS else Odometry)
        t = stamp_ns(msg.header.stamp)
        if name == STATUS:
            statuses.append((t, (msg.session_id, msg.epoch, msg.map_id, msg.map_sha256),
                             pose(msg.pose) if msg.state == 'tracking' and msg.header.frame_id == 'map' else None))
        elif msg.child_frame_id == 'base_link':
            if name == EKF:
                samples[t] = (msg.header.frame_id, planar(pose(msg.pose.pose)))
            elif msg.header.frame_id == 'odom':
                visual[t] = pose(msg.pose.pose)
    return samples, visual, statuses, steering


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    args = parser.parse_args()
    samples, visual, statuses, steering = read_bag(args.bag)
    rows = (row + (steering.get((row[0], *row[2:6])),)
            for row in aligned_rows(samples, visual, statuses))
    first = next(rows, None)
    if first is None:
        parser.exit(1, '比較可能な EKF・局所 VO・tracking の共通時刻がありません。\n')
    writer = csv.writer(sys.stdout)
    writer.writerow((*FIELDS, 'steering_command_rad'))
    writer.writerow(first)
    writer.writerows(rows)

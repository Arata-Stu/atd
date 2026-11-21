"""Ideal optical-frame IMU on the planar, pose-commanded Gazebo camera rig."""
from collections import deque
from math import atan2, cos, radians, sin

from sensor_profile import ACCEL_FRAME, GYRO_FRAME, IMU_FRAME

GRAVITY_M_S2 = 9.80665
NANO = 1_000_000_000


def yaw_delta(a, b):
    return atan2(sin(b - a), cos(b - a))


class RigImu:
    """Resample measured rig poses, then differentiate at the middle sample stamp."""

    def __init__(self, rate, forward_m, pitch_deg):
        self.dt_ns = NANO // rate
        self.dt = 1 / rate
        self.forward_m = forward_m
        self.cp = cos(radians(pitch_deg))
        self.sp = sin(radians(pitch_deg))
        self.previous_tick = None
        self.next_ns = 0
        self.poses = deque(maxlen=3)
        self.last_accel = None
        self.between = None

    def observe(self, stamp, x, y, yaw):
        previous = self.previous_tick
        if previous is None:
            self.previous_tick = stamp, x, y, yaw
            self.next_ns = -(-stamp // self.dt_ns) * self.dt_ns
            return
        t0, x0, y0, a0 = previous
        if stamp <= t0:
            return
        self.previous_tick = stamp, x, y, yaw
        while self.next_ns <= stamp:
            t = self.next_ns
            self.next_ns += self.dt_ns
            u = (t - t0) / (stamp - t0)
            a = a0 + u * yaw_delta(a0, yaw)
            # Differentiating the optical midpoint also measures tangential and
            # centripetal acceleration due to its offset ahead of the rear axle.
            self.poses.append((x0 + u * (x - x0) + self.forward_m * cos(a),
                               y0 + u * (y - y0) + self.forward_m * sin(a), a))
            if len(self.poses) == 3:
                yield from self._sample(t - self.dt_ns)

    def _sample(self, stamp):
        (x0, y0, a0), (x, y, heading), (x1, y1, a1) = self.poses
        dt2 = self.dt ** 2
        ax, ay = ((x1 - x) - (x - x0)) / dt2, ((y1 - y) - (y - y0)) / dt2
        forward = cos(heading) * ax + sin(heading) * ay
        left = -sin(heading) * ax + cos(heading) * ay
        yaw_rate = (yaw_delta(a0, heading) + yaw_delta(heading, a1)) / (2 * self.dt)
        # Rotate world specific force (acceleration minus downward gravity) to
        # pitched camera axes, then to optical x right, y down, z forward.
        accel = (-left, -self.sp * forward - self.cp * GRAVITY_M_S2,
                 self.cp * forward - self.sp * GRAVITY_M_S2)
        gyro = (0., -self.cp * yaw_rate, -self.sp * yaw_rate)
        if stamp % (NANO // 200):
            if self.last_accel is not None:
                yield "gyro", stamp, gyro, None
                self.between = stamp, gyro
            return
        if self.between is not None:
            middle, middle_gyro = self.between
            # 400 Hz gyroscope, 200 Hz accelerometer: one midpoint between
            # successive measured accelerations. The unified stamp is the gyro's.
            interpolated = tuple((a + b) / 2 for a, b in zip(self.last_accel, accel))
            yield "imu", middle, middle_gyro, interpolated
            self.between = None
        yield "gyro", stamp, gyro, None
        yield "accel", stamp, None, accel
        yield "imu", stamp, gyro, accel
        self.last_accel = accel


class ImuPublisher:
    def __init__(self, circuit, camera, rate):
        from gz.msgs.imu_pb2 import IMU
        from gz.transport import Node

        self.node = Node()
        self.message_type = IMU
        self.frames = {"imu": IMU_FRAME, "accel": ACCEL_FRAME, "gyro": GYRO_FRAME}
        self.publishers = {key: self.node.advertise(f"/{circuit}/{key}", IMU) for key in self.frames}
        self.sensor = RigImu(rate, camera["midpoint_forward_from_rear_axle_m"], camera["pitch_down_deg"])

    def update(self, stamp, pose):
        for kind, t, gyro, accel in self.sensor.observe(stamp, pose.x(), pose.y(), pose.yaw()):
            msg = self.message_type()
            msg.header.stamp.sec, msg.header.stamp.nsec = divmod(t, NANO)
            msg.header.data.add(key="frame_id", value=[self.frames[kind]])
            msg.orientation.w = 1.
            unavailable = [-1., 0., 0., 0., 0., 0., 0., 0., 0.]
            msg.orientation_covariance.data.extend(unavailable)
            if gyro is None:
                msg.angular_velocity_covariance.data.extend(unavailable)
            else:
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
            if accel is None:
                msg.linear_acceleration_covariance.data.extend(unavailable)
            else:
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
            self.publishers[kind].publish(msg)

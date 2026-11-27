"""Gazebo の最新 Depth から車体が通過できる操舵角を探す。"""
import math
from threading import Lock

from closed_loop_eval import FOOTPRINT

OPTICAL_FRAME = "camera_depth_optical_frame"
NEAR_M = .025
FAR_M = 5.
VOXEL_M = .025
BODY_HEIGHT_M = .17
GROUND_TOLERANCE_M = .012
LOOKAHEAD_M = 1.25
STEP_M = .035
COMMAND_AGE_NS = 150_000_000
NEAR_FIELD_M = .21
VERTICAL_PADDING_M = .025


def topic(course):
    return f"/{course}/depth/image"


class DepthProjection:
    def __init__(self, width, height, hfov, forward, height_above_floor, pitch):
        self.width, self.height = width, height
        self.focal = width / (2 * math.tan(hfov / 2))
        self.cx, self.cy = (width - 1) / 2, (height - 1) / 2
        self.forward, self.z = forward, height_above_floor
        self.c, self.s = math.cos(pitch), math.sin(pitch)
        self.horizontal = [(self.cx - u) / self.focal for u in range(width)]

    def obstacles(self, image):
        voxels = set()
        for v in range(0, self.height, 2):
            up = (self.cy - v) / self.focal
            x_scale = self.c + self.s * up
            z_scale = -self.s + self.c * up
            row = v * self.width
            for u in range(0, self.width, 2):
                d = image[row + u]
                if not NEAR_M <= d < FAR_M:
                    continue
                z = self.z + d * z_scale
                if not GROUND_TOLERANCE_M < z <= BODY_HEIGHT_M + VERTICAL_PADDING_M + VOXEL_M / 2:
                    continue
                x = self.forward + d * x_scale
                y = d * self.horizontal[u]
                if x <= LOOKAHEAD_M + .4 and abs(y) <= LOOKAHEAD_M + .4:
                    voxels.add(tuple(math.floor(value / VOXEL_M) for value in (x, y, z)))
        return [tuple((axis + .5) * VOXEL_M for axis in cell) for cell in voxels]

    def visible(self, image, x, y, z):
        dx, dz = x - self.forward, z - self.z
        d = self.c * dx - self.s * dz
        if d < NEAR_M:
            return False
        u = round(self.cx - self.focal * y / d)
        v = round(self.cy - self.focal * (self.s * dx + self.c * dz) / d)
        if not (0 <= u < self.width and 0 <= v < self.height):
            return False
        # Gazebo's +Inf denotes a ray beyond the far clip. NaN and -Inf supply no clearance.
        return d + VOXEL_M <= min(image[v * self.width + u], FAR_M)


class GapPlanner:
    def __init__(self, wheelbase, steering_limit, speed_limit, speed_tau, deceleration):
        self.wheelbase = wheelbase
        self.steering_limit = steering_limit
        self.speed_limit = speed_limit
        self.speed_tau = speed_tau
        self.deceleration = deceleration
        self.rear = FOOTPRINT["rear_axle_to_rear"]
        self.front = FOOTPRINT["vehicle_length"] - self.rear
        self.half_width = FOOTPRINT["vehicle_width"] / 2
        # Cover the displacement between sampled poses and the depth voxel quantization.
        self.padding = .055
        self.angles = [steering_limit * index / 10 for index in range(-10, 11)]
        self.arcs = [tuple(self.arc(angle)) for angle in self.angles]

    def arc(self, steering):
        curvature = math.tan(steering) / self.wheelbase
        for i in range(1, math.floor(LOOKAHEAD_M / STEP_M) + 1):
            distance = i * STEP_M
            yaw = distance * curvature
            s, c = math.sin(yaw), math.cos(yaw)
            x = s / curvature if curvature else distance
            y = (1 - c) / curvature if curvature else 0.
            yield distance, x, y, c, s

    def clearance(self, arc, obstacles, visible):
        clear = 0.
        for distance, x, y, c, s in arc:
            for ox, oy, oz in obstacles:
                dx, dy = ox - x, oy - y
                if (oz <= BODY_HEIGHT_M + VERTICAL_PADDING_M
                        and -self.rear - self.padding <= c * dx + s * dy <= self.front + self.padding
                        and abs(-s * dx + c * dy) <= self.half_width + self.padding):
                    return clear
            # Check forward visibility across the vehicle's leading edge. The camera
            # cannot see the underside and rear of the vehicle from its mount.
            if distance < NEAR_FIELD_M:
                continue
            front_x, front_y = x + c * self.front, y + s * self.front
            if not all(visible(front_x - s * side, front_y + c * side, height)
                       for side in (-self.half_width, 0., self.half_width)
                       for height in (.05, .15)):
                return clear
            clear = distance
        return clear

    def stopping_distance(self, speed):
        # Integrate the existing model's saturated deceleration and exponential tail.
        transition = self.deceleration * self.speed_tau
        tail_speed = min(speed, transition)
        braking = (speed**2 - tail_speed**2) / (2 * self.deceleration)
        return .04 + speed * (COMMAND_AGE_NS / 1e9) + braking + tail_speed * self.speed_tau

    def plan(self, obstacles, visible, speed, steering):
        required = self.stopping_distance(speed)
        distances = [self.clearance(arc, obstacles, visible) for arc in self.arcs]
        gaps = []
        for i, distance in enumerate(distances):
            if distance <= required:
                continue
            if not gaps or gaps[-1][-1] != i - 1:
                gaps.append([])
            gaps[-1].append(i)
        if not gaps:
            return steering, 0., {"state": "blocked", "clearance_m": max(distances), "gaps": []}

        def score(i, gap):
            edge = min(i - gap[0], gap[-1] - i)
            return (distances[i] + .025 * edge - .12 * abs(self.angles[i] - steering) / self.steering_limit
                    - .06 * abs(self.angles[i]) / self.steering_limit)

        index = max(((i, gap) for gap in gaps for i in gap), key=lambda item: score(*item))[0]
        clearance = distances[index]
        angle = self.angles[index]
        speed_limit = self.speed_limit * (1 - .35 * abs(angle) / self.steering_limit)
        low, high = 0., speed_limit
        for _ in range(10):
            candidate = (low + high) / 2
            if self.stopping_distance(candidate) <= clearance:
                low = candidate
            else:
                high = candidate
        return angle, low, {"state": "tracking_gap", "clearance_m": clearance,
                            "gaps": [[self.angles[gap[0]], self.angles[gap[-1]]] for gap in gaps]}


class DepthController:
    def __init__(self, course, camera, planner):
        from gz.msgs.image_pb2 import Image, R_FLOAT32
        from gz.transport import Node

        self.projection = DepthProjection(camera["width"], camera["height"], math.radians(camera["horizontal_fov_deg"]),
                                          camera["midpoint_forward_from_rear_axle_m"], camera["height_above_floor_m"],
                                          math.radians(camera["pitch_down_deg"]))
        self.planner = planner
        self.float32 = R_FLOAT32
        self.lock = Lock()
        self.pending = None
        self.last_stamp = 0
        self.node = Node()
        if not self.node.subscribe(Image, topic(course), self.receive):
            raise RuntimeError("Gazebo Depth の購読を開始できません")

    def receive(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nsec
        valid = (msg.pixel_format_type == self.float32 and msg.width == self.projection.width
                 and msg.height == self.projection.height and msg.step == msg.width * 4)
        data = msg.data if valid else b""
        with self.lock:
            if self.pending is None or stamp >= self.pending[0]:
                self.pending = stamp, data

    def update(self, now_ns, speed, steering):
        with self.lock:
            frame = self.pending
        if frame is None or frame[0] > now_ns or frame[0] <= self.last_stamp:
            return None
        stamp, data = frame
        self.last_stamp = stamp
        if now_ns - stamp > COMMAND_AGE_NS or len(data) != self.projection.width * self.projection.height * 4:
            return stamp, steering, 0., {"state": "invalid_or_stale_depth"}
        image = memoryview(data).cast("f")
        obstacles = self.projection.obstacles(image)
        angle, target, report = self.planner.plan(
            obstacles, lambda x, y, z: self.projection.visible(image, x, y, z), speed, steering)
        return stamp, angle, target, dict(report, obstacle_voxels=len(obstacles))

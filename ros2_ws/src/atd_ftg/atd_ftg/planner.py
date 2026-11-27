"""Depth から車体が通過できる操舵角を求める、ROS 非依存の処理。"""
import math

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


class DepthProjection:
    def __init__(self, width, height, fx, fy, cx, cy, origin, axes, *,
                 body_height=BODY_HEIGHT_M, allow_far_clip=False):
        self.width, self.height = width, height
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.origin, self.axes = origin, axes
        self.body_height = body_height
        self.allow_far_clip = allow_far_clip
        right, down, forward = axes
        self.columns = [tuple(axis * (u - cx) / fx for axis in right) for u in range(0, width, 2)]
        self.rows = [tuple(axis * (v - cy) / fy + offset for axis, offset in zip(down, forward))
                     for v in range(0, height, 2)]

    @classmethod
    def simulated(cls, width, height, hfov, forward, height_above_floor, pitch):
        focal = width / (2 * math.tan(hfov / 2))
        c, s = math.cos(pitch), math.sin(pitch)
        return cls(width, height, focal, focal, (width - 1) / 2, (height - 1) / 2,
                   (forward, 0., height_above_floor), ((0., -1., 0.), (-s, 0., -c), (c, 0., -s)),
                   allow_far_clip=True)

    def obstacles(self, image):
        voxels = set()
        x0, y0, z0 = self.origin
        max_height = self.body_height + VERTICAL_PADDING_M + VOXEL_M / 2
        for v, (row_x, row_y, row_z) in enumerate(self.rows):
            offset = v * 2 * self.width
            for u, (col_x, col_y, col_z) in enumerate(self.columns):
                d = image[offset + u * 2]
                if not NEAR_M <= d < FAR_M:
                    continue
                z = z0 + d * (row_z + col_z)
                if not GROUND_TOLERANCE_M < z <= max_height:
                    continue
                x = x0 + d * (row_x + col_x)
                y = y0 + d * (row_y + col_y)
                if x <= LOOKAHEAD_M + .4 and abs(y) <= LOOKAHEAD_M + .4:
                    voxels.add(tuple(math.floor(value / VOXEL_M) for value in (x, y, z)))
        return [tuple((axis + .5) * VOXEL_M for axis in cell) for cell in voxels]

    def visible(self, image, x, y, z):
        dx, dy, dz = (x - self.origin[0], y - self.origin[1], z - self.origin[2])
        right, down, forward = self.axes
        d = forward[0] * dx + forward[1] * dy + forward[2] * dz
        if d < NEAR_M:
            return False
        u = round(self.cx + self.fx * (right[0] * dx + right[1] * dy + right[2] * dz) / d)
        v = round(self.cy + self.fy * (down[0] * dx + down[1] * dy + down[2] * dz) / d)
        if not (0 <= u < self.width and 0 <= v < self.height):
            return False
        measured = image[v * self.width + u]
        # Gazebo reports +Inf beyond the far clip. A sensor's unknown return supplies no clearance.
        if measured == math.inf and not self.allow_far_clip:
            return False
        return d + VOXEL_M <= min(measured, FAR_M)


class GapPlanner:
    def __init__(self, wheelbase, steering_limit, speed_limit, speed_tau, deceleration, *, vehicle_length=.35,
                 vehicle_width=.19, rear_axle_to_rear=.1065, body_height=BODY_HEIGHT_M):
        self.wheelbase = wheelbase
        self.steering_limit = steering_limit
        self.speed_limit = speed_limit
        self.speed_tau = speed_tau
        self.deceleration = deceleration
        self.rear = rear_axle_to_rear
        self.front = vehicle_length - self.rear
        self.half_width = vehicle_width / 2
        self.body_height = body_height
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
                if (oz <= self.body_height + VERTICAL_PADDING_M
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
                       for height in (min(.05, self.body_height / 2), min(.15, self.body_height))):
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

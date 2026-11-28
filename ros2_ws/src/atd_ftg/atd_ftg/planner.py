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
CENTER_SLICE_M = .15
CENTER_FIRST_X_M = .45
CENTER_SLICE_COUNT = 10
CENTER_MAX_EXTRAPOLATION_M = .55
CENTER_WEIGHT = 2.


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
                if x < CENTER_FIRST_X_M + CENTER_SLICE_COUNT * CENTER_SLICE_M and abs(y) <= LOOKAHEAD_M + .4:
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

    def corridor_target(self, obstacles):
        # Use the inner observed surface on either side of each forward slice.
        slices = [[None, None] for _ in range(CENTER_SLICE_COUNT)]
        for x, y, z in obstacles:
            if x < CENTER_FIRST_X_M or not GROUND_TOLERANCE_M < z <= self.body_height + VERTICAL_PADDING_M:
                continue
            index = int((x - CENTER_FIRST_X_M) / CENTER_SLICE_M)
            if index >= len(slices) or abs(y) <= self.half_width:
                continue
            side = int(y < 0.)
            current = slices[index][side]
            if current is None or abs(y) < abs(current):
                slices[index][side] = y

        # Require both observed boundaries to continue through adjacent slices.
        runs = []
        current = []
        minimum_width = 2 * (self.half_width + self.padding)
        for index, (left, right) in enumerate(slices):
            if left is None or right is None or left - right < minimum_width:
                current = []
                continue
            x = CENTER_FIRST_X_M + (index + .5) * CENTER_SLICE_M
            if not current or max(abs(left - current[-1][1]), abs(right - current[-1][2])) > 2 * CENTER_SLICE_M:
                current = []
                runs.append(current)
            current.append((x, left, right))
        corridor = max(runs, key=len, default=[])
        if len(corridor) < 3:
            return None
        first_x = corridor[0][0]
        preferred_x = max(.95, first_x - CENTER_MAX_EXTRAPOLATION_M + CENTER_SLICE_M / 2)
        target_x = min(corridor[-1][0], preferred_x, 1.10)
        if first_x - target_x > CENTER_MAX_EXTRAPOLATION_M:
            return None
        mean_x = sum(x for x, _, _ in corridor) / len(corridor)
        mean_y = sum((left + right) / 2 for _, left, right in corridor) / len(corridor)
        slope = sum((x - mean_x) * ((left + right) / 2 - mean_y) for x, left, right in corridor)
        slope /= sum((x - mean_x)**2 for x, _, _ in corridor)
        return target_x, mean_y + slope * (target_x - mean_x)

    def lateral_at(self, angle, forward, clearance):
        curvature = math.tan(angle) / self.wheelbase
        offset = curvature * forward
        if abs(offset) >= 1.:
            return None
        travel = math.asin(offset) / curvature if curvature else forward
        if travel > clearance:
            return None
        return forward * offset / (1 + math.sqrt(1 - offset**2))

    def plan(self, obstacles, visible, speed, steering):
        required = self.stopping_distance(speed)
        collision_points = [point for point in obstacles if point[0] <= LOOKAHEAD_M + .4]
        distances = [self.clearance(arc, collision_points, visible) for arc in self.arcs]
        gaps = []
        for i, distance in enumerate(distances):
            if distance <= required:
                continue
            if not gaps or gaps[-1][-1] != i - 1:
                gaps.append([])
            gaps[-1].append(i)
        if not gaps:
            return steering, 0., {"state": "blocked", "clearance_m": max(distances), "gaps": []}

        target = self.corridor_target(obstacles)
        center_errors = {}
        if target is not None:
            for gap in gaps:
                for i in gap:
                    y = self.lateral_at(self.angles[i], target[0], distances[i])
                    if y is not None:
                        center_errors[i] = abs(y - target[1])

        def score(i, gap):
            edge = min(i - gap[0], gap[-1] - i)
            progress = distances[i]
            if center_errors:
                progress = min(progress, target[0] + .15) - CENTER_WEIGHT * center_errors[i]
            return (progress + .025 * edge - .12 * abs(self.angles[i] - steering) / self.steering_limit
                    - .06 * abs(self.angles[i]) / self.steering_limit)

        candidates = ((i, gap) for gap in gaps for i in gap if not center_errors or i in center_errors)
        index = max(candidates, key=lambda item: score(*item))[0]
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
        report = {"state": "tracking_gap", "clearance_m": clearance,
                  "gaps": [[self.angles[gap[0]], self.angles[gap[-1]]] for gap in gaps]}
        if center_errors:
            report.update(center_target_m=target, center_error_m=center_errors[index])
        return angle, low, report

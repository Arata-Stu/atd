"""Score the actual rear-axle motion in Gazebo world coordinates, independently of SLAM."""
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'apps/web/processing'))
from atd_processing.planning.footprint import bounds, close_boxes, rectangle, swept
from atd_processing.planning.geometry import cross, edges, inside, intersects, segment_distance


# Simulated RC footprint around the rear axle, in metres, matching the 2D benchmark.
FOOTPRINT = dict(vehicle_length=.35, vehicle_width=.19, rear_axle_to_rear=.1065, margin=0.)


class CourseEvaluation:
    def __init__(self, boundaries, samples, requested_laps):
        if type(requested_laps) is not int or requested_laps < 1:
            raise ValueError('周回数には正の整数を指定してください')
        self.left, self.right = boundaries['left'], boundaries['right']
        if len(self.left) != len(self.right) or len(samples) != len(self.left) + 1:
            raise ValueError('Gazebo の走路境界と周回経路の対応が不正です')
        self.walls = [(a, b, bounds((a, b))) for side in (self.left, self.right)
                      for a, b in edges(side, True)]
        self.center = []
        for (_, x, y, _), (_, nx, ny, _) in zip(samples, samples[1:]):
            length = math.hypot(nx - x, ny - y)
            self.center.append((x, y, (nx - x) / length, (ny - y) / length, length))

        # Cross three full-width sector gates in sequence, then the timing line.
        # This rejects rocking on the start line or passing on a parallel lane.
        stations = [s[0] for s in samples[:-1]]
        sector_indices = [min(range(len(stations)), key=lambda i: abs(stations[i] - f * samples[-1][0]))
                          for f in (.25, .5, .75)]
        self.gates = [(self.left[i], self.right[i]) for i in (*sector_indices, 0)]
        self.requested_laps = requested_laps
        self.next_gate = 0
        self.previous = None
        self.start = self.last_crossing = None
        self.distance = 0.
        self.lateral = None
        self.lap_times = []

    def _touches_boundary(self, previous, pose):
        # The footprint starts on the road. After that, any departure must intersect a wall.
        if self.previous is None:
            for corner in rectangle(pose[:2], pose[2], FOOTPRINT):
                if inside(corner, self.left) == inside(corner, self.right):
                    return True
        arc_padding = math.dist(previous[:2], pose[:2]) * abs(math.remainder(pose[2] - previous[2], math.tau)) / 4
        for _, hull, pad in swept([previous[:2], pose[:2]], False, FOOTPRINT, [previous[2], pose[2]]):
            box = bounds(hull)
            sides = edges(hull, True)
            pad += arc_padding
            for a, b, wall_box in self.walls:
                if close_boxes(box, wall_box, pad) and (
                    inside(a, hull) or any(segment_distance(a, b, p, q) <= pad for p, q in sides)
                ):
                    return True
        return False

    def _center_offset(self, x, y):
        nearest = math.inf
        offset = 0.
        for x0, y0, ux, uy, length in self.center:
            dx, dy = x - x0, y - y0
            t = min(length, max(0., dx * ux + dy * uy))
            squared = (dx - t * ux) ** 2 + (dy - t * uy) ** 2
            if squared < nearest:
                nearest, offset = squared, ux * dy - uy * dx
        return offset

    def report(self, status, sim_time_s, reason=''):
        return dict(status=status, reason=reason, laps_completed=len(self.lap_times),
                    laps_requested=self.requested_laps, lap_times_s=list(self.lap_times),
                    distance_m=self.distance, lateral_error_m=self.lateral,
                    lateral_reference='gazebo_course_center_left_positive',
                    started_sim_s=self.start, finish_sim_s=sim_time_s, sim_time_s=sim_time_s)

    def observe(self, sim_time_s, x_m, y_m, yaw_rad):
        pose = (x_m, y_m, yaw_rad)
        if not all(math.isfinite(v) for v in (sim_time_s, *pose)):
            return self.report('invalid_trace', None, 'Gazebo の観測姿勢か時刻が非有限です')
        if self.previous is None:
            self.start = self.last_crossing = sim_time_s
            prev_time, prev_pose = sim_time_s, pose
        else:
            prev_time, prev_pose = self.previous
            if sim_time_s <= prev_time:
                return self.report('invalid_trace', sim_time_s, 'Gazebo の観測時刻が進んでいません')
        self.distance += math.dist(prev_pose[:2], pose[:2])
        self.lateral = self._center_offset(x_m, y_m)
        if self._touches_boundary(prev_pose, pose):
            return self.report('off_course', sim_time_s, '車体の外形が舗装境界に達しました')
        self.previous = sim_time_s, pose
        left, right = self.gates[self.next_gate]
        before, after = cross(left, right, prev_pose), cross(left, right, pose)
        if before < 0 <= after and intersects(prev_pose[:2], pose[:2], left, right):
            self.next_gate = (self.next_gate + 1) % len(self.gates)
            if self.next_gate == 0:
                crossing = prev_time + (sim_time_s - prev_time) * (-before / (after - before))
                self.lap_times.append(crossing - self.last_crossing)
                self.last_crossing = crossing
                if len(self.lap_times) == self.requested_laps:
                    return self.report('completed', crossing)
        return None

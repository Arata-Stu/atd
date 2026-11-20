"""Monza の保存済み走行線を、Gazebo のシミュレーション時刻で移動するカメラ台。"""

from bisect import bisect_right
import json
from math import atan2, cos, sin
from pathlib import Path


def interpolate(samples, distances, distance):
    index = max(0, min(bisect_right(distances, distance) - 1, len(samples) - 2))
    start, end = samples[index:index + 2]
    weight = max(0.0, min(1.0, (distance - start[0]) / (end[0] - start[0])))
    delta_yaw = atan2(sin(end[3] - start[3]), cos(end[3] - start[3]))
    return (start[1] + weight * (end[1] - start[1]),
            start[2] + weight * (end[2] - start[2]), start[3] + weight * delta_yaw)


def travel_fraction(seconds, duration, repeat):
    progress = max(seconds, 0.0) / duration
    if repeat:
        # Reverse along the same poses on alternate passes (no teleport / yaw flip).
        return 1.0 - abs(progress % 2.0 - 1.0)
    return min(progress, 1.0)


class MonzaMotion:
    def configure(self, entity, sdf, ecm, event_mgr):
        from gz.sim import Link, Model
        from gz.math import Pose3d

        scene_file = Path(sdf.get_string("scene_file"))
        scene = json.loads(scene_file.read_text())
        self.model = Model(entity)
        self.link = Link(self.model.canonical_link(ecm))
        self.pose_type = Pose3d
        self.samples = scene["trajectory"]["samples_s_x_y_yaw"]
        self.distances = [point[0] for point in self.samples]
        self.duration = scene["trajectory"]["travel_seconds"]
        self.repeat = scene["trajectory"]["repeat"]
        self.trace = scene_file.with_name("rig_motion.jsonl")
        self.trace.write_text("")
        self.next_trace_s = 0.0

    def pre_update(self, info, ecm):
        if info.paused:
            return
        fraction = travel_fraction(info.sim_time.total_seconds(), self.duration, self.repeat)
        x, y, yaw = interpolate(self.samples, self.distances, fraction * self.distances[-1])
        self.model.set_world_pose_cmd(ecm, self.pose_type(x, y, 0, 0, 0, yaw))

    def post_update(self, info, ecm):
        seconds = info.sim_time.total_seconds()
        if info.paused or seconds < self.next_trace_s:
            return
        # Inspect the actual post-physics link, independent of the requested pose.
        pose = self.link.world_pose(ecm)
        sample = {"sim_time_s": seconds, "x_m": pose.x(), "y_m": pose.y(), "yaw_rad": pose.yaw()}
        with self.trace.open("a") as stream:
            stream.write(json.dumps(sample) + "\n")
        self.next_trace_s += min(.5, self.duration / 8)


def get_system():
    return MonzaMotion()

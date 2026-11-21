"""Monza の保存済み走行線を、Gazebo のシミュレーション時刻で移動するカメラ台。"""

from bisect import bisect_right
import json
from math import atan2, cos, hypot, sin
from pathlib import Path


def interpolate(samples, distances, distance):
    index = max(0, min(bisect_right(distances, distance) - 1, len(samples) - 2))
    start, end = samples[index:index + 2]
    weight = max(0.0, min(1.0, (distance - start[0]) / (end[0] - start[0])))
    delta_yaw = atan2(sin(end[3] - start[3]), cos(end[3] - start[3]))
    return (start[1] + weight * (end[1] - start[1]),
            start[2] + weight * (end[2] - start[2]), start[3] + weight * delta_yaw)


def travel_fraction(seconds, duration, repeat, *, closed=False, laps=1):
    progress = max(seconds, 0.0) / duration
    if not repeat and progress >= laps:
        return 1.0
    if closed:
        return progress % 1.0
    if repeat:
        # Reverse along the same poses on alternate passes (no teleport / yaw flip).
        return 1.0 - abs(progress % 2.0 - 1.0)
    return progress


class MonzaMotion:
    def __init__(self):
        self.closed = False
        self.finished = False
        self.start_s = 0.0
        self.laps = 1

    def configure(self, entity, sdf, ecm, event_mgr):
        from gz.sim import Link, Model
        from gz.math import Pose3d

        scene_file = Path(sdf.get_string("scene_file"))
        scene = json.loads(scene_file.read_text())
        self.model = Model(entity)
        self.link = Link(self.model.canonical_link(ecm))
        self.pose_type = Pose3d
        trajectory = scene["trajectory"]
        self.samples = trajectory["samples_s_x_y_yaw"]
        self.distances = [point[0] for point in self.samples]
        self.duration = trajectory["travel_seconds"]
        self.repeat = trajectory["repeat"]
        self.laps = trajectory.get("laps", 1)
        self.closed = trajectory.get("route") == "lap"
        self.start_s = None if trajectory.get("wait_for_capture") else 0.0
        self.capture_ready = scene_file.with_name("lap_capture.ready")
        self.lap_start = scene_file.with_name("lap_start.json")
        self.capture_finish = scene_file.with_name("lap_finished.ready") if self.start_s is None else None
        self.trace = scene_file.with_name("rig_motion.jsonl")
        self.trace.write_text("")
        self.next_trace_s = 0.0

    def pre_update(self, info, ecm):
        if info.paused:
            return
        seconds = info.sim_time.total_seconds()
        if self.start_s is None:
            if not self.capture_ready.is_file():
                return  # The SDF pose is the first sample; gravity is disabled on the rig.
            self.start_s = seconds
            started = self.lap_start.with_suffix(".tmp")
            started.write_text(json.dumps({"sim_time_s": seconds}) + "\n")
            started.replace(self.lap_start)
            self.next_trace_s = seconds
        fraction = travel_fraction(seconds - self.start_s, self.duration, self.repeat,
                                   closed=self.closed, laps=self.laps)
        x, y, yaw = interpolate(self.samples, self.distances, fraction * self.distances[-1])
        self.model.set_world_pose_cmd(ecm, self.pose_type(x, y, 0, 0, 0, yaw))

    def post_update(self, info, ecm):
        seconds = info.sim_time.total_seconds()
        # Final sample is already flushed when the completion marker appears.
        # Readers may validate the trace while the sensor streams continue.
        if info.paused or self.start_s is None or self.finished:
            return
        at_finish = self.closed and not self.repeat and seconds >= self.start_s + self.duration * self.laps
        if seconds < self.next_trace_s and not at_finish:
            return
        # Inspect the actual post-physics link, independent of the requested pose.
        pose = self.link.world_pose(ecm)
        sample = {"sim_time_s": seconds, "x_m": pose.x(), "y_m": pose.y(), "yaw_rad": pose.yaw()}
        with self.trace.open("a") as stream:
            stream.write(json.dumps(sample) + "\n")
        if at_finish:
            # A world-pose command can take another physics step to reach the link.
            _, x, y, _ = self.samples[-1]
            self.finished = hypot(pose.x() - x, pose.y() - y) <= .05
            if self.capture_finish is not None:
                if self.finished:
                    self.capture_finish.touch()
                elif seconds > self.start_s + self.duration * self.laps + 1:
                    self.capture_finish.with_suffix(".failed").touch()
        interval = min(.5, self.duration / (120 if self.closed else 8))
        if seconds >= self.next_trace_s:
            self.next_trace_s += interval


def get_system():
    return MonzaMotion()

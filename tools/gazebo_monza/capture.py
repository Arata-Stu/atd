#!/usr/bin/env python3
"""Measure exact-stamp ROS stereo delivery or encode a pair of videos."""
import argparse
import json
import math
import subprocess
import sys
import time
from collections import OrderedDict
from functools import partial
from pathlib import Path

from sensor_profile import CAMERA_RATES, DEFAULT_CAMERA_HZ, RESOLUTIONS

WIDTH, HEIGHT = RESOLUTIONS["640x480"]
VIDEO_HZ = 30
TOLERANCE = 0.01
NANO = 1_000_000_000
TOPICS = {
    "left": "/camera/camera/infra1/image_rect_raw",
    "right": "/camera/camera/infra2/image_rect_raw",
}
INFO_TOPICS = {side: topic.replace("image_rect_raw", "camera_info") for side, topic in TOPICS.items()}
FRAMES = {"left": "camera_infra1_optical_frame", "right": "camera_infra2_optical_frame"}
BASELINE_M = 0.095
PENDING_FRAMES = 16


def stamp_intervals(stamps):
    return [(b - a) / NANO for a, b in zip(stamps, stamps[1:])]


def missing_slots(intervals, camera_hz=DEFAULT_CAMERA_HZ):
    return sum(max(0, round(dt * camera_hz) - 1) for dt in intervals)


def udp_error_counters(path=Path("/proc/net/snmp")):
    # host networking shares these counters with all other processes on the Notebook.
    try:
        header, values = [line.split() for line in path.read_text().splitlines() if line.startswith("Udp:")]
        row = dict(zip(header, values))
        return {name: int(row[name]) for name in ("InErrors", "RcvbufErrors", "SndbufErrors")}
    except (OSError, ValueError, KeyError):
        return None


class StereoPairs:
    """Both sides must carry exactly the same simulation timestamp."""

    def __init__(self):
        self.pending = {side: OrderedDict() for side in TOPICS}
        # Keep only timestamps for the post-run overlap check. Pixel storage stays bounded.
        self.seen = {side: set() for side in TOPICS}
        self.received = dict.fromkeys(TOPICS, 0)
        self.duplicates = dict.fromkeys(TOPICS, 0)
        self.expired = dict.fromkeys(TOPICS, 0)
        self.out_of_order_pairs = 0
        self.stamps = []
        self.arrivals = []

    def feed(self, side, stamp, arrival, frame=None):
        self.received[side] += 1
        seen = self.seen[side]
        if stamp in seen:
            self.duplicates[side] += 1
            return None
        seen.add(stamp)
        other = "right" if side == "left" else "left"
        if stamp not in self.pending[other]:
            self.pending[side][stamp] = frame
            # Limit retention of unmatched full-size video frames.
            if len(self.pending[side]) > PENDING_FRAMES:
                self.pending[side].popitem(last=False)
                self.expired[side] += 1
            return None
        partner = self.pending[other].pop(stamp)
        if self.stamps and stamp <= self.stamps[-1]:
            self.out_of_order_pairs += 1
            return None
        self.stamps.append(stamp)
        self.arrivals.append(arrival)
        return (frame, partner) if side == "left" else (partner, frame)

    def summary(self, wall_seconds, *, image_wall_seconds=None, camera_hz=DEFAULT_CAMERA_HZ):
        count = len(self.stamps)
        intervals = stamp_intervals(self.stamps)
        arrival_intervals = [b - a for a, b in zip(self.arrivals, self.arrivals[1:])]
        span = (self.stamps[-1] - self.stamps[0]) / NANO if count else 0.0
        if image_wall_seconds is None:
            image_wall_seconds = wall_seconds
        streams = {}
        for side, received_stamps in self.seen.items():
            stamps = sorted(received_stamps)
            gaps = stamp_intervals(stamps)
            sim_span = (stamps[-1] - stamps[0]) / NANO if stamps else 0.0
            streams[side] = {
                "unique_images": len(stamps),
                "unique_wall_hz": len(stamps) / image_wall_seconds if image_wall_seconds > 0 else None,
                "unique_sim_hz": (len(stamps) - 1) / sim_span if sim_span > 0 else None,
                "first_sim_ns": stamps[0] if stamps else None,
                "last_sim_ns": stamps[-1] if stamps else None,
                "missing_sim_slots_between_images_estimate": missing_slots(gaps, camera_hz),
                "largest_sim_gap_ms": max(gaps) * 1000 if gaps else None,
            }
        shared = len(self.seen["left"] & self.seen["right"])
        return {
            "target_hz": camera_hz,
            "received_images": self.received,
            "image_measurement_wall_s": image_wall_seconds,
            "image_streams": streams,
            "pairing": {
                "pending_limit_per_eye": PENDING_FRAMES,
                "shared_image_stamps": shared,
                "shared_stamps_evicted_before_match": shared - count - self.out_of_order_pairs,
            },
            "duplicate_images": self.duplicates,
            "unpaired_images": {s: self.expired[s] + len(self.pending[s]) for s in TOPICS},
            "out_of_order_pairs": self.out_of_order_pairs,
            "unique_ordered_pairs": count,
            "first_pair_sim_ns": self.stamps[0] if count else None,
            "last_pair_sim_ns": self.stamps[-1] if count else None,
            "pair_span_sim_s": span,
            "measurement_wall_s": wall_seconds,
            "paired_wall_hz": count / wall_seconds if wall_seconds > 0 else None,
            "paired_sim_hz": (count - 1) / span if span > 0 else None,
            "missing_sim_slots_between_pairs_estimate": missing_slots(intervals, camera_hz),
            "largest_sim_gap_ms": max(intervals) * 1000 if intervals else None,
            "largest_arrival_gap_ms": max(arrival_intervals) * 1000 if intervals else None,
        }


def check_image(image, geometry=(WIDTH, HEIGHT)):
    width, height = geometry
    if ((image.width, image.height, image.encoding) != (width, height, "mono8")
            or image.step < width or len(image.data) != image.step * height):
        raise ValueError(f"Expected {width}x{height} mono8; received {image.width}x{image.height} "
                         f"{image.encoding}, step={image.step}, bytes={len(image.data)}")
    stamp = image.header.stamp
    if stamp.sec < 0 or not (0 <= stamp.nanosec < NANO):
        raise ValueError("Invalid ROS image timestamp")
    return stamp.sec * NANO + stamp.nanosec


def packed_pixels(image):
    data = memoryview(image.data).cast("B")
    if image.step == image.width:
        return data
    return b"".join(data[row * image.step:row * image.step + image.width] for row in range(image.height))


class Calibration:
    """Verify synchronized per-eye metadata on the first complete four-message sample."""

    def __init__(self, geometry=(WIDTH, HEIGHT)):
        self.width, self.height = geometry
        self.samples = OrderedDict()
        self.result = None

    def add(self, key, stamp, message=None):
        if self.result is not None:
            return
        sample = self.samples.setdefault(stamp, {})
        sample[key] = message
        if len(self.samples) > 32:
            self.samples.popitem(last=False)
        if set(sample) != {"pair", "left", "right"}:
            return
        result = {}
        for side in TOPICS:
            info = sample[side]
            if (info.width, info.height, info.header.frame_id) != (self.width, self.height, FRAMES[side]):
                raise ValueError(f"{side} CameraInfo geometry or optical frame mismatch")
            k, p = list(info.k), list(info.p)
            if (len(k) != 9 or len(p) != 12 or not all(math.isfinite(v) for v in (*k, *p))
                    or min(k[0], k[4], p[0], p[5]) <= 0
                    or not (0 < p[2] < self.width and 0 < p[6] < self.height)):
                raise ValueError(f"Invalid {side} camera intrinsics")
            result[side] = {"frame_id": info.header.frame_id, "width": info.width,
                            "height": info.height, "k": k, "p": p}
        lp, rp = result["left"]["p"], result["right"]["p"]
        baseline = lp[3] / lp[0] - rp[3] / rp[0]
        if (not math.isclose(lp[0], rp[0], rel_tol=0.001)
                or not math.isclose(lp[5], rp[5], rel_tol=0.001)
                or not math.isclose(lp[3] / lp[0], 0, abs_tol=0.001)
                or not math.isclose(baseline, BASELINE_M, abs_tol=0.001)):
            raise ValueError(f"Inconsistent stereo projection; P implies {baseline:.6f} m baseline")
        self.result = {"stamp_ns": stamp, "baseline_from_projection_m": baseline, **result}
        self.samples.clear()


class StereoVideo:
    """Sample real stereo pairs at 30 fps on the simulation time axis."""

    def __init__(self, output, duration, *, include_finish=False, geometry=(WIDTH, HEIGHT),
                 camera_hz=DEFAULT_CAMERA_HZ):
        self.output = output
        self.duration_ns = round(duration * NANO)
        self.camera_hz = camera_hz
        self.include_finish = include_finish
        self.frame_count = math.ceil(duration * VIDEO_HZ) + int(include_finish)
        self.written = 0
        self.repeated_frames = 0
        self.first_encoded_stamp = None
        self.last_encoded_stamp = None
        self.origin = None
        self.last_stamp = None
        self.last_frames = None
        self.files = [output / (side + ".partial.mp4") for side in TOPICS]
        width, height = geometry
        # Two stacked gray images in a single pipe keep encoder outputs in lockstep.
        command = [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin", "-n",
            "-f", "rawvideo", "-pixel_format", "gray", "-video_size", f"{width}x{height * 2}",
            "-framerate", str(VIDEO_HZ), "-i", "pipe:0", "-filter_complex_threads", "1",
            "-filter_complex",
            f"[0:v]split=2[a][b];[a]crop={width}:{height}:0:0[left];"
            f"[b]crop={width}:{height}:0:{height}[right]",
        ]
        for side, path in zip(TOPICS, self.files):
            command += ["-map", f"[{side}]", "-c:v", "libx264", "-preset", "veryfast",
                        "-crf", "18", "-threads", "2", "-pix_fmt", "yuv420p",
                        "-movflags", "+faststart", str(path)]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE)

    def add(self, stamp, frames):
        if self.origin is None:
            self.origin = stamp
        # Resample by capture time. Sparse inputs hold the previous real pair.
        while self.written < self.frame_count:
            finish = self.include_finish and self.written == self.frame_count - 1
            # Pose commands can take one physics update to affect an exposure.
            # The last video frame shows the parked goal after two camera ticks.
            offset = round(self.written * NANO / VIDEO_HZ)
            if finish:
                offset = self.duration_ns + math.ceil(2 * NANO / self.camera_hz)
            target = self.origin + offset
            if target > stamp:
                break
            # The one-shot rig holds its finish pose. Use a fresh pair from there.
            use_current = finish or target == stamp or self.last_frames is None
            selected_stamp = stamp if use_current else self.last_stamp
            selected_frames = frames if use_current else self.last_frames
            for pixels in selected_frames:
                self.process.stdin.write(pixels)
            if selected_stamp == self.last_encoded_stamp:
                self.repeated_frames += 1
            if self.first_encoded_stamp is None:
                self.first_encoded_stamp = selected_stamp
            self.last_encoded_stamp = selected_stamp
            self.written += 1
        self.last_stamp, self.last_frames = stamp, frames

    def close(self, success):
        try:
            if success:
                self.process.stdin.close()
                if self.process.wait(timeout=30) != 0 or self.written != self.frame_count:
                    raise RuntimeError("FFmpeg did not finish both stereo videos")
                for temp, side in zip(self.files, TOPICS):
                    temp.rename(self.output / f"{side}.mp4")
        finally:
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait()
            if not self.process.stdin.closed:
                try:
                    self.process.stdin.close()
                except BrokenPipeError:
                    pass
            for temp in self.files:
                temp.unlink(missing_ok=True)


def benchmark_passes(metrics):
    sim_hz = metrics["paired_sim_hz"]
    wall_hz = metrics["paired_wall_hz"]
    target_hz = metrics["target_hz"]
    return (sim_hz is not None and wall_hz is not None
            and wall_hz >= target_hz * (1 - TOLERANCE)
            and abs(sim_hz / target_hz - 1) <= TOLERANCE
            and metrics["missing_sim_slots_between_pairs_estimate"] == 0
            and metrics["out_of_order_pairs"] == 0)


def capture(args, report):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = rclpy.create_node("atd_monza_stereo_" + args.mode)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    pairs = StereoPairs()
    start = None
    first_pair_at = None
    video = None
    deadline = None
    success = False
    measure = args.mode == "benchmark"
    udp_start = None
    ready = args.output / f"{args.mode}.ready"
    ready_at = None
    geometry = RESOLUTIONS[args.resolution]
    calibration = Calibration(geometry)
    info_subscriptions = []
    lap_video = args.mode == "video" and args.route == "lap"
    scene_dir = args.output / "scene"
    pending_lap_start = scene_dir / "lap_start.json"
    armed = False

    def receive(side, image):
        nonlocal first_pair_at
        arrival = time.monotonic()
        if ready_at is None or (measure and deadline is not None and arrival >= deadline):
            return
        stamp = check_image(image, geometry)
        if image.header.frame_id != FRAMES[side]:
            raise ValueError(f"{side} image optical frame mismatch: {image.header.frame_id!r}")
        result = pairs.feed(side, stamp, arrival, None if measure else packed_pixels(image))
        if result is not None:
            if first_pair_at is None:
                first_pair_at = arrival
            calibration.add("pair", stamp)
            if video is not None:
                if lap_video and video.origin is None:
                    if not pending_lap_start.is_file():
                        # Calibration and encoder are ready while the rig is parked.
                        # Retain a real stereo pair at the starting pose.
                        video.last_stamp, video.last_frames = stamp, result
                        return
                    start_s = json.loads(pending_lap_start.read_text())["sim_time_s"]
                    if not math.isfinite(start_s) or start_s < 0:
                        raise ValueError("Invalid lap start simulation time")
                    video.origin = round(start_s * NANO)
                video.add(stamp, result)

    def receive_info(side, message):
        stamp = message.header.stamp
        if ready_at is not None:
            calibration.add(side, stamp.sec * NANO + stamp.nanosec, message)

    try:
        if not measure:
            video = StereoVideo(args.output, args.duration, include_finish=lap_video,
                                geometry=geometry, camera_hz=args.camera_hz)
        subscriptions = {}
        for side, topic in TOPICS.items():
            subscriptions[topic] = node.create_subscription(
                Image, topic, partial(receive, side), qos_profile_sensor_data)
            info = node.create_subscription(CameraInfo, INFO_TOPICS[side],
                                            partial(receive_info, side), qos_profile_sensor_data)
            subscriptions[INFO_TOPICS[side]] = info
            info_subscriptions.append(info)
        discovery_at = time.monotonic()
        while rclpy.ok():
            missing = [topic for topic, sub in subscriptions.items() if sub.get_publisher_count() == 0]
            if not missing:
                break
            if time.monotonic() - discovery_at >= args.startup_timeout:
                raise TimeoutError("No matched ROS publisher for " + ", ".join(missing))
            executor.spin_once(timeout_sec=0.05)
        if not rclpy.ok():
            raise RuntimeError("ROS shut down during publisher discovery")
        ready.touch(exist_ok=False)
        ready_at = time.monotonic()
        report.update(ready=True, publisher_discovery_wall_s=ready_at - discovery_at)
        while rclpy.ok():
            now = time.monotonic()
            if info_subscriptions and calibration.result is not None:
                for sub in info_subscriptions:
                    node.destroy_subscription(sub)
                info_subscriptions.clear()
            if calibration.result is None:
                if now - ready_at >= args.startup_timeout:
                    raise TimeoutError("No synchronized ROS image and CameraInfo stereo sample before startup timeout")
            elif measure:
                if start is None and now >= first_pair_at + args.warmup:
                    # Discovery and warm-up samples do not count towards either rate.
                    pairs = StereoPairs()
                    udp_start = udp_error_counters()
                    start = now
                    deadline = start + args.duration
                if deadline is not None and now >= deadline:
                    success = True
                    break
            else:
                if lap_video and not armed:
                    (scene_dir / "lap_capture.ready").touch(exist_ok=False)
                    armed = True
                if (video.origin is not None and video.written == video.frame_count
                        and pairs.stamps[-1] - video.origin >= video.duration_ns):
                    success = True
                    break
                if lap_video and video.origin is None and now - first_pair_at > args.startup_timeout:
                    raise TimeoutError("Calibrated stereo is ready; the camera rig did not release the lap")
                # A long lap on a slow renderer can legitimately take more than
                # ten real seconds per simulation second. Watch advancing pairs.
                last_progress = pairs.arrivals[-1] if lap_video else first_pair_at
                if now - last_progress > args.video_timeout:
                    raise TimeoutError("Video did not cover the requested simulation interval")
            executor.spin_once(timeout_sec=0.015)
        if not success:
            raise RuntimeError("ROS shut down before capture completed")
    finally:
        now = time.monotonic()
        if measure:
            elapsed = args.duration if success else (max(now - start, 0.0) if start else 0.0)
            image_elapsed = elapsed
            report["measurement_start_monotonic_s"] = start
            report["measurement_end_monotonic_s"] = start + elapsed if start is not None else None
            udp_end = udp_error_counters() if udp_start is not None else None
            report["udp_namespace_counter_deltas"] = (
                {key: udp_end[key] - value for key, value in udp_start.items()}
                if udp_end is not None and all(udp_end[key] >= value for key, value in udp_start.items())
                else None
            )
        else:
            elapsed = max(now - first_pair_at, 0.0) if first_pair_at is not None else 0.0
            image_elapsed = max(now - ready_at, 0.0) if ready_at is not None else 0.0
        report.update(pairs.summary(elapsed, image_wall_seconds=image_elapsed, camera_hz=args.camera_hz))
        if first_pair_at is not None and ready_at is not None:
            report["first_pair_wait_wall_s"] = first_pair_at - ready_at
        report["stereo_calibration"] = calibration.result
        try:
            if video is not None:
                report.update(video_fps=VIDEO_HZ, video_frames=video.written,
                              held_video_frames=video.repeated_frames)
                if lap_video:
                    report.update(lap_start_sim_ns=video.origin,
                                  first_encoded_pair_sim_ns=video.first_encoded_stamp,
                                  last_encoded_pair_sim_ns=video.last_encoded_stamp)
                video.close(success)
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.try_shutdown()

    if measure:
        passed = benchmark_passes(report)
        report["pass_target_hz"] = passed
        if args.camera_hz == 90:
            report["pass_90hz"] = passed
        report["status"] = "pass" if passed else "below_target"
        return 0 if passed else 2
    report.update(status="complete", videos=["left.mp4", "right.mp4"])
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mode", required=True, choices=("video", "benchmark"))
    parser.add_argument("--route", choices=("chicane", "lap"), default="chicane")
    parser.add_argument("--resolution", choices=RESOLUTIONS, default="640x480")
    parser.add_argument("--camera-hz", type=int, choices=CAMERA_RATES, default=DEFAULT_CAMERA_HZ)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--warmup", type=float, default=1.0, help="benchmark wall seconds after first pair")
    parser.add_argument("--startup-timeout", type=float, default=60.0, help="wall seconds to first pair")
    parser.add_argument("--video-timeout", type=float,
                        help="wall seconds without progress for a lap, or after first pair for a chicane")
    args = parser.parse_args()
    if args.video_timeout is None:
        args.video_timeout = 60. if args.route == "lap" else max(60, args.duration * 10)
    for key in ("duration", "startup_timeout", "video_timeout", "warmup"):
        value = getattr(args, key)
        if not math.isfinite(value) or (value < 0 if key == "warmup" else value <= 0):
            parser.error(f"Invalid {key}: {value}")
    args.output.mkdir(parents=True, exist_ok=True)
    target = args.output / f"{args.mode}.json"
    if target.exists() or (args.output / f"{args.mode}.ready").exists():
        parser.error(f"Use a fresh output directory; existing {args.mode} results in {args.output}")
    if args.mode == "video" and any((args.output / f"{s}{ext}").exists() for s in TOPICS
                                    for ext in (".mp4", ".partial.mp4")):
        parser.error("Existing stereo video files; use a fresh output directory")
    report = {"mode": args.mode, "route": args.route, "topics": TOPICS, "geometry": list(RESOLUTIONS[args.resolution]),
              "encoding": "mono8", "target_hz": args.camera_hz, "rate_tolerance_fraction": TOLERANCE,
              "requested_duration_s": args.duration, "duration_clock": "wall" if args.mode == "benchmark" else "simulation",
              "ready": False, "warmup_wall_s": args.warmup if args.mode == "benchmark" else 0}
    try:
        result = capture(args, report)
    except (Exception, KeyboardInterrupt) as exc:
        report.update(status="error", error=f"{type(exc).__name__}: {exc}")
        result = 1
    pending = target.with_suffix(".json.tmp")
    pending.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    pending.replace(target)
    print(json.dumps(report, ensure_ascii=False), flush=True)
    return result


if __name__ == "__main__":
    sys.exit(main())

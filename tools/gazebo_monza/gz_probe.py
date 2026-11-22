"""Gazebo Transport の画像を別プロセスで購読し、ROS と同じ実時間区間で集計する。"""
import argparse
from functools import partial
import json
from pathlib import Path
from threading import Lock
import time

from capture import NANO

def summarize(images, world, start, end):
    """Images are counted on callback arrival. Match all exact stamps in the window."""
    elapsed = end - start
    counts = {}
    stamps = {}
    for side, samples in images.items():
        selected = [stamp for arrival, stamp in samples if start <= arrival < end]
        counts[side] = len(selected)
        stamps[side] = set(selected)
    paired = sorted(stamps["left"] & stamps["right"])
    gaps = [b - a for a, b in zip(paired, paired[1:])]
    stats = [(arrival, stamp) for arrival, stamp in world if start <= arrival < end]
    wall_span = stats[-1][0] - stats[0][0] if len(stats) > 1 else None
    sim_span = (stats[-1][1] - stats[0][1]) / NANO if len(stats) > 1 else None
    return {
        "measurement_wall_s": elapsed,
        "received_images": counts,
        "eye_wall_hz": {side: count / elapsed for side, count in counts.items()},
        "duplicate_images": {side: counts[side] - len(unique) for side, unique in stamps.items()},
        "unpaired_unique_stamps": {side: len(unique) - len(paired) for side, unique in stamps.items()},
        "matching_stamps": len(paired),
        "paired_wall_hz": len(paired) / elapsed,
        "paired_sim_hz": (len(paired) - 1) * NANO / (paired[-1] - paired[0]) if gaps else None,
        "largest_sim_gap_ms": max(gaps) * 1000 / NANO if gaps else None,
        "world": {"samples": len(stats), "observed_wall_s": wall_span, "sim_span_s": sim_span,
                  "real_time_factor": sim_span / wall_span if wall_span and wall_span > 0 else None},
    }


def measure(output, circuit):
    from gz.msgs.image_pb2 import Image
    from gz.msgs.world_stats_pb2 import WorldStatistics
    from gz.transport import Node

    # Keep only (monotonic callback time, simulation stamp), never the pixel buffer.
    image_topics = {side: f"/{circuit}/{side}/image" for side in ("left", "right")}
    world_topic = f"/world/{circuit}/stats"
    images = {side: [] for side in image_topics}
    world = []
    lock = Lock()
    node = Node()

    def on_image(side, message):
        stamp = message.header.stamp
        with lock:
            images[side].append((time.monotonic(), stamp.sec * NANO + stamp.nsec))

    def on_world(message):
        stamp = message.sim_time
        with lock:
            world.append((time.monotonic(), stamp.sec * NANO + stamp.nsec))

    for side, topic in image_topics.items():
        if not node.subscribe(Image, topic, partial(on_image, side)):
            raise RuntimeError(f"Gazebo 画像の購読に失敗しました: {topic}")
    if not node.subscribe(WorldStatistics, world_topic, on_world):
        raise RuntimeError(f"Gazebo world 統計の購読に失敗しました: {world_topic}")

    listening_at = time.monotonic()
    (output / "gz_probe.ready").touch(exist_ok=False)
    # run.py owns the process lifetime if the ROS receiver exits without a report.
    benchmark = output / "benchmark.json"
    while not benchmark.is_file():
        time.sleep(0.05)
    metrics = json.loads(benchmark.read_text())
    start = metrics.get("measurement_start_monotonic_s")
    end = metrics.get("measurement_end_monotonic_s")
    window = "ros_measurement"
    if start is None or end is None or end <= start:
        # This includes discovery/warmup and must not be compared to ROS rates.
        start, end = listening_at, time.monotonic()
        window = "probe_lifetime_without_ros_window"
    with lock:
        report = summarize(images, world, start, end)
    report.update(window=window, image_topics=image_topics, world_topic=world_topic,
                  measurement_start_monotonic_s=start, measurement_end_monotonic_s=end)
    target = output / "gz_probe.json"
    temporary = target.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(target)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--circuit", default="odaiba")
    args = parser.parse_args()
    measure(args.output, args.circuit)

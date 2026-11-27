"""録画済み D455 左右画像の header.stamp を調べる。"""
import argparse
from itertools import pairwise
import json
from pathlib import Path
import struct

from .bag_drive_timeline import inspect


TOPICS = tuple(f'/camera/camera/infra{side}/image_rect_raw' for side in (1, 2))
IMAGE_TYPE = 'sensor_msgs/msg/Image'


def image_stamp(data):
    # Plain CDR の Image は encapsulation の直後に Header の sec/nanosec がある。
    representation = data[:2]
    if len(data) < 12 or representation not in (b'\x00\x00', b'\x00\x01'):
        return None
    sec, nsec = struct.unpack_from('<iI' if representation[1] else '>iI', data, 4)
    stamp = sec * 1_000_000_000 + nsec
    return stamp if stamp > 0 and nsec < 1_000_000_000 else None


def summarize(stamps, ordered, origin, period):
    backwards = sum(current < previous for previous, current in pairwise(stamps))
    missing = gap_count = longest = 0
    gaps = []
    for previous, current in pairwise(ordered):
        delta = current - previous
        longest = max(longest, delta)
        if delta >= 1.5 * period:
            frames = int(delta / period + 0.5) - 1
            missing += frames
            gap_count += 1
            if len(gaps) < 20:
                gaps.append(dict(after_s=(previous - origin) / 1e9, interval_ms=delta / 1e6,
                                 estimated_missing=frames))
    return dict(estimated_missing=missing if len(ordered) >= 2 else None, gap_count=gap_count,
                duplicates=len(stamps) - len(ordered), backwards=backwards,
                longest_ms=longest / 1e6 if longest else None, gaps=gaps)


def pair_stamps(left, right, tolerance):
    i = j = paired = 0
    while i < len(left) and j < len(right):
        delta = left[i] - right[j]
        if abs(delta) <= tolerance:
            paired += 1
            i += 1
            j += 1
        elif delta < 0:
            i += 1
        else:
            j += 1
    return dict(paired=paired, left_only=len(left) - paired, right_only=len(right) - paired,
                tolerance_ms=tolerance / 1e6)


def analyze(bag, fps):
    import rosbag2_py

    _, _, storage, _, _ = inspect(bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage), rosbag2_py.ConverterOptions('', ''))
    stored = {info.name: info for info in reader.get_all_topics_and_types()}
    selected = [name for name in TOPICS if name in stored and stored[name].type == IMAGE_TYPE
                and stored[name].serialization_format == 'cdr']
    stamps = {name: [] for name in TOPICS}
    invalid = dict.fromkeys(TOPICS, 0)
    if selected:
        reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
        while reader.has_next():
            name, data, _ = reader.read_next()
            stamp = image_stamp(data)
            if stamp is None:
                invalid[name] += 1
            else:
                stamps[name].append(stamp)
    period = 1e9 / fps
    ordered = {name: sorted(set(values)) for name, values in stamps.items()}
    origin = min((values[0] for values in ordered.values() if values), default=0)
    streams = [
        dict(topic=name, status='absent' if name not in stored else 'ready' if name in selected else 'unsupported',
             valid=len(stamps[name]), invalid=invalid[name], **summarize(stamps[name], ordered[name], origin, period))
        for name in TOPICS
    ]
    pairs = pair_stamps(ordered[TOPICS[0]], ordered[TOPICS[1]], period / 4) if all(ordered.values()) else None
    return dict(fps=fps, streams=streams, pairs=pairs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fps', type=int, choices=range(1, 121), required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps(analyze(args.bag.resolve(strict=True), args.fps)))

"""閉じた Gazebo MCAP で、周回を覆うステレオ・IMU と固定 TF を検査する。"""
from bisect import bisect_left, bisect_right
from collections import Counter
import math
from pathlib import Path
import sys

from capture import (BASELINE_M, Calibration, FRAMES, HEIGHT, INFO_TOPICS, NANO, TARGET_HZ,
                     TOPICS, WIDTH, check_image, missing_slots, stamp_intervals)
from sensor_profile import ACCEL_TOPIC, GYRO_TOPIC, IMU_TOPIC, imu_streams


TF = "/tf_static"
CAMERAS = {topic: side for streams in (TOPICS, INFO_TOPICS) for side, topic in streams.items()}
TYPES = {**dict.fromkeys(TOPICS.values(), "sensor_msgs/msg/Image"),
         **dict.fromkeys(INFO_TOPICS.values(), "sensor_msgs/msg/CameraInfo"),
         **dict.fromkeys((IMU_TOPIC, GYRO_TOPIC, ACCEL_TOPIC), "sensor_msgs/msg/Imu"),
         TF: "tf2_msgs/msg/TFMessage"}
MCAP_MAGIC = b"\x89MCAP0\r\n"
# 画像の空白区間を報告する目安。追跡の成否は再生時に判定する。
GAP_WARNING_NS = NANO // 4
UNIX_2000_NS = 946_684_800 * NANO


def load_bag_metadata(bag):
    # Gazebo 用イメージでは Isaac ROS の workspace を colcon build しない。
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2_ws/src/atd_mapping"))
    try:
        from atd_mapping.bag_inputs import load_metadata
    finally:
        sys.path.pop(0)
    return load_metadata(bag)


def inspect_metadata(bag, *, imu_hz=0):
    meta = load_bag_metadata(bag)
    paths = meta.get("relative_file_paths")
    if meta["storage_identifier"] != "mcap" or not isinstance(paths, list) or not paths:
        raise ValueError("metadata.yaml に MCAP ファイルがありません")
    sizes = {}
    for name in paths:
        if not isinstance(name, str) or Path(name).name != name or name in sizes:
            raise ValueError(f"MCAP ファイル名が不正です: {name!r}")
        file = bag / name
        if file.suffix != ".mcap" or file.is_symlink() or not file.is_file():
            raise ValueError(f"MCAP ファイルが見つかりません: {file}")
        sizes[name] = file.stat().st_size
        with file.open("rb") as stream:
            if stream.read(len(MCAP_MAGIC)) != MCAP_MAGIC or sizes[name] <= 2 * len(MCAP_MAGIC):
                raise ValueError(f"MCAP の先頭またはサイズが不正です: {file}")
            stream.seek(-len(MCAP_MAGIC), 2)
            if stream.read() != MCAP_MAGIC:
                raise ValueError(f"MCAP が正しく閉じられていません: {file}")

    counts = {}
    for entry in meta["topics_with_message_count"]:
        topic = entry["topic_metadata"]
        name = topic["name"]
        if name in counts:
            raise ValueError(f"metadata.yaml に topic が重複しています: {name}")
        counts[name] = entry["message_count"]
        if name in TYPES and (topic["type"] != TYPES[name] or topic.get("serialization_format") != "cdr"):
            raise ValueError(f"メッセージ型またはシリアライズ形式が不正です: {name}")
        if name == TF:
            profiles = topic.get("offered_qos_profiles")
            if isinstance(profiles, str):  # rosbag2 metadata version <= 8
                import yaml
                profiles = yaml.safe_load(profiles)
            if not isinstance(profiles, list) or not profiles or not all(
                isinstance(p, dict) and str(p.get("durability")).lower() in ("1", "transient_local")
                for p in profiles
            ):
                raise ValueError("/tf_static の transient_local QoS を確認できません")
    for name in (*CAMERAS, TF, *imu_streams(imu_hz)):
        if counts.get(name, 0) == 0:
            raise ValueError(f"必須の topic が空か欠落しています: {name}")
    if meta.get("message_count") != sum(counts.values()):
        raise ValueError("metadata.yaml のメッセージ総数が topic ごとの件数と一致しません")
    return counts, sizes


def read_messages(bag):
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from tf2_msgs.msg import TFMessage

    types = {Image: TOPICS.values(), CameraInfo: INFO_TOPICS.values(),
             Imu: (IMU_TOPIC, GYRO_TOPIC, ACCEL_TOPIC), TFMessage: [TF]}
    by_topic = {topic: kind for kind, topics in types.items() for topic in topics}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    while reader.has_next():
        topic, raw, receive_ns, _ = reader.read_next_ext()
        yield topic, deserialize_message(raw, by_topic[topic]) if topic in by_topic else None, receive_ns


def add_transforms(edges, message):
    for tf in message.transforms:
        parent, child = tf.header.frame_id, tf.child_frame_id
        t, q = tf.transform.translation, tf.transform.rotation
        position, rotation = (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)
        if not parent or not child or parent == child or {parent, child} & {"map", "odom"}:
            raise ValueError(f"記録に不正な固定 TF があります: {parent} -> {child}")
        if not all(map(math.isfinite, position)) or not math.isclose(math.hypot(*rotation), 1., abs_tol=1e-5):
            raise ValueError(f"固定 TF の平行移動または回転が不正です: {child}")
        if child in edges:
            old_parent, old_t, old_q = edges[child]
            # q と -q は同じ向きを表す。
            rotation_diff = min(math.dist(old_q, rotation), math.dist(old_q, [-x for x in rotation]))
            if old_parent != parent or math.dist(old_t, position) > 1e-6 or rotation_diff > 1e-6:
                raise ValueError(f"固定 TF が記録中に変化しました: {child}")
        edges[child] = parent, position, rotation


def check_extrinsics(edges, *, imu_hz=0):
    # run.transforms() が発行する Gazebo リグの固定フレーム。
    expected = {"camera_link": "base_link", **dict.fromkeys(FRAMES.values(), "camera_link")}
    imu_frames = {frame for frame, _ in imu_streams(imu_hz).values()}
    expected.update(dict.fromkeys(imu_frames, "camera_link"))
    if "base_link" in edges:
        raise ValueError("Gazebo の固定 TF のルートが base_link ではありません")
    for child, parent in expected.items():
        if child not in edges or edges[child][0] != parent:
            raise ValueError(f"必要な固定 TF がありません: {parent} -> {child}")
    left, right = (edges[FRAMES[s]] for s in ("left", "right"))
    baseline = tuple(l - r for l, r in zip(left[1], right[1]))
    if math.dist(baseline, (0., BASELINE_M, 0.)) > .001:
        raise ValueError(f"固定 TF のステレオ基線が不正です: {baseline} m")
    optical = [left, right]
    for frame in imu_frames:
        imu = edges[frame]
        if math.hypot(*imu[1]) > .001:
            raise ValueError("固定 TF の IMU がカメラの中央にありません")
        optical.append(imu)
    for _, _, rotation in optical:
        # camera_link で光軸 z は前、画像の x は右向き。
        aligned = abs(sum(a * b for a, b in zip(rotation, (-.5, .5, -.5, .5))))
        if not math.isclose(aligned, 1., abs_tol=1e-5):
            raise ValueError("固定 TF の optical frame の向きが不正です")
    return {child: parent for child, (parent, _, _) in edges.items()}


def check_imu(name, msg, frame):
    if msg.header.frame_id != frame:
        raise ValueError(f"{name} の IMU frame が不正です: {msg.header.frame_id}")
    if name != GYRO_TOPIC:
        accel = msg.linear_acceleration
        variance = msg.linear_acceleration_covariance[0]
        if variance < 0 or not all(map(math.isfinite, (variance, accel.x, accel.y, accel.z))):
            raise ValueError(f"{name} の IMU 加速度を確認できません")
    if name != ACCEL_TOPIC:
        gyro = msg.angular_velocity
        variance = msg.angular_velocity_covariance[0]
        if variance < 0 or not all(map(math.isfinite, (variance, gyro.x, gyro.y, gyro.z))):
            raise ValueError(f"{name} の IMU 角速度を確認できません")
    tm = msg.header.stamp
    if tm.sec < 0 or not 0 <= tm.nanosec < NANO:
        raise ValueError("IMU の時刻が不正です")
    return tm.sec * NANO + tm.nanosec


def imu_coverage(name, stamps, boundaries, frame, hz):
    before = bisect_right(stamps, boundaries[0]) - 1
    after = bisect_left(stamps, boundaries[-1])
    if before < 0 or after == len(stamps):
        raise ValueError(f"{name} の IMU ヘッダ時刻が周回の両端を含んでいません")
    interior = []
    for lap, (start, finish) in enumerate(zip(boundaries, boundaries[1:]), 1):
        count = bisect_left(stamps, finish) - bisect_right(stamps, start)
        if count <= 0:
            raise ValueError(f"{lap}周目の移動区間内に {name} の IMU サンプルがありません")
        interior.append(count)
    covered = stamps[before:after + 1]
    intervals = stamp_intervals(covered)
    # シミュレーションの IMU は等間隔で、欠落時の間隔は周期の整数倍になる。
    slots = [round(dt * hz) for dt in intervals]
    if 1 not in slots or any(n < 1 or abs(dt * hz - n) > .01 for dt, n in zip(intervals, slots)):
        raise ValueError(f"{name} の IMU ヘッダ時刻で {hz} Hz の周期を確認できません")
    missing = sum(n - 1 for n in slots)
    return {"frame": frame, "nominal_sim_hz": hz,
            "measured_sim_hz": (len(covered) - 1) * NANO / (covered[-1] - covered[0]),
            "first_covering_sim_ns": covered[0], "last_covering_sim_ns": covered[-1],
            "samples": len(covered), "interior_samples_per_lap": interior,
            "missing_slots_estimate": missing, "largest_gap_sim_s": max(intervals)}


def audit_bag(path, start_sim_ns, finish_sim_ns, *, laps=1, lap_duration_s=None,
              geometry=(WIDTH, HEIGHT), imu_hz=0, preroll_s=0):
    """ROS の画像ヘッダ時刻を使う。成功時は JSON に保存可能な検査結果を返す。"""
    if start_sim_ns <= 0 or finish_sim_ns <= start_sim_ns:
        raise ValueError("周回の発着時刻が不正です")
    if type(laps) is not int or laps < 1:
        raise ValueError("監査対象の周回数が不正です")
    if laps > 1 and (lap_duration_s is None or not math.isfinite(lap_duration_s) or lap_duration_s <= 0):
        raise ValueError("監査対象の周回時間が不正です")
    bag = Path(path)
    expected, files = inspect_metadata(bag, imu_hz=imu_hz)
    counts = Counter()
    imu_profile = imu_streams(imu_hz)
    imu_stamps = {name: [] for name in imu_profile}
    per_topic = {name: Counter() for name in CAMERAS}
    edges, first_info, signatures, last_stamp = {}, {}, {}, {}
    tf_arrival = {}
    receive_spans = {}
    max_clock_offset = 0
    large_offsets = 0
    for name, msg, receive_ns in read_messages(bag):
        counts[name] += 1
        if name in TYPES and receive_ns <= 0:
            raise ValueError(f"{name} の bag 記録時刻が 0 以下です")
        # この Gazebo world は時刻 0 から開始する。単発の実時計混入も検出する。
        if name in TYPES and finish_sim_ns < UNIX_2000_NS <= receive_ns:
            raise ValueError(f"{name} の bag 記録時刻が UNIX epoch の実時計になっています")
        if name == TF:
            add_transforms(edges, msg)
            for tf in msg.transforms:
                tf_arrival.setdefault(tf.child_frame_id, receive_ns)
        if name in imu_profile:
            stamp = check_imu(name, msg, imu_profile[name][0])
            imu_stamps[name].append(stamp)
        elif name in CAMERAS:
            side = CAMERAS[name]
            if msg.header.frame_id != FRAMES[side]:
                raise ValueError(f"{name} の optical frame が不正です: {msg.header.frame_id}")
            if name == TOPICS[side]:
                stamp = check_image(msg, geometry)
            else:
                tm = msg.header.stamp
                if tm.sec < 0 or not 0 <= tm.nanosec < NANO:
                    raise ValueError(f"{name} の時刻が不正です")
                stamp = tm.sec * NANO + tm.nanosec
                signature = (msg.width, msg.height, tuple(msg.k), tuple(msg.p))
                if side in signatures and signatures[side] != signature:
                    raise ValueError(f"{name} の校正値が記録中に変化しました")
                signatures[side] = signature
                first_info.setdefault(side, msg)
            per_topic[name][stamp] += 1
        else:
            continue
        if finish_sim_ns < UNIX_2000_NS <= stamp:
            raise ValueError(f"{name} のヘッダ時刻が UNIX epoch の実時計になっています")
        if stamp < last_stamp.get(name, stamp):
            raise ValueError(f"{name} のヘッダ時刻が再生順で逆行しています")
        if stamp == last_stamp.get(name) and name not in INFO_TOPICS.values():
            raise ValueError(f"{name} に同一時刻のサンプルが重複しています: {stamp}")
        last_stamp[name] = stamp
        # --use-sim-time による受信時刻と画像生成時刻は clock の更新単位・callback 遅延ぶんずれる。
        first, last = receive_spans.get(name, (receive_ns, receive_ns))
        receive_spans[name] = min(first, receive_ns), max(last, receive_ns)
        offset = abs(receive_ns - stamp)
        if offset > NANO:
            large_offsets += 1
        max_clock_offset = max(max_clock_offset, offset)
    if dict(counts) != {key: n for key, n in expected.items() if n}:
        raise ValueError(f"metadata.yaml の件数と読み戻した件数が一致しません: {dict(counts)}")
    parents = check_extrinsics(edges, imu_hz=imu_hz)
    required_frames = ["camera_link", *FRAMES.values()]
    required_frames.extend(frame for frame, _ in imu_profile.values())
    tf_ready_ns = max(tf_arrival[f] for f in required_frames)
    common = sorted(set.intersection(*(set(c) for c in per_topic.values())))
    before = bisect_right(common, start_sim_ns) - 1
    after = bisect_left(common, finish_sim_ns)
    if before < 0 or after == len(common):
        raise ValueError("左右画像と両 CameraInfo の同時刻の組が周回の両端を含んでいません")
    prestart_sensor_span_s = None
    if imu_profile:
        # 走行開始のゲートまでリグは静止。実際に bag に残った共通の記録区間を調べる。
        first = max([common[0], *(series[0] for series in imu_stamps.values())])
        prestart_sensor_span_s = (start_sim_ns - first) / NANO
        if preroll_s and prestart_sensor_span_s < preroll_s:
            raise ValueError(f"ステレオと IMU の走行前記録が {preroll_s} 秒に届きません "
                             f"({prestart_sensor_span_s:.3f} シミュレーション秒)")
        if preroll_s:
            for name, series in {"ステレオ": common, **imu_stamps}.items():
                window = series[bisect_right(series, start_sim_ns - preroll_s * NANO) - 1:
                                bisect_left(series, start_sim_ns) + 1]
                if any(b - a > GAP_WARNING_NS for a, b in zip(window, window[1:])):
                    raise ValueError(f"{name} の走行前記録に {GAP_WARNING_NS / NANO} 秒を超える空白があります")
    lap = common[before:after + 1]
    lap_sample_counts = []
    lap_start = start_sim_ns
    boundaries = [lap_start]
    for lap_index in range(1, laps + 1):
        lap_finish = (finish_sim_ns if lap_index == laps else
                      start_sim_ns + round(lap_index * lap_duration_s * NANO))
        count = bisect_left(common, lap_finish) - bisect_right(common, lap_start)
        if count <= 0:
            raise ValueError(f"{lap_index}周目の移動区間内に左右画像と両 CameraInfo の同時刻の組がありません")
        lap_sample_counts.append(count)
        boundaries.append(lap_finish)
        lap_start = lap_finish
    for name, (first, last) in receive_spans.items():
        if first > finish_sim_ns or last < start_sim_ns:
            raise ValueError(f"{name} の bag 記録時刻の区間がシミュレーションの周回時刻と重なりません")
    intervals = stamp_intervals(lap)
    gaps = [{"from_sim_ns": left, "to_sim_ns": right, "duration_s": (right - left) / NANO}
            for left, right in zip(lap, lap[1:]) if right - left > GAP_WARNING_NS]
    calibration = Calibration(geometry)
    calibration.add("pair", lap[0])
    for side in TOPICS:
        calibration.add(side, lap[0], first_info[side])

    # 離散時刻を画像・CameraInfo のどれかで観測した回数と、4本とも無い推定欠落枠を区別する。
    observed = {t for counts_at_stamp in per_topic.values() for t in counts_at_stamp if lap[0] <= t <= lap[-1]}
    streams = {}
    for name, counts_at_stamp in per_topic.items():
        stamps = sorted(observed.intersection(counts_at_stamp))
        streams[name] = {"unique_stamps": len(stamps),
                         "duplicates": sum(counts_at_stamp[t] - 1 for t in stamps),
                         "missing_at_observed_stamps": len(observed) - len(stamps),
                         "missing_slots_estimate": missing_slots(stamp_intervals(stamps))}
    return {"storage_id": "mcap", "file_bytes": files, "message_counts": dict(counts),
            "geometry": list(geometry), "imu_streams": {
                name: imu_coverage(name, imu_stamps[name], boundaries, frame, hz)
                for name, (frame, hz) in imu_profile.items()},
            "prestart_sensor_span_s": prestart_sensor_span_s,
            "laps_requested": laps, "interior_stereo_samples_per_lap": lap_sample_counts,
            "lap_start_sim_ns": start_sim_ns, "lap_finish_sim_ns": finish_sim_ns,
            "first_covering_sim_ns": lap[0], "last_covering_sim_ns": lap[-1],
            "complete_stereo_samples": len(lap), "nominal_sim_hz": TARGET_HZ,
            "largest_receive_header_offset_s": max_clock_offset / NANO,
            "receive_header_offset_over_1s": large_offsets,
            "coverage": "gaps_detected" if gaps else "continuous",
            "gaps": gaps, "largest_gap_sim_s": max(intervals), "gap_threshold_sim_s": GAP_WARNING_NS / NANO,
            "missing_complete_slots_estimate": missing_slots(intervals), "streams": streams,
            "tf_ready_sim_ns": tf_ready_ns, "tf_ready_before_lap": tf_ready_ns <= start_sim_ns,
            "stereo_calibration": calibration.result, "static_tf_parents": parents}

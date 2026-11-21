"""Record Gazebo laps and publish the audited bag for offline stereo mapping."""
from functools import partial
from importlib import import_module
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from capture import (Calibration, FRAMES, HEIGHT, INFO_TOPICS, NANO, StereoPairs, TARGET_HZ,
                     TOPICS, WIDTH, check_image)


RECORDER = "atd_gazebo_lap_recorder"
TOPIC_NAMES = (*TOPICS.values(), *INFO_TOPICS.values(), "/tf_static")
# Use the same latched TF policy that the offline mapper uses to replay this bag.
TF_QOS = Path(__file__).resolve().parents[2] / "ros2_ws/src/atd_mapping/config/replay_qos.yaml"


def check_recorder():
    hint = "Gazebo の rosbag2 / MCAP の準備が必要です。Notebook ホストで atd gazebo prepare を実行してください"
    try:
        import_module("ros2bag.verb.record")
        from rosbag2_py import get_registered_readers, get_registered_writers
    except ImportError as exc:
        raise RuntimeError(hint) from exc
    if "mcap" not in get_registered_writers() or "mcap" not in get_registered_readers():
        raise RuntimeError(hint)


def check_destination(final):
    if final.parent.name != "rosbags" or not final.parent.is_dir():
        raise ValueError(f"rosbags 直下の新規 bag 保存先を指定してください: {final}")
    if os.path.lexists(final):
        raise FileExistsError(f"bag 保存先は既に存在します: {final}")


def staging_path(final):
    check_destination(final)
    # The Web library scans all descendants of rosbags. Unfinished bags and
    # even their metadata must stay outside it until the full audit succeeds.
    workdir = Path(tempfile.mkdtemp(prefix=".gazebo-", dir=final.parent.parent))
    return workdir / final.name


def command(pending):
    return ["ros2", "bag", "record", "--storage", "mcap", "--output", str(pending),
            "--node-name", RECORDER, "--use-sim-time", "--disable-keyboard-controls",
            "--qos-profile-overrides-path", str(TF_QOS), "--topics", *TOPIC_NAMES]


def wait_for_sample(children, wait_for, *, min_stamp=1, required_edges=()):
    """Observe a calibrated stereo pair only at the two stationary endpoints."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage

    pairs = StereoPairs()
    calibration = Calibration()
    missing_edges = set(required_edges)

    def on_image(side, image):
        stamp = check_image(image)
        if image.header.frame_id != FRAMES[side]:
            raise ValueError(f"{side} image optical frame mismatch: {image.header.frame_id}")
        if stamp >= min_stamp and pairs.feed(side, stamp, time.monotonic()) is not None:
            calibration.add("pair", stamp)

    def on_info(side, info):
        stamp = info.header.stamp.sec * NANO + info.header.stamp.nanosec
        if stamp >= min_stamp:
            calibration.add(side, stamp, info)

    def on_tf(message):
        missing_edges.difference_update((tf.header.frame_id, tf.child_frame_id) for tf in message.transforms)

    with rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO), rclpy.create_node(
        "atd_gazebo_record_check"
    ) as node:
        def subscribed():
            return all(any(sub.node_name == RECORDER for sub in node.get_subscriptions_info_by_topic(topic))
                       for topic in TOPIC_NAMES)

        wait_for(subscribed, children, 60, "recorder の画像・CameraInfo・固定TF購読")
        for side, topic in TOPICS.items():
            node.create_subscription(Image, topic, partial(on_image, side), qos_profile_sensor_data)
            node.create_subscription(CameraInfo, INFO_TOPICS[side], partial(on_info, side), qos_profile_sensor_data)
        if missing_edges:
            node.create_subscription(TFMessage, "/tf_static", on_tf, QoSProfile(
                depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def received():
            rclpy.spin_once(node, timeout_sec=.05)
            return calibration.result is not None and not missing_edges

        wait_for(received, children, 60, "同時刻の左右画像・CameraInfo と必要な固定TF", poll_interval=0)


def close_recorder(proc):
    if proc.poll() is not None:
        raise RuntimeError(f"recorder が保存要求前に終了しました（{proc.returncode}）。recorder.log を確認してください")
    try:
        # Lyrical's record CLI handles SIGTERM by stopping the writer and closing metadata.
        os.killpg(proc.pid, signal.SIGTERM)
        code = proc.wait(timeout=90)
    except subprocess.TimeoutExpired as exc:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
        raise TimeoutError("MCAP の保存完了を確認できません。recorder.log を確認してください") from exc
    if code != 0:
        raise RuntimeError(f"MCAP の保存に失敗しました（recorder 終了値 {code}）。recorder.log を確認してください")


def audit_and_publish(pending, final, timing, report, trace):
    from bag_audit import audit_bag
    # Gazebo 専用イメージから、replay と同じ保存形式を使う。
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2_ws/src/atd_bringup"))
    try:
        from atd_bringup.replay_gt_source import store_ground_truth
    finally:
        sys.path.pop(0)

    laps = report["laps_requested"]
    print(f"{laps}周分のステレオ入力と固定TFを保存済み MCAP で検査しています。", flush=True)
    audit = audit_bag(pending, timing["start_sim_ns"], timing["finish_sim_ns"],
                      laps=laps, lap_duration_s=report["lap_duration_s"])
    report["bag_audit"] = audit
    store_ground_truth(pending, trace, audit["file_bytes"])
    check_destination(final)
    pending.rename(final)
    del report["partial_bag"]
    report["bag_path"] = str(final)
    try:
        pending.parent.rmdir()
    except OSError as exc:
        print(f"bag は保存済みです。作業ディレクトリが残っています: {exc}", file=sys.stderr)
    print(f"{laps}周のステレオ {audit['complete_stereo_samples']} 組、最大間隔 "
          f"{audit['largest_gap_sim_s'] * 1000:.1f} ms、MCAP {sum(audit['file_bytes'].values()) / 1e9:.2f} GB",
          flush=True)
    if audit["coverage"] == "gaps_detected":
        print("画像欠落あり。周回中にステレオ入力が途切れた区間があります。"
              "この bag はオフライン地図作成に使用できます。", flush=True)

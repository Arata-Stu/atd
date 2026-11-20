"""Notebook 上の独立した Gazebo サーバー、ROS ブリッジと計測を管理する。"""
import argparse
from contextlib import ExitStack
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import world

HERE = Path(__file__).resolve().parent


def bridge_topics():
    topics = [{"ros_topic_name": "/clock", "gz_topic_name": "/world/monza/clock",
               "ros_type_name": "rosgraph_msgs/msg/Clock", "gz_type_name": "gz.msgs.Clock",
               "direction": "GZ_TO_ROS", "qos_profile": "CLOCK"}]
    for index, side in enumerate(("left", "right"), start=1):
        for gz_suffix, ros_suffix, msg in (("image", "image_rect_raw", "Image"),
                                           ("camera_info", "camera_info", "CameraInfo")):
            topics.append({"gz_topic_name": f"/monza/{side}/{gz_suffix}",
                           "ros_topic_name": f"/camera/camera/infra{index}/{ros_suffix}",
                           "gz_type_name": f"gz.msgs.{msg}", "ros_type_name": f"sensor_msgs/msg/{msg}",
                           "direction": "GZ_TO_ROS", "qos_profile": "SENSOR_DATA", "lazy": False})
    return topics


def stop_group(process):
    # ros2 run / gz sim が起動した子プロセスも一緒に終了する。
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=4)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def wait_for(check, processes, timeout, what):
    deadline = time.monotonic() + timeout
    while True:
        for name, proc in processes.items():
            if proc.poll() is not None:
                raise RuntimeError(f"{name} が終了しました（{proc.returncode}）。{name}.log を参照してください")
        if check():
            return
        if time.monotonic() >= deadline:
            raise TimeoutError(f"{what} を {timeout} 秒以内に確認できませんでした")
        time.sleep(.10)


def unpause():
    cmd = ["gz", "service", "-s", "/world/monza/control", "--reqtype", "gz.msgs.WorldControl",
           "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req", "pause: false"]
    try:
        reply = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
    except subprocess.TimeoutExpired:
        return False
    return reply.returncode == 0 and "data: true" in reply.stdout


def transforms(camera):
    # シミュレーション用後軸中心の仮定値。床基準のレンズ高から差し引く。
    axle_height = .0325
    yield ("base_link", "camera_link", camera["midpoint_forward_from_rear_axle_m"], 0,
           camera["height_above_floor_m"] - axle_height, 0, math.radians(camera["pitch_down_deg"]), 0)
    for side, sign in (("left", 1), ("right", -1)):
        yield ("camera_link", camera["optical_frames"][side], 0, sign * camera["baseline_m"] / 2,
               0, -math.pi / 2, 0, -math.pi / 2)


def inspect_motion(scene_dir, scene, metrics):
    trace = scene_dir / "rig_motion.jsonl"
    if not trace.is_file():
        raise RuntimeError("カメラ台の移動記録がありません。gazebo.log の monza_motion を確認してください")
    samples = [json.loads(row) for row in trace.read_text().splitlines() if row]
    xy = [(p["x_m"], p["y_m"]) for p in samples]
    travelled = sum(math.dist(a, b) for a, b in zip(xy, xy[1:]))
    complete = None
    if metrics["mode"] == "video":
        endpoint = scene["trajectory"]["samples_s_x_y_yaw"][-1][1:3]
        # The trace is sampled every 0.5 s or faster on short video runs.
        sample_spacing = scene["trajectory"]["end_s_m"] * min(.5 / scene["trajectory"]["travel_seconds"], .125)
        complete = bool(xy) and math.dist(xy[-1], endpoint) <= sample_spacing + .05
    if (len(xy) > 1 and travelled < .005) or complete is False:
        raise RuntimeError("カメラ台が予定区間を移動していません。scene/rig_motion.jsonl を確認してください")
    if len(xy) < 2 and metrics.get("pair_span_sim_s", 0) >= .55:
        raise RuntimeError("カメラの移動記録が途中で途切れました")
    return {"pose_samples": len(xy), "sampled_distance_m": travelled, "reached_corner_exit": complete}


def run(args, report):
    diagnose = args.mode == "diagnose"
    mode = "benchmark" if diagnose else args.mode
    out = args.output
    scene_dir = out / "scene"
    scene = world.build(scene_dir, height=args.camera_height, pitch_deg=args.camera_pitch_deg,
                        travel_seconds=args.duration if mode == "video" else 10.0, repeat=mode == "benchmark")
    config = out / "bridge.yaml"
    config.write_text(json.dumps(bridge_topics(), indent=2) + "\n")
    report.update(ros_domain_id=os.environ.get("ROS_DOMAIN_ID", "0"),
                  dds_profile=os.environ.get("FASTDDS_DEFAULT_PROFILES_FILE"), scene="scene/scene.json",
                  simulated_rear_axle_height_m=.0325)
    if shutil.which("nvidia-smi"):
        info = subprocess.run(["nvidia-smi", "--query-gpu=name,driver_version,memory.total",
                               "--format=csv,noheader"], capture_output=True, text=True, timeout=10)
        report["visible_gpus"] = info.stdout.strip().splitlines()
    with ExitStack() as stack:
        children = {}

        def start(name, command):
            log = stack.enter_context((out / f"{name}.log").open("w"))
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            stack.callback(stop_group, proc)
            children[name] = proc
            return proc

        start("gazebo", ["gz", "sim", "-s", "-v", "4", "--headless-rendering", str(scene_dir / "world.sdf")])
        start("bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                         "-p", f"config_file:={config}"])
        for parent, child, x, y, z, roll, pitch, yaw in transforms(scene["camera"]):
            start("tf_" + child, ["ros2", "run", "tf2_ros", "static_transform_publisher",
                                  "--frame-id", parent, "--child-frame-id", child,
                                  "--x", str(x), "--y", str(y), "--z", str(z),
                                  "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw)])
        probe = None
        if diagnose:
            probe = start("gz_probe", [sys.executable, str(HERE / "gz_probe.py"), "--output", str(out)])
            wait_for((out / "gz_probe.ready").is_file, children, 30, "Gazebo 側の診断購読の準備")
        receiver = start("capture", [sys.executable, str(HERE / "capture.py"), "--output", str(out),
                                     "--mode", mode, "--duration", str(args.duration)])
        wait_for((out / f"{mode}.ready").is_file, children, 70, "左右カメラの ROS publisher と subscriber の接続")
        wait_for(unpause, children, 45, "Gazebo の再生開始")
        servers = {k: v for k, v in children.items() if v is not receiver and v is not probe}
        wait_for(lambda: receiver.poll() is not None, servers,
                 max(150, args.duration * (11 if mode == "video" else 1) + 120), "ステレオ計測の完了")
        exit_code = receiver.returncode
        if probe is not None:
            if not (out / "benchmark.json").is_file():
                raise RuntimeError("ROS の計測区間を取得できません。capture.log を確認してください")
            wait_for(lambda: probe.poll() is not None, servers, 20, "Gazebo 側の診断集計")
            if probe.returncode != 0:
                raise RuntimeError("Gazebo 側の集計に失敗しました。gz_probe.log と benchmark.json を確認してください")

    metrics_path = out / f"{mode}.json"
    if not metrics_path.is_file():
        raise RuntimeError("受信結果を取得できません。capture.log を確認してください")
    metrics = json.loads(metrics_path.read_text())
    report["capture_status"] = metrics["status"]
    if diagnose:
        gz = json.loads((out / "gz_probe.json").read_text())
        report["gz_probe"] = "gz_probe.json"
        window = "ROS と同じ計測区間" if gz["window"] == "ros_measurement" else "ROS 計測開始前を含む全区間"
        print(f"Gazebo（{window} {gz['measurement_wall_s']:.1f} 秒）  左 {gz['received_images']['left']} 枚 / "
              f"右 {gz['received_images']['right']} 枚  同時刻 {gz['matching_stamps']} 組 "
              f"({gz['paired_wall_hz']:.2f} 組/秒)", flush=True)
        rtf = gz["world"]["real_time_factor"]
        if rtf is not None:
            print(f"Gazebo 時間の進み {rtf:.3f} シミュレーション秒/実秒", flush=True)
    if "received_images" in metrics:
        eyes = metrics["received_images"]
        print(f"ROS 受信  左 {eyes['left']} 枚 / 右 {eyes['right']} 枚  "
              f"同時刻・順序通り {metrics['unique_ordered_pairs']} 組", flush=True)
        if diagnose:
            pairing = metrics["pairing"]
            print(f"ROS 左右の共通時刻 {pairing['shared_image_stamps']} 組、"
                  f"照合バッファから外れた後に相手が届いたもの {pairing['shared_stamps_evicted_before_match']} 組")
            counters = metrics.get("udp_namespace_counter_deltas")
            if counters is not None:
                print(f"Notebook 全体の UDP 受信バッファ不足エラー増分 {counters['RcvbufErrors']}")
    if metrics["status"] == "error" or exit_code not in (0, 2):
        raise RuntimeError(metrics.get("error", f"capture exit {exit_code}"))
    report["motion"] = inspect_motion(scene_dir, scene, metrics)
    if mode == "video":
        # Join the original matched eyes into one seekable movie after measurement.
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin", "-n",
                        "-i", str(out / "left.mp4"), "-i", str(out / "right.mp4"),
                        "-filter_complex", "hstack=inputs=2", "-c:v", "libx264", "-crf", "19",
                        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out / "stereo.mp4")],
                       check=True, timeout=max(60, args.duration * 4))
        print(f"動画 {metrics['video_frames']} フレーム中、前の画像を再表示 {metrics['held_video_frames']} フレーム")
    print(f"ROS 実時間 {metrics['paired_wall_hz']:.2f} 組/秒", flush=True)
    if mode == "benchmark":
        print("90 Hz 判定 " + ("達成" if metrics["pass_90hz"] else "未達"), flush=True)
    return exit_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("benchmark", "video", "diagnose"), required=True)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--camera-height", type=float, default=.12)
    parser.add_argument("--camera-pitch-deg", type=float, default=0)
    args = parser.parse_args()
    if args.duration is None:
        args.duration = {"video": 10.0, "benchmark": 60.0, "diagnose": 20.0}[args.mode]
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be finite and positive")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        parser.error("既存の結果を保護するため、新しい空の出力先を指定してください")
    report = {"mode": args.mode, "requested_duration_s": args.duration}
    try:
        result = run(args, report)
        report["status"] = "complete" if result == 0 else "below_target"
    except (OSError, ValueError, RuntimeError, TimeoutError, KeyboardInterrupt, subprocess.SubprocessError) as exc:
        report.update(status="error", error=str(exc) or "interrupted")
        result = 1
        print(report["error"], file=sys.stderr)
    finally:
        ogre_log = Path.home() / ".gz/rendering/ogre2.log"
        if ogre_log.is_file():
            shutil.copyfile(ogre_log, args.output / "ogre2.log")
            report["renderer_log_lines"] = [line.strip() for line in ogre_log.read_text(errors="replace").splitlines()
                                            if "GL_RENDERER" in line or "EGL Subsystem" in line]
        (args.output / "run.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    return result


if __name__ == "__main__":
    sys.exit(main())

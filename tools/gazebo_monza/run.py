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
import lap_bag
from monza_motion import interpolate, travel_fraction
from sensor_profile import (ACCEL_FRAME, ACCEL_TOPIC, CAMERA_RATES, DEFAULT_CAMERA_HZ,
                            GYRO_FRAME, GYRO_TOPIC, IMU_MODES, IMU_TOPIC, RESOLUTIONS)

HERE = Path(__file__).resolve().parent


def bridge_topics(circuit="odaiba", imu_hz=0):
    topics = [{"ros_topic_name": "/clock", "gz_topic_name": f"/world/{circuit}/clock",
               "ros_type_name": "rosgraph_msgs/msg/Clock", "gz_type_name": "gz.msgs.Clock",
               "direction": "GZ_TO_ROS", "qos_profile": "CLOCK"}]
    for index, side in enumerate(("left", "right"), start=1):
        for gz_suffix, ros_suffix, msg in (("image", "image_rect_raw", "Image"),
                                           ("camera_info", "camera_info", "CameraInfo")):
            topics.append({"gz_topic_name": f"/{circuit}/{side}/{gz_suffix}",
                           "ros_topic_name": f"/camera/camera/infra{index}/{ros_suffix}",
                           "gz_type_name": f"gz.msgs.{msg}", "ros_type_name": f"sensor_msgs/msg/{msg}",
                           "direction": "GZ_TO_ROS", "qos_profile": "SENSOR_DATA", "lazy": False})
    if imu_hz:
        for name, topic in (("imu", IMU_TOPIC), ("accel", ACCEL_TOPIC), ("gyro", GYRO_TOPIC)):
            topics.append({"gz_topic_name": f"/{circuit}/{name}", "ros_topic_name": topic,
                           "gz_type_name": "gz.msgs.IMU", "ros_type_name": "sensor_msgs/msg/Imu",
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


def check_children(processes):
    for name, proc in processes.items():
        if proc.poll() is not None:
            raise RuntimeError(f"{name} が終了しました（{proc.returncode}）。{name}.log を参照してください")


def wait_for(check, processes, timeout, what, check_external=None, *, poll_interval=.10):
    deadline = None if timeout is None else time.monotonic() + timeout
    while True:
        check_children(processes)
        if check_external is not None:
            check_external()
        if check():
            return
        if deadline is not None and time.monotonic() >= deadline:
            raise TimeoutError(f"{what} を {timeout} 秒以内に確認できませんでした")
        if poll_interval:
            time.sleep(poll_interval)


def unpause(circuit):
    cmd = ["gz", "service", "-s", f"/world/{circuit}/control", "--reqtype", "gz.msgs.WorldControl",
           "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req", "pause: false"]
    try:
        reply = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
    except subprocess.TimeoutExpired:
        return False
    return reply.returncode == 0 and "data: true" in reply.stdout


def start_stream(children, wait_for_stereo, circuit="odaiba", check_external=None, *, imu_hz=0):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSCompatibility, qos_check_compatible
    from rclpy.signals import SignalHandlerOptions

    topics = bridge_topics(circuit, imu_hz)
    with rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO), rclpy.create_node(
        "atd_gazebo_startup", parameter_overrides=[Parameter("use_sim_time", value=True)]
    ) as node:
        def bridge_ready():
            if node.count_publishers("/clock") > 1:
                raise RuntimeError("同じ ROS ドメインに /clock の発行元が複数あります")
            return all(node.count_publishers(t["ros_topic_name"]) for t in topics)

        wait_for(bridge_ready, children, 60, "ROS の /clock と選択したセンサーの publisher", check_external)
        if wait_for_stereo:
            last_missing = None

            def subscribers_ready():
                nonlocal last_missing
                missing = []
                for topic in topics:
                    if topic["ros_type_name"] not in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo"):
                        continue
                    name = topic["ros_topic_name"]
                    publishers = node.get_publishers_info_by_topic(name)
                    subscribers = node.get_subscriptions_info_by_topic(name)
                    if not any(
                        sub.topic_type == pub.topic_type == topic["ros_type_name"]
                        and qos_check_compatible(pub.qos_profile, sub.qos_profile)[0] != QoSCompatibility.ERROR
                        for pub in publishers for sub in subscribers
                    ):
                        missing.append(name)
                if missing and missing != last_missing:
                    print("一時停止中。対応する QoS の購読待ち: " + ", ".join(missing), flush=True)
                last_missing = missing
                return not missing

            wait_for(subscribers_ready, children, None, "左右のステレオ購読", check_external)
        wait_for(lambda: unpause(circuit), children, 45, "Gazebo の再生開始", check_external)
        first_stamp = None

        def clock_advancing():
            nonlocal first_stamp
            rclpy.spin_once(node, timeout_sec=0.1)
            stamp = node.get_clock().now().nanoseconds
            if first_stamp is None and stamp > 0:
                first_stamp = stamp
            return first_stamp is not None and stamp > first_stamp

        wait_for(clock_advancing, children, 45, "ROS /clock の進行", check_external)


def transforms(camera, imu_hz=0):
    # シミュレーション用後軸中心の仮定値。床基準のレンズ高から差し引く。
    axle_height = .0325
    yield ("base_link", "camera_link", camera["midpoint_forward_from_rear_axle_m"], 0,
           camera["height_above_floor_m"] - axle_height, 0, math.radians(camera["pitch_down_deg"]), 0)
    for side, sign in (("left", 1), ("right", -1)):
        yield ("camera_link", camera["optical_frames"][side], 0, sign * camera["baseline_m"] / 2,
               0, -math.pi / 2, 0, -math.pi / 2)
    if imu_hz:
        for frame in (ACCEL_FRAME, GYRO_FRAME):
            yield ("camera_link", frame, 0, 0, 0, -math.pi / 2, 0, -math.pi / 2)


def inspect_lap(scene_dir, trajectory, samples, metrics):
    """Check the post-physics trace from release through the final requested finish."""
    video = metrics.get("mode", "video") == "video"
    laps = trajectory.get("laps", 1)
    release_file = scene_dir / "lap_start.json"
    start = 0.0
    if release_file.is_file():
        start = json.loads(release_file.read_text())["sim_time_s"]
    elif trajectory.get("wait_for_capture"):
        incomplete = {"lap_completed": False, "lap_start_sim_s": None, "laps_requested": laps, "laps_completed": 0}
        if video:
            incomplete["video_endpoints_covered"] = False
        return incomplete
    lap_seconds = trajectory["travel_seconds"]
    duration = lap_seconds * laps
    end = start + duration
    profile = trajectory["samples_s_x_y_yaw"]
    length = trajectory["end_s_m"]
    distances = [row[0] for row in profile]
    endpoint = profile[-1][1:3]
    times = []
    errors = []
    finish_error = None
    # The transform can be applied one physics step after the pre-update command.
    step_s = trajectory.get("physics_step_s", 1 / DEFAULT_CAMERA_HZ)
    max_allowed_gap = trajectory.get("trace_interval_s", min(.5, lap_seconds / 8)) + 2 * step_s
    verified = True
    completed_laps = 0
    for sample in samples:
        t = sample["sim_time_s"]
        if t < start - 1e-6:
            continue
        xy = (sample["x_m"], sample["y_m"])
        expected = [interpolate(profile, distances, length * travel_fraction(
            t - start - lag, lap_seconds, False, closed=True, laps=laps)) for lag in (0., step_s)]
        errors.append(min(math.dist(xy, p[:2]) for p in expected))
        interval_valid = (0 < t - times[-1] <= max_allowed_gap) if times else t - start <= step_s * 2
        verified = verified and interval_valid and errors[-1] <= .05
        times.append(t)
        if verified:
            # An intermediate crossing may fall between two trace samples.
            completed_laps = min(laps - 1, int((t - start + 1e-6) / lap_seconds))
        if t >= end - 1e-6:
            finish_error = math.dist(xy, endpoint)
            if finish_error <= .05:
                break
    max_gap = max((b - a for a, b in zip(times, times[1:])), default=None)
    max_error = max(errors, default=None)
    complete = verified and len(times) >= 3 and finish_error is not None and finish_error <= .05
    result = {"lap_completed": complete, "laps_completed": laps if complete else completed_laps,
              "laps_requested": laps, "lap_start_sim_s": start, "lap_finish_sim_s": end,
              "lap_trace_samples": len(times), "observed_finish_sim_s": times[-1] if complete else None,
              "maximum_trace_gap_s": max_gap, "maximum_path_error_m": max_error,
              "finish_position_error_m": finish_error}
    if video:
        first = metrics.get("first_encoded_pair_sim_ns")
        last = metrics.get("last_encoded_pair_sim_ns")
        start_ns = round(start * 1e9)
        result["video_endpoints_covered"] = (first is not None and last is not None
                                             and metrics.get("lap_start_sim_ns") == start_ns
                                             and first <= start_ns and last >= start_ns + round(duration * 1e9))
    return result


def inspect_motion(scene_dir, scene, metrics):
    trace = scene_dir / "rig_motion.jsonl"
    if not trace.is_file():
        raise RuntimeError("カメラ台の移動記録がありません。gazebo.log の monza_motion を確認してください")
    samples = [json.loads(row) for row in trace.read_text().splitlines() if row]
    xy = [(p["x_m"], p["y_m"]) for p in samples]
    travelled = sum(math.dist(a, b) for a, b in zip(xy, xy[1:]))
    observation = {"pose_samples": len(xy), "sampled_distance_m": travelled}
    if metrics["mode"] in ("bag", "map", "video", "serve"):
        observation.update(inspect_lap(scene_dir, scene["trajectory"], samples, metrics))
        return observation
    if len(xy) > 1 and travelled < .005:
        raise RuntimeError("カメラ台が予定区間を移動していません。scene/rig_motion.jsonl を確認してください")
    if len(xy) < 2 and metrics.get("pair_span_sim_s", 0) >= .55:
        raise RuntimeError("カメラの移動記録が途中で途切れました")
    return observation


def drive_laps(children, out, scene, report, check_external=None):
    scene_dir = out / "scene"
    (scene_dir / "lap_capture.ready").touch(exist_ok=False)
    print(f"スタート位置から{scene['trajectory'].get('laps', 1)}周走行します。", flush=True)

    trace = scene_dir / "rig_motion.jsonl"
    finished = scene_dir / "lap_finished.ready"
    trace_size = -1
    last_progress = time.monotonic()

    def lap_finished():
        nonlocal trace_size, last_progress
        if (scene_dir / "lap_finished.failed").is_file():
            raise RuntimeError("予定のゴール時刻を過ぎてもカメラ台の到達を確認できません")
        if finished.is_file():
            return True
        size = trace.stat().st_size if trace.is_file() else 0
        if size != trace_size:
            trace_size, last_progress = size, time.monotonic()
        if time.monotonic() - last_progress >= 90:
            raise TimeoutError("90実時間秒の間、カメラ台の姿勢記録が進んでいません")
        return False

    wait_for(lap_finished, children, None, "ゴールまでの走行", check_external)
    motion = inspect_motion(scene_dir, scene, {"mode": report.get("mode", "map")})
    report["motion"] = motion
    if not motion["lap_completed"]:
        raise RuntimeError("カメラ台の実際の姿勢から全周の走行とゴール到達を確認できません")
    return {"start_sim_ns": round(1e9 * motion["lap_start_sim_s"]),
            "finish_sim_ns": round(1e9 * motion["observed_finish_sim_s"])}


def map_laps(children, out, circuit, scene, report):
    def check_mapper():
        if (out / "mapping.cancel").exists():
            raise RuntimeError("地図作成を中断しました")
        failed = out / "mapping.failed.json"
        if failed.is_file():
            failure = json.loads(failed.read_text())
            raise RuntimeError(f"cuVSLAM の地図作成に失敗しました: {failure.get('error', failure)}")

    # Let the clock and stereo run at the initial pose until the mapper has
    # actually tracked it. Waiting for odometry with the world paused deadlocks.
    start_stream(children, False, circuit, check_mapper, imu_hz=report["imu_hz"])
    (out / "gazebo.ready").touch(exist_ok=False)
    print("スタート位置で cuVSLAM の初期追跡を待っています。", flush=True)
    wait_for((out / "mapping.ready").is_file, children, 240, "cuVSLAM の初期追跡", check_mapper)
    timing = drive_laps(children, out, scene, report, check_mapper)
    marker = out / "lap.complete.json"
    pending = marker.with_suffix(".tmp")
    pending.write_text(json.dumps(timing) + "\n")
    pending.replace(marker)
    print("ゴールを確認しました。cuVSLAM が終端まで処理し地図を保存するのを待っています。", flush=True)
    saved = out / "mapping.saved.json"
    wait_for(saved.is_file, children, 300, "cuVSLAM 地図の保存完了", check_mapper)
    report["mapping"] = json.loads(saved.read_text())
    return 0


def record_laps(children, out, circuit, scene, report, final):
    # Advance the clock with the rig parked. The Lyrical recorder waits for
    # /clock before it starts discovering the selected topics.
    imu_hz = report["imu_hz"]
    geometry = tuple(report["geometry"])
    start_stream(children, False, circuit, imu_hz=imu_hz)
    pending = lap_bag.staging_path(final)
    report["partial_bag"] = str(pending)
    print(f"MCAP を記録します。途中の保存先 {pending}", flush=True)
    with (out / "recorder.log").open("w") as log:
        proc = subprocess.Popen(lap_bag.command(pending, imu_hz=imu_hz), stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        children["recorder"] = proc
        try:
            edges = {(p, c) for p, c, *_ in transforms(scene["camera"], imu_hz)}
            lap_bag.wait_for_sample(children, wait_for, required_edges=edges, geometry=geometry, imu_hz=imu_hz,
                                    preroll_s=lap_bag.IMU_PREROLL_S if imu_hz else 0)
            timing = drive_laps(children, out, scene, report)
            # Keep recording on the stationary finishing pose through the last exposure.
            lap_bag.wait_for_sample(children, wait_for, min_stamp=timing["finish_sim_ns"] + 250_000_000,
                                    geometry=geometry, imu_hz=imu_hz)
        finally:
            del children["recorder"]
            lap_bag.close_recorder(proc)
    return pending, timing


def run(args, report):
    diagnose = args.mode == "diagnose"
    mode = "benchmark" if diagnose else args.mode
    out = args.output
    scene_dir = out / "scene"
    closed_loop = mode == "closed-loop"
    lap_video = mode == "video"
    finite_serve = mode == "serve" and args.laps is not None
    finite_run = mode in ("video", "map", "bag") or finite_serve
    width, height = RESOLUTIONS[args.resolution]
    imu_hz = IMU_MODES[args.imu]
    report.update(geometry=[width, height], camera_hz=args.camera_hz, imu_hz=imu_hz,
                  accel_hz=200 if imu_hz else 0, gyro_hz=imu_hz)
    if mode == "bag":
        lap_bag.check_recorder()
        lap_bag.check_destination(args.bag_output)
    scene = world.build(scene_dir, height=args.camera_height, pitch_deg=args.camera_pitch_deg,
                        travel_seconds=args.duration if finite_run else None, speed_mps=args.speed,
                        laps=args.laps or 1, repeat=not (finite_run or closed_loop), circuit=args.circuit, route=args.route,
                        wait_for_capture=(lap_video or finite_serve or mode in ("map", "bag")),
                        record_ground_truth=mode == "bag", resolution=args.resolution,
                        camera_hz=args.camera_hz, imu_hz=imu_hz, command_driven=closed_loop)
    trajectory = scene["trajectory"]
    total_seconds = trajectory["total_travel_seconds"]
    duration = total_seconds if finite_run else args.duration
    report.update(requested_duration_s=duration, speed_mps=trajectory["speed_mps"],
                  laps_requested=trajectory["laps"],
                  lap_duration_s=trajectory["travel_seconds"],
                  total_travel_seconds=total_seconds)
    if closed_loop:
        print(f"{args.circuit} / {args.route}  操舵・スロットル指令で {trajectory['laps']}周を評価", flush=True)
    elif finite_run:
        schedule = f"{trajectory['laps']}周  走行時間 {duration:.2f} シミュレーション秒"
    else:
        schedule = "繰り返し走行"
    if not closed_loop:
        print(f"{args.circuit} / {args.route}  RC速度 {trajectory['speed_mps']:.3g} m/s  {schedule}", flush=True)
    inertial = f"加速度 200 Hz・ジャイロ／統合 {imu_hz} Hz" if imu_hz else "OFF"
    print(f"ステレオ {width}×{height} @ {args.camera_hz} Hz  IMU {inertial}", flush=True)
    if mode == "bag":
        report["lap_length_m"] = trajectory["end_s_m"]
        image_bytes = 2 * width * height * args.camera_hz * duration
        print(f"左右画像の容量目安 合計 {image_bytes / 1e9:.2f} GB "
              f"（1周 {image_bytes / trajectory['laps'] / 1e9:.2f} GB × {trajectory['laps']}周、"
              f"mono8 {width}×{height} × 左右2枚 × {args.camera_hz} Hz）", flush=True)
    config = out / "bridge.yaml"
    config.write_text(json.dumps(bridge_topics(args.circuit, imu_hz), indent=2) + "\n")
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

        clock_param = "use_sim_time:=true" if mode in ("bag", "map", "serve", "closed-loop") else "use_sim_time:=false"
        start("gazebo", ["gz", "sim", "-s", "-v", "4", "--headless-rendering", str(scene_dir / "world.sdf")])
        start("bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                         "-p", f"config_file:={config}", "-p", clock_param,
                         "-p", "override_timestamps_with_wall_time:=false"])
        for parent, child, x, y, z, roll, pitch, yaw in transforms(scene["camera"], imu_hz):
            start("tf_" + child, ["ros2", "run", "tf2_ros", "static_transform_publisher",
                                  "--frame-id", parent, "--child-frame-id", child,
                                  "--x", str(x), "--y", str(y), "--z", str(z),
                                  "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
                                  "--ros-args", "-p", clock_param])
        if closed_loop:
            from closed_loop_trial import run_trial
            return run_trial(children, out, scene, report, start_stream, wait_for, check_children)
        if mode == "map":
            return map_laps(children, out, args.circuit, scene, report)
        if mode == "bag":
            pending, timing = record_laps(children, out, args.circuit, scene, report, args.bag_output)
            stack.close()  # Release the renderer and bridge before reading the full MCAP back.
            lap_bag.audit_and_publish(pending, args.bag_output, timing, report, scene_dir / "rig_motion.jsonl")
            return 0
        if mode == "serve":
            start_stream(children, args.wait_for_stereo, args.circuit, imu_hz=imu_hz)
            if finite_serve:
                drive_laps(children, out, scene, report)
                print(f"{args.laps}周のゴールを確認しました。配信を終了します。", flush=True)
                return 0
            print("配信中。/clock は Gazebo 時刻で進みます。Ctrl+C で終了します。", flush=True)
            while True:
                check_children(children)
                time.sleep(0.5)
        probe = None
        if diagnose:
            probe = start("gz_probe", [sys.executable, str(HERE / "gz_probe.py"), "--output", str(out), "--circuit", args.circuit])
            wait_for((out / "gz_probe.ready").is_file, children, 30, "Gazebo 側の診断購読の準備")
        receiver = start("capture", [sys.executable, str(HERE / "capture.py"), "--output", str(out),
                                     "--mode", mode, "--route", args.route, "--duration", str(duration),
                                     "--resolution", args.resolution, "--camera-hz", str(args.camera_hz)])
        wait_for((out / f"{mode}.ready").is_file, children, 70, "左右カメラの ROS publisher と subscriber の接続")
        wait_for(lambda: unpause(args.circuit), children, 45, "Gazebo の再生開始")
        servers = {k: v for k, v in children.items() if v is not receiver and v is not probe}
        timeout = None if lap_video else max(150, duration * (11 if mode == "video" else 1) + 120)
        wait_for(lambda: receiver.poll() is not None, servers, timeout, "ステレオ計測の完了")
        exit_code = receiver.returncode
        if lap_video and exit_code == 0:
            try:
                wait_for((scene_dir / "lap_finished.ready").is_file, servers, 60,
                         "ゴール位置に到達したカメラ台の姿勢記録")
            except TimeoutError:
                # Stop the server before reading a potentially incomplete trace.
                report["finish_trace_timeout"] = True
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
    if lap_video and (scene_dir / "rig_motion.jsonl").is_file():
        report["motion"] = inspect_motion(scene_dir, scene, metrics)
    if metrics["status"] == "error" or exit_code not in (0, 2):
        raise RuntimeError(metrics.get("error", f"capture exit {exit_code}"))
    if "motion" not in report:
        report["motion"] = inspect_motion(scene_dir, scene, metrics)
    if lap_video:
        if report.get("finish_trace_timeout") or not report["motion"]["lap_completed"]:
            raise RuntimeError("実際のカメラ台の姿勢から全周の走行とゴール到達を確認できません")
        if not report["motion"]["video_endpoints_covered"]:
            raise RuntimeError("左右動画でスタート・ゴール両地点の撮影時刻を確認できません")
        print(f"カメラ台の{trajectory['laps']}周を確認  走行距離 {trajectory['end_s_m'] * trajectory['laps']:.2f} m", flush=True)
        print(f"左右動画でスタートとゴールを撮影  受信ペア間の最大間隔 "
              f"{metrics['largest_sim_gap_ms']:.1f} ms（シミュレーション時刻）", flush=True)
    if mode == "video":
        # Join the original matched eyes into one seekable movie after measurement.
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin", "-n",
                        "-i", str(out / "left.mp4"), "-i", str(out / "right.mp4"),
                        "-filter_complex", "hstack=inputs=2", "-c:v", "libx264", "-crf", "19",
                        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out / "stereo.mp4")],
                       check=True, timeout=max(60, duration * 4))
        print(f"動画 {metrics['video_frames']} フレーム中、前の画像を再表示 {metrics['held_video_frames']} フレーム")
    print(f"ROS 実時間 {metrics['paired_wall_hz']:.2f} 組/秒", flush=True)
    if mode == "benchmark":
        if metrics["paired_sim_hz"] is not None:
            print(f"画像スタンプ基準 {metrics['paired_sim_hz']:.2f} 組/シミュレーション秒、"
                  f"観測ペア間の欠け推定 {metrics['missing_sim_slots_between_pairs_estimate']} 組", flush=True)
        verdict = "達成" if metrics["pass_target_hz"] else "未達"
        print(f"{args.camera_hz} Hz 判定（実時間配信を含む） {verdict}", flush=True)
    return exit_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("benchmark", "video", "diagnose", "serve", "map", "bag", "closed-loop"), required=True)
    parser.add_argument("--bag-output", type=Path, help="bag の確定保存先（rosbags/直下の新規ディレクトリ）")
    parser.add_argument("--circuit", "--course", choices=world.circuits.circuit_ids(), default="odaiba")
    parser.add_argument("--route", choices=("lap",), default="lap")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--speed", type=float, help="RC スケールの走行速度（m/s、シミュレーション時刻）")
    parser.add_argument("--laps", type=int, help="走行する周回数（正の整数）")
    parser.add_argument("--wait-for-stereo", action="store_true")
    parser.add_argument("--camera-height", type=float, default=.12)
    parser.add_argument("--camera-pitch-deg", type=float, default=0)
    parser.add_argument("--resolution", choices=RESOLUTIONS, default="640x480")
    parser.add_argument("--camera-hz", type=int, choices=CAMERA_RATES, default=DEFAULT_CAMERA_HZ)
    parser.add_argument("--imu", choices=IMU_MODES, default="off", help="ジャイロ・統合IMUのHz。加速度は200 Hz")
    args = parser.parse_args()
    if args.mode == "closed-loop":
        if args.duration is not None or args.speed is not None:
            parser.error("closed-loop の速度は制御指令から決まります。周回数は --laps で指定してください")
        if args.laps is None:
            args.laps = 2
    if (args.mode == "bag") != (args.bag_output is not None):
        parser.error("--bag-output は --mode bag で指定してください")
    if args.laps is not None:
        if args.laps < 1:
            parser.error("--laps は1以上の整数を指定してください")
        if args.mode in ("benchmark", "diagnose"):
            parser.error("実時間計測の benchmark / diagnose では --duration で計測秒数を指定してください")
    if args.wait_for_stereo and args.mode != "serve":
        parser.error("--wait-for-stereo requires --mode serve")
    if args.mode == "serve" and args.duration is not None:
        parser.error("serve では --speed と、終了する周回数を指定する場合は --laps を使ってください")
    if args.speed is not None:
        if not math.isfinite(args.speed) or args.speed <= 0:
            parser.error("--speed は有限の正数を m/s で指定してください")
        if args.duration is not None and args.mode in ("video", "bag", "map"):
            parser.error("--speed と1周あたりの --duration は同時に指定できません")
    if args.mode not in ("serve", "closed-loop") and args.duration is None:
        if args.mode == "benchmark":
            args.duration = 60.
        elif args.mode == "diagnose":
            args.duration = 20.
        elif args.speed is None:
            args.duration = world.default_travel_seconds(args.route, args.circuit)
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("duration must be finite and positive")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        parser.error("既存の結果を保護するため、新しい空の出力先を指定してください")
    report = {"circuit": args.circuit, "mode": args.mode, "route": args.route, "requested_duration_s": args.duration}
    if args.mode in ("bag", "serve", "map", "closed-loop"):
        # docker run --init forwards both sigint from the terminal and docker stop's SIGTERM.
        signal.signal(signal.SIGINT, signal.default_int_handler)
        signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        result = run(args, report)
        if args.mode == "closed-loop":
            report["status"] = report["closed_loop"]["status"]
        else:
            report["status"] = "complete" if result == 0 else "below_target"
    except KeyboardInterrupt:
        report.update(status="stopped")
        result = 0 if args.mode == "serve" and args.laps is None else 130
    except (OSError, ValueError, RuntimeError, TimeoutError, subprocess.SubprocessError) as exc:
        report.update(status="error", error=str(exc))
        result = 1
        print(report["error"], file=sys.stderr)
    finally:
        if "partial_bag" in report:
            print(f"未公開の記録先 {report['partial_bag']}", file=sys.stderr)
        ogre_log = Path.home() / ".gz/rendering/ogre2.log"
        if ogre_log.is_file():
            shutil.copyfile(ogre_log, args.output / "ogre2.log")
            report["renderer_log_lines"] = [line.strip() for line in ogre_log.read_text(errors="replace").splitlines()
                                            if "GL_RENDERER" in line or "EGL Subsystem" in line]
        (args.output / "run.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    return result


if __name__ == "__main__":
    sys.exit(main())

"""Notebook の Gazebo と Isaac ROS で指定周回を走り、検証済みの地図を公開する。"""
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

PROJECT = Path(__file__).resolve().parents[2]
ISAAC = "atd_pc_dev"
INSIDE = Path("/workspaces/atd")


def read_json(path):
    return json.loads(path.read_text())


def check_isaac_mounts(results):
    cmd = subprocess.run(["docker", "container", "inspect", ISAAC], capture_output=True, text=True)
    if cmd.returncode:
        raise RuntimeError("Notebook ホストで先に atd dev を起動してください")
    container, = json.loads(cmd.stdout)
    if not container["State"]["Running"] or container["HostConfig"]["NetworkMode"] != "host":
        raise RuntimeError("ホストネットワークで起動した atd_pc_dev が必要です。atd dev を確認してください")
    mounts = {m["Destination"]: m for m in container["Mounts"]}
    for destination, source in [(str(INSIDE), PROJECT), ("/data/results", results)]:
        mount = mounts.get(destination, {})
        if mount.get("Type") != "bind" or Path(mount["Source"]).resolve() != source:
            raise RuntimeError(f"atd_pc_dev の {destination} のマウント元を {source} に揃えてください")
        if destination == "/data/results" and not mount["RW"]:
            raise RuntimeError("atd_pc_dev の /data/results に書き込めません")


def wait_exit(process, timeout):
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def worker(directory, launch, cancel_name="mapping.cancel"):
    """docker exec のクライアントが止まっても、この試行の ROS launch に終了を伝える。"""
    cancel = directory / cancel_name
    if cancel.exists():
        return 1
    proc = subprocess.Popen(launch, start_new_session=True)
    try:
        while proc.poll() is None and not cancel.exists():
            time.sleep(.25)
        if cancel.exists():
            return 1
        return proc.returncode
    finally:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=15)
            except ProcessLookupError:
                proc.wait()
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()


def verify_and_publish(results, out, circuit):
    report = read_json(out / "run.json")
    lap = read_json(out / "lap.complete.json")
    saved = read_json(out / "mapping.saved.json")
    scene = read_json(out / "scene/scene.json")
    trajectory = scene.get("trajectory", {})
    motion = report["motion"]
    laps = trajectory.get("laps", 1)
    if ((out / "mapping.failed.json").exists() or saved["status"] != "complete" or report["status"] != "complete"
            or report["circuit"] != circuit or not motion["lap_completed"]
            or motion.get("laps_requested", 1) != laps or motion.get("laps_completed", 1) != laps
            or not 0 < lap["start_sim_ns"] < lap["finish_sim_ns"]
            or any(saved["tracking"][key] != lap[key] for key in ("start_sim_ns", "finish_sim_ns"))):
        raise RuntimeError("全周の走行・保存を確認できません。run.json と mapping.*.json を確認してください")
    lap["laps_completed"] = laps
    if "speed_mps" in trajectory:
        lap["speed_mps"] = trajectory["speed_mps"]
    stage = out / "map-staging"
    db = stage / "nvidia/cuvslam_map"
    files = {p.name: p.stat().st_size for p in db.glob("*.mdb")
             if p.name != "lock.mdb" and not p.is_symlink() and p.is_file() and p.stat().st_size > 0}
    if not files or files != saved["databases"]:
        raise RuntimeError("保存した cuVSLAM 地図が保存サービス確認時のデータベースと一致しません")
    key = f"gazebo-{circuit}-{out.name}"
    manifest = {"map_id": key, "source": "gazebo_live_cuvslam", "circuit": circuit,
                "run": out.relative_to(results).as_posix(), "lap": lap, "save": saved,
                "geography": scene["geography"]}
    with (stage / "mapping.json").open("x") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    maps = results / "maps"
    maps.mkdir(exist_ok=True)
    target = maps / key
    # Reserve exclusively so an existing map (including an empty directory) is preserved.
    target.mkdir()
    try:
        stage.rename(target)
    except OSError:
        target.rmdir()
        raise
    return target


def coordinate(out, gazebo, mapper, stop_gazebo):
    sim = subprocess.Popen(gazebo, start_new_session=True)
    ros = None
    try:
        deadline = time.monotonic() + 180
        while not (out / "gazebo.ready").is_file():
            if sim.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("Gazebo の起動に失敗しました。試行フォルダの run.json と gazebo.log を確認してください")
            time.sleep(.25)
        print("ステレオ配信を確認しました。cuVSLAM の追跡開始を待っています。", flush=True)
        with (out / "mapping.log").open("x") as log:
            ros = subprocess.Popen(mapper, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            phase = None
            shutdown_deadline = None
            while True:
                sim_rc, ros_rc = sim.poll(), ros.poll()
                failed = out / "mapping.failed.json"
                if failed.is_file():
                    raise RuntimeError(str(read_json(failed).get("error") or failed.read_text()))
                if sim_rc not in (None, 0) or ros_rc not in (None, 0):
                    raise RuntimeError(f"地図作成が中断しました (Gazebo={sim_rc}, cuVSLAM={ros_rc})。run.json と mapping.log を確認してください")
                if sim_rc is not None or ros_rc is not None:
                    if not (out / "mapping.saved.json").is_file():
                        raise RuntimeError("地図保存前にプロセスが終了しました。mapping.log と run.json を確認してください")
                    if sim_rc == ros_rc == 0:
                        return
                    shutdown_deadline = shutdown_deadline or time.monotonic() + 60
                    if time.monotonic() > shutdown_deadline:
                        raise RuntimeError("保存後の Gazebo / ROS の終了を確認できません")
                if phase != "saving" and (out / "lap.complete.json").is_file():
                    print("ゴールに到達しました。地図を保存しています。", flush=True)
                    phase = "saving"
                elif phase is None and (out / "mapping.ready").is_file():
                    print("追跡を開始しました。指定した周回数で走行します。", flush=True)
                    phase = "driving"
                time.sleep(.25)
    finally:
        if sim.poll() is None or (ros is not None and ros.poll() is None):
            (out / "mapping.cancel").touch()
            if sim.poll() is None:
                stop_gazebo()
                wait_exit(sim, 12)
            if ros is not None:
                wait_exit(ros, 25)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="role", required=True)
    helper = modes.add_parser("worker")
    helper.add_argument("--output", type=Path, required=True)
    host = modes.add_parser("host")
    host.add_argument("--output", type=Path, required=True)
    host.add_argument("--results", type=Path, required=True)
    for name in ("domain", "circuit", "container", "profile"):
        host.add_argument("--" + name, required=True)
    host.add_argument("--dry-run", action="store_true")
    for mode in (helper, host):
        mode.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("実行コマンドがありません")
    if args.role == "worker":
        return worker(args.output, command)
    out, results = args.output.resolve(), args.results.resolve()
    remote = Path("/data/results") / out.relative_to(results)
    launch = ["ros2", "launch", "atd_bringup", "gazebo_mapping.launch.py",
              f"run_dir:={remote}", f"map_dir:={remote}/map-staging/nvidia/cuvslam_map"]
    mapper = ["docker", "exec", "-u", "admin", ISAAC, "/usr/local/bin/atd-shell", "env",
              f"ROS_DOMAIN_ID={args.domain}", f"FASTDDS_DEFAULT_PROFILES_FILE={args.profile}",
              "python3", str(INSIDE / "tools/gazebo_monza/map_job.py"), "worker", "--output", str(remote),
              "--", *launch]
    if args.dry_run:
        print(shlex.join(command))
        print(shlex.join(mapper))
        return 0
    check_isaac_mounts(results)

    def stop_gazebo():
        try:
            subprocess.run(["docker", "stop", "--time", "8", args.container], timeout=15,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except subprocess.TimeoutExpired:
            print("Gazebo コンテナの終了確認がタイムアウトしました。", file=sys.stderr)

    signal.signal(signal.SIGTERM, signal.default_int_handler)
    coordinate(out, command, mapper, stop_gazebo)
    target = verify_and_publish(results, out, args.circuit)
    print(f"cuVSLAM 地図を保存しました　{target / 'nvidia/cuvslam_map'}")
    if read_json(out / "mapping.saved.json")["tracking"].get("coverage") == "interrupted":
        print(f"周回途中に追跡欠けがあります。時刻と最大間隔は {target / 'mapping.json'} を確認してください。")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("地図作成を中断しました", file=sys.stderr)
        sys.exit(130)
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as exc:
        sys.exit(f"Gazebo mapping: {exc}")

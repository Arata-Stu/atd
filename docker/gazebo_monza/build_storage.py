"""Notebook のローカル Docker ストレージを計測しながら専用イメージを準備する。"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time


def mount_points():
    mounts = []
    for line in Path("/proc/self/mountinfo").read_text().splitlines():
        escaped = line.split(" ")[4]
        path = re.sub(r"\\([0-7]{3})", lambda m: chr(int(m[1], 8)), escaped)
        mounts.append(Path(path))
    return sorted(mounts, key=lambda p: len(p.parts), reverse=True)


def measured_filesystems(roots, mounts):
    # /var/lib/docker の中身は一般ユーザーが読めない場合がある。マウント表の
    # 最長一致から同じファイルシステム上の空き容量を読む。
    result = {}
    for name, root in roots.items():
        root = Path(os.path.realpath(root))
        mount = next((m for m in mounts if m == root or m in root.parents), None)
        if mount is None:
            raise RuntimeError(f"{root} のホストマウントが見つかりません")
        device = os.stat(mount).st_dev
        item = result.setdefault(device, {"mount": str(mount), "paths": {}})
        item["paths"][name] = str(root)
    return list(result.values())


def free_bytes(mount):
    stat = os.statvfs(mount)
    return stat.f_bavail * stat.f_frsize


def docker_snapshot(output, phase):
    for name, command in (
        ("docker", ["docker", "system", "df", "-v"]),
        ("builder", ["docker", "buildx", "du", "--builder", "default"]),
    ):
        with (output / f"{name}-{phase}.txt").open("w") as stream:
            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
            if result.returncode:
                stream.write(f"\n計測コマンド終了コード {result.returncode}\n")


def image_size(image):
    result = subprocess.run(["docker", "image", "inspect", "-f", "{{.Size}}", image],
                            text=True, capture_output=True)
    return int(result.stdout.strip()) if result.returncode == 0 else None


def measure_build(context, image, output):
    docker_root = subprocess.check_output(
        ["docker", "info", "-f", "{{.DockerRootDir}}"], text=True).strip()
    if not os.path.isabs(docker_root):
        raise RuntimeError(f"DockerRootDir を特定できません: {docker_root!r}")
    roots = {"docker_root": docker_root}
    # Engine の containerd image store は別のディスクに置かれる場合がある。
    containerd = os.environ.get("ATD_CONTAINERD_ROOT", "/var/lib/containerd")
    if os.path.exists(containerd) or "ATD_CONTAINERD_ROOT" in os.environ:
        roots["containerd_root"] = containerd
    filesystems = measured_filesystems(roots, mount_points())
    docker_snapshot(output, "before")
    report = {"image": image, "sampling_interval_s": 0.5, "filesystems": filesystems,
              "image_size_before_bytes": image_size(image)}
    for fs in filesystems:
        fs["free_before_bytes"] = fs["min_observed_free_bytes"] = free_bytes(fs["mount"])
    started = time.monotonic()
    command = ["docker", "buildx", "build", "--builder", "default", "--load",
               "--progress", "plain", "--tag", image, str(context)]
    try:
        with subprocess.Popen(command) as build:
            try:
                while build.poll() is None:
                    for fs in filesystems:
                        fs["min_observed_free_bytes"] = min(
                            fs["min_observed_free_bytes"], free_bytes(fs["mount"]))
                    time.sleep(report["sampling_interval_s"])
            except BaseException:
                build.terminate()
                raise
            report["build_exit_code"] = build.returncode
    finally:
        report["elapsed_wall_s"] = round(time.monotonic() - started, 2)
        for fs in filesystems:
            fs["free_after_bytes"] = free_bytes(fs["mount"])
            fs["min_observed_free_bytes"] = min(fs["min_observed_free_bytes"], fs["free_after_bytes"])
            fs["net_used_bytes"] = fs["free_before_bytes"] - fs["free_after_bytes"]
            fs["peak_added_bytes_sampled"] = fs["free_before_bytes"] - fs["min_observed_free_bytes"]
        report["image_size_after_bytes"] = image_size(image)
        (output / "storage.json").write_text(json.dumps(report, indent=2) + "\n")
        for fs in filesystems:
            print(f"{fs['mount']}  実使用増分 {fs['net_used_bytes'] / 2**30:+.2f} GiB"
                  f"  ビルド中の最大増分（標本値） {fs['peak_added_bytes_sampled'] / 2**30:.2f} GiB")
        docker_snapshot(output, "after")
    return report["build_exit_code"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("context", type=Path)
    parser.add_argument("image")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    sys.exit(measure_build(args.context, args.image, args.output))

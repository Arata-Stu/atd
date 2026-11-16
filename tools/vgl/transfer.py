"""SCP で export bundle を送り、検証後に Jetson の保存先へ昇格する。"""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import shlex
import subprocess
import sys
import uuid

if __package__:
    from .bundle import validate_bundle
else:
    from bundle import validate_bundle


def validate_target(value: str) -> str:
    if value.startswith('-') or not re.fullmatch(r"(?:[A-Za-z0-9._-]+@)?[A-Za-z0-9][A-Za-z0-9._-]*", value):
        raise ValueError("SSH 接続先は SSH alias または user@host で指定してください")
    return value


def validate_remote_path(value: str) -> str:
    path = Path(value)
    if not path.is_absolute() or path == Path(path.anchor) or any(part == ".." for part in path.parts):
        raise ValueError("remote path は / 以外の絶対パスで指定してください（.. は使用不可）")
    if any(character in value for character in ("\n", "\r", "\0")):
        raise ValueError("remote path に改行や NUL は使えません")
    return value


def validate_port(value: str | int) -> int:
    try:
        port = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError("SSH port は 1〜65535 の整数です") from error
    if not 1 <= port <= 65535:
        raise ValueError("SSH port は 1〜65535 の整数です")
    return port


# NVIDIA / ROS / ATD Python package に依存しない受信処理を stdin で SSH へ渡す。
# SCP の staging は唯一の UUID 名。既存の最終 export を上書きしない。
RECEIVER = r'''
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import sys

def checked_id(value):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,95}", value):
        raise ValueError("invalid asset/export ID")
    return value

def confined(path, root):
    if not path.resolve().is_relative_to(root):
        raise ValueError("remote path escapes result directory")
    return path

def validate(root):
    if root.is_symlink() or not root.is_dir():
        raise ValueError("incoming bundle must be an ordinary directory")
    manifest_path = root / "manifest.json"
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise ValueError("manifest.json missing or symlink")
    manifest = json.loads(manifest_path.read_text())
    if not isinstance(manifest, dict) or type(manifest.get("schema_version")) is not int or manifest["schema_version"] != 1 or manifest.get("kind") != "atd.vgl.export":
        raise ValueError("unsupported manifest")
    if manifest.get("asset_id") != asset or manifest.get("export_id") != export:
        raise ValueError("manifest IDs do not match requested destination")
    model = manifest.get("model", {})
    if not isinstance(model, dict) or model.get("family") != "aliked_lightglue" or model.get("mode") not in ("aliked", "package"):
        raise ValueError("unsupported model family/mode")
    if any(type(model.get(key)) is not int or model[key] <= 0 for key in ("width", "height")):
        raise ValueError("invalid image size")
    if model.get("top_k") != 2048:
        raise ValueError("unsupported top_k")
    files = manifest.get("files")
    if not isinstance(files, dict) or not files:
        raise ValueError("manifest files must be a nonempty object")
    required = {"models/aliked_lightglue/aliked.onnx",
                "configs/isaac/keypoint_creation_config.pb.txt",
                "configs/isaac/matching_task_worker_config.pb.txt"}
    expected = {"manifest.json"}
    for name, record in files.items():
        relative = PurePosixPath(name)
        if (not isinstance(name, str) or relative.is_absolute() or "\\" in name or any(ord(c) < 32 for c in name)
                or any(part in ("", ".", "..") for part in name.split("/"))
                or relative.parts[0] not in ("models", "configs")):
            raise ValueError("unsafe manifest file path")
        if (not isinstance(record, dict) or set(record) != {"size", "sha256"} or type(record.get("size")) is not int or record["size"] < 0
                or not re.fullmatch(r"[0-9a-f]{64}", record.get("sha256", ""))):
            raise ValueError("invalid file digest/size")
        path = confined(root / name, root.resolve())
        if path.is_symlink() or not path.is_file():
            raise ValueError("missing/unsafe file: " + name)
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        if path.stat().st_size != record["size"] or digest.hexdigest() != record["sha256"]:
            raise ValueError("file hash/size mismatch: " + name)
        expected.add(name)
    if not required.issubset(files) or any(files[name]["size"] <= 0 for name in required) or not any(
            name.startswith("models/aliked_lightglue/lightglue") and name.endswith(".onnx")
            and record["size"] > 0 for name, record in files.items()):
        raise ValueError("required model/config files missing")
    actual = set()
    for directory, dirs, names in os.walk(root, followlinks=False):
        for name in dirs + names:
            path = Path(directory) / name
            mode = path.lstat().st_mode
            if stat.S_ISLNK(mode) or not (stat.S_ISREG(mode) or stat.S_ISDIR(mode)):
                raise ValueError("symlink/special file in bundle")
            if stat.S_ISREG(mode):
                actual.add(path.relative_to(root).as_posix())
    if actual != expected:
        raise ValueError("bundle has missing or unlisted files")
    return manifest_path.read_bytes()

try:
    action, result_arg, asset, export, incoming = sys.argv[1:]
    checked_id(asset)
    checked_id(export)
    if not re.fullmatch(r"\.incoming-[0-9a-f]{32}", incoming):
        raise ValueError("invalid staging name")
    raw_result = Path(result_arg)
    if not raw_result.is_absolute() or ".." in raw_result.parts or not raw_result.is_dir():
        raise ValueError("remote result directory must exist and be absolute")
    result = raw_result.resolve()
    if result == Path("/"):
        raise ValueError("remote result directory cannot be root")
    for path in (result / "models", result / "models/vgl", result / "models/vgl" / asset,
                 result / "models/vgl" / asset / "exported"):
        if path.is_symlink():
            raise ValueError("remote model parent cannot be a symlink")
    parent = confined(result / "models" / "vgl" / asset / "exported", result)
    parent.mkdir(parents=True, exist_ok=True)
    confined(parent, result)
    stage = confined(parent / incoming, result)
    destination = confined(parent / export, result)
    if action == "prepare":
        if os.path.lexists(stage):
            raise ValueError("staging path already exists")
        print("SCP staging: " + str(stage))
    elif action == "commit":
        source_manifest = validate(stage)
        if os.path.lexists(destination):
            if validate(destination) != source_manifest:
                raise ValueError("destination already exists with different content; choose a new export ID")
            shutil.rmtree(stage)
            print("UNCHANGED: " + str(destination))
        else:
            stage.rename(destination)
            print("VERIFIED: " + str(destination))
    else:
        raise ValueError("unknown receiver action")
except (OSError, ValueError, TypeError, KeyError, IndexError) as error:
    sys.exit("VGL transfer failed (incoming data is preserved): " + str(error))
'''


def _ssh_python(target: str, port: int, arguments: list[str]) -> None:
    command = shlex.join(["python3", "-", *arguments])
    subprocess.run(["ssh", "-p", str(port), "--", target, command],
                   input=RECEIVER, text=True, check=True)


def send_bundle(bundle: Path, target: str, remote_results: str, port: int = 22) -> None:
    target = validate_target(target)
    port = validate_port(port)
    remote_results = validate_remote_path(remote_results)
    manifest = validate_bundle(bundle)
    asset, export = manifest["asset_id"], manifest["export_id"]
    incoming = ".incoming-" + uuid.uuid4().hex
    arguments = [remote_results, asset, export, incoming]
    _ssh_python(target, port, ["prepare", *arguments])
    stage = str(Path(remote_results) / "models/vgl" / asset / "exported" / incoming)
    print("SCP: " + str(bundle) + " -> " + target + ":" + stage, flush=True)
    # OpenSSH 9+ の既定 SFTP モード。remote path は shell quote を含めず一引数で渡す。
    subprocess.run(["scp", "-P", str(port), "-r", "--", str(bundle.resolve()), target + ":" + stage], check=True)
    _ssh_python(target, port, ["commit", *arguments])


def remote_build(target: str, remote_project: str, arguments: list[str], port: int = 22) -> None:
    target = validate_target(target)
    port = validate_port(port)
    remote_project = validate_remote_path(remote_project)
    command = shlex.join(["bash", str(Path(remote_project) / "scripts/vgl.sh"),
                          "--host", "jetson", "build", *arguments])
    subprocess.run(["ssh", "-p", str(port), "--", target, command], check=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("send", "remote-build"))
    parser.add_argument("--ssh-target", required=True)
    parser.add_argument("--port", default="22")
    parser.add_argument("--remote-results")
    parser.add_argument("--remote-project")
    parser.add_argument("--bundle", type=Path)
    args, remainder = parser.parse_known_args(argv)
    if remainder[:1] == ["--"]:
        remainder = remainder[1:]
    try:
        if args.action == "send":
            if not args.bundle or not args.remote_results or remainder:
                parser.error("send には --bundle と --remote-results が必要です")
            send_bundle(args.bundle, args.ssh_target, args.remote_results, args.port)
        else:
            if not args.remote_project:
                parser.error("remote-build には --remote-project が必要です")
            remote_build(args.ssh_target, args.remote_project, remainder, args.port)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print("VGL: " + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Jetson 上で公式の VGL builder を実行し、出力を対象環境と結び付ける。"""

from __future__ import annotations

from contextlib import contextmanager
import ctypes
import importlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import signal
import tempfile
import uuid

if __package__:
    from .bundle import export_path, model_root, sha256, utc_now, validate_bundle, validate_id, write_json
else:
    from bundle import export_path, model_root, sha256, utc_now, validate_bundle, validate_id, write_json


def _block(text: str, name: str) -> tuple[int, int]:
    masked = re.sub(r'"(?:\\.|[^"\\])*"|#[^\n]*|//[^\n]*',
                    lambda match: ' ' * len(match.group()), text)
    matches = list(re.finditer(r'\b' + re.escape(name) + r'\s*\{', masked))
    if len(matches) != 1:
        raise ValueError(f'{name} block は一つ必要です')
    begin, depth = matches[0].end(), 1
    for end in range(begin, len(masked)):
        depth += (masked[end] == '{') - (masked[end] == '}')
        if depth == 0:
            return begin, end
    raise ValueError(f'{name} block が閉じられていません')


def fixed_extractor_config(text: str, width: int, height: int) -> str:
    if width <= 0 or height <= 0:
        raise ValueError('画像サイズは正の整数で指定してください')
    begin, end = _block(text, 'aliked_detector')
    inner_begin, inner_end = _block(text[begin:end], 'tensorrt_config')
    profile_begin, profile_end = _block(text[begin + inner_begin:begin + inner_end], 'input_dimension')
    lo, hi = begin + inner_begin + profile_begin, begin + inner_begin + profile_end
    profile = re.sub(r'"(?:\\.|[^"\\])*"|#[^\n]*|//[^\n]*',
                     lambda match: ' ' * len(match.group()), text[lo:hi])
    for key in ('min', 'opt', 'max'):
        values = [int(value) for value in re.findall(r'\b' + key + r'\s*:\s*(\d+)', profile)]
        if len(values) != 4 or values[:2] != [1, 3]:
            raise ValueError('対応しない ALIKED NCHW profile です')
    counters = dict.fromkeys(('min', 'opt', 'max'), 0)
    pieces, cursor = [], 0
    for match in re.finditer(r'\b(min|opt|max)\s*:\s*(\d+)', profile):
        key = match.group(1)
        value = (1, 3, height, width)[counters[key]]
        counters[key] += 1
        start, end = match.span(2)
        pieces.extend((text[lo + cursor:lo + start], str(value)))
        cursor = end
    pieces.append(text[lo + cursor:hi])
    replacement = ''.join(pieces)
    return text[:lo] + replacement + text[hi:]


def package_paths() -> tuple[Path, Path]:
    # ament_index の探索順と package marker を使う。分離した export venv に
    # ROS の PYTHONPATH を戻さず、ros2 CLI / ament_index_python にも依存しない。
    # https://github.com/ament/ament_index/tree/rolling/ament_index_python/ament_index_python
    entries = [entry for entry in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if entry]
    if not entries:
        raise ValueError('AMENT_PREFIX_PATH が未設定または空です。atd-shell から ROS 環境を読み込んでください')
    prefixes = [Path(entry) for entry in entries]
    for prefix in prefixes:
        if not prefix.is_absolute():
            raise ValueError('AMENT_PREFIX_PATH に相対 prefix は使えません: ' + str(prefix))
    package = 'isaac_ros_visual_mapping'
    for prefix in prefixes:
        marker = prefix / 'share/ament_index/resource_index/packages' / package
        if not marker.is_file():
            continue
        share = prefix / 'share' / package
        if not share.is_dir():
            raise ValueError('ament index に package はありますが share ディレクトリがありません: ' + str(share))
        return prefix, share
    raise ValueError('AMENT_PREFIX_PATH 内に isaac_ros_visual_mapping がありません。ATD image を --build で更新してください')


def hardware_info() -> dict:
    # この工程には PyTorch / ONNX exporter を import しない。
    trt = importlib.import_module('tensorrt')
    if platform.system() != 'Linux' or platform.machine() != 'aarch64':
        raise ValueError('この build は Jetson (Linux / aarch64) 上で実行してください')
    marker = Path('/etc/nv_tegra_release')
    model_file = Path('/proc/device-tree/model')
    if not marker.is_file() and not (model_file.is_file() and b'jetson' in model_file.read_bytes().lower()):
        raise ValueError('Jetson / Tegra の機種情報を確認できません')
    cuda = ctypes.CDLL('libcuda.so.1')

    def call(name, *args):
        result = getattr(cuda, name)(*args)
        if result:
            raise ValueError(f'CUDA {name} が失敗しました: {result}')

    call('cuInit', 0)
    device, driver, major, minor = (ctypes.c_int() for _ in range(4))
    name, gpu_uuid = ctypes.create_string_buffer(256), ctypes.create_string_buffer(16)
    call('cuDeviceGet', ctypes.byref(device), 0)
    call('cuDeviceGetName', name, len(name), device)
    call('cuDeviceGetUuid', gpu_uuid, device)
    call('cuDeviceComputeCapability', ctypes.byref(major), ctypes.byref(minor), device)
    call('cuDriverGetVersion', ctypes.byref(driver))
    try:
        packages = subprocess.check_output(
            ['dpkg-query', '-W', '-f=${Package}=${Version}\n',
             'ros-lyrical-isaac-ros-visual-mapping', 'python3-libnvinfer'], text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        packages = 'unavailable'
    return {'architecture': platform.machine(), 'gpu_name': name.value.decode(),
            'gpu_uuid': gpu_uuid.raw.hex(), 'compute_capability': [major.value, minor.value],
            'cuda_driver_version': driver.value, 'tensorrt_version': trt.__version__,
            'tegra_release': marker.read_text().strip() if marker.is_file() else '',
            'packages': packages, 'atd_runtime_revision': os.environ.get('ATD_RUNTIME_REV', ''),
            'cuda_visible_devices': os.environ.get('CUDA_VISIBLE_DEVICES', '')}


def inspect_engine(path: Path, expected_image_shape: list[int] | None = None) -> dict:
    trt = importlib.import_module('tensorrt')
    logger = trt.Logger(trt.Logger.WARNING)
    trt.init_libnvinfer_plugins(logger, '')
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(path.read_bytes())
    if engine is None:
        raise ValueError('engine をこの GPU / TensorRT で読み込めません: ' + str(path))
    tensors = []
    for index in range(engine.num_io_tensors):
        name = engine.get_tensor_name(index)
        tensors.append({'name': name, 'shape': list(engine.get_tensor_shape(name)),
                        'mode': str(engine.get_tensor_mode(name)), 'dtype': str(engine.get_tensor_dtype(name))})
    if expected_image_shape is not None:
        image = next((tensor for tensor in tensors if tensor['name'] == 'image'), None)
        if image is None or image['shape'] != expected_image_shape:
            raise ValueError('ALIKED engine の入力画像サイズが export と異なります')
    del engine
    del runtime
    return {'sha256': sha256(path), 'size': path.stat().st_size, 'tensors': tensors}


def _engines(output: Path) -> dict[str, Path]:
    directory = output / 'aliked_lightglue'
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError('engine は build 出力内の通常ディレクトリに保存してください')
    result = {}
    for kind, pattern in (('extractor', 'aliked_*.engine'), ('matcher', 'lightglue_aliked_*.engine')):
        paths = list(directory.glob(pattern))
        if len(paths) != 1 or paths[0].is_symlink() or not paths[0].is_file() or paths[0].stat().st_size == 0:
            raise ValueError(f'{kind} engine は一つ必要です: {directory}/{pattern}')
        result[kind] = paths[0]
    return result


@contextmanager
def engine_builder_commands(binaries: Path, bundle: Path, manifest: dict, output: Path):
    # SDK は --output_model_dir のほかに --model_dir の ONNX の隣にも engine を書き込む。
    with tempfile.TemporaryDirectory(prefix='.builder-inputs-', dir=output) as directory:
        inputs = Path(directory)
        for name, info in manifest['files'].items():
            destination = inputs / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(bundle / name, destination)
            if sha256(destination) != info['sha256']:
                raise ValueError('builder 入力のコピーが export manifest と一致しません: ' + name)
        model_args = ['--model_dir', str(inputs / 'models'), '--output_model_dir', str(output)]
        config = inputs / 'configs/isaac'
        yield [
            [str(binaries / 'export_extractor_engine'), '--configure_file',
             str(config / 'keypoint_creation_config.pb.txt'), *model_args],
            [str(binaries / 'export_lightglue_engine'), '--worker_config_file',
             str(config / 'matching_task_worker_config.pb.txt'), *model_args],
        ]


def run_builder(command: list[str]) -> None:
    # GPU builder の stdout/stderr も Python 側の job.log と端末へ残す。
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, bufsize=1, start_new_session=True)
    try:
        for line in process.stdout:
            print(line, end='', flush=True)
        status = process.wait()
        if status:
            raise subprocess.CalledProcessError(status, command)
    except BaseException:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        raise
    finally:
        if process.stdout is not None:
            process.stdout.close()


def verify_build(output: Path, bundle: Path, hardware: dict | None = None) -> dict:
    manifest = validate_bundle(bundle)
    report_file = output / 'build.json'
    if output.is_symlink() or not report_file.is_file() or report_file.is_symlink():
        raise ValueError('build 完了情報がありません: ' + str(output))
    report = json.loads(report_file.read_text())
    hardware = hardware if hardware is not None else hardware_info()
    if (type(report.get('schema_version')) is not int or report['schema_version'] != 1 or report.get('status') != 'complete'
            or report.get('asset_id') != manifest['asset_id']
            or report.get('export_id') != manifest['export_id']
            or report.get('bundle_sha256') != sha256(bundle / 'manifest.json')
            or report.get('hardware') != hardware):
        raise ValueError('build のモデル・export・対象環境が現在と一致しません。別 build ID で生成してください')
    paths = _engines(output)
    shape = [1, 3, manifest['model']['height'], manifest['model']['width']]
    current = {kind: inspect_engine(path, shape if kind == 'extractor' else None)
               for kind, path in paths.items()}
    if report.get('engines') != current:
        raise ValueError('build 済み engine が変更されています')
    onnx = output / 'aliked_lightglue/aliked.onnx'
    if onnx.is_symlink() or not onnx.is_file() or sha256(onnx) != manifest['files']['models/aliked_lightglue/aliked.onnx']['sha256']:
        raise ValueError('runtime の ALIKED ONNX が export と一致しません')
    return report


def build(results: Path, asset_id: str, export_id: str, target_id: str, build_id: str, log_dir: Path) -> Path:
    bundle = export_path(results, asset_id, export_id)
    manifest = validate_bundle(bundle)
    if manifest['asset_id'] != asset_id or manifest['export_id'] != export_id:
        raise ValueError('bundle の ID が保存先と一致しません')
    target = model_root(results, asset_id) / 'engines' / validate_id(target_id)
    for path in (target.parent, target):
        if path.is_symlink():
            raise ValueError('engine 保存先に symlink は使えません')
    output = target / validate_id(build_id)
    hardware = hardware_info()  # exporter を実行する前に GPU / TRT を確認。
    if output.exists() or output.is_symlink():
        verify_build(output, bundle, hardware)
        print('検証済み engine を再利用: ' + str(output), flush=True)
        return output
    prefix, _ = package_paths()
    binaries = prefix / 'bin/visual_mapping'
    for name in ('export_extractor_engine', 'export_lightglue_engine'):
        binary = binaries / name
        if not binary.is_file() or not os.access(binary, os.X_OK):
            raise ValueError('公式 builder を実行できません: ' + str(binary))
    target.mkdir(parents=True, exist_ok=True)
    staging = target / ('.partial-' + build_id + '-' + uuid.uuid4().hex)
    staging.mkdir()
    report = {'schema_version': 1, 'asset_id': asset_id, 'export_id': export_id,
              'target_id': target_id, 'build_id': build_id, 'created_at': utc_now(),
              'bundle_sha256': sha256(bundle / 'manifest.json'), 'hardware': hardware,
              'log_dir': str(log_dir), 'status': 'building'}
    try:
        with engine_builder_commands(binaries, bundle, manifest, staging) as commands:
            write_json(log_dir / 'build-plan.json', {**report, 'commands': commands})
            for command in commands:
                print('実行: ' + repr(command), flush=True)
                run_builder(command)
                if command is commands[0]:
                    extractors = list((staging / 'aliked_lightglue').glob('aliked_*.engine'))
                    if len(extractors) != 1:
                        raise ValueError('ALIKED engine が生成されていません')
                    inspect_engine(extractors[0], [1, 3, manifest['model']['height'], manifest['model']['width']])
        paths = _engines(staging)
        model_output = staging / 'aliked_lightglue/aliked.onnx'
        shutil.copyfile(bundle / 'models/aliked_lightglue/aliked.onnx', model_output)
        for license_file in (bundle / 'models/aliked_lightglue').glob('LICENSE*'):
            shutil.copyfile(license_file, model_output.parent / license_file.name)
        report['engines'] = {kind: inspect_engine(path, [1, 3, manifest['model']['height'], manifest['model']['width']]
                                               if kind == 'extractor' else None)
                             for kind, path in paths.items()}
        # build 中に入力が変更されていないことも再確認する。
        validate_bundle(bundle)
        if report['bundle_sha256'] != sha256(bundle / 'manifest.json'):
            raise ValueError('build 中に入力 manifest が変更されました')
        report.update(status='complete', completed_at=utc_now())
        write_json(staging / 'build.json', report)
        verify_build(staging, bundle, hardware)
        if output.exists():
            raise ValueError('同じ build ID が別プロセスで作成されました')
        staging.rename(output)
        print('vgl_model_dir: ' + str(output), flush=True)
        return output
    except BaseException as error:
        report.update(status='failed', error=str(error), failed_at=utc_now())
        write_json(log_dir / 'build-failed.json', report)
        if staging.exists():
            failed = log_dir / 'failed-output'
            if not failed.exists():
                staging.rename(failed)
        raise

"""ROS の system Python で実行する Notebook 専用 VGL 準備 worker。"""
from __future__ import annotations

import ctypes
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import uuid

# Web venv に TensorRT / ROS Python を取り込まず、固定した処理で既存資産を利用する。
REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO))
from tools.vgl.bundle import validate_bundle, sha256, write_json, model_root, utc_now
from tools.vgl.build_engine import inspect_engine, package_paths
if __package__:
    from .vgl_transaction import check_preparable, recover, publish
else:
    from vgl_transaction import check_preparable, recover, publish


def gpu_identity():
    import tensorrt
    if platform.machine() not in ('x86_64', 'AMD64') or platform.system() != 'Linux':
        raise ValueError('Web 地図準備は NVIDIA GPU のある Linux Notebook 専用です')
    cuda = ctypes.CDLL('libcuda.so.1')
    if cuda.cuInit(0):
        raise ValueError('CUDA の初期化に失敗しました')
    device, driver = ctypes.c_int(), ctypes.c_int()
    name, identity = ctypes.create_string_buffer(256), ctypes.create_string_buffer(16)
    for function, args in [('cuDeviceGet', (ctypes.byref(device), 0)),
                           ('cuDeviceGetName', (name, 256, device)),
                           ('cuDeviceGetUuid', (identity, device)),
                           ('cuDriverGetVersion', (ctypes.byref(driver),))]:
        if getattr(cuda, function)(*args):
            raise ValueError('CUDA GPU の識別に失敗しました: ' + function)
    return dict(gpu_uuid=identity.raw.hex(), gpu_name=name.value.decode(),
                cuda_driver_version=driver.value, tensorrt_version=tensorrt.__version__, architecture=platform.machine())


def run(argv):
    print('実行: ' + repr(list(map(str, argv))), flush=True)
    subprocess.run(list(map(str, argv)), check=True)


def prepare(results, map_id, model_relative):
    results = Path(results).resolve()
    folder = results / 'maps' / map_id
    recover(folder)
    check_preparable(folder)
    bundle = results / model_relative
    if not bundle.resolve().is_relative_to(results) or bundle.is_symlink():
        raise ValueError('モデルは結果保存先内の export bundle を選択してください')
    manifest = validate_bundle(bundle)
    hardware = gpu_identity()
    shape = [1, 3, manifest['model']['height'], manifest['model']['width']]
    prefix, share = package_paths()
    binaries = prefix / 'bin/visual_mapping'
    target = model_root(results, manifest['asset_id']) / 'engines' / ('notebook-' + hardware['gpu_uuid'])
    target.mkdir(parents=True, exist_ok=True)
    engine_stage = Path(tempfile.mkdtemp(prefix='.partial-', dir=target))
    build_id = manifest['export_id'] + '-' + uuid.uuid4().hex[:12]
    engines = target / build_id
    report = dict(schema_version=1, asset_id=manifest['asset_id'], export_id=manifest['export_id'],
                  target_id=target.name, build_id=build_id, status='building', hardware=hardware,
                  bundle_sha256=sha256(bundle / 'manifest.json'), created_at=utc_now())
    try:
        run([binaries / 'export_extractor_engine', '--configure_file', bundle / 'configs/isaac/keypoint_creation_config.pb.txt',
             '--model_dir', bundle / 'models', '--output_model_dir', engine_stage])
        run([binaries / 'export_lightglue_engine', '--worker_config_file', bundle / 'configs/isaac/matching_task_worker_config.pb.txt',
             '--model_dir', bundle / 'models', '--output_model_dir', engine_stage])
        extractor = list((engine_stage / 'aliked_lightglue').glob('aliked_*.engine'))
        matcher = list((engine_stage / 'aliked_lightglue').glob('lightglue_aliked_*.engine'))
        if len(extractor) != 1 or len(matcher) != 1:
            raise ValueError('公式 builder の engine 出力を一意に確認できません')
        report['engines'] = dict(extractor=inspect_engine(extractor[0], shape), matcher=inspect_engine(matcher[0]))
        shutil.copyfile(bundle / 'models/aliked_lightglue/aliked.onnx', engine_stage / 'aliked_lightglue/aliked.onnx')
        for license_file in (bundle / 'models/aliked_lightglue').glob('LICENSE*'):
            shutil.copyfile(license_file, engine_stage / 'aliked_lightglue' / license_file.name)
        validate_bundle(bundle)
        if report['bundle_sha256'] != sha256(bundle / 'manifest.json'):
            raise ValueError('engine build 中に export が更新されました')
        report.update(status='complete', completed_at=utc_now())
        write_json(engine_stage / 'build.json', report)
        os.rename(engine_stage, engines)
    finally:
        if engine_stage.exists():
            failed = target / ('failed-' + build_id)
            write_json(engine_stage / 'build-failed.json', report)
            os.rename(engine_stage, failed)
    print('Notebook 実行用 engine: ' + str(engines), flush=True)
    with tempfile.TemporaryDirectory(prefix='.vgl-', dir=folder) as temporary:
        temporary = Path(temporary)
        config = temporary / 'vgl_runtime_config'
        shutil.copytree(share / 'configs/isaac', config)
        shutil.copytree(bundle / 'configs/isaac', config, dirs_exist_ok=True)
        required = ('keypoint_creation_config.pb.txt', 'localizer_config.pb.txt', 'matching_config.pb.txt', 'query_map_config.pb.txt')
        if any(not (config / name).is_file() for name in required):
            raise ValueError('公式 VGL 設定が不足しています')
        output = temporary / 'cuvgl_map'
        # SDK が必要な入力を提供するか --help でも確認し、未知の版へ黙って fallback しない。
        help_text = subprocess.check_output(['ros2', 'run', 'isaac_ros_visual_mapping', 'create_cuvgl_map.py', '--help'], text=True)
        for flag in ('--map_folder', '--raw_image_folder', '--config_folder_path', '--model_dir', '--binary_folder_path', '--extract_feature', '--build_bow_index'):
            if flag not in help_text:
                raise ValueError('この SDK の公式 VGL CLI に必要な引数がありません: ' + flag)
        run(['ros2', 'run', 'isaac_ros_visual_mapping', 'create_cuvgl_map.py', f'--map_folder={output}',
             f'--raw_image_folder={folder / "nvidia/map_frames/rectified"}', f'--config_folder_path={config}',
             f'--model_dir={engines}', f'--binary_folder_path={binaries}', '--feature_type=aliked', '--extract_feature', '--build_bow_index'])
        if not output.is_dir() or not any(p.is_file() and p.stat().st_size for p in output.rglob('*')):
            raise ValueError('VGL の非空成果物がありません')
        publish(folder, config, output, dict(model_export=model_relative,
            runtime_model_dir=str(engines.relative_to(results)), input_shape=shape,
            bundle_sha256=report['bundle_sha256'], asset_id=manifest['asset_id'],
            export_id=manifest['export_id'], completed_at=utc_now()))



if __name__ == '__main__':
    prepare(sys.argv[1], sys.argv[2], sys.argv[3])

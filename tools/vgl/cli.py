#!/usr/bin/env python3
"""VGL export / build の非対話 backend。対話の入口は scripts/vgl.sh。"""

from __future__ import annotations

import argparse
import contextlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import sys
import traceback
import uuid

if __package__:
    from .bundle import (export_path, make_manifest, model_root, sha256, utc_now,
                         validate_bundle, validate_id, write_json, copy_inputs)
    from .build_engine import build, fixed_extractor_config, hardware_info, package_paths, verify_build
else:
    from bundle import (export_path, make_manifest, model_root, sha256, utc_now,
                        validate_bundle, validate_id, write_json, copy_inputs)
    from build_engine import build, fixed_extractor_config, hardware_info, package_paths, verify_build


class Tee:
    def __init__(self, original, log):
        self.original, self.log = original, log

    def write(self, value):
        self.original.write(value)
        self.log.write(value)
        self.flush()
        return len(value)

    def flush(self):
        self.original.flush()
        self.log.flush()


def aliked_output_contract(outputs, *, require_static: bool) -> dict:
    expected = {'keypoints': [2048, 2], 'descriptors': [2048, 128], 'scores': [2048]}
    if len(outputs) != 3 or {value.name for value in outputs} != set(expected):
        raise ValueError('ALIKED の ONNX output 名が異なります')
    shapes = {}
    for value in outputs:
        tensor = value.type.tensor_type
        shape = [dimension.dim_value for dimension in tensor.shape.dim]
        target = expected[value.name]
        if (tensor.elem_type != 1 or len(shape) != len(target)
                or any(actual > 0 and actual != wanted for actual, wanted in zip(shape, target))):
            raise ValueError(f'ALIKED output {value.name} は FP32 / {target} が必要です: {shape}')
        if require_static and shape != target:
            raise ValueError('package モードでは TopK=2048 の静的な出力形状が必要です。aliked モードで export してください')
        shapes[value.name] = shape
    return {'output_shapes': shapes,
            'top_k_validation': 'onnx_static' if all(shapes[key] == expected[key] for key in expected)
            else 'pinned_exporter'}


def onnx_contract(path: Path, width: int | None, height: int | None, *, require_static_top_k: bool = False) -> dict:
    import onnx

    graph = onnx.load(str(path), load_external_data=True)
    onnx.checker.check_model(str(path))
    inputs = list(graph.graph.input)
    if len(inputs) != 1 or inputs[0].name != 'image':
        raise ValueError('ALIKED の ONNX input image が一つ必要です')
    tensor = inputs[0].type.tensor_type
    shape = [dimension.dim_value for dimension in tensor.shape.dim]
    if tensor.elem_type != onnx.TensorProto.FLOAT or len(shape) != 4 or shape[:2] != [1, 3]:
        raise ValueError('ALIKED は FP32 / NCHW / batch 1 / RGB input が必要です')
    actual_height, actual_width = shape[2:]
    for label, actual, requested in (('width', actual_width, width), ('height', actual_height, height)):
        if actual <= 0 and not requested:
            raise ValueError(f'dynamic ONNX は --{label} を指定してください')
        if actual > 0 and requested is not None and actual != requested:
            raise ValueError(f'ONNX の {label} は {actual} です。指定サイズと一致しません')
    actual_width = actual_width if actual_width > 0 else width
    actual_height = actual_height if actual_height > 0 else height
    outputs = [value.name for value in graph.graph.output]
    output_contract = aliked_output_contract(list(graph.graph.output), require_static=require_static_top_k)
    return {'family': 'aliked_lightglue', 'width': actual_width, 'height': actual_height,
            'top_k': 2048, 'input_shape': [1, 3, actual_height, actual_width],
            'input_dtype': 'float32', 'output_names': outputs, **output_contract}


def prepare_export(args, log_dir: Path) -> Path:
    if platform.system() != 'Linux' or platform.machine() != 'x86_64':
        raise ValueError('Notebook の Linux / x86_64 Docker 内で export してください')
    asset_id, export_id = validate_id(args.asset_id), validate_id(args.export_id)
    destination = export_path(args.result_dir, asset_id, export_id)
    if destination.exists() or destination.is_symlink():
        existing = validate_bundle(destination)
        raise ValueError('同じ export ID は上書きしません。別の ID を指定してください: ' + existing['export_id'])
    source_share = Path(args.source_dir) if args.source_dir else package_paths()[1]
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = destination.parent / ('.partial-' + export_id + '-' + uuid.uuid4().hex)
    staging.mkdir()
    source = {'package_share': str(source_share), 'mode': args.mode, 'origin': 'installed NVIDIA model assets'}
    try:
        copy_inputs(source_share, staging)
        model_path = staging / 'models/aliked_lightglue/aliked.onnx'
        if args.mode == 'aliked':
            if __package__:
                from .export_model import export_aliked
            else:
                from export_model import export_aliked
            width, height = args.width or 848, args.height or 480
            upstream = model_root(args.result_dir, asset_id) / 'source/aliked'
            # source/ もホストへ永続化される。未知の symlink 先へ書き込まない。
            if upstream.parent.is_symlink() or upstream.is_symlink():
                raise ValueError('source 保存先に symlink は使えません')
            source['replaced_onnx_sha256'] = sha256(model_path)
            model_path.unlink()  # 新規 staging 内の配布 ONNX だけを差し替える。
            source['export'] = export_aliked(model_path, upstream, width, height)
            shutil.copyfile(upstream / 'LICENSE', model_path.parent / 'LICENSE.aliked')
        else:
            width, height = args.width, args.height
        model = onnx_contract(model_path, width, height, require_static_top_k=args.mode == 'package')
        model['mode'] = args.mode
        config = staging / 'configs/isaac/keypoint_creation_config.pb.txt'
        config.write_text(fixed_extractor_config(config.read_text(), model['width'], model['height']))
        make_manifest(staging, asset_id, export_id, model, source)
        if destination.exists():
            raise ValueError('同じ export ID が別プロセスで作成されました')
        staging.rename(destination)
        print('export bundle: ' + str(destination), flush=True)
        print('manifest SHA256: ' + sha256(destination / 'manifest.json'), flush=True)
        return destination
    except BaseException:
        if staging.exists():
            staging.rename(log_dir / 'failed-export')
        raise


def doctor(args) -> int:
    failures = 0
    if args.host == 'pc':
        checks = [('Notebook', platform.system() == 'Linux' and platform.machine() == 'x86_64'),
                  ('ONNX', importlib.util.find_spec('onnx') is not None),
                  ('PyTorch', importlib.util.find_spec('torch') is not None)]
        for label, passed in checks:
            print(f'[{"OK" if passed else "FAIL"}] {label}')
            failures += not passed
        if not failures:
            import torch
            import torchvision
            if not torch.cuda.is_available():
                print('[FAIL] PyTorch CUDA: GPU が使えません')
                failures += 1
            else:
                torchvision.ops.deform_conv2d(
                    torch.zeros(1, 1, 4, 4, device='cuda'), torch.zeros(1, 18, 2, 2, device='cuda'),
                    torch.ones(1, 1, 3, 3, device='cuda'))
                torch.cuda.synchronize()
                print('[OK] CUDA / torchvision deform_conv2d')
    else:
        try:
            print(json.dumps(hardware_info(), ensure_ascii=False, indent=2))
        except (OSError, ImportError, ValueError, RuntimeError) as error:
            print('[FAIL] Jetson GPU / TensorRT: ' + str(error))
            failures += 1
    try:
        prefix, share = package_paths()
        missing = [str(prefix / 'bin/visual_mapping' / name)
                   for name in ('export_lightglue_engine', 'export_extractor_engine')
                   if not os.access(prefix / 'bin/visual_mapping' / name, os.X_OK)]
        if missing:
            raise ValueError('公式 builder がありません: ' + ', '.join(missing))
        print('[OK] isaac_ros_visual_mapping: ' + str(share))
    except ValueError as error:
        print('[FAIL] ' + str(error))
        failures += 1
    return int(bool(failures))


def run(args, log_dir: Path) -> int:
    if args.action == 'doctor':
        return doctor(args)
    if args.action == 'export':
        prepare_export(args, log_dir)
    elif args.action == 'build':
        output = build(Path(args.result_dir), args.asset_id, args.export_id, args.target_id, args.build_id, log_dir)
        write_json(log_dir / 'output.json', {'vgl_model_dir': str(output)})
    elif args.action == 'verify':
        bundle = export_path(args.result_dir, args.asset_id, args.export_id)
        manifest = validate_bundle(bundle)
        if manifest['asset_id'] != args.asset_id or manifest['export_id'] != args.export_id:
            raise ValueError('選択 ID と manifest が異なります')
        if args.build_id:
            output = model_root(args.result_dir, args.asset_id) / 'engines' / validate_id(args.target_id) / validate_id(args.build_id)
            verify_build(output, bundle)
            print('[OK] engine の入力・hash・対象 GPU と実読み込みを確認しました')
        else:
            print('[OK] export bundle の全ファイル hash・size を確認しました')
    elif args.action == 'list':
        root = Path(args.result_dir) / 'models/vgl'
        for manifest_file in sorted(root.glob('*/exported/*/manifest.json')):
            try:
                manifest = validate_bundle(manifest_file.parent)
                print(f'{manifest["asset_id"]}\t{manifest["export_id"]}\t{manifest["model"]["width"]}x{manifest["model"]["height"]}\tOK')
            except (OSError, ValueError) as error:
                print(f'{manifest_file.parent}\tFAIL: {error}')
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['doctor', 'export', 'build', 'verify', 'list'])
    parser.add_argument('--host', choices=['pc', 'jetson'],
                        default='jetson' if platform.machine() == 'aarch64' else 'pc')
    parser.add_argument('--result-dir', type=Path, default=Path('/data/results'))
    parser.add_argument('--asset-id')
    parser.add_argument('--export-id')
    parser.add_argument('--build-id')
    parser.add_argument('--target-id', default='jetson_orin_nano')
    parser.add_argument('--mode', choices=['aliked', 'package'], default='aliked')
    parser.add_argument('--width', type=int)
    parser.add_argument('--height', type=int)
    parser.add_argument('--source-dir', type=Path)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    try:
        if args.action in ('export', 'build', 'verify'):
            validate_id(args.asset_id)
            validate_id(args.export_id)
        if args.action == 'build' and not args.build_id:
            args.build_id = utc_now().replace(':', '').replace('-', '') + '-' + uuid.uuid4().hex[:8]
        if args.build_id:
            validate_id(args.build_id)
        validate_id(args.target_id)
        if not args.result_dir.is_absolute():
            raise ValueError('結果先は絶対パスで指定してください')
        if args.source_dir is not None and not args.source_dir.is_absolute():
            raise ValueError('source-dir はコンテナ内の絶対パスで指定してください')
        if any(value is not None and value <= 0 for value in (args.width, args.height)):
            raise ValueError('画像サイズは正の整数で指定してください')
        if args.dry_run:
            print('実行予定: ' + json.dumps(vars(args), ensure_ascii=False, default=str))
            return 0
        if args.action == 'export' and args.host != 'pc':
            raise ValueError('モデル export は Notebook で実行してください')
        if args.action == 'build' and args.host != 'jetson':
            raise ValueError('engine build は Jetson で実行してください')
        results = args.result_dir.resolve(strict=True)
        if not results.is_dir():
            raise ValueError('結果先はディレクトリを指定してください')
        runs = results / 'runs'
        if runs.is_symlink():
            raise ValueError('runs 保存先に symlink は使えません')
        run_id = 'vgl-' + args.action + '-' + utc_now().replace(':', '').replace('-', '') + '-' + uuid.uuid4().hex[:8]
        log_dir = runs / run_id
        log_dir.mkdir(parents=True)
        print('VGL job log: ' + str(log_dir), flush=True)
        with (log_dir / 'job.log').open('w') as log:
            with contextlib.redirect_stdout(Tee(sys.stdout, log)), contextlib.redirect_stderr(Tee(sys.stderr, log)):
                write_json(log_dir / 'job.json', {'status': 'running', 'started_at': utc_now(),
                                                 'action': args.action, 'asset_id': args.asset_id,
                                                 'export_id': args.export_id, 'build_id': args.build_id})
                previous_signals = {}
                def interrupted(number, frame):
                    raise KeyboardInterrupt('signal ' + str(number))
                try:
                    for number in (signal.SIGTERM, signal.SIGHUP):
                        previous_signals[number] = signal.signal(number, interrupted)
                    status = run(args, log_dir)
                except BaseException as error:
                    write_json(log_dir / 'result.json', {'status': 'failed', 'error': str(error), 'completed_at': utc_now()})
                    traceback.print_exc()
                    return 130 if isinstance(error, KeyboardInterrupt) else 1
                finally:
                    for number, handler in previous_signals.items():
                        signal.signal(number, handler)
                write_json(log_dir / 'result.json', {'status': 'complete' if status == 0 else 'failed',
                                                   'exit_code': status, 'completed_at': utc_now()})
                return status
    except (OSError, ValueError) as error:
        print('ATD VGL: ' + str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())

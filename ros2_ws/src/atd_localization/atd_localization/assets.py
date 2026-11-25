"""Resolve an ATD saved map and its GPU-specific localization engine."""
import ctypes
import hashlib
import json
from pathlib import Path
import platform
import re
import shutil

from atd_mapping.capture_config import CONFIG_FILES, digest, inspect_engines, require_file, tracking_workflow


def _identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}', value):
        raise ValueError('地図・モデル・build の ID が不正です')
    return value


def _path(root, relative):
    value = Path(relative)
    if value.is_absolute() or '..' in value.parts:
        raise ValueError('資産ディレクトリからの相対パスが必要です: ' + str(value))
    current = root
    for part in value.parts:
        current = current / part
        if current.is_symlink():
            raise ValueError('資産に symlink は使えません: ' + str(current))
    return current


def _object(path):
    data = require_file(path).read_bytes()
    result = json.loads(data)
    if not isinstance(result, dict):
        raise ValueError('JSON オブジェクトが必要です: ' + str(path))
    return result, hashlib.sha256(data).hexdigest()


def _sha(value):
    return isinstance(value, str) and bool(re.fullmatch('[0-9a-f]{64}', value))


def _source(results_root, map_id):
    root = Path(results_root).resolve(strict=True)
    folder = _path(root, Path('maps') / _identifier(map_id))
    doc, hdmap_hash = _object(_path(folder, 'hdmap/map.json'))
    profile, _ = _object(_path(folder, 'vgl.json'))
    source = doc.get('source', {})
    if (doc.get('id') != map_id or doc.get('frame') != 'map'
            or not isinstance(source, dict) or source.get('kind') != 'nvidia_offline'):
        raise ValueError('選択した ID と一致する map 座標系の NVIDIA 保存地図が必要です')
    if not _sha(source.get('metadata_sha256')) or not isinstance(source.get('bag'), str) or not source['bag']:
        raise ValueError('地図作成 bag の識別情報が不足しています')
    if profile.get('status') != 'complete' or not _sha(profile.get('bundle_sha256')):
        raise ValueError('この地図の VGL 準備を完了してください')
    shape = profile.get('input_shape')
    if not isinstance(shape, list) or len(shape) != 4 or shape[:2] != [1, 3] or any(type(v) is not int or v < 1 for v in shape):
        raise ValueError('VGL の入力画像サイズが不正です')
    inputs = dict(source.get('inputs', {}), ground_constraint=source.get('mapping_options', {}).get('ground_constraint'))
    workflow = tracking_workflow(inputs)
    if workflow['base_frame'] != 'base_link' or any(workflow[k] in ('base_link', 'map', 'odom') for k in ('left_frame', 'right_frame')):
        raise ValueError('車両の base_link と別々の左右 optical frame で作成した地図が必要です')
    fingerprint = profile.get('cuvslam_sha256')
    if not isinstance(fingerprint, dict) or not any(k.endswith('.mdb') and _sha(v) for k, v in fingerprint.items()):
        raise ValueError('cuVSLAM 地図の内容 hash が未記録です')
    map_hash = hashlib.sha256(json.dumps(fingerprint, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    # If an HDMap background is published, it must have been captured in this map.
    if source.get('capture_relative') or source.get('pointcloud_status') == 'ready':
        relative = source.get('capture_relative')
        if not isinstance(relative, str) or not relative.startswith('hdmap/captures/'):
            raise ValueError('HDMap 点群の出典情報がありません')
        capture, _ = _object(_path(folder, Path(relative) / 'manifest.json'))
        if (capture.get('schema') != 'atd.capture.v1' or capture.get('map_id') != map_id
                or capture.get('source_map_sha256') != fingerprint
                or capture.get('bag_metadata_sha256') != source['metadata_sha256']
                or not profile.get('transaction_id') or capture.get('vgl_transaction') != profile['transaction_id']):
            raise ValueError('HDMap 点群と選択した cuVSLAM・VGL 地図の出典が一致しません')
    metadata = dict(map_id=map_id, map_sha256=map_hash, hdmap_sha256=hdmap_hash, workflow=workflow,
                    input_shape=shape, source_bag=source['bag'], source_metadata_sha256=source['metadata_sha256'])
    return root, folder, metadata, profile


def map_metadata(results_root, map_id):
    """Read the persisted identities and stereo settings, without using CUDA."""
    from .orb_assets import is_orb_map, orb_map_metadata
    if is_orb_map(results_root, map_id):
        return orb_map_metadata(results_root, map_id)
    return _source(results_root, map_id)[2]


def verified_hdmap(results_root, metadata):
    """Read precisely the HDMap identified by a localization session."""
    folder = _path(Path(results_root), Path('maps') / _identifier(metadata['map_id']))
    data = require_file(_path(folder, 'hdmap/map.json')).read_bytes()
    if hashlib.sha256(data).hexdigest() != metadata['hdmap_sha256']:
        raise ValueError('HDMap が変更されました。地図を再選択してください')
    return data


def save_validation_hdmap(results_root, metadata, run_dir):
    """Keep the exact reference lines and boundaries used by this validation run."""
    data = verified_hdmap(results_root, metadata)
    with (run_dir / 'hdmap.json').open('xb') as stream:
        stream.write(data)


def _verify_tree(directory, expected):
    if not directory.is_dir() or not isinstance(expected, dict) or not expected:
        raise ValueError('地図・設定の記録 hash がありません: ' + str(directory))
    actual = {}
    for item in directory.rglob('*'):
        key = item.relative_to(directory).as_posix()
        if item.is_symlink() or not (item.is_dir() or item.is_file()):
            raise ValueError('地図・設定にリンクや特殊ファイルは使えません: ' + str(item))
        if item.is_dir():
            actual[key + '/'] = 'directory'
        else:
            actual[key] = digest(item)
    if actual != expected:
        raise ValueError('VGL 作成時の内容 hash と一致しません: ' + str(directory))


def gpu_signature():
    """Identity recorded by the Notebook and Jetson builders for CUDA device 0."""
    import tensorrt
    cuda = ctypes.CDLL('libcuda.so.1')
    device = ctypes.c_int()
    uuid = ctypes.create_string_buffer(16)
    for name, args in [('cuInit', (0,)), ('cuDeviceGet', (ctypes.byref(device), 0)),
                       ('cuDeviceGetUuid', (uuid, device))]:
        if getattr(cuda, name)(*args):
            raise ValueError('CUDA GPU を識別できません: ' + name)
    return dict(gpu_uuid=uuid.raw.hex(), tensorrt_version=tensorrt.__version__, architecture=platform.machine())


def _resolve_engine(root, profile, selected, hardware):
    asset, export = _identifier(profile.get('asset_id')), _identifier(profile.get('export_id'))
    model = _path(root, Path('models/vgl') / asset)
    bundle = _path(model, Path('exported') / export)
    manifest, bundle_hash = _object(_path(bundle, 'manifest.json'))
    model_data = manifest.get('model', {})
    if (manifest.get('schema_version') != 1 or manifest.get('asset_id') != asset or manifest.get('export_id') != export
            or manifest.get('kind') != 'atd.vgl.export' or bundle_hash != profile['bundle_sha256']
            or profile['input_shape'] != [1, 3, model_data.get('height'), model_data.get('width')]):
        raise ValueError('VGL 地図と転送済みのモデル export・入力サイズが一致しません')
    builds = _path(model, 'engines')
    if selected:
        chosen = _path(root, selected)
        if chosen.parent.parent != builds:
            raise ValueError('対応する models/vgl/<asset>/engines/<target>/<build> を指定してください')
        candidates = [chosen / 'build.json']
    else:
        candidates = sorted(builds.glob('*/*/build.json'))
    matches = []
    for candidate in candidates:
        if any(part.startswith('.') for part in candidate.relative_to(builds).parts):
            continue
        report, _ = _object(_path(builds, candidate.relative_to(builds)))
        if (report.get('schema_version') == 1 and report.get('status') == 'complete'
                and report.get('asset_id') == asset and report.get('export_id') == export
                and report.get('bundle_sha256') == bundle_hash and report.get('build_id') == candidate.parent.name
                and report.get('target_id') == candidate.parent.parent.name
                and all(report.get('hardware', {}).get(k) == v for k, v in hardware.items())):
            matches.append((candidate.parent, report))
    if not matches:
        raise ValueError('この GPU・TensorRT・地図に対応する engine がありません。対象機で VGL engine を準備してください')
    if len(matches) > 1:
        # The map records its Notebook build. Compare only with compatible local builds
        # so a transferred map can still use the Jetson's own engine.
        recorded = profile.get('runtime_model_dir')
        preferred = [match for match in matches if match[0].relative_to(root).as_posix() == recorded]
        if not preferred:
            choices = ', '.join(str(path.relative_to(root)) for path, _ in matches)
            raise ValueError('候補が複数あります。engine_dir で build を選択してください: ' + choices)
        matches = preferred
    runtime, report = matches[0]
    engine_folder = _path(runtime, 'aliked_lightglue')
    onnx = require_file(_path(engine_folder, 'aliked.onnx'))
    if digest(onnx) != manifest.get('files', {}).get('models/aliked_lightglue/aliked.onnx', {}).get('sha256'):
        raise ValueError('engine に付属する ALIKED ONNX の hash が一致しません')
    binaries = {}
    for kind, pattern in [('extractor', 'aliked_*.engine'), ('matcher', 'lightglue_aliked_*.engine')]:
        found = list(engine_folder.glob(pattern))
        if len(found) != 1 or digest(require_file(found[0])) != report.get('engines', {}).get(kind, {}).get('sha256'):
            raise ValueError('build に記録した engine の内容と一致しません: ' + kind)
        binaries[kind] = found[0]
    return runtime, binaries


def prepare_runtime(results_root, map_id, engine_dir, working_dir, hardware=None, inspect=inspect_engines):
    root, folder, metadata, profile = _source(results_root, map_id)
    nvidia = _path(folder, 'nvidia')
    slam = _path(nvidia, 'cuvslam_map')
    vgl = _path(nvidia, 'cuvgl_map')
    config = _path(folder, 'vgl_runtime_config')
    if digest(require_file(_path(nvidia, 'map_frames/rectified/frames_meta.json'))) != profile.get('frames_meta_sha256'):
        raise ValueError('VGL 地図の作成元の保存画像・姿勢情報が一致しません')
    _verify_tree(vgl, profile.get('cuvgl_sha256'))
    _verify_tree(config, profile.get('config_sha256'))
    for relative in ('keyframes/frames_meta.json', 'bow_index.pb'):
        require_file(vgl / relative)
    if not any(p.is_file() and p.stat().st_size for p in (vgl / 'vocabulary').rglob('*')):
        raise ValueError('VGL vocabulary が空です')
    for name in CONFIG_FILES:
        require_file(config / name)
    runtime, engines = _resolve_engine(root, profile, engine_dir, gpu_signature() if hardware is None else hardware)
    inspect(engines, metadata['input_shape'])
    # LMDB may create lock files when opened. Only the private copy is passed to cuVSLAM.
    private_slam = Path(working_dir) / 'cuvslam_map'
    shutil.copytree(slam, private_slam, symlinks=True)
    _verify_tree(private_slam, profile['cuvslam_sha256'])
    databases = list(private_slam.glob('*.mdb'))
    if not databases:
        raise ValueError('cuVSLAM 地図に .mdb ファイルがありません')
    for database in databases:
        require_file(database)
    return dict(**metadata, map_dir=private_slam, map_root=nvidia, config_dir=config, model_dir=runtime)

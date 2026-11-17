"""保存地図の作業コピーから点群を取得し、map.json を最後に更新する。"""
from __future__ import annotations

from functools import lru_cache
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import uuid

from atd_processing.vgl_transaction import fingerprint
from atd_processing.vgl_assets import validate_runtime_layout
from .store import atomic_json, confined, read_json


DOMAIN = 94
DEFAULTS = dict(replay_rate=1.0, ready_timeout_s=120.0, drain_timeout_s=30.0,
                settle_s=3.0, tail_tolerance_s=0.5)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


@lru_cache(maxsize=8)
def _collector_probe(interval):
    if not shutil.which('ros2'):
        return False
    script = ('set -e; source /opt/ros/lyrical/setup.bash; '
              'if [ -n "${ISAAC_ROS_WS:-}" ] && [ -f "$ISAAC_ROS_WS/install/setup.bash" ]; '
              'then source "$ISAAC_ROS_WS/install/setup.bash"; fi; '
              'exec ros2 pkg prefix atd_mapping')
    try:
        return subprocess.run(['bash', '-c', script], stdin=subprocess.DEVNULL,
                              capture_output=True, timeout=5).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def collector_available():
    return _collector_probe(int(time.monotonic() / 5))


def ready_document(document):
    return document.get('source', {}).get('kind') != 'nvidia_offline' or document.get('source', {}).get('pointcloud_status') == 'ready'


def _assets(store, folder, profile, model_relative):
    """ROS-free asset validation; GPU / TensorRT loading is repeated by the collector."""
    if profile.get('status') != 'complete':
        raise ValueError('VGL 地図の準備が完了していません')
    shape = profile.get('input_shape')
    if (not isinstance(shape, list) or len(shape) != 4 or shape[:2] != [1, 3]
            or any(type(v) is not int or v <= 0 for v in shape)):
        raise ValueError('VGL 入力画像サイズを確認できません')
    map_hash = fingerprint(confined(folder, 'nvidia/cuvslam_map'))
    if not map_hash or map_hash != profile.get('cuvslam_sha256'):
        raise ValueError('VGL と元の cuVSLAM 地図の対応を確認できません。対応情報を記録した地図を再準備してください')
    frames = confined(folder, 'nvidia/map_frames/rectified/frames_meta.json')
    if digest(frames) != profile.get('frames_meta_sha256'):
        raise ValueError('VGL 地図と保存画像・pose の対応が一致しません')
    for relative, field in (('nvidia/cuvgl_map', 'cuvgl_sha256'), ('vgl_runtime_config', 'config_sha256')):
        if fingerprint(confined(folder, relative)) != profile.get(field):
            raise ValueError('VGL 地図または実行設定が変更されています: ' + relative)
    models = confined(store.result_root, model_relative)
    build = read_json(confined(models, 'build.json'))
    bundle = confined(store.result_root, profile['model_export'])
    repo = Path(__file__).resolve().parents[5]
    if str(repo) not in sys.path:
        sys.path.insert(0, str(repo))
    from tools.vgl.bundle import validate_bundle
    manifest = validate_bundle(bundle)
    if (build.get('status') != 'complete' or build.get('asset_id') != profile.get('asset_id')
            or build.get('export_id') != profile.get('export_id')
            or build.get('bundle_sha256') != profile.get('bundle_sha256')
            or digest(bundle / 'manifest.json') != profile.get('bundle_sha256')
            or shape != [1, 3, manifest['model']['height'], manifest['model']['width']]):
        raise ValueError('選択した engine と VGL 地図のモデル・画像サイズが一致しません')
    layout = validate_runtime_layout(folder / 'nvidia/cuvgl_map', folder / 'vgl_runtime_config', models,
        manifest['files']['models/aliked_lightglue/aliked.onnx']['sha256'])
    for kind in ('extractor', 'matcher'):
        expected = build.get('engines', {}).get(kind, {}).get('sha256')
        if not expected or digest(layout['engines'][kind]) != expected:
            raise ValueError('engine の内容 hash が一致しません: ' + kind)
    return dict(input_shape=shape, map_sha256=map_hash, model_dir=str(models),
                config_sha256=profile['config_sha256'], cuvgl_sha256=profile['cuvgl_sha256'])


def preflight(store, key, bag=None, model_dir=None):
    folder, document = store.folder(key), store.load_map(key)
    source = document.get('source', {})
    result = dict(ready=False, issues=[], revision=document['revision'], source_kind=source.get('kind', ''),
                  pointcloud_status=source.get('pointcloud_status', 'unavailable'),
                  source_bag=source.get('bag', ''), bag=bag if bag is not None else source.get('bag', ''),
                  model_dir=model_dir or '', vgl_ready=False,
                  capabilities=dict(ros=bool(shutil.which('ros2')), collector=collector_available()))
    if source.get('kind') != 'nvidia_offline':
        result['issues'].append('rosbag から作成した NVIDIA 保存地図を選択してください')
    if source.get('pointcloud_status') == 'ready' or source.get('capture_relative'):
        result['issues'].append('既に点群を取得しています。編集座標を保持するため再取得は行いません')
    try:
        bag_path = confined(store.record_root, result['bag'])
        if not source.get('metadata_sha256') or digest(confined(bag_path, 'metadata.yaml')) != source['metadata_sha256']:
            raise ValueError('地図生成時と同じ rosbag を選択してください（metadata hash 不一致）')
        inputs = source.get('inputs', {})
        required = ('left_image', 'right_image', 'left_info', 'right_info', 'base_frame', 'left_frame', 'right_frame')
        if any(not isinstance(inputs.get(k), str) or not inputs[k] for k in required):
            raise ValueError('地図生成時の topic / frame 設定がありません')
        if type(source.get('mapping_options', {}).get('ground_constraint')) is not bool:
            raise ValueError('地図生成時の平面拘束設定が未記録です。設定を記録する現行の地図作成から再準備してください')
    except (ValueError, OSError, KeyError) as error:
        result['issues'].append(str(error))
    try:
        profile = read_json(confined(folder, 'vgl.json'))
        result['model_dir'] = model_dir if model_dir is not None else profile.get('runtime_model_dir', '')
        result.update(_assets(store, folder, profile, result['model_dir']))
        # API は相対パスを提示し、ROS job への変換時だけ絶対パスを使う。
        result['model_dir'] = model_dir if model_dir is not None else profile.get('runtime_model_dir', '')
        result['vgl_ready'] = True
    except (ValueError, OSError, KeyError, TypeError) as error:
        result['issues'].append('VGL 地図・Notebook 用モデルを先に準備してください: ' + str(error))
    if not result['capabilities']['ros']:
        result['issues'].append('Notebook の ROS / CUDA Docker 内で実行してください')
    if not result['capabilities']['collector']:
        result['issues'].append('atd_mapping が未buildです。Notebook の ROS workspace を build してください')
    result['ready'] = not result['issues']
    return result


def payload(store, key, body):
    result = preflight(store, key, body.get('bag'), body.get('model_dir'))
    if type(body.get('revision')) is not int or body['revision'] != result['revision']:
        raise ValueError('地図 revision が変わりました。再読み込みしてください')
    if not result['ready']:
        raise ValueError('\n'.join(result['issues']))
    return dict(id=key, revision=result['revision'], bag=result['bag'], model_dir=result['model_dir'],
                map_sha256=result['map_sha256'])


def normalize(snapshot, key):
    provenance = snapshot.get('provenance', {})
    if (snapshot.get('schema') != 'atd.snapshot.v1' or snapshot.get('frame') != 'map'
            or provenance.get('localized_in_existing_map') is not True
            or provenance.get('map_id') != key):
        raise ValueError('保存地図への localize 成功と map 座標を確認できません')
    points, trajectory = snapshot.get('points'), snapshot.get('trajectory', [])
    if not isinstance(points, list) or not 1 <= len(points) <= 5_000_000:
        raise ValueError('有効な点群がありません（1〜500万点）')
    for group in (points, trajectory):
        if not isinstance(group, list):
            raise ValueError('点群・軌跡の形式が不正です')
        for point in group:
            if (not isinstance(point, (list, tuple)) or len(point) != 3
                    or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 1e6 for v in point)):
                raise ValueError('点群・軌跡に不正な map 座標があります')
    stride = max(1, math.ceil(len(points) / 250000))
    return dict(frame='map', points=points[::stride], trajectory=trajectory,
                source_count=len(points), display_stride=stride, provenance=provenance)


def _copy_tree(source, target, check):
    fingerprint(source)  # source 内のリンク・特殊ファイルを拒否する。
    def copy_file(src, dst):
        with open(src, 'rb') as reader, open(dst, 'wb') as writer:
            while block := reader.read(1024 * 1024):
                check()
                writer.write(block)
        return dst
    shutil.copytree(source, target, copy_function=copy_file)


def capture(store, task, request):
    request = payload(store, request['id'], request)
    key, folder = request['id'], store.folder(request['id'])
    before = store.load_map(key)
    source = before['source']
    bag = confined(store.record_root, request['bag'])
    profile = read_json(confined(folder, 'vgl.json'))
    models = confined(store.result_root, request['model_dir'])
    work = Path(tempfile.mkdtemp(prefix='.capture-', dir=store.maps_root))
    published = None
    stage = None
    committed = False
    try:
        task.log('元の保存地図を保護するため、点群取得用コピーを準備しています')
        for relative in ('nvidia/cuvslam_map', 'nvidia/cuvgl_map', 'vgl_runtime_config'):
            target = work / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            _copy_tree(confined(folder, relative), target, task.check)
        if fingerprint(work / 'nvidia/cuvslam_map') != request['map_sha256']:
            raise ValueError('点群取得用コピーの地図 hash が一致しません')
        workflow = dict(source['inputs'], tf_static='/tf_static', ros_domain_id=DOMAIN,
                        ground_constraint=source['mapping_options']['ground_constraint'])
        job = dict(map_id=key, map_dir=str(work / 'nvidia/cuvslam_map'), bag=str(bag), output=str(work),
                   workflow=workflow, capture=DEFAULTS,
                   vgl=dict(map_root=str(work / 'nvidia'), model_dir=str(models),
                            config_dir=str(work / 'vgl_runtime_config'), input_shape=profile['input_shape']))
        job_file = work / 'capture_job.json'
        atomic_json(job_file, job)
        task.run(['ros2', 'run', 'atd_mapping', 'capture_snapshot', '--job', str(job_file)],
                 env=dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'))
        task.check()
        snapshot_path = confined(work, 'snapshot.json')
        snapshot = read_json(snapshot_path)
        cloud = normalize(snapshot, key)
        # 外部変更や再生中の bag 差替えを成果物に取り込まない。
        if (digest(bag / 'metadata.yaml') != source['metadata_sha256']
                or fingerprint(folder / 'nvidia/cuvslam_map') != request['map_sha256']):
            raise ValueError('点群取得中に元 bag または保存地図が変更されました')
        captures = folder / 'hdmap/captures'
        captures.mkdir(parents=True, exist_ok=True)
        stage = Path(tempfile.mkdtemp(prefix='.partial-', dir=captures))
        atomic_json(stage / 'snapshot.json', snapshot)
        atomic_json(stage / 'cloud.json', cloud)
        atomic_json(stage / 'manifest.json', dict(schema='atd.capture.v1', map_id=key,
            source_map_sha256=request['map_sha256'], bag_metadata_sha256=source['metadata_sha256'],
            vgl_transaction=profile.get('transaction_id'), model_dir=request['model_dir'], created=time.time()))
        task.check()
        with store.lock('map-' + key):
            document = store.load_map(key)
            if document['revision'] != request['revision'] or document.get('source', {}).get('pointcloud_status') == 'ready':
                raise ValueError('点群取得中に地図の状態が変わりました')
            published = captures / uuid.uuid4().hex
            os.rename(stage, published)
            relative = published.relative_to(folder).as_posix()
            document['source'].update(pointcloud_status='ready', capture_relative=relative,
                                      point_count=cloud['source_count'])
            document.update(revision=document['revision'] + 1, updated=time.time())
            task.check()
            atomic_json(folder / 'hdmap/map.json', document)
            committed = True
        return dict(map=key, revision=document['revision'], pointcloud_ready=True, point_count=cloud['source_count'])
    finally:
        # marker 公開後の成果物は保持する。未公開 staging も入力ではなく再生成可能な作業コピーだけ。
        if published is not None and not committed:
            current = store.load_map(key)
            if current.get('source', {}).get('capture_relative') != published.relative_to(folder).as_posix():
                shutil.rmtree(published)
        if stage is not None and stage.exists():
            shutil.rmtree(stage)
        shutil.rmtree(work)

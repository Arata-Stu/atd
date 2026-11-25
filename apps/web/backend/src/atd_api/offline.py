"""地図作成をバックエンドの CLI に委譲し、完成物を検査する。"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile
import time

from .jobs import Cancelled, ros_program
from .store import atomic_json, confined, identifier, read_json
from .transfers import inventory


TOPIC = re.compile(r'/[A-Za-z_][A-Za-z_0-9/]*$')
FRAME = re.compile(r'[A-Za-z_][A-Za-z_0-9/]*$')
ODOM_POSES = Path('nvidia/poses/odom_poses.tum')
OFFLINE_SLAM_MAX_MAP_SIZE = 0
MAP_DOMAIN = 93
ORB_BUILD_HINT = ('ORB-SLAM3 対応の Notebook 環境で atd-build --packages-up-to atd_orbslam3 atd_bringup '
                  '--cmake-args -DATD_REQUIRE_ORBSLAM3=ON を実行してください')
DEFAULT_INPUTS = dict(left_image='/camera/camera/infra1/image_rect_raw', right_image='/camera/camera/infra2/image_rect_raw',
    left_info='/camera/camera/infra1/camera_info', right_info='/camera/camera/infra2/camera_info',
    base_frame='base_link', left_frame='camera_infra1_optical_frame', right_frame='camera_infra2_optical_frame')


def build_payload(store, body):
    key = identifier(body['id'])
    if (store.maps_root / key).exists():
        raise ValueError('同名の地図があります')
    bag = confined(store.record_root, body['bag'])
    metadata = confined(bag, 'metadata.yaml', exists=False)
    if not metadata.is_file():
        raise ValueError('metadata.yaml のある bag を選択してください')
    inputs = dict(DEFAULT_INPUTS, **{k: body[k] for k in DEFAULT_INPUTS if k in body})
    backend = body.get('backend', 'cuvslam')
    if backend == 'orbslam3':
        if inputs != DEFAULT_INPUTS:
            raise ValueError('ORB-SLAM3 の地図作成には D455 標準のステレオ topic・frame を使います')
        rate = body.get('rate', .5)
        if type(rate) not in (int, float) or not math.isfinite(rate) or not .05 <= rate <= 4.:
            raise ValueError('ORB-SLAM3 の再生倍率は 0.05〜4 で指定してください')
        if ros_program('atd_orbslam3', 'orbslam3_node') is None:
            raise ValueError(ORB_BUILD_HINT)
        sha = hashlib.sha256(metadata.read_bytes()).hexdigest()
        if body.get('metadata_sha256', sha) != sha:
            raise ValueError('要求後に bag の metadata.yaml が更新されました。記録を選び直してください')
        return dict(id=key, bag=bag.relative_to(store.record_root).as_posix(), backend=backend, rate=rate,
                    metadata_sha256=sha)
    if backend != 'cuvslam':
        raise ValueError('未知の地図作成バックエンドです')
    if 'rate' in body:
        raise ValueError('再生倍率を指定できる地図作成バックエンドは ORB-SLAM3 です')
    for name, value in inputs.items():
        if not (TOPIC if name in ('left_image', 'right_image', 'left_info', 'right_info') else FRAME).fullmatch(value):
            raise ValueError('topic または frame が不正です: ' + name)
    if not (shutil.which('ros2') or os.access('/opt/ros/lyrical/bin/ros2', os.X_OK)):
        raise ValueError('ROS 2 がありません。Notebook の ROS/CUDA コンテナ内で実行してください')
    return dict(id=key, bag=body['bag'], **inputs)


def blank(key, title=None):
    return dict(schema='atd.hdmap.v1', id=key, title=title or key, revision=1, frame='map', closed=True,
                left=[], right=[], custom=[], lines={}, source={}, updated=time.time())


def map_build(store, task, payload, *, validate_source=None):
    payload = build_payload(store, payload)
    if payload.get('backend') == 'orbslam3':
        return _orb_map_build(store, task, payload)
    key = payload['id']
    store.maps_root.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.build-', dir=store.maps_root))
    try:
        bag = confined(store.record_root, payload['bag'])
        config = stage / 'camera_topics.json'
        atomic_json(config, dict(stereo_cameras=[dict(name='realsense', left=payload['left_image'], right=payload['right_image'],
            left_camera_info=payload['left_info'], right_camera_info=payload['right_info'],
            left_frame_id=payload['left_frame'], right_frame_id=payload['right_frame'])]))
        nvidia = stage / 'nvidia'
        nvidia.mkdir()
        command = ['ros2', 'run', 'isaac_mapping_ros', 'create_map_offline.py', f'--sensor_data_bag={bag}',
            f'--base_output_folder={nvidia}', f'--camera_topic_config={config}', f"--base_link_name={payload['base_frame']}",
            '--use_raw_image=False', '--use_cusfm=False', '--use_cuvslam_opencv_compat=True',
            '--override', 'cuvslam.cfg_planar=false',
            '--override', f'cuvslam.cfg_slam_max_map_size={OFFLINE_SLAM_MAX_MAP_SIZE}',
            '--steps_to_run', 'edex', 'compute_poses']
        task.run(command, env=dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_DOMAIN_ID=str(MAP_DOMAIN)))
        candidates = sorted({p.resolve() for p in nvidia.rglob('cuvslam_map') if p.is_dir()})
        if len(candidates) != 1 or not candidates[0].is_relative_to(nvidia):
            raise ValueError('公式出力内の cuvslam_map 実体を一意に確認できません')
        if not any(p.stat().st_size for p in candidates[0].glob('*.mdb')):
            raise ValueError('非空の cuVSLAM .mdb 地図がありません')
        # NVIDIA の run 内部の名称を維持し、ATD が保持する map_dir を nvidia/ に揃える。
        run = candidates[0].parent
        if run != nvidia:
            holder = stage / '.nvidia-run'
            shutil.move(str(run), holder)
            shutil.rmtree(nvidia)
            os.rename(holder, nvidia)
        files = inventory(nvidia, task.check)
        task.check()
        document = blank(key)
        document['source'] = dict(kind='nvidia_offline', bag=payload['bag'], frame='map', pointcloud_status='pending',
                                 metadata_sha256=hashlib.sha256((bag / 'metadata.yaml').read_bytes()).hexdigest(), inputs=payload,
                                 mapping_options=dict(ground_constraint=False, slam_max_map_size=OFFLINE_SLAM_MAX_MAP_SIZE))
        # 標準 workflow の連続 VO は、指定した rig 基準の姿勢を ROS 軸で保存する。
        poses = files.get(ODOM_POSES.relative_to('nvidia').as_posix())
        if poses and poses['size'] > 0:
            document['source']['odom_poses'] = dict(
                path=ODOM_POSES.as_posix(), sha256=poses['sha256'], frame=payload['base_frame'])
        if validate_source:
            validate_source(document['source'])
        atomic_json(stage / 'hdmap/map.json', document)
        atomic_json(stage / 'mapping.json', dict(command=command, source=document['source'], completed=time.time()))
        target = confined(store.maps_root, key, exists=False)
        if target.exists():
            raise ValueError('同名の地図が作成されています')
        os.rename(stage, target)
        return dict(map=key, cuvslam_ready=True, pointcloud_ready=False)
    except BaseException:
        # 失敗時もログは runs/web/jobs に残る。未公開の公式診断データも保持する。
        if stage.exists():
            failed = store.result_root / 'runs' / ('map-failed-' + task.key)
            if not failed.exists():
                shutil.move(str(stage), failed)
                task.log('未完成の診断データ: ' + str(failed))
        raise


def _orb_map_build(store, task, payload):
    key = payload['id']
    replay = ros_program('atd_bringup', 'replay_vslam')
    if replay is None:
        raise ValueError(ORB_BUILD_HINT)
    bag = confined(store.record_root, payload['bag'])
    command = [str(replay), '--backend', 'orbslam3', '--save-map', key, '--tracking', 'vo', '--headless',
               '--rate', str(payload['rate']), '--', str(bag)]
    env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_DOMAIN_ID=str(MAP_DOMAIN),
               ATD_REPLAY_RECORD_ROOT=str(store.record_root), ATD_REPLAY_RESULTS_ROOT=str(store.result_root),
               FASTDDS_DEFAULT_PROFILES_FILE=str(Path(__file__).resolve().parents[5] / 'docker/gazebo_monza/fastdds.xml'))
    try:
        # Signal the replay supervisor directly so it can stop both ROS processes and discard its staging directory.
        task.run(command, env=env, ros=True)
    except Cancelled as exc:
        if (store.maps_root / key).exists():
            raise Cancelled(f'中止要求を受けました。地図 {key} は公開済みです') from exc
        raise
    reference = store.live_reference(key)
    doc = reference['document']
    source = doc.get('source', {})
    if (doc.get('id') != key or doc.get('frame') != 'map' or source.get('kind') != 'orbslam3'
            or source.get('bag') != payload['bag'] or source.get('metadata_sha256') != payload['metadata_sha256']
            or source.get('pointcloud_status') != 'ready' or not reference['map_sha256']
            or not confined(store.folder(key), 'hdmap/cloud.json').is_file()):
        raise ValueError('公開された ORB-SLAM3 地図・表示点群と作成要求の対応を確認できません')
    return dict(map=key, orbslam3_ready=True, pointcloud_ready=True)


def _vgl_bundle():
    repo = str(Path(__file__).resolve().parents[5])
    if repo not in sys.path:
        sys.path.insert(0, repo)
    from tools.vgl import bundle
    return bundle


def vgl_exports(store):
    items = []
    for path, manifest, error in _vgl_bundle().list_exports(store.result_root):
        if error is None:
            items.append(dict(asset_id=manifest['asset_id'], export_id=manifest['export_id'],
                              width=manifest['model']['width'], height=manifest['model']['height'],
                              model_dir=path.relative_to(store.result_root).as_posix()))
    return dict(items=items, result_root=str(store.result_root))


def vgl_payload(store, key, body):
    identifier(key)
    folder = confined(store.maps_root, key)
    models = confined(store.result_root, body['model_dir'])
    frames = folder / 'nvidia/map_frames/rectified'
    if not (frames / 'frames_meta.json').is_file():
        raise ValueError('公式保存画像 map_frames/rectified/frames_meta.json がありません')
    from atd_processing.vgl_transaction import check_preparable
    check_preparable(folder)
    if not shutil.which('ros2'):
        raise ValueError('ROS 2 / GPU のある Notebook 環境で実行してください')
    _vgl_bundle().validate_bundle(models)
    return dict(id=key, model_dir=body['model_dir'])


def vgl_build(store, task, payload):
    payload = vgl_payload(store, payload['id'], payload)
    repo = Path(__file__).resolve().parents[5]
    helper = repo / 'apps/web/processing/atd_processing/vgl_prepare.py'
    try:
        task.run(['/usr/bin/python3', str(helper), str(store.result_root), payload['id'], payload['model_dir']], ros=True)
    except BaseException:
        # 取消で helper が途中終了しても、自ジョブの未完了公開を戻して再試行可能にする。
        from atd_processing.vgl_transaction import recover
        recover(store.folder(payload['id']))
        raise
    profile = read_json(store.folder(payload['id']) / 'vgl.json')
    if profile.get('status') != 'complete':
        raise ValueError('完成した VGL 地図を確認できません')
    return dict(map=payload['id'], vgl_ready=True, runtime_model_dir=profile['runtime_model_dir'])

"""ROS/GPU 処理は固定された公式 CLI に委譲し、完成物を検査する。"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile
import time

from .store import atomic_json, confined, identifier, read_json
from .transfers import inventory


TOPIC = re.compile(r'/[A-Za-z_][A-Za-z_0-9/]*$')
FRAME = re.compile(r'[A-Za-z_][A-Za-z_0-9/]*$')
ODOM_POSES = Path('nvidia/poses/odom_poses.tum')
DEFAULT_INPUTS = dict(left_image='/camera/camera/infra1/image_rect_raw', right_image='/camera/camera/infra2/image_rect_raw',
    left_info='/camera/camera/infra1/camera_info', right_info='/camera/camera/infra2/camera_info',
    base_frame='base_link', left_frame='camera_infra1_optical_frame', right_frame='camera_infra2_optical_frame')


def build_payload(store, body):
    key = identifier(body['id'])
    if (store.maps_root / key).exists():
        raise ValueError('同名の地図があります')
    bag = confined(store.record_root, body['bag'])
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError('metadata.yaml のある bag を選択してください')
    inputs = dict(DEFAULT_INPUTS, **{k: body[k] for k in DEFAULT_INPUTS if k in body})
    for name, value in inputs.items():
        if not (TOPIC if name in ('left_image', 'right_image', 'left_info', 'right_info') else FRAME).fullmatch(value):
            raise ValueError('topic または frame が不正です: ' + name)
    if not shutil.which('ros2'):
        raise ValueError('ROS 2 がありません。Notebook の ROS/CUDA コンテナ内で実行してください')
    return dict(id=key, bag=body['bag'], **inputs)


def blank(key, title=None):
    return dict(schema='atd.hdmap.v1', id=key, title=title or key, revision=1, frame='map', closed=True,
                left=[], right=[], custom=[], lines={}, source={}, updated=time.time())


def map_build(store, task, payload):
    payload = build_payload(store, payload)
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
            '--override', 'cuvslam.cfg_planar=false', '--steps_to_run', 'edex', 'compute_poses']
        task.run(command, env=dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_DOMAIN_ID='93'))
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
                                 mapping_options=dict(ground_constraint=False))
        # 標準 workflow の連続 VO は、指定した rig 基準の姿勢を ROS 軸で保存する。
        poses = files.get(ODOM_POSES.relative_to('nvidia').as_posix())
        if poses and poses['size'] > 0:
            document['source']['odom_poses'] = dict(
                path=ODOM_POSES.as_posix(), sha256=poses['sha256'], frame=payload['base_frame'])
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
    repo = Path(__file__).resolve().parents[5]
    sys.path.insert(0, str(repo))
    from tools.vgl.bundle import validate_bundle
    validate_bundle(models)
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

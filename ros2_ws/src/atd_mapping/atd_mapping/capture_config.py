"""保存地図を使う共通のステレオ設定と、点群取得用の検証・起動設定。"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re

import yaml

CONFIG_FILES = ('keypoint_creation_config.pb.txt', 'localizer_config.pb.txt',
                'matching_config.pb.txt', 'query_map_config.pb.txt')
DEFAULT_CAPTURE = dict(replay_rate=1.0, ready_timeout_s=120.0, drain_timeout_s=30.0,
                       settle_s=3.0, tail_tolerance_s=0.5)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def require_file(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or not path.stat().st_size:
        raise ValueError('非空の通常ファイルが必要です: ' + str(path))
    return path


def inspect_engines(engines, shape):
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.WARNING)
    trt.init_libnvinfer_plugins(logger, '')
    runtime = trt.Runtime(logger)
    for kind, path in engines.items():
        engine = runtime.deserialize_cuda_engine(path.read_bytes())
        if engine is None:
            raise ValueError('この GPU / TensorRT で engine を読めません: ' + str(path))
        try:
            if kind == 'extractor' and list(engine.get_tensor_shape('image')) != shape:
                raise ValueError('ALIKED engine と地図の入力 shape が一致しません')
        finally:
            del engine


def tracking_workflow(inputs):
    workflow = dict(inputs)
    required = ('left_image', 'right_image', 'left_info', 'right_info', 'base_frame', 'left_frame', 'right_frame')
    for key in required:
        expression = r'/[A-Za-z_][A-Za-z0-9_/]*' if key in required[:4] else r'[A-Za-z_][A-Za-z0-9_/]*'
        if (not isinstance(workflow.get(key), str) or not re.fullmatch(expression, workflow[key])
                or workflow[key].endswith('/') or '//' in workflow[key]):
            raise ValueError('topic / frame が不正です: ' + key)
    if workflow['left_frame'] == workflow['right_frame']:
        raise ValueError('左右 optical frame は別々に指定してください')
    if len({workflow[k] for k in required[:4]}) != 4:
        raise ValueError('左右画像・CameraInfo topic は別々に指定してください')
    workflow.setdefault('tf_static', '/tf_static')
    if workflow['tf_static'] != '/tf_static':
        raise ValueError('車体とステレオカメラを結ぶ /tf_static の入力が必要です')
    if type(workflow.get('ground_constraint')) is not bool:
        raise ValueError('地図作成時の ground_constraint を boolean で明示してください')
    return workflow


def component_configuration(config_dir, workflow, map_dir, map_root, model_dir, runtime_config, *,
                            namespace, use_sim_time, capture_visualization):
    """NVIDIA stereo tracking parameters and remaps shared by live localization and capture."""
    slam = yaml.safe_load((config_dir / 'capture_cuvslam.yaml').read_text())['/**/visual_slam']['ros__parameters']
    global_ = yaml.safe_load((config_dir / 'capture_vgl.yaml').read_text())['/**/visual_global_localization']['ros__parameters']
    common = dict(use_sim_time=use_sim_time, base_frame=workflow['base_frame'], map_frame='map', odom_frame='odom',
                  camera_optical_frames=[workflow['left_frame'], workflow['right_frame']])
    slam.update(common, tracking_mode=0, load_map_folder_path=str(map_dir), save_map_folder_path='',
                localize_on_startup=False, enable_request_hint=True,
                enable_ground_constraint_in_odometry=workflow['ground_constraint'],
                enable_ground_constraint_in_slam=workflow['ground_constraint'])
    if not capture_visualization:
        slam.update(enable_slam_visualization=False, enable_landmarks_view=False)
    global_.update(common, map_dir=str(map_root / 'cuvgl_map'), model_dir=str(model_dir), config_dir=str(runtime_config))
    if global_['publish_map_to_base_tf'] or global_['publish_map_to_odom_tf']:
        raise ValueError('動的 TF は cuVSLAM だけが配信する必要があります')
    remaps = {}
    for kind, prefix in (('cuvslam', 'visual_slam'), ('vgl', 'visual_localization')):
        remaps[kind] = [(f'{prefix}/{topic}_{index}', workflow[key]) for topic, index, key in (
            ('image', 0, 'left_image'), ('image', 1, 'right_image'), ('camera_info', 0, 'left_info'), ('camera_info', 1, 'right_info'))]
    hint, request_hint = namespace + '/pose_hint', namespace + '/request_hint'
    remaps['cuvslam'] += [('visual_slam/initial_pose', hint), ('visual_slam/trigger_hint', request_hint)]
    remaps['vgl'] += [('visual_localization/pose', hint), ('visual_localization/trigger_localization', request_hint),
                      ('/diagnostics', namespace + '/vgl_diagnostics')]
    return dict(parameters={'cuvslam': slam, 'vgl': global_}, remaps=remaps, hint_topic=hint)


def resolve_capture(config_dir, job, inspect=None):
    config_dir = Path(config_dir)
    workflow = tracking_workflow(job['workflow'])
    if type(workflow.get('ros_domain_id')) is not int or not 1 <= workflow['ros_domain_id'] <= 232:
        raise ValueError('隔離 ROS domain は1〜232で指定してください')
    capture = dict(DEFAULT_CAPTURE, **job.get('capture', {}))
    if set(capture) != set(DEFAULT_CAPTURE):
        raise ValueError('未知の capture option です')
    bounds = dict(replay_rate=(.05, 4), ready_timeout_s=(1, 600), drain_timeout_s=(1, 300),
                  settle_s=(.1, 30), tail_tolerance_s=(.01, 5))
    for key, (low, high) in bounds.items():
        value = capture[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError('capture option が範囲外です: ' + key)
    if capture['settle_s'] >= capture['drain_timeout_s']:
        raise ValueError('settle_s は drain_timeout_s より短くしてください')
    map_dir = Path(job['map_dir']).resolve(strict=True)
    if not map_dir.is_dir() or not list(map_dir.glob('*.mdb')):
        raise ValueError('保存 cuVSLAM 地図がありません')
    vgl = job.get('vgl')
    if not isinstance(vgl, dict):
        raise ValueError('VGL による初期位置取得が必要です。原点hintへfallbackしません')
    map_root = Path(vgl['map_root']).resolve(strict=True)
    model_dir = Path(vgl['model_dir']).resolve(strict=True)
    runtime_config = Path(vgl['config_dir']).resolve(strict=True)
    if map_dir != map_root / 'cuvslam_map':
        raise ValueError('cuVSLAM 地図と VGL の map_root が一致しません')
    shape = vgl['input_shape']
    if not isinstance(shape, list) or len(shape) != 4 or shape[:2] != [1, 3] or any(type(v) is not int or v <= 0 for v in shape):
        raise ValueError('VGL input_shape は正の NCHW [1,3,H,W] が必要です')
    for relative in ('keyframes/frames_meta.json', 'bow_index.pb'):
        require_file(map_root / 'cuvgl_map' / relative)
    vocabulary = map_root / 'cuvgl_map/vocabulary'
    if not vocabulary.is_dir() or not any(p.is_file() and p.stat().st_size for p in vocabulary.rglob('*')):
        raise ValueError('VGL vocabulary がありません')
    files = {p.name: require_file(p) for p in map_dir.glob('*.mdb')}
    configs = {name: require_file(runtime_config / name) for name in CONFIG_FILES}
    engines = {}
    for kind, pattern in (('extractor', 'aliked_*.engine'), ('matcher', 'lightglue_aliked_*.engine')):
        candidates = list((model_dir / 'aliked_lightglue').glob(pattern))
        if len(candidates) != 1:
            raise ValueError('VGL engine を一意に選べません: ' + kind)
        engines[kind] = require_file(candidates[0])
    onnx = require_file(model_dir / 'aliked_lightglue/aliked.onnx')
    if inspect:
        inspect(engines, shape)
    components = component_configuration(config_dir, workflow, map_dir, map_root, model_dir, runtime_config,
        namespace='/atd_mapping', use_sim_time=True, capture_visualization=True)
    components['parameters']['cuvslam']['slam_max_map_size'] = 0
    return dict(**components, workflow=workflow, capture=capture, input_shape=shape,
                assets=dict(cuvslam_sha256={name: digest(path) for name, path in files.items()},
                            config_sha256={name: digest(path) for name, path in configs.items()},
                            onnx_sha256=digest(onnx), engine_sha256={k: digest(p) for k, p in engines.items()}))

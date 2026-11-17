"""ROS-free VGL runtime layout checks shared by publication and capture preflight."""
from __future__ import annotations

import hashlib
from pathlib import Path

CONFIG_FILES = ('keypoint_creation_config.pb.txt', 'localizer_config.pb.txt',
                'matching_config.pb.txt', 'query_map_config.pb.txt')


def _directory(path):
    path = Path(path)
    if path.is_symlink() or not path.is_dir():
        raise ValueError('VGL の通常directoryが必要です: ' + str(path))
    return path


def _file(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or not path.stat().st_size:
        raise ValueError('VGL の非空の通常ファイルが必要です: ' + str(path))
    return path


def validate_runtime_layout(cuvgl_map, config_dir, model_dir, expected_onnx_sha256):
    """Do not publish or advertise a layout that the collector must reject."""
    cuvgl_map, config_dir, model_dir = map(_directory, (cuvgl_map, config_dir, model_dir))
    _file(_directory(cuvgl_map / 'keyframes') / 'frames_meta.json')
    _file(cuvgl_map / 'bow_index.pb')
    vocabulary = _directory(cuvgl_map / 'vocabulary')
    vocabulary_files = []
    for path in vocabulary.rglob('*'):
        if path.is_symlink():
            raise ValueError('VGL vocabulary に symlink は使えません')
        if path.is_file():
            if path.stat().st_size:
                vocabulary_files.append(path)
        elif not path.is_dir():
            raise ValueError('VGL vocabulary に特殊ファイルは使えません')
    if not vocabulary_files:
        raise ValueError('VGL vocabulary の非空ファイルがありません')
    for name in CONFIG_FILES:
        _file(config_dir / name)
    models = _directory(model_dir / 'aliked_lightglue')
    onnx = _file(models / 'aliked.onnx')
    with onnx.open('rb') as stream:
        actual_onnx_sha256 = hashlib.file_digest(stream, 'sha256').hexdigest()
    if not expected_onnx_sha256 or actual_onnx_sha256 != expected_onnx_sha256:
        raise ValueError('runtime の ALIKED ONNX が VGL 作成元の export と一致しません')
    engines = {}
    for kind, pattern in (('extractor', 'aliked_*.engine'), ('matcher', 'lightglue_aliked_*.engine')):
        candidates = list(models.glob(pattern))
        if len(candidates) != 1:
            raise ValueError('VGL engine を一意に確認できません: ' + kind)
        engines[kind] = _file(candidates[0])
    return {'engines': engines, 'onnx_sha256': actual_onnx_sha256}

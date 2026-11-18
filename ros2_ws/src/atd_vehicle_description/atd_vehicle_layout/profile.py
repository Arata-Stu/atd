"""実測前の値はnullで保持する。保存は再起動時のstatic TF反映と分離する。"""
from __future__ import annotations

import math
import os
from pathlib import Path
import tempfile

import yaml

DIMENSIONS = ('wheelbase', 'front_track', 'rear_track', 'body_length', 'body_width', 'body_height', 'rear_overhang')
SOURCES = ('unmeasured', 'cad_candidate', 'manual')
FRAMES = {'base': 'base_link', 'rear_axle': 'rear_axle', 'front_axle': 'front_axle', 'camera': 'camera_link'}
CAD_REFERENCE = {
    'source': 'cad_candidate', 'path': 'hardware/cad/tt02/frame.3mf',
    'sha256': '1a36617b0864c304103ca44e0005683183dd1aa6c68565f88f442f79a72708bd',
    'axis_extent_m': {'x': .12, 'y': .2105, 'z': .0192},
    'caveat': 'CADのframe板の境界寸法です。CAD軸XYZと車体TF軸の対応は未確定で、実車の全長・全幅・全高、車軸位置、路面高さ、camera_link取付位置には流用できません。',
}


def default_profile():
    return {'version': 1, 'dimensions': {key: {'value_m': None, 'source': 'unmeasured'} for key in DIMENSIONS},
            'camera': {'enabled': False, 'translation_m': {key: None for key in ('x', 'y', 'z')},
                       'rotation_rpy_rad': {key: None for key in ('roll', 'pitch', 'yaw')}, 'source': 'unmeasured'}}


def _keys(value, expected):
    if not isinstance(value, dict) or set(value) != set(expected):
        raise ValueError('必要なキー: ' + ', '.join(expected))


def _number(value, low, high, positive=False):
    if value is None:
        return None
    if type(value) not in (int, float) or not low <= value <= high or not math.isfinite(value) or positive and value == 0:
        raise ValueError(f'有限の数値（{low}〜{high}）または未測定のnullを指定してください')
    return float(value)


def _source(value, known):
    if value not in SOURCES or (value == 'unmeasured') == known:
        raise ValueError('未測定はnullとunmeasured、入力済み値はmanualまたはcad_candidateを指定してください')
    return value


def validate(value):
    _keys(value, ('version', 'dimensions', 'camera'))
    if type(value['version']) is not int or value['version'] != 1:
        raise ValueError('version は整数1を指定してください')
    _keys(value['dimensions'], DIMENSIONS)
    result = default_profile()
    for key in DIMENSIONS:
        field = value['dimensions'][key]
        _keys(field, ('value_m', 'source'))
        number = _number(field['value_m'], 0, 20, positive=key != 'rear_overhang')
        result['dimensions'][key] = {'value_m': number, 'source': _source(field['source'], number is not None)}
    camera = value['camera']
    _keys(camera, ('enabled', 'translation_m', 'rotation_rpy_rad', 'source'))
    if type(camera['enabled']) is not bool:
        raise ValueError('camera.enabled はboolを指定してください')
    known = []
    for group, axes, limit in (('translation_m', ('x', 'y', 'z'), 20),
                               ('rotation_rpy_rad', ('roll', 'pitch', 'yaw'), math.pi)):
        _keys(camera[group], axes)
        for axis in axes:
            number = _number(camera[group][axis], -limit, limit)
            result['camera'][group][axis] = number
            known.append(number is not None)
    if camera['enabled'] and not all(known):
        raise ValueError('camera TFを有効にするには位置xyzとroll/pitch/yawの全6値が必要です')
    result['camera']['enabled'] = camera['enabled']
    result['camera']['source'] = _source(camera['source'], any(known))
    return result


class _Loader(yaml.SafeLoader):
    pass


def _mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=True)
        if type(key) is not str or key in result:
            raise ValueError('YAMLキーの型または重複が不正です')
        result[key] = loader.construct_object(value_node, deep=True)
    return result


_Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def load(path):
    path = Path(path)
    if path.stat().st_size > 65536:
        raise ValueError('設定は64 KiB以下にしてください')
    return validate(yaml.load(path.read_text(encoding='utf-8'), Loader=_Loader))


def save(path, value):
    value = validate(value)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent, delete=False) as stream:
            temporary = stream.name
            yaml.safe_dump(value, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)

"""補助CHの役割設定の検査・永続化。走行用CH1〜3はPico側で固定。"""
from __future__ import annotations

import os
from pathlib import Path
import tempfile

import yaml

ROLES = ('recording', 'auxiliary_1', 'auxiliary_2')
CHANNELS = tuple(range(4, 17))
FIXED_ROLES = {'steering': 1, 'throttle': 2, 'authority': 3}


def default_profile():
    return {'version': 1, 'debounce_ms': 100, 'roles': {
        role: {'channel': 0, 'low_threshold': 700, 'high_threshold': 1300, 'inverted': False}
        for role in ROLES}}


def _keys(value, names):
    if not isinstance(value, dict) or set(value) != set(names):
        raise ValueError('必要なキー: ' + ', '.join(names))


def validate(value):
    _keys(value, ('version', 'debounce_ms', 'roles'))
    if type(value['version']) is not int or value['version'] != 1:
        raise ValueError('version は整数1を指定してください')
    if type(value['debounce_ms']) is not int or not 0 <= value['debounce_ms'] <= 1000:
        raise ValueError('debounce_ms は0〜1000の整数を指定してください')
    _keys(value['roles'], ROLES)
    result = default_profile()
    result['debounce_ms'] = value['debounce_ms']
    used = set()
    for role in ROLES:
        binding = value['roles'][role]
        _keys(binding, ('channel', 'low_threshold', 'high_threshold', 'inverted'))
        channel = binding['channel']
        if type(channel) is not int or channel not in (0, *CHANNELS):
            raise ValueError('役割のCHは未割当(0)またはCH4〜16を選択してください')
        if channel and channel in used:
            raise ValueError('同じCHに複数の役割は割り当てられません')
        low, high = binding['low_threshold'], binding['high_threshold']
        if type(low) is not int or type(high) is not int or not 0 <= low < high <= 2047:
            raise ValueError('閾値は0〜2047の整数で、低側 < 高側としてください')
        if type(binding['inverted']) is not bool:
            raise ValueError('反転はboolで指定してください')
        if channel:
            used.add(channel)
        result['roles'][role] = dict(binding)
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


def save(path, profile):
    value = validate(profile)
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

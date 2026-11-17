from __future__ import annotations

from contextlib import contextmanager
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
import time

from atd_processing.geometry import points, corridor


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}', value):
        raise ValueError('ID は英数字で始まる96文字以内の英数字・_・-・. で指定してください')
    return value


def confined(root, relative='', exists=True):
    root = Path(root).resolve()
    value = Path(relative)
    if value.is_absolute() or '..' in value.parts:
        raise ValueError('保存先からの相対パスを指定してください')
    path = root / value
    current = path
    while current != root:
        if current.is_symlink():
            raise ValueError('symlink は扱いません')
        current = current.parent
    if exists and not path.exists():
        raise ValueError('ファイルまたはディレクトリが見つかりません')
    return path


def read_json(path):
    return json.loads(Path(path).read_text())


def atomic_json(path, data):
    path = Path(path)
    if path.is_symlink():
        raise ValueError('symlink への保存はできません')
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(data, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


class Store:
    def __init__(self, record_root, result_root):
        self.record_root = Path(record_root).resolve()
        self.result_root = Path(result_root).resolve()
        self.maps_root = self.result_root / 'maps'
        self.state_root = self.result_root / 'runs/web'
        for directory in (self.record_root, self.maps_root, self.state_root):
            directory.mkdir(parents=True, exist_ok=True)

    @contextmanager
    def lock(self, key):
        with (self.state_root / (hashlib.sha256(str(key).encode()).hexdigest() + '.lock')).open('a') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            yield

    def folder(self, key):
        return confined(self.maps_root, identifier(key))

    def load_map(self, key):
        return read_json(confined(self.folder(key), 'hdmap/map.json'))

    def list_maps(self):
        result = []
        for directory in sorted(self.maps_root.iterdir()):
            if directory.name.startswith('.') or directory.is_symlink() or not (directory / 'hdmap/map.json').is_file():
                continue
            result.append(self.load_map(directory.name))
        return result

    def create_map(self, key, title=''):
        key = identifier(key)
        with self.lock('maps'):
            folder = confined(self.maps_root, key, False)
            if folder.exists():
                raise ValueError('同じ ID の地図があります')
            folder.mkdir()
            document = dict(id=key, title=title or key, revision=1, frame='map', closed=True,
                            left=[], right=[], custom=[], lines={}, source={}, updated=time.time())
            atomic_json(folder / 'hdmap/map.json', document)
        return document

    def save_map(self, key, body):
        with self.lock('map-' + identifier(key)):
            current = self.load_map(key)
            if type(body.get('revision')) is not int or body['revision'] != current['revision']:
                raise ValueError('地図の revision が変わりました。再読込してください')
            if body.get('frame', 'map') != 'map' or type(body.get('closed', True)) is not bool:
                raise ValueError('座標系は map、closed は boolean で指定してください')
            changed = dict(closed=body.get('closed', current['closed']))
            for field in ('left', 'right', 'custom'):
                changed[field] = points(body.get(field, current[field]))
            if changed['left'] and changed['right']:
                corridor(changed)
            invalidate = any(changed[field] != current[field] for field in ('left', 'right', 'closed'))
            custom_changed = changed['custom'] != current['custom']
            current.update(changed)
            current['title'] = str(body.get('title', current['title']))[:160]
            if invalidate:
                current['lines'] = {}
            elif custom_changed:
                current['lines'].pop('custom', None)
            current.update(revision=current['revision'] + 1, updated=time.time())
            atomic_json(self.folder(key) / 'hdmap/map.json', current)
        return current

    def save_line(self, key, revision, kind, line):
        with self.lock('map-' + identifier(key)):
            current = self.load_map(key)
            if current['revision'] != revision:
                raise ValueError('生成中に地図が更新されました。再生成してください')
            current['lines'][kind] = line
            current.update(revision=revision + 1, updated=time.time())
            atomic_json(self.folder(key) / 'hdmap/map.json', current)
            atomic_json(self.folder(key) / 'lines' / (kind + '.json'), dict(map_id=key, revision=current['revision'], **line))
        return current

    def records(self):
        result = []
        for directory, dirs, files in os.walk(self.record_root, followlinks=False):
            parent = Path(directory)
            dirs[:] = [name for name in dirs if not name.startswith('.') and not (parent / name).is_symlink()]
            if 'metadata.yaml' in files and not (parent / 'metadata.yaml').is_symlink():
                relative = parent.relative_to(self.record_root).as_posix()
                result.append(dict(id=relative, name=parent.name, bytes=sum(p.stat().st_size for p in parent.rglob('*') if p.is_file() and not p.is_symlink())))
                dirs[:] = []
        return result

    def connection(self):
        path = self.state_root / 'connection.json'
        default = dict(host='', user='', port=22, record_root='', map_root='', bridge_port=8790)
        return dict(default, **read_json(path)) if path.is_file() else default

    def save_connection(self, data):
        allowed = {'host', 'user', 'port', 'record_root', 'map_root', 'bridge_port'}
        if set(data) - allowed:
            raise ValueError('接続設定に未知の項目があります')
        value = dict(self.connection(), **data)
        from .transfers import connection
        connection(value)
        if type(value['bridge_port']) is not int or not 1024 <= value['bridge_port'] <= 65535:
            raise ValueError('bridge port は1024〜65535で指定してください')
        with self.lock('connection'):
            atomic_json(self.state_root / 'connection.json', value)
        return value

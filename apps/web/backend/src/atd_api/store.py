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

from atd_processing.geometry import points, corridor, validate_raceline_limits


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
        self.maps_root = confined(self.result_root, 'maps', exists=False)
        self.state_root = confined(self.result_root, 'runs/web', exists=False)
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

    def live_reference(self, key):
        folder = self.folder(key)
        raw = confined(folder, 'hdmap/map.json').read_bytes()
        document = json.loads(raw)
        source = document.get('source', {})
        if isinstance(source, dict) and source.get('kind') == 'orbslam3':
            # The ROS loader verifies both large assets before opening the Atlas. The
            # live view compares identities without re-reading the Atlas every refresh.
            manifest = read_json(confined(folder, 'orbslam3/manifest.json'))
            if (not isinstance(manifest, dict) or manifest.get('schema') != 'atd.orbslam3.map.v1'
                    or not isinstance(manifest.get('files'), dict)):
                raise ValueError('ORB-SLAM3 地図の manifest が不正です')
            files = manifest['files']
            frozen = {name: files.get(name) for name in ('atlas.osa', 'camera.yml')}
            encoded = json.dumps(frozen, sort_keys=True, separators=(',', ':')).encode()
            map_digest = hashlib.sha256(encoded).hexdigest()
            if (map_digest != source.get('map_sha256')
                    or any(not re.fullmatch('[0-9a-f]{64}', str(value)) for value in frozen.values())):
                raise ValueError('HDMap と ORB-SLAM3 の地図識別情報が一致しません')
            return dict(document=document, hdmap_sha256=hashlib.sha256(raw).hexdigest(), map_sha256=map_digest)
        marker = confined(folder, 'vgl.json', exists=False)
        profile = read_json(marker) if marker.is_file() else {}
        fingerprint = profile.get('cuvslam_sha256')
        map_digest = None
        if profile.get('status') == 'complete' and isinstance(fingerprint, dict) and fingerprint:
            encoded = json.dumps(fingerprint, sort_keys=True, ensure_ascii=True, separators=(',', ':'))
            map_digest = hashlib.sha256(encoded.encode('utf-8')).hexdigest()
        return dict(document=document, hdmap_sha256=hashlib.sha256(raw).hexdigest(), map_sha256=map_digest)

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
            if (current.get('source', {}).get('kind') == 'nvidia_offline'
                    and current['source'].get('pointcloud_status') != 'ready'
                    and any(body.get(field, current[field]) != current[field] for field in ('left', 'right', 'custom', 'closed'))):
                raise ValueError('保存地図から点群を取得してから HDMap を編集してください')
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
            if kind == 'centerline':
                current['lines'].pop('raceline', None)
            current.update(revision=revision + 1, updated=time.time())
            atomic_json(self.folder(key) / 'hdmap/map.json', current)
            atomic_json(self.folder(key) / 'lines' / (kind + '.json'), dict(map_id=key, revision=current['revision'], **line))
        return current

    def apply_raceline_calibration(self, key, revision, adoption):
        """Save a server-verified selection under the map's optimistic revision lock."""
        with self.lock('map-' + identifier(key)):
            current = self.load_map(key)
            if type(revision) is not int or revision != current['revision']:
                raise ValueError('地図の revision が変わりました。再読込してください')
            previous = current.get('raceline_calibration')
            if adoption is None:
                if previous is None:
                    return current
                del current['raceline_calibration']
            else:
                validate_raceline_limits(adoption['limits'])
                if previous is not None and all(previous.get(k) == v for k, v in adoption.items()):
                    return current
                current['raceline_calibration'] = dict(adoption, applied_at=time.time())
            current.update(revision=revision + 1, updated=time.time())
            atomic_json(self.folder(key) / 'hdmap/map.json', current)
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
        data_root = '/home/atd/workspaces/atd/data'
        default = dict(host='', user='atd', port=22, bridge_port=8790,
                       record_root=f'{data_root}/record/rosbags', map_root=f'{data_root}/results/maps')
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

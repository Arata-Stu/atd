"""Notebook 地図の複製と、HDMap の編集内容または地図全体の削除。"""
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import time
from typing import Literal

from fastapi import APIRouter, Request
from pydantic import BaseModel, ConfigDict, Field

from .store import atomic_json, confined, identifier, read_json


router = APIRouter(prefix='/maps')


class Selection(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    scope: Literal['map', 'hdmap']
    revision: int | None = Field(default=None, ge=1)
    gazebo_manifest_sha256: str | None = Field(default=None, pattern='^[0-9a-f]{64}$')


class Duplicate(Selection):
    destination: str


def gazebo_entry(store, key):
    folder = store.folder(key)
    marker = confined(folder, 'mapping.json')
    raw = marker.read_bytes()
    manifest = json.loads(raw)
    if (not isinstance(manifest, dict) or manifest.get('source') != 'gazebo_live_cuvslam'
            or manifest.get('map_id') != key):
        raise ValueError('Gazebo 保存地図の manifest と ID が一致しません')
    db = confined(folder, 'nvidia/cuvslam_map')
    if not any(p.name != 'lock.mdb' and not p.is_symlink() and p.is_file() and p.stat().st_size
               for p in db.glob('*.mdb')):
        raise ValueError('Gazebo 保存地図のデータベースが見つかりません')
    return dict(id=key, gazebo_manifest_sha256=hashlib.sha256(raw).hexdigest())


def gazebo_maps(store):
    result = []
    for folder in sorted(store.maps_root.iterdir()):
        if (folder.is_symlink() or not folder.is_dir() or folder.name.startswith('.')
                or (folder / 'hdmap/map.json').exists() or not (folder / 'mapping.json').is_file()):
            continue
        try:
            result.append(gazebo_entry(store, folder.name))
        except (ValueError, OSError):
            continue
    return result


def current_map(store, key, selection):
    confined(store.result_root, 'maps')
    if selection.get('gazebo_manifest_sha256') is not None:
        hdmap = confined(store.folder(key), 'hdmap/map.json', exists=False)
        if hdmap.exists() or selection.get('revision') is not None or selection['scope'] != 'map':
            raise ValueError('HDMap のない Gazebo 保存地図は地図全体を指定してください')
        if selection['gazebo_manifest_sha256'] != gazebo_entry(store, key)['gazebo_manifest_sha256']:
            raise ValueError('Gazebo 保存地図が変わりました。再読み込みしてください')
        return None
    document = store.load_map(key)
    if document.get('id') != key or selection.get('revision') is None or document.get('revision') != selection['revision']:
        raise ValueError('地図の ID または revision が変わりました。再読み込みしてください')
    return document


def destination(store, name):
    target = confined(store.maps_root, identifier(name), exists=False)
    if target.exists():
        raise ValueError('同じ ID の地図があります')
    return target


def _copy_tree(source, stage, check):
    def validate(directory, names):
        check()
        for name in names:
            path = Path(directory) / name
            if path.is_symlink() or not (path.is_file() or path.is_dir()):
                raise ValueError('地図内のリンク・特殊ファイルは複製できません: ' + str(path))
        return []
    shutil.copytree(source, stage, ignore=validate)


def _rewrite_id(path, field, source, target):
    value = read_json(path)
    if value.get(field) != source:
        raise ValueError(f'{path.name} の地図 ID が複製元と一致しません')
    value[field] = target
    atomic_json(path, value)


def duplicate_map(store, task, payload):
    key, new_id = payload['id'], payload['destination']
    current = current_map(store, key, payload)
    source = store.folder(key)
    destination(store, new_id)
    with tempfile.TemporaryDirectory(dir=store.maps_root, prefix='.duplicate-') as tmp:
        stage = Path(tmp) / new_id
        if payload['scope'] == 'map':
            _copy_tree(source, stage, task.check)
            if current is None:
                _rewrite_id(stage / 'mapping.json', 'map_id', key, new_id)
            for path in stage.glob('hdmap/captures/*/manifest.json'):
                # snapshot と cloud の計測時の provenance は保持する。manifest は所属地図を示す。
                _rewrite_id(path, 'map_id', key, new_id)
            for path in stage.glob('exports/*/manifest.json'):
                _rewrite_id(path, 'map_id', key, new_id)
            for path in stage.glob('exports/*/hdmap.yaml'):
                _rewrite_id(path, 'id', key, new_id)
            for path in stage.glob('lines/*.json'):
                _rewrite_id(path, 'map_id', key, new_id)
        else:
            (stage / 'hdmap').mkdir(parents=True)
            info = current.get('source', {})
            capture = info.get('capture_relative')
            cloud = confined(source, Path(capture or 'hdmap') / 'cloud.json', exists=False)
            current['source'] = dict(kind='hdmap_copy', original_map=key)
            if cloud.is_file():
                shutil.copyfile(cloud, stage / 'hdmap/cloud.json')
                current['source']['pointcloud_status'] = 'ready'
            elif capture or info.get('pointcloud_status') == 'ready':
                raise ValueError('元地図の背景点群が見つかりません')
            for kind, line in current['lines'].items():
                atomic_json(stage / 'lines' / (identifier(kind) + '.json'),
                            dict(line, map_id=new_id, revision=current['revision'] + 1))
        if current is not None:
            current.update(id=new_id, title=new_id, revision=current['revision'] + 1, updated=time.time())
            atomic_json(stage / 'hdmap/map.json', current)
        task.check()
        with store.lock('maps'):
            current_map(store, key, payload)
            target = destination(store, new_id)
            stage.rename(target)
    return dict(map=new_id, scope=payload['scope'])


@router.post('/{key}/duplicate')
def start_duplicate(key: str, body: Duplicate, request: Request):
    store, jobs = request.app.state.store, request.app.state.jobs
    current_map(store, key, body.model_dump())
    destination(store, body.destination)
    return jobs.start('map_duplicate', dict(id=key, **body.model_dump()),
                      ['map:' + key, 'map:' + body.destination], f'地図を複製: {key} → {body.destination}')


@router.delete('/{key}')
def remove(key: str, body: Selection, request: Request):
    store, jobs = request.app.state.store, request.app.state.jobs
    identifier(key)
    with tempfile.TemporaryDirectory(dir=confined(store.result_root, 'maps'), prefix='.delete-') as tmp:
        removed = Path(tmp)
        with jobs.registry(), store.lock('maps'), store.lock('map-' + key):
            jobs.require_available(['map:' + key])
            current = current_map(store, key, body.model_dump())
            folder = store.folder(key)
            if body.scope == 'map':
                folder.rename(removed / key)
            else:
                # map.json は保存地図と取得済み点群の対応も保持する。編集レイヤだけを初期化する。
                current.update(left=[], right=[], custom=[], lines={}, closed=True,
                               revision=current['revision'] + 1, updated=time.time())
                current.pop('raceline_calibration', None)
                moved = []
                try:
                    for name in ('lines', 'exports'):
                        path = confined(folder, name, exists=False)
                        if path.exists():
                            path.rename(removed / name)
                            moved.append(name)
                    atomic_json(confined(folder, 'hdmap/map.json'), current)
                except BaseException:
                    for name in moved:
                        (removed / name).rename(folder / name)
                    raise
    return dict(map=key, scope=body.scope)

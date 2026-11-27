"""Notebook の保存地図と別 bag を選択し、既存の ROS 検証 CLI をジョブで実行する。"""
from functools import lru_cache
import hashlib
import os
import re
from pathlib import Path
import subprocess
import time
from typing import Literal

from fastapi import APIRouter, Request
from pydantic import BaseModel, ConfigDict, Field

from .jobs import ROS_SETUP
from .store import confined, identifier, read_json

router = APIRouter()
DOMAIN = 95
BUILD_HINT = ('Notebook Docker で /workspaces/atd/scripts/build.sh --packages-up-to atd_localization '
              'を実行し、validate_bag を準備してください')


class ValidationRun(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    map_id: str
    bag: str
    revision: int = Field(ge=1)
    rate: float = Field(default=1.0, ge=.05, le=4., allow_inf_nan=False)
    engine_dir: str | None = None
    map_correction_source: Literal['cuvslam', 'vgl'] = 'cuvslam'
    vgl_frequency: float | None = Field(default=None, ge=0., allow_inf_nan=False)


@lru_cache(maxsize=1)
def _probe(_interval):
    try:
        completed = subprocess.run(['bash', '-c', ROS_SETUP + 'exec ros2 pkg prefix atd_localization'],
                                   stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=5)
        prefix = Path(completed.stdout.strip())
        executable = prefix / 'lib/atd_localization/validate_bag'
        if completed.returncode == 0 and prefix.is_absolute() and executable.is_file() and os.access(executable, os.X_OK):
            return executable
    except (OSError, subprocess.TimeoutExpired):
        pass
    return None


def validator():
    return _probe(int(time.monotonic() // 5))


def _check_map(reference, key):
    doc = reference['document']
    source = doc.get('source', {})
    if (doc.get('id') != key or doc.get('frame') != 'map' or not isinstance(source, dict) or source.get('kind') != 'nvidia_offline'
            or not isinstance(source.get('inputs'), dict) or source['inputs'].get('base_frame') != 'base_link'):
        raise ValueError('base_link で作成した NVIDIA 地図を選択してください')
    if (not isinstance(source.get('bag'), str) or not source['bag']
            or not re.fullmatch('[0-9a-f]{64}', str(source.get('metadata_sha256', '')))):
        raise ValueError('地図作成 bag の識別情報が不足しています')
    if not reference['map_sha256']:
        raise ValueError('この地図の VGL 準備を完了してください')


def _bag(store, bag_id, source):
    path = Path(bag_id)
    if not path.parts:
        raise ValueError('記録ルートからの相対 bag パスを選択してください')
    metadata = confined(store.record_root, path / 'metadata.yaml')
    if not metadata.is_file():
        raise ValueError('bag に metadata.yaml がありません')
    with metadata.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    if path == Path(source['bag']) or digest == source['metadata_sha256']:
        raise ValueError('地図作成に使用した bag と同じです。別走行の bag を選択してください')
    return path.as_posix(), digest


def _engines(store, key):
    profile = read_json(confined(store.folder(key), 'vgl.json'))
    asset = identifier(profile.get('asset_id'))
    folder = confined(store.result_root, Path('models/vgl') / asset / 'engines', exists=False)
    dirs = []
    for path in sorted(folder.glob('*/*/build.json')):
        if any(p.startswith('.') for p in path.relative_to(folder).parts):
            continue
        try:
            record = read_json(confined(folder, path.relative_to(folder)))
        except (OSError, ValueError):
            continue
        if (isinstance(record, dict) and record.get('status') == 'complete' and record.get('asset_id') == asset
                and record.get('export_id') == profile.get('export_id')
                and record.get('bundle_sha256') == profile.get('bundle_sha256')):
            dirs.append(path.parent.relative_to(store.result_root).as_posix())
    return dirs


def options(store, selected):
    maps, bags, engines = [], [], []
    source = None
    if selected is not None:
        identifier(selected)
    for folder in sorted(store.maps_root.iterdir()):
        if folder.is_symlink() or folder.name.startswith('.') or not (folder / 'hdmap/map.json').is_file():
            continue
        entry = dict(id=folder.name, title=folder.name, revision=0, source_bag='', has_raceline=False,
                     ready=False, issue='')
        try:
            reference = store.live_reference(folder.name)
            doc = reference['document']
            entry.update(title=doc.get('title', folder.name), revision=doc['revision'],
                         source_bag=doc.get('source', {}).get('bag', ''),
                         has_raceline=bool(doc.get('lines', {}).get('raceline', {}).get('points')))
            _check_map(reference, folder.name)
            if folder.name == selected:
                engines = _engines(store, selected)
                source = doc['source']
            entry['ready'] = True
        except (OSError, ValueError, KeyError, AttributeError, TypeError) as error:
            entry['issue'] = str(error)
        maps.append(entry)
    if source:
        for record in sorted(store.records(), key=lambda item: item['id']):
            issue = ''
            try:
                _bag(store, record['id'], source)
            except (ValueError, OSError) as error:
                issue = str(error)
            bags.append(dict(record, eligible=not issue, issue=issue))
    available = validator() is not None
    return dict(maps=maps, bags=bags, engine_dirs=engines, available=available,
                unavailable_reason='' if available else BUILD_HINT)


def payload(store, body):
    if body.vgl_frequency is not None and body.map_correction_source != 'vgl':
        raise ValueError('VGL 照合頻度は地図補正元に cuVGL を選んだ場合に指定できます')
    reference = store.live_reference(body.map_id)
    _check_map(reference, body.map_id)
    doc = reference['document']
    if doc['revision'] != body.revision:
        raise ValueError('地図が更新されています。地図を再選択してください')
    bag, sha = _bag(store, body.bag, doc['source'])
    if body.engine_dir is not None and body.engine_dir not in _engines(store, body.map_id):
        raise ValueError('地図の VGL export に対応する engine を選択してください')
    if validator() is None:
        raise ValueError(BUILD_HINT)
    return dict(map_id=body.map_id, bag=bag, rate=body.rate, engine_dir=body.engine_dir, metadata_sha256=sha,
                map_correction_source=body.map_correction_source, vgl_frequency=body.vgl_frequency,
                hdmap_sha256=reference['hdmap_sha256'], map_sha256=reference['map_sha256'])


@router.get('/localization/options')
def validation_options(request: Request, map_id: str | None = None):
    return options(request.app.state.store, map_id)


@router.post('/localization/runs')
def start_validation(request: Request, body: ValidationRun):
    p = payload(request.app.state.store, body)
    return request.app.state.jobs.start('localization_validation', p,
        ['map:' + p['map_id'], 'record:' + p['bag'], f'ros-domain:{DOMAIN}'],
        f"自己位置検証: {p['map_id']} / {p['bag']}")


def execute(store, task, request):
    key, bag = request['map_id'], request['bag']
    reference = store.live_reference(key)
    _check_map(reference, key)
    source = reference['document']['source']
    _, bag_hash = _bag(store, bag, source)
    if (bag_hash != request['metadata_sha256'] or reference['hdmap_sha256'] != request['hdmap_sha256']
            or reference['map_sha256'] != request['map_sha256']):
        raise ValueError('要求後に地図または bag が更新されました。入力を選び直してください')
    executable = validator()
    if executable is None:
        raise ValueError(BUILD_HINT)
    # Exec the installed program directly so cancellation waits for its own ROS launch/player cleanup.
    command = [str(executable), '--run-id', task.key, '--map', key, '--bag', bag, '--rate', str(request['rate']),
               '--domain', str(DOMAIN), '--results-root', str(store.result_root), '--record-root', str(store.record_root)]
    if request['engine_dir'] is not None:
        command.extend(['--engine-dir', request['engine_dir']])
    correction = request.get('map_correction_source', 'cuvslam')
    frequency = request.get('vgl_frequency')
    if correction == 'vgl':
        command.extend(['--map-correction-source', correction])
        if frequency is not None:
            command.extend(['--vgl-frequency', str(frequency)])
    env = dict(os.environ, ATD_REPLAY_DDS=os.environ.get('ATD_REPLAY_DDS', 'shm'))
    code = task.run(command, env=env, ros=True, allowed_exit_codes=(0, 2))
    path = confined(store.result_root, Path('runs/localization') / identifier(task.key) / 'report.json')
    report = read_json(path)
    expected = dict(map_id=key, bag=bag, map_sha256=request['map_sha256'], hdmap_sha256=request['hdmap_sha256'],
                    metadata_sha256=bag_hash, source_bag=source['bag'], source_metadata_sha256=source['metadata_sha256'],
                    schema='atd.localization.validation.v1', frame='map', body_frame='base_link',
                    ros_domain_id=DOMAIN, replay_rate=request['rate'])
    outcomes = ('tracked_to_end',) if code == 0 else ('partial_tracking', 'no_tracking')
    if correction == 'vgl':
        expected['vgl_frequency_hz'] = 1.0 if frequency is None else frequency
    if (report.get('map_correction_source', report.get('backend', 'cuvslam')) != correction
            or any(report.get(k) != value for k, value in expected.items()) or report.get('outcome') not in outcomes):
        raise ValueError('検証結果の出典・終了状態が要求と一致しません。ログを確認してください')
    return dict(run_id=task.key, map_id=key, bag=bag, outcome=report['outcome'])

"""Notebook 上の F1 サンプル、編集用コピー、再現可能な閉ループ評価。"""
from __future__ import annotations

import csv
import hashlib
import io
import json
import tempfile
from pathlib import Path
from typing import Literal

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel, ConfigDict, Field

from .planning_replay import _core
from .store import atomic_json, confined, identifier, read_json

SAMPLES = Path(__file__).resolve().parents[5] / 'samples/f1tenth'
REPORTS = Path('runs/simulation_benchmarks')
router = APIRouter(prefix='/simulation')


class CourseOptions(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    settings: dict = Field(default_factory=dict)
    line: Literal['centerline', 'raceline'] = 'raceline'
    source: Literal['sample', 'workspace'] = 'sample'


class Batch(CourseOptions):
    course_ids: list[str] = Field(min_length=1, max_length=64)
    laps: int = Field(default=2, ge=1, le=5)
    trials: int = Field(default=3, ge=1, le=3)


class Trace(CourseOptions):
    laps: int = Field(default=2, ge=1, le=5)
    trial: int = Field(default=1, ge=1, le=3)


def catalog():
    return read_json(SAMPLES / 'catalog.json')


def included():
    return {row['id']: row for row in catalog()['tracks'] if row['included']}


def sample_folder(key):
    if key not in included():
        raise ValueError('ベンチマークに含まれるコースを選択してください')
    return confined(SAMPLES / 'maps', identifier(key))


def working_id(key):
    return 'f1tenth-' + identifier(key)


def editable_id(store, key):
    path = confined(store.maps_root, working_id(key) + '/hdmap/map.json', exists=False)
    if path.is_file() and read_json(path).get('source', {}).get('benchmark_course') == key:
        return working_id(key)
    return None


def document_for(store, key, source):
    folder = sample_folder(key)
    if source == 'sample':
        return read_json(folder / 'hdmap/map.json')
    edit = editable_id(store, key)
    if not edit:
        raise ValueError(f'{key} の編集用 HDMap を先に作成してください')
    return store.load_map(edit)


def engine():
    _core('atd_planning')
    _core('atd_control', 'simulation')
    from atd_processing import benchmark
    return benchmark


@router.get('/courses')
def courses(request: Request):
    store = request.app.state.store
    data = catalog()
    return dict(upstream=data['upstream'], items=[
        dict(row, editable_id=editable_id(store, row['id']) if row['included'] else None)
        for row in data['tracks']])


@router.post('/courses/{key}/edit')
def edit_course(key: str, request: Request):
    folder = sample_folder(key)
    store = request.app.state.store
    name = working_id(key)
    with store.lock('maps'):
        target = confined(store.maps_root, name, exists=False)
        if target.exists():
            if editable_id(store, key) == name:
                return dict(map_id=name)
            raise ValueError(f'同じ地図 ID {name} が既に使われています')
        # 地図と両ラインが揃った時点で公開する。既存の編集結果を再インポートしない。
        with tempfile.TemporaryDirectory(dir=store.maps_root, prefix='.sample-') as tmp:
            stage = Path(tmp) / name
            document = read_json(folder / 'hdmap/map.json')
            document['id'] = name
            document['source'] = dict(document.get('source', {}), benchmark_course=key)
            atomic_json(stage / 'hdmap/map.json', document)
            for kind in ('centerline', 'raceline'):
                line = read_json(folder / 'lines' / (kind + '.json'))
                line['map_id'] = name
                atomic_json(stage / 'lines' / (kind + '.json'), line)
            stage.rename(target)
    return dict(map_id=name)


@router.post('/courses/{key}/compare')
def compare_course(key: str, body: Trace, request: Request):
    doc = document_for(request.app.state.store, key, body.source)
    return JSONResponse(engine().compare_course(doc, body.settings, line=body.line, laps=body.laps, trial=body.trial))


def signature(document, kind):
    line = document.get('lines', {}).get(kind)
    if not line:
        raise ValueError(f'{document["id"]} の {kind} を地図画面で生成してください')
    # 入力に直接使う境界・経路・速度を固定する。タイトルや最適化ログの違いで比較を阻害しない。
    fields = dict(left=document['left'], right=document['right'], closed=document['closed'], kind=kind,
                  points=line['points'], profile=line['profile'], speeds=line.get('speeds'))
    return hashlib.sha256(json.dumps(fields, allow_nan=False, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def prepare(store, body):
    keys = body.course_ids
    if len(set(keys)) != len(keys):
        raise ValueError('同じコースを複数回指定できません')
    # 投稿時の入力を検証し、worker 実行までの編集を検出する。
    hashes = {key: signature(document_for(store, key, body.source), body.line) for key in keys}
    _core('atd_planning')
    simulation = _core('atd_control', 'simulation')
    return dict(body.model_dump(), settings=simulation.validate_settings(body.settings), course_sha256=hashes)


@router.post('/benchmarks')
def start(body: Batch, request: Request):
    store = request.app.state.store
    payload = prepare(store, body)
    return request.app.state.jobs.start('simulation_benchmark', payload, ['simulation-benchmark'],
                                        f'走行ベンチマーク（{len(body.course_ids)} コース）')


def make_report(report_id, payload, rows):
    # 未コミットの実装や Docker の読み取り専用 checkout も、実行したコードそのもので識別する。
    benchmark = engine()
    from atd_processing.planning import footprint
    modules = (benchmark, footprint, _core('atd_planning'), _core('atd_control'),
               _core('atd_control', 'speed'), _core('atd_control', 'simulation'))
    source_hashes = {m.__name__: hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for m in modules}
    return dict(schema='atd.simulation_benchmark.v1', id=report_id, implementation_sha256=source_hashes,
                upstream=catalog()['upstream'], options=payload, courses=rows)


def evaluate_row(store, key, payload, check=lambda: None):
    doc = document_for(store, key, payload['source'])
    digest = signature(doc, payload['line'])
    if digest != payload['course_sha256'][key]:
        raise ValueError(f'{key} の境界またはラインが開始後に更新されました。再実行してください')
    row = engine().evaluate_course(doc, payload['settings'], line=payload['line'],
                                   laps=payload['laps'], trials=payload['trials'], check=check)
    return dict(row, course_id=key, map_id=doc['id'], course_sha256=digest)


def execute(store, task, payload):
    rows = []
    for i, key in enumerate(payload['course_ids'], 1):
        task.check()
        task.log(f'{i}/{len(payload["course_ids"])}  {key}')
        rows.append(evaluate_row(store, key, payload, task.check))
    task.check()
    report = make_report(task.key, payload, rows)
    atomic_json(store.result_root / REPORTS / task.key / 'report.json', report)
    return dict(report_id=task.key, course_count=len(rows))


def report_path(store, key):
    if key == 'baseline':
        return SAMPLES / 'baseline.json'
    return confined(store.result_root, REPORTS / identifier(key) / 'report.json')


@router.get('/benchmarks')
def list_reports(request: Request):
    root = request.app.state.store.result_root / REPORTS
    reports = sorted((p for p in root.glob('*/report.json') if not p.is_symlink() and not p.parent.is_symlink()),
                     key=lambda p: p.stat().st_mtime, reverse=True)
    return dict(items=[dict(id=p.parent.name, saved=p.stat().st_mtime) for p in reports],
                baseline=(SAMPLES / 'baseline.json').is_file())


@router.get('/benchmarks/{key}')
def report(key: str, request: Request):
    return read_json(report_path(request.app.state.store, key))


@router.get('/benchmarks/{key}/summary.csv')
def summary_csv(key: str, request: Request):
    data = read_json(report_path(request.app.state.store, key))
    out = io.StringIO()
    writer = csv.writer(out)
    columns = ('mode', 'trials', 'finishes', 'finish_rate', 'collisions', 'mean_lap_time_s', 'best_lap_time_s',
               'mean_trial_rms_lateral_error_m')
    writer.writerow(('course_id', 'line', 'course_sha256', *columns))
    for course in data['courses']:
        for result in course['results']:
            writer.writerow([course['course_id'], course['line'], course['course_sha256'], *(result.get(col) for col in columns)])
    return Response(out.getvalue(), media_type='text/csv', headers={
        'Content-Disposition': f'attachment; filename="benchmark-{key}.csv"'})

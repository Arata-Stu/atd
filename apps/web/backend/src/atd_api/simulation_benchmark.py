"""お台場サンプルと Notebook に保存された閉路の HDMap を使う閉ループ評価。"""
from __future__ import annotations

import csv
import hashlib
import io
import json
from pathlib import Path
from typing import Literal

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel, ConfigDict, Field

from .planning_replay import _core
from .sample_map import ODAIBA_COURSE, odaiba_document
from .store import atomic_json, confined, identifier, read_json

REPORTS = Path('runs/simulation_benchmarks')
router = APIRouter(prefix='/simulation')


class CourseOptions(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    settings: dict = Field(default_factory=dict)
    line: Literal['centerline', 'raceline'] = 'raceline'
    source: Literal['workspace', 'sample', 'mixed'] | None = None


class Batch(CourseOptions):
    course_ids: list[str] = Field(min_length=1, max_length=64)
    laps: int = Field(default=2, ge=1, le=5)
    trials: int = Field(default=3, ge=1, le=3)


class Trace(CourseOptions):
    laps: int = Field(default=2, ge=1, le=5)
    trial: int = Field(default=1, ge=1, le=3)


def course_lines(document):
    if not document.get('closed') or document.get('source', {}).get('kind') == 'f1tenth_benchmark':
        return []
    lines = document.get('lines', {})
    return [kind for kind in ('raceline', 'centerline')
            if lines.get(kind) and lines[kind].get('points') and lines[kind].get('profile')]


def document_for(store, key, line):
    document = odaiba_document() if key == ODAIBA_COURSE else store.load_map(key)
    if line not in course_lines(document):
        raise ValueError(f'{key} の評価には、閉路の HDMap と生成済みの {line} が必要です')
    return document


def course_source(keys, requested):
    source = 'workspace'
    if ODAIBA_COURSE in keys:
        source = 'sample' if len(keys) == 1 else 'mixed'
    if requested is not None and requested != source:
        raise ValueError('指定された地図の種類とコース ID が一致しません')
    return source


def engine():
    _core('atd_planning')
    _core('atd_control', 'simulation')
    from atd_processing import benchmark
    return benchmark


@router.get('/courses')
def courses(request: Request):
    sample = odaiba_document()
    items = [dict(id=ODAIBA_COURSE, name=sample['title'] + '（同梱サンプル）', lines=course_lines(sample),
                  sample='odaiba')]
    for document in request.app.state.store.list_maps():
        lines = course_lines(document)
        if lines:
            items.append(dict(id=document['id'], name=document.get('title') or document['id'], lines=lines))
    return dict(items=items)


@router.post('/courses/{key}/compare')
def compare_course(key: str, body: Trace, request: Request):
    course_source([key], body.source)
    doc = document_for(request.app.state.store, key, body.line)
    result = engine().compare_course(doc, body.settings, line=body.line, laps=body.laps, trial=body.trial)
    result['course']['id'] = key
    return JSONResponse(result)


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
    source = course_source(keys, body.source)
    # 投稿時の入力を検証し、worker 実行までの編集を検出する。
    hashes = {key: signature(document_for(store, key, body.line), body.line) for key in keys}
    _core('atd_planning')
    simulation = _core('atd_control', 'simulation')
    return dict(body.model_dump(), source=source, settings=simulation.validate_settings(body.settings), course_sha256=hashes)


@router.post('/benchmarks')
def start(body: Batch, request: Request):
    store = request.app.state.store
    payload = prepare(store, body)
    resources = ['simulation-benchmark']
    resources.extend('map:' + key for key in body.course_ids if key != ODAIBA_COURSE)
    return request.app.state.jobs.start('simulation_benchmark', payload, resources,
                                        f'走行ベンチマーク（{len(body.course_ids)} コース）')


def make_report(report_id, payload, rows):
    # 未コミットの実装や Docker の読み取り専用 checkout も、実行したコードそのもので識別する。
    benchmark = engine()
    from atd_processing.planning import footprint
    modules = (benchmark, footprint, _core('atd_planning'), _core('atd_control'),
               _core('atd_control', 'speed'), _core('atd_control', 'simulation'))
    source_hashes = {m.__name__: hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for m in modules}
    return dict(schema='atd.simulation_benchmark.v1', id=report_id, implementation_sha256=source_hashes,
                options=payload, courses=rows)


def evaluate_row(store, key, payload, check=lambda: None):
    doc = document_for(store, key, payload['line'])
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
    return confined(store.result_root, REPORTS / identifier(key) / 'report.json')


@router.get('/benchmarks')
def list_reports(request: Request):
    root = request.app.state.store.result_root / REPORTS
    reports = sorted((p for p in root.glob('*/report.json') if not p.is_symlink() and not p.parent.is_symlink()),
                     key=lambda p: p.stat().st_mtime, reverse=True)
    return dict(items=[dict(id=p.parent.name, saved=p.stat().st_mtime) for p in reports])


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

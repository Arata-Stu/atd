"""保存した別 bag の全観測を Jetson と共通の planner へ時刻順に入力する。"""
from contextlib import closing
from dataclasses import asdict, dataclass
from functools import lru_cache
import hashlib
from importlib import import_module
import json
import math
import os
from pathlib import Path
import sqlite3
import sys
import tempfile
from typing import Literal

from fastapi import APIRouter, Query, Request
from pydantic import BaseModel, ConfigDict

from .jobs import ACTIVE
from .localization_results import RUNS, _map, _report, _uint, trajectory_rows
from .store import confined, identifier

router = APIRouter()
Line = Literal['raceline', 'centerline', 'custom']


class Selection(BaseModel):
    model_config = ConfigDict(extra='forbid')
    line: Line


@lru_cache(maxsize=4)
def _core(name='atd_planning', module='core'):
    # ROS に依存しない同じ checkout の planner / controller で bag の観測を処理する。
    package = Path(__file__).resolve().parents[5] / 'ros2_ws/src' / name
    if str(package) not in sys.path:
        sys.path.insert(0, str(package))
    return import_module(f'{name}.{module}')


@lru_cache(maxsize=1)
def _algorithm():
    return hashlib.sha256(Path(_core().__file__).read_bytes()).hexdigest()


def _signature(path):
    stat = path.stat()
    return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)


@lru_cache(maxsize=4)
def _read_report(folder, signature):
    return _report(folder)


@dataclass
class Source:
    run_id: str
    line: str
    folder: Path
    report: dict
    start: int
    end: int
    reference: object
    index: Path

    @property
    def resource(self):
        return 'planning-replay:' + self.index.stem

    @property
    def duration(self):
        return (self.end - self.start) / 1e9


def source(store, run_id, line):
    folder = confined(store.result_root, RUNS / identifier(run_id))
    report_stat = _signature(confined(folder, 'report.json'))
    report = _read_report(folder, report_stat)
    if report['outcome'] in ('failed', 'cancelled'):
        raise ValueError('完了した別 bag の自己位置検証を選択してください')
    if report['metadata_sha256'] == report['source_metadata_sha256'] or report['bag'] == report['source_bag']:
        raise ValueError('地図作成と異なる走行の検証結果を指定してください')
    snapshot = confined(folder, 'hdmap.json', exists=False)
    files = [confined(folder, 'trajectory.csv')]
    if snapshot.is_file():
        files.append(snapshot)
    else:
        map_dir = store.folder(report['map_id'])
        files.extend([confined(map_dir, 'hdmap/map.json'), confined(map_dir, 'vgl.json')])
    signature = (report_stat, *(_signature(path) for path in files))
    return _reference(store, run_id, line, signature)


@lru_cache(maxsize=8)
def _reference(store, run_id, line, signature):
    folder = confined(store.result_root, RUNS / run_id)
    report = _read_report(folder, signature[0])
    document, issue = _map(store, folder, report)
    if issue:
        raise ValueError(issue)
    stereo = report.get('stereo')
    if not isinstance(stereo, dict):
        raise ValueError('検証結果にステレオ観測の時刻範囲がありません')
    start, end = (_uint(stereo.get(key)) for key in ('first_stamp_ns', 'last_stamp_ns'))
    samples = _uint(report.get('pose_samples'))
    if (start >= end or bool(samples) != (report['outcome'] != 'no_tracking')
            or (samples and (not isinstance(report.get('session_id'), str) or not report['session_id']))):
        raise ValueError('走行記録の観測範囲または自己位置セッションが不正です')
    reference = _core().ReferenceLine.from_document(document, line)
    # 元ファイルまたは計算実装の変更時は以前の派生結果を再利用しない。
    key = hashlib.sha256(json.dumps([1, _algorithm(), run_id, line, signature]).encode()).hexdigest()
    index = confined(store.state_root, Path('planning') / (key + '.sqlite'), exists=False)
    return Source(run_id, line, folder, report, start, end, reference, index)


def _open_index(path):
    return sqlite3.connect(path.as_uri() + '?mode=ro&immutable=1', uri=True)


def _info(src, jobs):
    summary = dict(id=src.run_id, line=src.line, ready=src.index.is_file(), duration_s=src.duration,
                   pose_samples=src.report['pose_samples'], valid_samples=None, job_id=None,
                   control_config=dict(asdict(_core('atd_control').LookaheadConfig()), wheelbase_m=None))
    if summary['ready']:
        with closing(_open_index(src.index)) as db:
            summary['valid_samples'] = db.execute('SELECT valid_samples FROM summary').fetchone()[0]
        return summary
    job = next((item for item in jobs.list() if src.resource in item['resources']), None)
    if job:
        summary['job_id'] = job['id']
    return summary


@router.get('/localization/runs/{run_id}/planning')
def status(request: Request, run_id: str, line: Line):
    return _info(source(request.app.state.store, run_id, line), request.app.state.jobs)


@router.post('/localization/runs/{run_id}/planning')
def start(request: Request, run_id: str, selection: Selection):
    store, jobs = request.app.state.store, request.app.state.jobs
    src = source(store, run_id, selection.line)
    with store.lock(src.resource):
        info = _info(src, jobs)
        if info['ready'] or (info['job_id'] and jobs.get(info['job_id'])['status'] in ACTIVE | {'orphaned'}):
            return info
        job = jobs.start('planning_replay', dict(run_id=run_id, line=src.line, resource=src.resource),
                         [src.resource], f'前方軌道の検証: {src.report["bag"]} / {src.line}')
        return dict(info, job_id=job['id'])


def _observations(src, task):
    planner = _core().Planner(src.reference)
    fields = ('qx', 'qy', 'qz', 'qw', 'speed_mps', 'z_m')
    for segment, i, stamp, (x, y), row in trajectory_rows(src.folder, src.report, src.start, src.end, fields):
        interval = src.report['tracking_intervals'][segment]
        if i == 0:
            planner.reset()
        if i % 128 == 0:
            task.check()
        speed = float(row['speed_mps'])
        if not math.isfinite(speed):
            raise ValueError('走行記録の推定速度が有限値ではありません')
        try:
            if not math.isfinite(float(row['z_m'])):
                raise ValueError('車体高さに非有限値があります')
            yaw = _core().yaw_from_quaternion(*(float(row[k]) for k in fields[:4]))
        except ValueError as error:
            planner.reset()
            yaw = None
            result = dict(reason=str(error), progress_m=None, lateral_error_m=None, target_speed_mps=None)
        else:
            result = planner.update(stamp_ns=stamp, x=x, y=y, yaw=yaw, speed_mps=speed,
                                    session_id=src.report['session_id'], epoch=interval['epoch'], include_trajectory=False)
        yield (stamp, interval['last_stamp_ns'], x, y, yaw, speed, result['progress_m'],
               result['lateral_error_m'], result['target_speed_mps'], result['reason'])


def execute(store, task, payload):
    src = source(store, payload['run_id'], payload['line'])
    if src.resource != payload['resource']:
        raise ValueError('検証結果の入力が変わりました。再度選択してください')
    task.log(f'{src.report["pose_samples"]} 点の自己位置を観測順に処理します。')
    src.index.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.planning-', suffix='.sqlite', dir=src.index.parent)
    os.close(fd)
    tmp = Path(name)
    try:
        with closing(sqlite3.connect(tmp)) as db:
            db.execute('CREATE TABLE samples (stamp INTEGER, segment_end INTEGER, x REAL, y REAL, yaw REAL, '
                       'estimated REAL, progress REAL, lateral REAL, target REAL, reason TEXT)')
            db.executemany('INSERT INTO samples VALUES (?,?,?,?,?,?,?,?,?,?)', _observations(src, task))
            db.execute('CREATE INDEX sample_time ON samples(stamp)')
            db.execute('CREATE TABLE summary AS SELECT count(progress) AS valid_samples FROM samples')
            db.commit()
        if src.resource != source(store, src.run_id, src.line).resource:
            raise ValueError('計算中に元の自己位置検証結果が変更されました')
        task.check()
        os.replace(tmp, src.index)
    finally:
        tmp.unlink(missing_ok=True)
    return dict(run_id=src.run_id, line=src.line)


def _planner_frame(src, time_s, horizon_m):
    if time_s > src.duration or not src.index.is_file():
        raise ValueError('計算済みの走行時間内を選択してください')
    stamp = src.start + round(time_s * 1e9)
    with closing(_open_index(src.index)) as db:
        row = db.execute('SELECT * FROM samples WHERE stamp <= ? ORDER BY stamp DESC, rowid DESC LIMIT 1', (stamp,)).fetchone()
    out = dict(id=src.run_id, line=src.line, time_s=time_s, sample_s=None, valid=False, reason='この時刻は追跡区間外です',
               vehicle=None, progress_m=None, lateral_error_m=None, target_speed_mps=None, estimated_speed_mps=None,
               trajectory=[], trajectory_speeds_mps=[])
    if row is None or stamp > row[1]:
        return out
    observed, _, x, y, yaw, speed, progress, lateral, target, reason = row
    if stamp - observed > round(_core().MAX_AGE_S * 1e9):
        return dict(out, reason='この時刻の自己位置が古くなっています')
    out.update(sample_s=(observed-src.start)/1e9, estimated_speed_mps=speed, reason=reason)
    if yaw is not None:
        out['vehicle'] = dict(x=x, y=y, yaw=yaw)
    if progress is not None:
        points, speeds = src.reference.preview(progress, horizon_m)
        out.update(valid=True, progress_m=progress, target_speed_mps=target, lateral_error_m=lateral,
                   trajectory=points, trajectory_speeds_mps=speeds)
    return out


@router.get('/localization/runs/{run_id}/planning/frame')
def frame(request: Request, run_id: str, line: Line, time_s: float = Query(ge=0, allow_inf_nan=False),
          min_lookahead_m: float | None = Query(None, gt=0, allow_inf_nan=False),
          lookahead_time_s: float | None = Query(None, ge=0, allow_inf_nan=False),
          max_lookahead_m: float | None = Query(None, gt=0, allow_inf_nan=False),
          wheelbase_m: float | None = Query(None, gt=0, allow_inf_nan=False)):
    core = _core('atd_control')
    overrides = dict(min_lookahead_m=min_lookahead_m, lookahead_time_s=lookahead_time_s,
                     max_lookahead_m=max_lookahead_m)
    config = core.LookaheadConfig(**{key: value for key, value in overrides.items() if value is not None})
    src = source(request.app.state.store, run_id, line)
    out = _planner_frame(src, time_s, max(_core().DEFAULT_HORIZON_M, config.max_lookahead_m))
    if out['valid']:
        control = core.pure_pursuit(out['trajectory'], **out['vehicle'], speed_mps=out['estimated_speed_mps'],
                                    wheelbase_m=wheelbase_m, config=config)
    else:
        control = core.unavailable(out['reason'])
    out['control'] = dict(control, config=dict(asdict(config), wheelbase_m=wheelbase_m))
    return out

"""同じ別走行・ラインの全観測で FF + PID を一度計算し、任意時刻を比較する。"""
from contextlib import closing, ExitStack
from dataclasses import asdict, dataclass
from functools import lru_cache
import hashlib
import json
import os
from pathlib import Path
import sqlite3
from tempfile import TemporaryDirectory
from typing import Annotated

from fastapi import APIRouter, Query, Request
from pydantic import BaseModel, ConfigDict, Field

from . import planning_replay as planning
from .jobs import ACTIVE
from .speed_tables import HASH, load_table
from .store import confined

router = APIRouter()
GAIN = Annotated[float, Field(ge=0, allow_inf_nan=False)]


class Comparison(BaseModel):
    model_config = ConfigDict(extra='forbid')
    line: planning.Line
    table_sha256: str = Field(pattern=HASH)
    kp: GAIN | None = None
    ki: GAIN | None = None
    kd: GAIN | None = None


class Removal(Comparison):
    index_id: str = Field(pattern=HASH)


class Seek(Comparison):
    time_s: float = Field(ge=0, allow_inf_nan=False)


def _speed():
    return planning._core('atd_control', 'speed')


def _manual():
    return planning._core('atd_control', 'manual_inputs')


def _key(parts):
    return hashlib.sha256(json.dumps(parts, sort_keys=True, allow_nan=False).encode()).hexdigest()


@lru_cache(maxsize=1)
def _algorithms():
    core = Path(_speed().__file__)
    return tuple(hashlib.sha256(p.read_bytes()).hexdigest() for p in
                 (core, Path(_manual().__file__), core.with_name('bag_propo.py'), Path(__file__)))


@lru_cache(maxsize=16)
def _metadata_digest(path, signature):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _recording(store, report):
    bag = confined(store.record_root, report['bag'], exists=False)
    metadata = confined(bag, 'metadata.yaml', exists=False)
    if not metadata.is_file():
        return None, None, '元の bag が Notebook にないため、手動入力は欠測です'
    if _metadata_digest(metadata, planning._signature(metadata)) != report['metadata_sha256']:
        return None, None, 'bag の metadata が自己位置検証時と異なるため、手動入力は欠測です'
    files = []
    for path in sorted(bag.rglob('*')):
        if path.is_symlink():
            raise ValueError('bag 内の symlink は読み込めません')
        if path.is_file():
            files.append((str(path.relative_to(bag)), planning._signature(path)))
    _, manual, extractor, _ = _algorithms()
    return bag, _key([report['bag'], files, manual, extractor]), ''


@dataclass
class Source:
    plan: planning.Source
    table: object
    config: object
    bag: Path | None
    input_index: Path | None
    input_reason: str
    index: Path

    @property
    def resource(self):
        return 'speed-replay:' + self.index.stem


def source(store, run_id, choice):
    plan = planning.source(store, run_id, choice.line)
    doc = load_table(store, choice.table_sha256)
    config = _speed().PIDConfig(**{k: getattr(choice, k) for k in ('kp', 'ki', 'kd') if getattr(choice, k) is not None})
    bag, fingerprint, issue = _recording(store, plan.report)
    directory = confined(store.state_root, 'speed', exists=False)
    key = _key([_algorithms(), plan.index.stem, doc, asdict(config), fingerprint, issue])
    return Source(plan, _speed().FeedforwardTable.from_document(doc), config, bag,
                  directory / ('inputs-' + fingerprint + '.sqlite') if fingerprint else None,
                  issue, planning.result_index(store, plan.folder, 'speed', key + '.sqlite'))


def _status(src, jobs):
    ready = src.index.is_file()
    result = dict(id=src.plan.run_id, line=src.plan.line, index_id=src.index.stem, duration_s=src.plan.duration,
                  table_sha256=src.table.proposal_sha256, table_run_id=src.table.run_id,
                  config=asdict(src.config), ready=ready, planning_ready=src.plan.index.is_file(),
                  propo_reason=src.input_reason, joy_reason=src.input_reason, job_id=None)
    if ready:
        with closing(planning._open_index(src.index)) as db:
            result['propo_reason'], result['joy_reason'] = db.execute('SELECT propo_reason,joy_reason FROM summary').fetchone()
    else:
        job = next((j for j in jobs.list() if src.resource in j['resources']), None)
        if job:
            result['job_id'] = job['id']
    return result


@router.get('/localization/runs/{run_id}/speed')
def status(request: Request, run_id: str, choice: Annotated[Comparison, Query()]):
    return _status(source(request.app.state.store, run_id, choice), request.app.state.jobs)


@router.delete('/localization/runs/{run_id}/speed')
def remove(request: Request, run_id: str, choice: Annotated[Removal, Query()]):
    store, jobs = request.app.state.store, request.app.state.jobs
    with store.lock('analysis:localization:' + run_id), jobs.registry():
        src = source(store, run_id, choice)
        if src.index.stem != choice.index_id:
            raise ValueError('速度比較の元データが変わりました。結果を再読み込みしてください。')
        jobs.require_available([src.resource])
        src.index.unlink(missing_ok=True)
    return dict(deleted=run_id, line=choice.line)


@router.post('/localization/runs/{run_id}/speed')
def start(request: Request, run_id: str, choice: Comparison):
    store, jobs = request.app.state.store, request.app.state.jobs
    with store.lock('analysis:localization:' + run_id):
        src = source(store, run_id, choice)
        if not src.plan.index.is_file():
            raise ValueError('先にこの走行・ラインの前方軌道を計算してください')
        result = _status(src, jobs)
        if result['ready'] or (result['job_id'] and jobs.get(result['job_id'])['status'] in ACTIVE | {'orphaned'}):
            return result
        job = jobs.start('speed_replay', dict(run_id=run_id, selection=choice.model_dump(), resource=src.resource),
                         src.plan.job_resources(src.resource), f'速度制御の比較: {src.plan.report["bag"]} / {choice.line}')
        return dict(result, job_id=job['id'])


def _extract(store, task, src, working):
    if src.input_index is None:
        return
    src.input_index.parent.mkdir(parents=True, exist_ok=True)
    with store.lock(src.input_index):
        if src.input_index.exists():
            return
        path = working / 'inputs.sqlite'
        task.run(['ros2', 'run', 'atd_control', 'extract_propo', '--bag', str(src.bag), '--output', str(path)])
        task.check()
        # colcon に古い抽出器が残っていても、現行版のキャッシュとして登録しない。
        try:
            with closing(planning._open_index(path)) as db:
                db.execute('SELECT topic,messages FROM topics LIMIT 0')
                db.execute('SELECT stamp,path,raw,reason FROM propo LIMIT 0')
                db.execute('SELECT effective,stamp,throttle,enabled,reason FROM joy LIMIT 0')
        except sqlite3.Error as exc:
            raise ValueError('手動入力の抽出形式を確認できません。Notebook で atd_control を再ビルドしてください') from exc
        # 元の bag が抽出中に変わった場合は共有キャッシュに登録しない。
        _, digest, _ = _recording(store, src.plan.report)
        if src.input_index.name != 'inputs-' + str(digest) + '.sqlite':
            raise ValueError('抽出中に元の bag が変更されました')
        os.replace(path, src.input_index)


class InputCursor:
    """適用時刻順の入力を一度だけ走査する。各行の先頭は適用時刻、その後が元観測。"""
    def __init__(self, rows):
        self.rows = iter(rows)
        self.upcoming = next(self.rows, None)
        self.latest = None

    def at(self, stamp):
        while self.upcoming and self.upcoming[0] <= stamp:
            self.latest = self.upcoming[1:]
            self.upcoming = next(self.rows, None)
        return self.latest


def _measurements(src, planner, pico, joy, missing, task):
    speed = _speed()
    controller = speed.SpeedController(src.table, src.config)
    pico, joy = InputCursor(pico), InputCursor(joy)
    next_interval = 0
    segment = -1
    query = 'SELECT stamp,segment_end,target,estimated,reason FROM samples ORDER BY stamp,rowid'
    for i, (stamp, end, target, estimate, reason) in enumerate(planner.execute(query)):
        if i % 128 == 0:
            task.check()
        if i == next_interval:
            segment += 1
            next_interval += src.plan.report['tracking_intervals'][segment]['samples']
            controller.reset()
        if target is None:
            controller.reset()
            result = speed.speed_unavailable(reason)
            inputs = _manual().match_manual(stamp, pico_missing=reason, joy_missing=reason)
        else:
            result = controller.update(stamp, target, estimate, segment)
            inputs = _manual().match_manual(stamp, pico.at(stamp), joy.at(stamp), **missing)
        result.update(time_s=(stamp-src.plan.start)/1e9, segment=segment, **inputs)
        yield stamp, end, json.dumps(result, ensure_ascii=False, allow_nan=False, separators=(',', ':'))


def execute(store, task, payload):
    choice = Comparison.model_validate(payload['selection'])
    src = source(store, payload['run_id'], choice)
    if src.resource != payload['resource'] or not src.plan.index.is_file():
        raise ValueError('元の記録・校正表・前方軌道の版が変わりました。再度選択してください')
    src.index.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix='.speed-', dir=src.index.parent) as directory:
        work = Path(directory)
        _extract(store, task, src, work)
        task.log('全観測を時刻順に処理し、Pico の操作経路・プロポ CH2・DualSense スロットルを照合します。')
        with ExitStack() as stack:
            planner = stack.enter_context(closing(planning._open_index(src.plan.index)))
            pico, joy = (), ()
            missing = dict(pico_missing=src.input_reason, joy_missing=src.input_reason)
            if src.input_index:
                recorded = stack.enter_context(closing(planning._open_index(src.input_index)))
                counts = dict(recorded.execute('SELECT topic,messages FROM topics'))
                if not counts.get('/vehicle/pico/telemetry'):
                    missing['pico_missing'] = 'この bag に Pico telemetry が記録されていません'
                if not counts.get('/teleop/command'):
                    missing['joy_missing'] = 'この bag に DualSense 操作要求が記録されていません'
                pico = recorded.execute('SELECT stamp,stamp,path,raw,reason FROM propo ORDER BY rowid')
                joy_rows = recorded.execute('SELECT effective,stamp,throttle,enabled,reason FROM joy ORDER BY effective,rowid')
                joy = ((effective, stamp, value, bool(enabled) if enabled is not None else None, why)
                       for effective, stamp, value, enabled, why in joy_rows)
            db = stack.enter_context(closing(sqlite3.connect(work / 'index.sqlite')))
            db.execute('CREATE TABLE samples (stamp INTEGER, segment_end INTEGER, sample TEXT)')
            db.executemany('INSERT INTO samples VALUES (?,?,?)', _measurements(src, planner, pico, joy, missing, task))
            db.execute('CREATE INDEX speed_stamp ON samples(stamp)')
            db.execute('CREATE TABLE summary (propo_reason TEXT, joy_reason TEXT)')
            db.execute('INSERT INTO summary VALUES (?,?)', (missing['pico_missing'], missing['joy_missing']))
            db.commit()
        if source(store, src.plan.run_id, choice).resource != src.resource:
            raise ValueError('計算中に元の記録・前方軌道・校正表が変更されました')
        task.check()
        os.replace(work / 'index.sqlite', src.index)
    return dict(run_id=src.plan.run_id, line=src.plan.line, table_sha256=choice.table_sha256)


@router.get('/localization/runs/{run_id}/speed/frame')
def frame(request: Request, run_id: str, choice: Annotated[Seek, Query()]):
    src = source(request.app.state.store, run_id, choice)
    if not src.index.is_file() or choice.time_s > src.plan.duration:
        raise ValueError('計算済みの速度比較と走行時間内の時刻を選択してください')
    stamp = src.plan.start + round(choice.time_s * 1e9)
    with closing(planning._open_index(src.index)) as db:
        history = [json.loads(r[0]) for r in db.execute('SELECT sample FROM samples WHERE stamp BETWEEN ? AND ? '
                   'ORDER BY stamp,rowid', (stamp-5_000_000_000, stamp))]
        row = db.execute('SELECT stamp,segment_end,sample FROM samples WHERE stamp<=? ORDER BY stamp DESC,rowid DESC LIMIT 1',
                         (stamp,)).fetchone()
    if row and stamp <= row[1] and stamp - row[0] <= round(planning._core().MAX_AGE_S * 1e9):
        sample = json.loads(row[2])
    else:
        issue = 'この時刻に有効な自己位置観測がありません'
        sample = dict(_speed().speed_unavailable(issue), time_s=None, segment=None,
                      **_manual().match_manual(stamp, pico_missing=issue, joy_missing=issue))
    return dict(id=run_id, line=choice.line, time_s=choice.time_s, table_sha256=choice.table_sha256,
                sample=sample, history=history)

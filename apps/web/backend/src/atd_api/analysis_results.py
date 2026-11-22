"""Notebook の解析履歴から明示的に選んだ実行の保存物を削除する。"""
from contextlib import ExitStack
import fcntl
from pathlib import Path
import re
from tempfile import TemporaryDirectory
from typing import Literal

from fastapi import APIRouter, HTTPException, Request

from . import calibration, localization_results, replay_results, simulation_benchmark
from .jobs import ACTIVE
from .store import confined, identifier, read_json

router = APIRouter()
ROOTS = dict(replay=replay_results.RUNS, localization=localization_results.RUNS,
             calibration=calibration.RUNS, benchmark=simulation_benchmark.REPORTS)
PARENT_ACTION = dict(localization='localization_validation', calibration='calibration',
                     benchmark='simulation_benchmark')


def _legacy_indexes(jobs, states, run_id):
    """従来の共有ディレクトリの SQLite は、生成要求が証明できるものだけを扱う。"""
    indexes = set()
    for state in states:
        kind = {'planning_replay': 'planning', 'speed_replay': 'speed'}.get(state['action'])
        if not kind:
            continue
        request_file = confined(jobs.root, identifier(state['id']) + '.request.json', exists=False)
        if not request_file.is_file():
            continue
        request = read_json(request_file)
        payload = request['payload']
        if payload.get('run_id') != run_id:
            continue
        resource = payload['resource']
        prefix = kind + '-replay:'
        if resource in state['resources'] and resource.startswith(prefix) and re.fullmatch('[0-9a-f]{64}', resource[len(prefix):]):
            indexes.add(confined(jobs.store.state_root, Path(kind) / (resource[len(prefix):] + '.sqlite'), exists=False))
    return indexes


def require_idle(states, jobs, kind, run_id, line=None):
    for state in states:
        if state['status'] not in ACTIVE | {'orphaned'}:
            continue
        busy = state['action'] == PARENT_ACTION.get(kind) and state['id'] == run_id
        if kind == 'localization' and state['action'] in ('planning_replay', 'speed_replay'):
            payload = read_json(confined(jobs.root, identifier(state['id']) + '.request.json'))['payload']
            job_line = payload.get('line') or payload.get('selection', {}).get('line')
            busy = payload.get('run_id') == run_id and (line is None or job_line == line)
        if kind == 'calibration' and state['action'] == 'calibration' and not busy:
            payload = read_json(confined(jobs.root, identifier(state['id']) + '.request.json'))['payload']
            busy = payload.get('reference_run') == run_id
        if busy:
            raise HTTPException(409, '処理中のジョブがこの解析結果を使用しています。')


@router.delete('/analysis-results/{kind}/{run_id}')
def delete_result(kind: Literal['replay', 'localization', 'calibration', 'benchmark'], run_id: str, request: Request):
    store, jobs = request.app.state.store, request.app.state.jobs
    identifier(run_id)
    folder = confined(store.result_root, ROOTS[kind] / run_id, exists=False)
    if not folder.is_dir():
        raise HTTPException(404, '選択した解析結果が見つかりません。')
    with TemporaryDirectory(prefix='.delete-', dir=folder.parent) as tmp:
        with store.lock(f'analysis:{kind}:{run_id}'), jobs.registry(), ExitStack() as stack:
            if not folder.is_dir():
                raise HTTPException(404, '選択した解析結果が見つかりません。')
            legacy_indexes = set()
            if kind == 'replay':
                summary = replay_results._summary(folder)
                if summary['status'] == 'running':
                    raise HTTPException(409, '再生中の結果は終了後に削除してください。')
                samples = confined(folder, 'samples.jsonl', exists=False)
                if samples.exists():
                    stream = stack.enter_context(samples.open('rb'))
                    try:
                        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError:
                        raise HTTPException(409, '再生結果の保存が終了するまでお待ちください。')
            else:
                states = jobs._states()
                require_idle(states, jobs, kind, run_id)
                if kind == 'localization':
                    localization_results._report(folder)
                    legacy_indexes = _legacy_indexes(jobs, states, run_id)
                elif kind == 'calibration':
                    calibration._report(folder)
                else:
                    report = read_json(confined(folder, 'report.json'))
                    if report.get('schema') != 'atd.simulation_benchmark.v1' or report.get('id') != run_id:
                        raise ValueError('選択したベンチマークの実行 ID を確認できません')
            staged = Path(tmp)
            moved = {}
            try:
                for index in legacy_indexes:
                    if index.is_file():
                        destination = staged / (index.parent.name + '-' + index.name)
                        index.rename(destination)
                        moved[destination] = index
                folder.rename(staged / 'result')
            except OSError:
                for previous, original in moved.items():
                    previous.rename(original)
                raise
    return {'deleted': run_id}

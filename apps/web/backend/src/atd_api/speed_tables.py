"""校正画面で確認した前進用 CH2 表を保存し、比較に使う版を選択する。"""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import re

from fastapi import APIRouter, Request
from pydantic import BaseModel, ConfigDict, Field

from .calibration import feedforward_document
from .live_input import call
from .planning_replay import _core
from .store import atomic_json, confined, read_json

router = APIRouter()
TABLES = Path('runs/vehicle/feedforward')
HASH = r'^[0-9a-f]{64}$'


class TableSelection(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    proposal_sha256: str = Field(pattern=HASH)


class Adoption(TableSelection):
    run_id: str


class LiveSelection(TableSelection):
    instance_id: str = Field(min_length=1)


def _digest(doc):
    data = json.dumps(doc, ensure_ascii=False, sort_keys=True, allow_nan=False, separators=(',', ':'))
    return hashlib.sha256(data.encode()).hexdigest()


def load_table(store, proposal_sha256):
    if not isinstance(proposal_sha256, str) or not re.fullmatch(HASH, proposal_sha256):
        raise ValueError('採用済みの表を選んでください')
    saved = read_json(confined(store.result_root, TABLES / (proposal_sha256 + '.json')))
    doc = saved.get('table') if isinstance(saved, dict) else None
    if (not isinstance(doc, dict) or doc.get('proposal_sha256') != proposal_sha256
            or _digest(doc) != saved.get('table_sha256')):
        raise ValueError('保存したスロットル表の版または内容が一致しません')
    _core('atd_control', 'speed').FeedforwardTable.from_document(doc)
    return doc


def _summary(doc):
    return {k: doc[k] for k in ('run_id', 'bag', 'proposal_sha256', 'speed_min_mps', 'speed_max_mps', 'direction')}


@router.get('/speed/tables')
def tables(request: Request):
    store = request.app.state.store
    folder = confined(store.result_root, TABLES, exists=False)
    items, issues = [], []
    for path in sorted(folder.glob('*.json')):
        try:
            items.append(_summary(load_table(store, path.stem)))
        except (ValueError, OSError, KeyError) as error:
            issues.append(f'{path.name}: {error}')
    return dict(items=sorted(items, key=lambda item: (item['run_id'], item['proposal_sha256'])),
                issues=issues, defaults=asdict(_core('atd_control', 'speed').PIDConfig()))


@router.post('/speed/tables')
def adopt(request: Request, body: Adoption):
    store = request.app.state.store
    doc = feedforward_document(store, body.run_id, body.proposal_sha256)
    # 詳細な走行証跡は校正結果に保持し、監視 node には補間に必要な点と出典を渡す。
    doc['points'] = [{k: p[k] for k in ('speed_mps', 'throttle_raw')} for p in doc['points']]
    _core('atd_control', 'speed').FeedforwardTable.from_document(doc)
    path = confined(store.result_root, TABLES / (body.proposal_sha256 + '.json'), exists=False)
    with store.lock(path):
        if path.exists():
            if load_table(store, body.proposal_sha256) != doc:
                raise ValueError('同じ校正結果 ID に異なるスロットル表が保存されています')
        else:
            atomic_json(path, dict(table=doc, table_sha256=_digest(doc)))
    return dict(table=_summary(doc))


@router.put('/live/speed-table')
def select_live(request: Request, body: LiveSelection):
    table = load_table(request.app.state.store, body.proposal_sha256)
    return call(request, '/live/speed-table', 'PUT', dict(instance_id=body.instance_id, table=table))

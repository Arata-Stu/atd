"""別走行検証の確定レポートと観測軌跡を読む。元の ns は整数のまま照合する。"""
from bisect import bisect_left
import csv
import hashlib
import json
import math
from pathlib import Path
import re

from fastapi import APIRouter, Request

from .store import confined, identifier, read_json

router = APIRouter()
RUNS = Path('runs/localization')
OUTCOMES = ('tracked_to_end', 'partial_tracking', 'no_tracking', 'failed', 'cancelled')
STATES = ('waiting_hint', 'localizing', 'tracking', 'lost', 'stale')
SAMPLE_BUDGET = 6000


def _report(folder):
    value = read_json(confined(folder, 'report.json'))
    if not isinstance(value, dict) or value.get('schema') != 'atd.localization.validation.v1':
        raise ValueError('自己位置検証のレポート形式を確認してください')
    identifier(value.get('map_id'))
    if (value.get('outcome') not in OUTCOMES or value.get('frame') != 'map'
            or value.get('body_frame') != 'base_link' or not all(
                isinstance(value.get(k), str) and re.fullmatch('[0-9a-f]{64}', value[k])
                for k in ('map_sha256', 'hdmap_sha256', 'metadata_sha256', 'source_metadata_sha256'))
            or not all(isinstance(value.get(k), str) and value[k] for k in ('bag', 'source_bag'))):
        raise ValueError('自己位置検証の出典または実行結果が不正です')
    return value


def _entry(run_id, report):
    return dict(id=run_id, map_id=report['map_id'], bag=report['bag'], outcome=report['outcome'],
                map_correction_source=report.get('map_correction_source', report.get('backend', 'cuvslam')),
                vgl_frequency_hz=report.get('vgl_frequency_hz'))


def list_runs(store, map_id=None):
    if map_id is not None:
        identifier(map_id)
    root = confined(store.result_root, RUNS, exists=False)
    entries = []
    if root.is_dir():
        for folder in root.iterdir():
            if folder.is_symlink() or not folder.is_dir() or folder.name.startswith('.'):
                continue
            try:
                run_id = identifier(folder.name)
                report = _report(folder)
                saved_at = (folder / 'report.json').stat().st_mtime
            except (ValueError, OSError):
                continue
            if map_id is None or report['map_id'] == map_id:
                entries.append(dict(_entry(run_id, report), saved_at=saved_at))
    return sorted(entries, key=lambda entry: (entry['saved_at'], entry['id']), reverse=True)


def _points(value):
    return isinstance(value, list) and all(
        isinstance(xy, list) and len(xy) == 2 and all(type(v) in (int, float) and math.isfinite(v) for v in xy)
        for xy in value)


def _map(store, folder, report):
    # A byte-identical snapshot still describes this run after edits to the current map.
    try:
        snapshot = confined(folder, 'hdmap.json', exists=False)
        if snapshot.exists():
            raw = snapshot.read_bytes()
            if hashlib.sha256(raw).hexdigest() != report['hdmap_sha256']:
                raise ValueError('保存した HDMap の内容が検証時の SHA256 と一致しません')
            doc = json.loads(raw)
        else:
            current = store.live_reference(report['map_id'])
            if any(current[key] != report[key] for key in ('hdmap_sha256', 'map_sha256')):
                raise ValueError('現在の地図・Raceline が検証時の版と一致しません')
            doc = current['document']
        if not isinstance(doc, dict) or doc.get('id') != report['map_id'] or doc.get('frame') != 'map':
            raise ValueError('検証時の地図 ID・座標系と一致しません')
        if (not all(_points(doc.get(k)) for k in ('left', 'right', 'custom')) or not isinstance(doc.get('lines'), dict)
                or any(not isinstance(line, dict) or not _points(line.get('points')) for line in doc['lines'].values())):
            raise ValueError('保存した HDMap の境界・ライン点列が不正です')
        return doc, None
    except (ValueError, OSError) as error:
        return None, str(error)


def _uint(value):
    if type(value) is not int or value < 0:
        raise ValueError('検証レポートの時刻・試料数・追跡世代が不正です')
    return value


def _metric(value):
    if type(value) not in (float, int) or not math.isfinite(value) or value < 0:
        raise ValueError('検証レポートの集計値が不正です')
    return value


def trajectory_rows(folder, report, start_ns, end_ns, required=()):
    """CSV の全観測をレポートの区間・順序と照合して順に返す。"""
    intervals = report['tracking_intervals']
    total = _uint(report['pose_samples'])
    max_gap_ns = round(_metric(report['gap_tolerance_s']) * 1e9)
    if not isinstance(intervals, list) or report['trajectory'] != 'trajectory.csv':
        raise ValueError('検証レポートの追跡区間・軌跡ファイルが不正です')
    if sum(_uint(i['samples']) for i in intervals) != total:
        raise ValueError('検証レポートの区間と総試料数が一致しません')
    last_stamp, last_epoch = -1, -1
    with confined(folder, 'trajectory.csv').open(newline='') as stream:
        rows = csv.DictReader(stream, strict=True)
        if not {'stamp_ns', 'epoch', 'x_m', 'y_m', *required}.issubset(rows.fieldnames or []):
            raise ValueError('軌跡 CSV の列を確認してください')
        for segment, interval in enumerate(intervals):
            first, end, epoch, count = (_uint(interval[k]) for k in ('first_stamp_ns', 'last_stamp_ns', 'epoch', 'samples'))
            if (count < 1 or not start_ns - 5_000_000 <= first <= end <= end_ns + 5_000_000
                    or first < last_stamp or epoch < last_epoch):
                raise ValueError('追跡区間の範囲または順序が不正です')
            for i in range(count):
                row = next(rows, None)
                if row is None:
                    raise ValueError('軌跡 CSV の試料数がレポートより少なくなっています')
                stamp = int(row['stamp_ns'])
                xy = [float(row['x_m']), float(row['y_m'])]
                if (int(row['epoch']) != epoch or not all(map(math.isfinite, xy))
                        or not first <= stamp <= end or i == 0 and stamp != first
                        or i == count - 1 and stamp != end or i > 0 and not 0 < stamp - last_stamp <= max_gap_ns
                        or last_epoch == epoch and stamp == last_stamp):
                    raise ValueError('軌跡 CSV の時刻・世代・位置が追跡区間と一致しません')
                last_stamp, last_epoch = stamp, epoch
                yield segment, i, stamp, xy, row
        if next(rows, None) is not None:
            raise ValueError('軌跡 CSV の試料数がレポートより多くなっています')


def _segments(folder, report, start_ns, end_ns):
    """各区間の始点と終点を残し、表示する軌跡の内部だけを間引く。"""
    stride = max(1, (_uint(report['pose_samples']) + SAMPLE_BUDGET - 1) // SAMPLE_BUDGET)
    segments = []
    for segment, index, _, xy, _ in trajectory_rows(folder, report, start_ns, end_ns):
        interval = report['tracking_intervals'][segment]
        if index == 0:
            segments.append(dict(epoch=interval['epoch'], start_s=(interval['first_stamp_ns']-start_ns)/1e9,
                                 end_s=(interval['last_stamp_ns']-start_ns)/1e9, samples=interval['samples'], points=[]))
        if index == interval['samples'] - 1 or index % stride == 0:
            segments[-1]['points'].append(xy)
    return segments


def _interruptions(events, intervals):
    # 非追跡状態の stamp は通知時刻、復帰は観測時刻。両者の時刻順を仮定しない。
    # 状態通知の順に、喪失/更新途絶を次の追跡開始サンプルと対応させる。
    starts = [(row['epoch'], row['first_stamp_ns']) for row in intervals]
    resumed = {}
    last_anchor = -1
    tracking_seen = False
    cause = None
    priority = {'stale': 1, 'lost': 2}
    for e in events:
        state = e['state']
        if state == 'tracking':
            # 最初の tracking 通知がすでに CSV に記録済みの stamp の場合は重複が除かれ、
            # この区間の CSV は次の観測から始まる。新しい姿勢がなければ直前の中断を保持する。
            i = bisect_left(starts, (e['epoch'], e['stamp_ns']), lo=last_anchor + 1)
            if i < len(starts) and starts[i][0] == e['epoch']:
                if cause and tracking_seen:
                    resumed[i] = cause
                cause = None
                last_anchor = i
                tracking_seen = True
        elif tracking_seen and (cause is None or priority.get(state, 0) > priority.get(cause['state'], 0)):
            cause = e
    return resumed, cause


def _gap(before, after, end_s, cause):
    kind = cause['state'] if cause and cause['state'] in ('lost', 'stale') else 'gap'
    reason = cause['reason'] if cause else '有効な姿勢の観測が途切れています'
    left = dict(time_s=before['end_s'], point=before['points'][-1])
    right = dict(time_s=after['start_s'], point=after['points'][0]) if after else None
    return dict(kind=kind, reason=reason, start_s=left['time_s'], end_s=right['time_s'] if right else max(end_s, left['time_s']),
                before=left, after=right)


def load_result(store, run_id):
    folder = confined(store.result_root, RUNS / identifier(run_id))
    report = _report(folder)
    doc, issue = _map(store, folder, report)
    result = dict(**_entry(run_id, report), source_bag=report['source_bag'], map_sha256=report['map_sha256'],
                  hdmap_sha256=report['hdmap_sha256'], document=doc, map_issue=issue,
                  error=report.get('error'), segments=[], gaps=[], states=[], pose_samples=None, displayed_samples=0,
                  first_tracking_delay_s=None, tracking_time_fraction=None, stereo_duration_s=None)
    if report['outcome'] in ('failed', 'cancelled'):
        return result
    try:
        start, end = (_uint(report['stereo'][k]) for k in ('first_stamp_ns', 'last_stamp_ns'))
        events = report['state_changes']
        fraction = _metric(report['tracking_time_fraction'])
        if (start >= end or not isinstance(events, list) or fraction > 1 or type(report['tail_tracked']) is not bool
                or any(e['state'] not in STATES or not isinstance(e['reason'], str) for e in events)):
            raise ValueError('検証レポートの時刻・状態遷移または追跡率が不正です')
        duration = (end - start) / 1e9
        states = [dict(time_s=max(0., min(duration, (_uint(e['stamp_ns'])-start)/1e9)), epoch=_uint(e['epoch']),
                       state=e['state'], reason=e['reason']) for e in events]
        segments = _segments(folder, report, start, end)
        if (bool(segments) != (report['outcome'] != 'no_tracking')
                or report['tail_tracked'] != (report['outcome'] == 'tracked_to_end')):
            raise ValueError('検証の判定と追跡試料・末尾状態が一致しません')
        resumed, tail = _interruptions(events, report['tracking_intervals'])
        gaps = [_gap(before, after, duration, resumed.get(index))
                for index, (before, after) in enumerate(zip(segments, segments[1:]), start=1)]
        # A single fix exactly at the end has zero duration; it is not evidence of a loss.
        if segments and not report['tail_tracked'] and (tail or duration - segments[-1]['end_s'] > _metric(report['tail_tolerance_s'])):
            gaps.append(_gap(segments[-1], None, duration, tail))
        return dict(result, segments=segments, states=states, gaps=gaps, pose_samples=report['pose_samples'],
                    displayed_samples=sum(len(s['points']) for s in segments), stereo_duration_s=duration,
                    first_tracking_delay_s=_metric(report['first_tracking_delay_s']) if segments else None,
                    tracking_time_fraction=fraction)
    except (KeyError, TypeError, csv.Error, OverflowError) as error:
        raise ValueError('自己位置検証のレポートと軌跡 CSV を照合できません') from error


@router.get('/localization/runs')
def history(request: Request, map_id: str | None = None):
    return dict(items=list_runs(request.app.state.store, map_id))


@router.get('/localization/runs/{run_id}')
def result(request: Request, run_id: str):
    return load_result(request.app.state.store, run_id)

"""Notebook の results に保存した Gazebo 閉ループ試行の読み取り専用 API。"""
import hashlib
import json
from pathlib import Path

from fastapi import APIRouter, Request
from atd_processing.geometry import points
from atd_processing.gazebo_review import (
    DisplaySeries, GroundTruth, LEGACY_FOOTPRINT, PoseComparison, Road, mesh_boundaries,
    motion_sample, number, timing_sample,
)

from .store import confined, identifier

ROOT = Path('runs/gazebo')
router = APIRouter(prefix='/simulation/gazebo-runs')
LINES = ('raceline', 'centerline', 'custom')


def artifact(folder, name):
    return confined(folder, name, exists=False)


def read_bytes(path):
    if path.stat().st_size > 32 * 1024 * 1024:
        raise ValueError(f'{path.name} が大きすぎます')
    return path.read_bytes()


def document(folder, name):
    path = artifact(folder, name)
    if not path.is_file():
        return {}
    obj = json.loads(read_bytes(path))
    if not isinstance(obj, dict):
        raise ValueError(f'{name} の JSON の形式が不正です')
    return obj


def rows(folder, name, warnings):
    path = artifact(folder, name)
    if path.is_file():
        with path.open('rb') as stream:
            while line := stream.readline(1 << 20):
                if len(line) == 1 << 20:
                    raise ValueError(f'{name} の記録行が大きすぎます')
                if not line.endswith(b'\n'):
                    warnings.append(f'{name} の書き込み途中の末尾行を除外しました。')
                    break
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError(f'{name} の記録行が不正です')
                yield row


def label(value):
    return value if isinstance(value, str) else None


def metadata(folder):
    host = document(folder, 'closed_loop.host.json')
    result = document(folder, 'closed_loop.result.json')
    identity = document(folder, 'closed_loop.reference.json')
    # run.json is also written when startup fails before the trial can emit a result.
    job = document(folder, 'run.json') if not result and not host else {}
    closed = bool(result or host or identity or job.get('mode') == 'closed-loop' or '-closed-loop-' in folder.name)
    if not closed:
        return None
    if not result and isinstance(job.get('closed_loop'), dict):
        result = job['closed_loop']
    status = label(result.get('status')) or label(host.get('status')) or ('error' if job.get('error') else 'unknown')
    times = result.get('lap_times_s')
    if not isinstance(times, list) or any(number(t) is None or t < 0 for t in times):
        times = []
    info = dict(id=folder.name, circuit=folder.parent.name, status=status,
                reason=label(result.get('reason')) or label(host.get('reason')) or label(job.get('error')) or '',
                started=number(host.get('started')), map_id=label(identity.get('map_id') or host.get('map_id')),
                line=label(identity.get('line') or host.get('line')), max_speed_mps=number(host.get('max_speed_mps')),
                laps_requested=number(result.get('laps_requested', job.get('laps_requested'))),
                laps_completed=number(result.get('laps_completed')), distance_m=number(result.get('distance_m')),
                lap_times_s=times)
    return info, result, identity


@router.get('')
def list_runs(request: Request):
    root = artifact(request.app.state.store.result_root, ROOT)
    items = []
    for folder in root.glob('*/*'):
        if folder.is_symlink() or folder.parent.is_symlink() or not folder.is_dir():
            continue
        try:
            identifier(folder.name)
            identifier(folder.parent.name)
            info = metadata(folder)
        except (OSError, ValueError):
            # One incomplete or malformed archive must not hide other runs.
            continue
        if info:
            items.append((info[0]['started'] or folder.stat().st_mtime, info[0]))
    return dict(items=[item for _, item in sorted(items, key=lambda pair: pair[0], reverse=True)])


def road_for(folder, warnings):
    try:
        scene = document(folder, 'scene/scene.json')
        evaluation = scene.get('course_evaluation')
        if evaluation is not None:
            if not isinstance(evaluation, dict) or evaluation.get('frame') != 'gazebo_world':
                raise ValueError('評価境界の座標系が不正です')
            return Road(evaluation, evaluation.get('footprint'))
        mesh = artifact(folder, 'scene/floor.obj')
        if mesh.is_file():
            if mesh.stat().st_size > 16 * 1024 * 1024:
                raise ValueError('舗装メッシュが大きすぎます')
            with mesh.open() as stream:
                boundaries = mesh_boundaries(stream)
            road = Road(boundaries, LEGACY_FOOTPRINT)
            warnings.append('旧形式のため、保存済みの舗装メッシュと当時の評価器の車体寸法（全長0.35 m・幅0.19 m・後端まで0.1065 m）から境界距離を復元しました。')
            return road
        raise ValueError('走行時の舗装境界がありません')
    except (ValueError, OSError) as error:
        warnings.append(f'車体の境界距離を表示できません。{error}')
        return None


def reference(store, folder, run, identity, comparison, warnings):
    if comparison.anchor is None:
        return [], 'missing'
    try:
        map_id = identifier(run['map_id'])
        line = run['line']
        if line not in LINES:
            raise ValueError('走行時に選択したラインが不明です')
        expected = identity.get('hdmap_sha256')
        snapshot = artifact(folder, 'hdmap.json')
        if snapshot.is_file():
            raw = read_bytes(snapshot)
            if not expected or hashlib.sha256(raw).hexdigest() != expected or identity.get('frame') != 'map':
                raise ValueError('試行の地図コピーと記録した SHA-256 または座標系が一致しません')
            source = 'snapshot'
        else:
            current = artifact(store.result_root, Path('maps') / map_id / 'hdmap/map.json')
            if not current.is_file():
                raise ValueError(f'{map_id} の走行時コピーと Notebook の地図を確認できません')
            raw = read_bytes(current)
            if expected and hashlib.sha256(raw).hexdigest() == expected:
                source = 'verified_map'
            else:
                source = 'current_map'
                warnings.append('走行時と同一のラインを確認できません。現在の同名 HDMap を参考表示に使えます。')
        doc = json.loads(raw)
        if not isinstance(doc, dict) or doc.get('id') != map_id or doc.get('frame') != 'map' or doc.get('closed') is not True:
            raise ValueError('地図 ID・座標系・閉路の指定が一致しません')
        lines = doc.get('lines')
        saved = lines.get(line) if isinstance(lines, dict) else None
        if not isinstance(saved, dict):
            raise ValueError(f'{map_id} に {line} の点列がありません')
        coords = points(saved.get('points'), minimum=3)
        return [comparison.to_world(*p) for p in coords], source
    except (ValueError, OSError) as error:
        warnings.append(f'選択ラインを表示できません。{error}')
        return [], 'missing'


def review(store, circuit, key):
    folder = confined(store.result_root, ROOT / identifier(circuit) / identifier(key))
    meta = metadata(folder) if folder.is_dir() else None
    if not meta:
        raise ValueError('閉ループの試行が見つかりません')
    run, result, identity = meta
    contact = None
    if 'off_course' in (result.get('status'), result.get('course_outcome_during_stop')):
        contact = number(result.get('finish_sim_s', result.get('sim_time_s')))
    warnings = []
    truth = GroundTruth()
    motion = DisplaySeries(contact)
    trace_file = 'scene/rig_motion.jsonl'
    for row in rows(folder, trace_file, warnings):
        motion.append(motion_sample(row, truth))
        control = row.get('control_state')
        if isinstance(control, dict):
            for field in ('line', 'map_id'):
                if not run[field] and label(control.get(field)):
                    run[field] = control[field]
    samples = motion.result()
    if not samples:
        warnings.append('GT と操舵の記録がありません。')
    elif not any(s['image_to_apply_ms'] is not None for s in samples):
        warnings.append('指令を採用した時刻が記録されていないか、有効な指令がありません。')
    road = road_for(folder, warnings)
    if road is not None:
        for sample in samples:
            sample['clearance_m'] = road.clearance(sample)
    comparison = PoseComparison(truth, contact)
    localization = 'closed_loop.localization.jsonl'
    if artifact(folder, localization).is_file():
        for row in rows(folder, localization, warnings):
            comparison.observe(row)
    if comparison.anchor is None and samples:
        for row in rows(folder, trace_file, []):
            state = row.get('control_state')
            if isinstance(state, dict):
                comparison.observe(state, legacy=True)
        if comparison.anchor is not None:
            warnings.append('独立した自己位置ログに対応点がないため、保存された制御状態に含まれる画像時刻の推定姿勢を使っています。')
    if comparison.anchor is None:
        warnings.append('同じシミュレーション時刻で GT と対応する推定姿勢がなく、位置合わせできません。')
    ref, source = reference(store, folder, run, identity, comparison, warnings)
    timing = DisplaySeries(contact)
    last_t = None
    for row in rows(folder, 'closed_loop.vslam.jsonl', warnings):
        vslam = timing_sample(row)
        if vslam is not None and (last_t is None or vslam['t'] > last_t):
            timing.append(vslam)
            last_t = vslam['t']
    if not timing.count:
        warnings.append('画像に対応する cuVSLAM の実行時間は記録されていません。')
    if max(motion.stride, comparison.series.stride, timing.stride) > 1:
        warnings.append('長い記録は全体表示を間引いています。接触判定の前後10秒は記録の各時刻を保持しています。')
    return dict(run=run, samples=samples, estimates=comparison.series.result(), vslam=timing.result(),
                contact_s=contact, anchor_s=comparison.anchor_s, reference=ref, reference_source=source,
                warnings=warnings, boundaries=road.boundaries if road else dict(left=[], right=[]),
                footprint=road.footprint if road else None)


@router.get('/{circuit}/{key}')
def detail(circuit: str, key: str, request: Request):
    return review(request.app.state.store, circuit, key)

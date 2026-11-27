"""Notebook の results に保存した Gazebo 閉ループ試行の読み取り専用 API。"""
import hashlib
import json
import math
from pathlib import Path

from fastapi import APIRouter, Request
from atd_processing.geometry import points
from atd_processing.gazebo_review import (
    DisplaySeries, GroundTruth, LEGACY_FOOTPRINT, PoseComparison, Road, mesh_boundaries,
    motion_sample, number, timing_sample,
)

from .planning_replay import _core
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


def raceline_speeds(doc, saved, point_count):
    if 'speeds' not in saved:
        try:
            return _core().ReferenceLine.from_document(doc, 'raceline').speeds
        except ValueError:
            return []
    speeds = saved['speeds']
    if (not isinstance(speeds, list) or len(speeds) != point_count
            or any(number(v) is None or v < 0 for v in speeds)):
        return []
    return speeds


def references(store, folder, run, identity, selections, comparison, warnings):
    if comparison.anchor is None:
        return {}, 'missing'
    try:
        map_id = identifier(run['map_id'])
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
        saved_lines = doc.get('lines')
        if not isinstance(saved_lines, dict):
            raise ValueError(f'{map_id} に走行ラインがありません')
    except (ValueError, OSError) as error:
        warnings.append(f'選択ラインを表示できません。{error}')
        return {}, 'missing'
    output = {}
    for name in selections:
        try:
            saved = saved_lines.get(name)
            if not isinstance(saved, dict):
                raise ValueError(f'{map_id} に {name} の点列がありません')
            coords = points(saved.get('points'), minimum=3)
            output[name] = dict(points=[comparison.to_world(*p) for p in coords],
                                speeds_mps=raceline_speeds(doc, saved, len(coords)) if name == 'raceline' else [])
        except ValueError as error:
            warnings.append(f'{name} を表示できません。{error}')
    return output, source if output else 'missing'


def ekf_review(folder, identity, comparison, truth, contact, warnings):
    """Use the bag tool's image-time comparison and the existing map-to-world anchor."""
    if comparison.anchor is None or not artifact(folder, 'closed_loop.ekf.jsonl').is_file():
        return [], []
    if not identity.get('map_sha256') or not identity.get('map_id'):
        warnings.append('EKF と対応させる保存地図を確認できません。')
        return [], []
    _core('atd_mapping', 'frames')
    core = _core('atd_localization', 'rpm_comparison')

    def odometry(name):
        for row in rows(folder, f'closed_loop.{name}.jsonl', warnings):
            p = core.RigidPose.checked(row['xyz'], row['xyzw'])
            if name == 'ekf':
                yield row['stamp_ns'], (row['frame'], core.planar(p))
            elif row['frame'] == 'odom':
                yield row['stamp_ns'], p

    def statuses():
        for row in rows(folder, 'closed_loop.localization.jsonl', warnings):
            tracking = (row['session_id'], row['epoch'], identity['map_id'], identity['map_sha256'])
            p = None
            if row['state'] == 'tracking':
                p = core.RigidPose.checked((row['x_m'], row['y_m'], row['z_m']), row['orientation_xyzw'])
            yield row['header_stamp_ns'], tracking, p

    commands = core.StampCursor((r['stamp_ns'], r['steering_rad'])
                                for r in rows(folder, 'closed_loop.commands.jsonl', warnings))
    series = DisplaySeries(contact)
    peaks = {}
    anchors = {}

    def save_anchor(frame, ns, tx, ty, angle):
        anchors[frame] = (ns, tx, ty, angle, len(anchors))


    last_key, segment = None, 0
    for values in core.aligned_stream(odometry('ekf'), odometry('vo'), statuses(), on_anchor=save_anchor):
        row = dict(zip(core.FIELDS, values, strict=True))
        ns = row['stamp_ns']
        gt = truth.at(ns)
        if gt is None:
            continue
        key = row['ekf_frame'], row['vslam_session'], row['epoch']
        if key != last_key:
            segment += 1
            last_key = key
        x, y = comparison.to_world(row['aligned_ekf_x_m'], row['aligned_ekf_y_m'])
        yaw = math.remainder(row['aligned_ekf_yaw_rad'] + comparison.anchor[-1], math.tau)
        item = dict(t=ns/1e9, segment=segment, x=x, y=y, yaw_rad=yaw,
                    lateral_error_m=-math.sin(gt[2])*(x-gt[0]) + math.cos(gt[2])*(y-gt[1]),
                    yaw_error_rad=math.remainder(yaw-gt[2], math.tau), steering_rad=commands.exact(ns))
        for field in ('correction_forward_m', 'correction_left_m', 'correction_yaw_deg'):
            item[field] = number(row[field])
            if item[field] is not None and (field not in peaks or abs(item[field]) > abs(peaks[field][field])):
                peaks[field] = item
        series.append(item)
    # Preserve the largest correction on each axis even in long, decimated runs.
    retained = {item['t']: item for item in series.result()}
    retained.update((item['t'], item) for item in peaks.values())
    # The EKF keeps running during visual tracking loss. Use each local origin's
    # fixed alignment for its full trajectory after the first common observation.
    path = DisplaySeries(contact)
    for ns, (frame, (px, py, yaw)) in odometry('ekf'):
        anchor = anchors.get(frame)
        if anchor is None or ns < anchor[0]:
            continue
        _, tx, ty, angle, segment = anchor
        dx, dy = core.rotate_xy(px, py, angle)
        x, y = comparison.to_world(tx+dx, ty+dy)
        path.append(dict(t=ns/1e9, segment=segment, x=x, y=y,
                         yaw_rad=math.remainder(yaw+angle+comparison.anchor[-1], math.tau)))
    return [retained[t] for t in sorted(retained)], path.result()


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
    changes = []
    active = run['line']
    last_stamp = -1
    for entry in rows(folder, 'closed_loop.lines.jsonl', warnings):
        ns, name = entry.get('stamp_ns'), entry.get('line')
        if type(ns) is not int or not 0 <= ns < 2**31 * 10**9 or ns < last_stamp or name not in LINES:
            raise ValueError('参照ライン変更履歴の時刻またはライン名が不正です')
        last_stamp = ns
        if name != active:
            changes.append(dict(t=ns/1e9, line=name))
            active = name
    names = {item['line'] for item in changes}
    if run['line'] in LINES:
        names.add(run['line'])
    lines, source = references(store, folder, run, identity, sorted(names), comparison, warnings)
    initial = lines.pop(run['line'], {})
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
    ekf, ekf_path = ekf_review(folder, identity, comparison, truth, contact, warnings)
    if not ekf and artifact(folder, 'closed_loop.ekf.jsonl').is_file():
        warnings.append('EKF と VSLAM・GT に共通する観測がありません。IMU 設定とセンサー入力を確認してください。')
    return dict(run=run, samples=samples, ekf=ekf, ekf_path=ekf_path, estimates=comparison.series.result(), vslam=timing.result(),
                contact_s=contact, anchor_s=comparison.anchor_s, reference=initial.get('points', []),
                reference_speeds_mps=initial.get('speeds_mps', []), reference_source=source,
                line_changes=changes, additional_references=lines,
                warnings=warnings, boundaries=road.boundaries if road else dict(left=[], right=[]),
                footprint=road.footprint if road else None)


@router.get('/{circuit}/{key}')
def detail(circuit: str, key: str, request: Request):
    return review(request.app.state.store, circuit, key)

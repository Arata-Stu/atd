"""Notebook で bag と連続 VO 姿勢を照合し、車両校正レポートを作る。"""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from uuid import uuid4

from atd_processing.geometry import validate_raceline_limits
from atd_processing.vehicle_limits import derive_candidates

from . import offline
from .store import confined, identifier, read_json


RUNS = Path('runs/vehicle_calibration')
POSE_FILE = offline.ODOM_POSES
ARTIFACTS = ('summary.json', 'response.png', 'timeseries.csv')


class InvalidPoseEvidence(ValueError):
    """bag に対応する記録済み姿勢の証跡が壊れており、自動生成にも流用できない。"""


def _valid_frame(frame):
    return isinstance(frame, str) and offline.FRAME.fullmatch(frame) and '//' not in frame and not frame.endswith('/')


def _bag(store, relative):
    path = confined(store.record_root, relative)
    metadata = confined(path, 'metadata.yaml')
    if not metadata.is_file() or not metadata.stat().st_size:
        raise ValueError('記録停止後の metadata.yaml がある bag を選択してください')
    return path.relative_to(store.record_root).as_posix(), hashlib.sha256(metadata.read_bytes()).hexdigest()


def _pose(store, key, bag, digest):
    folder = confined(store.result_root, Path('maps') / identifier(key))
    manifest = read_json(confined(folder, 'mapping.json'))
    source = manifest.get('source') if isinstance(manifest, dict) else None
    if (not isinstance(source, dict) or source.get('kind') != 'nvidia_offline'
            or source.get('bag') != bag or source.get('metadata_sha256') != digest):
        raise ValueError('姿勢の生成元と選択した bag が一致しません')
    inputs = source.get('inputs')
    base_frame = inputs.get('base_frame') if isinstance(inputs, dict) else None
    info = dict(map_id=key, base_frame=base_frame, pose_frame=None, pose_sha256=None)
    try:
        path = confined(folder, POSE_FILE)
        if not path.is_file() or not path.stat().st_size:
            raise ValueError('非空の odom_poses.tum が見つかりません')
        if 'odom_poses' in source:
            evidence = source['odom_poses']
            if (not isinstance(evidence, dict) or evidence.get('path') != POSE_FILE.as_posix()
                    or not _valid_frame(evidence.get('frame'))):
                raise ValueError('odom_poses のパスまたはフレームが不正です')
            if evidence['frame'] != base_frame:
                raise ValueError('odom_poses のフレームと地図生成設定の base_frame が一致しません')
            with path.open('rb') as stream:
                actual = hashlib.file_digest(stream, 'sha256').hexdigest()
            if evidence.get('sha256') != actual:
                raise ValueError('odom_poses.tum の SHA-256 が一致しません')
            info.update(pose_frame=evidence['frame'], pose_sha256=actual)
    except (OSError, ValueError) as error:
        if 'odom_poses' in source:
            raise InvalidPoseEvidence(f'地図 {key} の生成記録を確認してください。{error}') from error
        raise
    return path, info


def _candidates(store, bag, digest):
    matches, warnings = [], []
    for folder in confined(store.result_root, 'maps').iterdir():
        try:
            _, item = _pose(store, folder.name, bag, digest)
            matches.append(item)
        except InvalidPoseEvidence as error:
            warnings.append(str(error))
        except (OSError, ValueError):
            continue
    if warnings and not matches:
        raise InvalidPoseEvidence(' '.join(sorted(warnings)))
    return sorted(matches, key=lambda item: item['map_id']), sorted(warnings)


def _report(folder):
    for name in ARTIFACTS:
        path = confined(folder, name)
        if not path.is_file() or not path.stat().st_size:
            raise ValueError('解析結果が未完成です')
    summary = read_json(folder / 'summary.json')
    if not isinstance(summary, dict) or summary.get('schema_version') != 1:
        raise ValueError('校正解析の summary.json を確認できません')
    source = summary.get('source')
    eligible, total = summary.get('eligible_samples'), summary.get('timeseries_samples')
    if (not isinstance(source, dict) or not isinstance(source.get('bag'), str)
            or not isinstance(source.get('metadata_sha256'), str) or not isinstance(summary.get('analysis'), dict)
            or type(total) is not int or type(eligible) is not int or not 0 <= eligible <= total or total < 1):
        raise ValueError('校正解析の出典または試料数が不正です')
    return summary


def _run_folder(store, key):
    return confined(store.result_root, RUNS / identifier(key))


def _run_entry(store, key):
    folder = _run_folder(store, key)
    summary = _report(folder)
    bag = Path(summary['source']['bag'])
    if bag.is_relative_to(store.record_root):
        bag = bag.relative_to(store.record_root)
    entry = dict(id=key, bag=bag.as_posix(), metadata_sha256=summary['source']['metadata_sha256'],
                 eligible_samples=summary['eligible_samples'], timeseries_samples=summary['timeseries_samples'],
                 completed=(folder / 'summary.json').stat().st_mtime)
    coverage = summary['analysis'].get('coverage')
    reasons = coverage.get('exclusion_reasons') if isinstance(coverage, dict) else None
    if isinstance(reasons, dict) and all(isinstance(reason, str) and type(count) is int and count >= 0
                                         for reason, count in reasons.items()):
        entry['exclusion_reasons'] = reasons
    return entry


def artifact(store, run, file):
    if file not in ARTIFACTS:
        raise ValueError('指定できる成果物は summary.json、response.png、timeseries.csv です')
    folder = _run_folder(store, run)
    _report(folder)
    return confined(folder, file)


def candidates(store, run):
    folder = _run_folder(store, run)
    summary = _report(folder)
    source = summary['source']
    bag = Path(source['bag'])
    if bag.is_relative_to(store.record_root):
        bag = bag.relative_to(store.record_root)
    digest = hashlib.sha256()
    # Hash the same CSV bytes that supply the candidate evidence.
    with confined(folder, 'timeseries.csv').open('rb') as stream:
        def lines():
            for line in stream:
                digest.update(line)
                yield line.decode('utf-8')

        try:
            proposal = derive_candidates(summary, csv.DictReader(lines(), strict=True))
        except csv.Error as error:
            raise ValueError('校正の時系列 CSV を読み込めません') from error
        for remainder in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(remainder)
    for key, candidate in proposal['limits'].items():
        if candidate['value'] is not None:
            try:
                validate_raceline_limits({key: candidate['value']})
            except ValueError as error:
                candidate.update(value=None, reason=f'観測候補 {candidate["value"]} は Raceline 設定の範囲外です。{error}')
    proposal.update(schema_version=1, run_id=run, bag=bag.as_posix(), source=dict(
        metadata_sha256=source['metadata_sha256'], poses_sha256=source.get('poses_sha256'),
        pose_frame=source.get('pose_frame'), timeseries_sha256=digest.hexdigest()))
    content = json.dumps(proposal, ensure_ascii=False, sort_keys=True, allow_nan=False, separators=(',', ':'))
    proposal['proposal_sha256'] = hashlib.sha256(content.encode('utf-8')).hexdigest()
    return proposal


def reviewed_candidates(store, run, expected):
    proposal = candidates(store, run)
    if proposal['proposal_sha256'] != expected:
        raise ValueError('校正結果または算出方法が変わりました。候補を再読込して確認してください')
    return proposal


def feedforward_document(store, run_id, proposal_sha256):
    proposal = reviewed_candidates(store, run_id, proposal_sha256)
    table = proposal['feedforward']
    if not table['usable']:
        raise ValueError(table['reason'])
    return dict(schema='atd.throttle_feedforward.v1', run_id=run_id, bag=proposal['bag'],
                proposal_sha256=proposal_sha256, source=proposal['source'], **table)


def selected_limits(proposal, fields):
    if not fields or len(set(fields)) != len(fields):
        raise ValueError('採用する制限を重複なく選択してください')
    limits = {}
    for key in fields:
        candidate = proposal['limits'].get(key)
        if candidate is None or candidate['value'] is None:
            raise ValueError('この制限には採用可能な候補がありません: ' + key)
        limits[key] = candidate['value']
    return dict(run_id=proposal['run_id'], bag=proposal['bag'], proposal_sha256=proposal['proposal_sha256'],
                limits=limits)


def options(store, bag=None):
    root = confined(store.result_root, RUNS, exists=False)
    runs = []
    if root.is_dir():
        for folder in root.iterdir():
            try:
                runs.append(_run_entry(store, folder.name))
            except (OSError, ValueError):
                continue
    poses, warnings, digest = [], [], None
    if bag is not None:
        bag, digest = _bag(store, bag)
        poses, warnings = _candidates(store, bag, digest)
    return dict(poses=poses, warnings=warnings, metadata_sha256=digest,
                runs=sorted(runs, key=lambda run: run['completed'], reverse=True))


def _reference(store, key, digest):
    if key is None:
        return None
    run = _run_entry(store, key)
    if run['metadata_sha256'] == digest or not run['eligible_samples']:
        raise ValueError('比較元には有効試料がある別の走行の run を指定してください')
    return _run_folder(store, key) / 'summary.json'


def payload(store, body):
    bag, digest = _bag(store, body['bag'])
    frame = body.get('pose_frame')
    if frame is not None and not _valid_frame(frame):
        raise ValueError('姿勢が表す機体フレームを指定してください')
    wheelbase = body.get('wheelbase_m')
    if wheelbase is not None and (type(wheelbase) not in (float, int) or not math.isfinite(wheelbase) or wheelbase <= 0):
        raise ValueError('実測ホイールベースは m 単位の正の有限値で指定してください')
    reference = body.get('reference_run')
    _reference(store, reference, digest)
    if body.get('map_id') is None:
        matches, _ = _candidates(store, bag, digest)
        if len(matches) > 1:
            raise ValueError('対応する VO 軌跡が複数あります。生成元の地図を選択してください')
        info = matches[0] if matches else None
    else:
        _, info = _pose(store, body['map_id'], bag, digest)
    generate = info is None
    if generate:
        if frame is not None and frame != offline.DEFAULT_INPUTS['base_frame']:
            raise ValueError('入力した姿勢フレームが新規生成の設定と一致しません')
        key = 'calibration-' + uuid4().hex
    else:
        key = info['map_id']
        frame = _resolve_frame(frame, info, require_evidence=False)
    return dict(bag=bag, metadata_sha256=digest, pose_frame=frame, map_id=key, generate=generate,
                pose_sha256=info['pose_sha256'] if info else None, wheelbase_m=wheelbase, reference_run=reference)


def _resolve_frame(requested, info, *, require_evidence):
    confirmed = info['pose_frame']
    if confirmed:
        if requested is not None and requested != confirmed:
            raise ValueError(f'入力した姿勢フレームと生成記録の {confirmed} が一致しません')
        return confirmed
    if require_evidence:
        raise ValueError('生成した軌跡にフレームの証跡がありません。生成結果を確認してください')
    if requested is None:
        raise ValueError('この既存軌跡にはフレームの記録がありません。確認した姿勢フレームを指定してください')
    return requested


def resources(request):
    keys = ['record:' + request['bag'], 'map:' + request['map_id']]
    if request['generate']:
        keys.extend(['ros-domain:93', 'offline-mapping'])
    return keys


def _stage(task, name, message):
    task.check()
    task.state['stage'] = name
    task.log(message)


def execute(store, task, request):
    _stage(task, 'preflight', '校正パッケージと入力 bag を確認しています')
    bag, digest = _bag(store, request['bag'])
    if digest != request['metadata_sha256']:
        raise ValueError('要求後に bag の metadata が変わりました。選び直してください')
    bag_path = confined(store.record_root, bag)
    reference = _reference(store, request['reference_run'], digest)
    key, generate = request['map_id'], request['generate']
    analyzer = ['ros2', 'run', 'atd_vehicle_calibration', 'analyze']
    check = ['ros2', 'run', 'atd_vehicle_calibration', 'check_bag', '--bag', str(bag_path)]
    if generate:
        check.append('--require-stereo')
    try:
        task.run([*analyzer, '--help'])
        checked = task.run(check, allowed_exit_codes=(0, 3))
    except (OSError, ValueError) as exc:
        raise ValueError('Notebook Docker 内で /workspaces/atd/scripts/build.sh --packages-up-to '
                         'atd_vehicle_calibration を実行し、校正パッケージを準備してください') from exc
    if checked == 3:
        raise ValueError('bag の事前検査に失敗しました。記録トピック・型・件数・ファイルの詳細をログで確認してください')
    _stage(task, 'pose', '公式 cuVSLAM で odom 姿勢を生成しています' if generate else '保存済みの odom 姿勢を照合しています')
    if generate:
        offline.map_build(store, task, dict(id=key, bag=bag))
    poses, info = _pose(store, key, bag, digest)
    expected_sha = request.get('pose_sha256')
    if expected_sha and expected_sha != info['pose_sha256']:
        raise ValueError('要求後に軌跡または生成記録が変わりました。選び直してください')
    frame = _resolve_frame(request.get('pose_frame'), info, require_evidence=generate)
    _stage(task, 'analysis', '推定速度・加速度・操舵応答を計算しています')
    run_id = identifier(task.key)
    output = confined(store.result_root, RUNS / run_id, exists=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise ValueError('同名の解析結果がすでにあります')
    with TemporaryDirectory(prefix='.calibration-', dir=output.parent) as staging:
        report = Path(staging) / 'report'
        cmd = [*analyzer, '--bag', str(bag_path), '--poses', str(poses), '--pose-frame', frame,
               '--output', str(report)]
        if request['wheelbase_m'] is not None:
            cmd.extend(['--wheelbase-m', str(request['wheelbase_m'])])
        if reference:
            cmd.extend(['--reference-summary', str(reference)])
        code = task.run(cmd, allowed_exit_codes=(0, 2))
        summary = _report(confined(Path(staging), 'report'))
        source = summary['source']
        eligible = summary['eligible_samples']
        if (source.get('bag') != str(bag_path) or source.get('metadata_sha256') != digest
                or source.get('poses') != str(poses) or source.get('pose_frame') != frame
                or (info['pose_sha256'] and source.get('poses_sha256') != info['pose_sha256'])
                or code != (0 if eligible else 2)):
            raise ValueError('解析結果の入力または終了コードが要求と一致しません')
        task.check()
        report.rename(output)
    return dict(run_id=run_id, map_id=key, pose_frame=frame, pose_reused=not generate, eligible_samples=eligible,
                timeseries_samples=summary['timeseries_samples'], outcome='ok' if eligible else 'no_eligible_samples')

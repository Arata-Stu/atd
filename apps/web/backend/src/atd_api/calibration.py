"""Notebook で bag と連続 VO 姿勢を照合し、車両校正レポートを作る。"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from uuid import uuid4

from . import offline
from .store import confined, identifier, read_json


RUNS = Path('runs/vehicle_calibration')
POSE_FILE = Path('nvidia/poses/odom_poses.tum')
ARTIFACTS = ('summary.json', 'response.png', 'timeseries.csv')


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
    path = confined(folder, POSE_FILE, exists=False)
    if not path.is_file() or not path.stat().st_size:
        raise ValueError('非空の odom_poses.tum が見つかりません')
    inputs = source.get('inputs')
    base_frame = inputs.get('base_frame') if isinstance(inputs, dict) else None
    return path, dict(map_id=key, base_frame=base_frame)


def _candidates(store, bag, digest):
    matches = []
    for folder in confined(store.result_root, 'maps').iterdir():
        try:
            _, item = _pose(store, folder.name, bag, digest)
            matches.append(item)
        except (OSError, ValueError):
            continue
    return sorted(matches, key=lambda item: item['map_id'])


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
    return dict(id=key, bag=bag.as_posix(), metadata_sha256=summary['source']['metadata_sha256'],
                eligible_samples=summary['eligible_samples'], timeseries_samples=summary['timeseries_samples'],
                completed=(folder / 'summary.json').stat().st_mtime)


def artifact(store, run, file):
    if file not in ARTIFACTS:
        raise ValueError('指定できる成果物は summary.json、response.png、timeseries.csv です')
    folder = _run_folder(store, run)
    _report(folder)
    return confined(folder, file)


def options(store, bag=None):
    root = confined(store.result_root, RUNS, exists=False)
    runs = []
    if root.is_dir():
        for folder in root.iterdir():
            try:
                runs.append(_run_entry(store, folder.name))
            except (OSError, ValueError):
                continue
    poses, digest = [], None
    if bag is not None:
        bag, digest = _bag(store, bag)
        poses = _candidates(store, bag, digest)
    return dict(poses=poses, metadata_sha256=digest, runs=sorted(runs, key=lambda run: run['completed'], reverse=True))


def _reference(store, key, digest):
    if key is None:
        return None
    run = _run_entry(store, key)
    if run['metadata_sha256'] == digest or not run['eligible_samples']:
        raise ValueError('比較元には有効試料がある別の走行の run を指定してください')
    return _run_folder(store, key) / 'summary.json'


def payload(store, body):
    bag, digest = _bag(store, body['bag'])
    frame = body['pose_frame']
    if not isinstance(frame, str) or not offline.FRAME.fullmatch(frame) or '//' in frame or frame.endswith('/'):
        raise ValueError('姿勢が表す機体フレームを指定してください')
    wheelbase = body.get('wheelbase_m')
    if wheelbase is not None and (type(wheelbase) not in (float, int) or not math.isfinite(wheelbase) or wheelbase <= 0):
        raise ValueError('実測ホイールベースは m 単位の正の有限値で指定してください')
    reference = body.get('reference_run')
    _reference(store, reference, digest)
    key = body.get('map_id')
    if key is None:
        matches = _candidates(store, bag, digest)
        if len(matches) > 1:
            raise ValueError('対応する VO 軌跡が複数あります。生成元の地図を選択してください')
        if matches:
            key = matches[0]['map_id']
    generate = key is None
    if generate:
        key = 'calibration-' + uuid4().hex
    else:
        _pose(store, key, bag, digest)
    return dict(bag=bag, metadata_sha256=digest, pose_frame=frame, map_id=key, generate=generate,
                wheelbase_m=wheelbase, reference_run=reference)


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
    poses, _ = _pose(store, key, bag, digest)
    _stage(task, 'analysis', '推定速度・加速度・操舵応答を計算しています')
    run_id = identifier(task.key)
    output = confined(store.result_root, RUNS / run_id, exists=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise ValueError('同名の解析結果がすでにあります')
    with TemporaryDirectory(prefix='.calibration-', dir=output.parent) as staging:
        report = Path(staging) / 'report'
        cmd = [*analyzer, '--bag', str(bag_path), '--poses', str(poses), '--pose-frame', request['pose_frame'],
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
                or source.get('poses') != str(poses) or source.get('pose_frame') != request['pose_frame']
                or code != (0 if eligible else 2)):
            raise ValueError('解析結果の入力または終了コードが要求と一致しません')
        task.check()
        report.rename(output)
    return dict(run_id=run_id, map_id=key, pose_reused=not generate, eligible_samples=eligible,
                timeseries_samples=summary['timeseries_samples'], outcome='ok' if eligible else 'no_eligible_samples')

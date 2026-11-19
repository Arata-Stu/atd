"""手動走行と出典が一致する cuVSLAM 成果物の車体前後速度を読む。"""
from contextlib import closing
import csv
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sqlite3
from tempfile import TemporaryDirectory

from . import calibration, localization_results as localization, offline
from .planning_replay import _core, _signature
from .store import confined, identifier, read_json

NS = 1_000_000_000
FILES = {
    'calibration': (calibration.RUNS, 'summary.json', 'timeseries.csv', '車両校正'),
    'localization': (localization.RUNS, 'report.json', 'trajectory.csv', '自己位置検証'),
    'odom': (Path('maps'), 'mapping.json', offline.ODOM_POSES, '連続 VO 軌跡'),
}


@lru_cache(maxsize=64)
def _digest(path, signature):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def _seconds(value):
    if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
        raise ValueError('速度推定の時刻・許容間隔が不正です')
    return round(value * NS)


@dataclass
class Source:
    kind: str
    path: Path
    report: dict
    signature: tuple


def _source(store, bag, digest, source_id):
    kind, _, key = source_id.partition(':')
    if kind not in FILES:
        raise ValueError('速度の出典を確認してください')
    root, report_name, csv_name, _ = FILES[kind]
    folder = confined(store.result_root, root / identifier(key))
    report_path = confined(folder, report_name)
    path = confined(folder, csv_name)
    if not path.is_file():
        raise ValueError('速度の元データを読み取れません')
    # report と軌跡のいずれかが更新されればインデックスを使い直さない。
    report_stat, data_stat = _signature(report_path), _signature(path)
    report = _report(kind, folder, report_stat)
    source = report if kind == 'localization' else report['source']
    recorded_bag = Path(source['bag'])
    if recorded_bag.is_relative_to(store.record_root):
        recorded_bag = recorded_bag.relative_to(store.record_root)
    if recorded_bag.as_posix() != bag or source['metadata_sha256'] != digest:
        raise ValueError('速度の出典が選択した走行・bag の metadata と一致しません')
    if kind == 'odom':
        proof = source.get('odom_poses') or {}
        if (source.get('kind') != 'nvidia_offline' or source.get('inputs', {}).get('base_frame') != 'base_link'
                or proof.get('frame') != 'base_link' or proof.get('path') != offline.ODOM_POSES.as_posix()
                or proof.get('sha256') != _digest(path, data_stat)):
            raise ValueError('車体フレームで生成した連続 VO 軌跡の証跡が一致しません')
    elif kind == 'calibration':
        motion = report.get('motion', {})
        mount = motion.get('body_to_pose', {})
        if (source.get('body_frame') != 'base_link' or source.get('association') != 'mapping_metadata_sha256'
                or Path(source.get('poses', '')).name != 'odom_poses.tum'
                or not re.fullmatch('[0-9a-f]{64}', source.get('poses_sha256') or '')
                or motion.get('base_frame') != 'base_link' or motion.get('world_frame') != 'odom'
                or not calibration._valid_frame(source.get('pose_frame'))
                or motion.get('input_pose_frame') != source['pose_frame']
                or mount.get('parent') != 'base_link' or mount.get('child') != source['pose_frame']):
            raise ValueError('校正結果の連続 VO 軌跡・車体フレームの証跡を確認できません')
        frames = _core('atd_vehicle_calibration', 'inputs')
        translation = frames.finite_vector(mount['translation'])
        rotation = frames.quaternion(mount['rotation'])
        if source['pose_frame'] == 'base_link' and (any(abs(v) > 1e-8 for v in translation)
                                                    or any(abs(v) > 1e-8 for v in rotation[:3])):
            raise ValueError('校正結果の base_link 自身の変換が不正です')
        _seconds(motion.get('max_gap_s'))
    elif (report['outcome'] not in ('tracked_to_end', 'partial_tracking') or not report.get('pose_samples')
          or report['source_bag'] == bag or report['source_metadata_sha256'] == digest):
        raise ValueError('この bag の有効な別走行検証結果を確認できません')
    return Source(kind, path, report, (digest, report_stat, data_stat))


@lru_cache(maxsize=32)
def _report(kind, folder, signature):
    if kind == 'localization':
        return localization._report(folder)
    if kind == 'calibration':
        return calibration._report(folder)
    return read_json(folder / 'mapping.json')


def _bag_digest(store, bag):
    path = confined(confined(store.record_root, bag), 'metadata.yaml')
    return _digest(path, _signature(path))


def sources(store, bag):
    """同一の相対パス・metadata の証跡がある、明示選択可能な出典。"""
    digest = _bag_digest(store, bag)
    items = []
    for kind, (root, _, _, label) in FILES.items():
        directory = confined(store.result_root, root, exists=False)
        if not directory.is_dir():
            continue
        for folder in sorted(directory.iterdir()):
            if not folder.is_dir() or folder.is_symlink() or folder.name.startswith('.'):
                continue
            source_id = f'{kind}:{folder.name}'
            try:
                _source(store, bag, digest, source_id)
            except (OSError, TypeError, KeyError, ValueError, AttributeError, OverflowError):
                continue
            items.append(dict(id=source_id, label=f'{label} · {folder.name}'))
    return items


def _samples(src):
    """有効速度または欠測、連続区間の識別子、次の観測までの上限 [ns]。"""
    if src.kind == 'localization':
        report = src.report
        first, last = (report['stereo'][field] for field in ('first_stamp_ns', 'last_stamp_ns'))
        limit = _seconds(report['gap_tolerance_s'])
        for segment, _, t, _, row in localization.trajectory_rows(src.path.parent, report, first, last, ('speed_mps',)):
            yield t, float(row['speed_mps']), segment, limit
    elif src.kind == 'calibration':
        limit = _seconds(src.report['motion']['max_gap_s'])
        count = 0
        with src.path.open(newline='') as stream:
            rows = csv.DictReader(stream, strict=True)
            required = {'stamp', 'motion_valid', 'motion_segment', 'vo_vx_mps'}
            if not required.issubset(rows.fieldnames or []):
                raise ValueError('校正時系列に符号付き速度・観測時刻・区間がありません')
            for row in rows:
                count += 1
                if row['motion_valid'] == 'False':
                    yield None, None, None, limit
                    continue
                if row['motion_valid'] != 'True':
                    raise ValueError('推定速度の有効フラグが不正です')
                yield _seconds(float(row['stamp'])), float(row['vo_vx_mps']), int(row['motion_segment']), limit
        if count != src.report['timeseries_samples']:
            raise ValueError('校正時系列の試料数が summary.json と一致しません')
    else:
        motion = _core('atd_vehicle_calibration', 'motion')
        poses = motion.load_tum(src.path, pose_frame='base_link')
        # mapping manifest が姿勢のフレーム自体を base_link と確認した軌跡のみ。
        estimate = motion.estimate_motion(poses, pose_frame='base_link', static_transforms=[])
        limit = _seconds(estimate['provenance']['max_gap_s'])
        previous = None
        for row in estimate['samples']:
            if previous is not None and row['source_index'] != previous + 1:
                yield None, None, None, limit
            previous = row['source_index']
            yield _seconds(row['stamp']), row['velocity_body'][0], row['segment'], limit


def _spans(samples):
    previous = None
    last = -1
    segment = -1
    for stamp, speed, group, limit in samples:
        if stamp is not None and (stamp < last or (stamp == last and (not previous or previous[2] == group))
                                  or not math.isfinite(speed)):
            raise ValueError('速度試料の時刻または符号付き速度が不正です')
        contiguous = bool(previous and stamp is not None and previous[2] == group and stamp - last <= limit)
        if previous and stamp != last:
            # 異なる区間の手前や最終試料は、その観測時刻だけに限定する。
            yield last, stamp if contiguous else last + 1, previous[1], segment
        if stamp is not None:
            if not contiguous:
                segment += 1
            previous = (stamp, speed, group)
            last = stamp
        else:
            previous = None
    if previous:
        yield last, last + 1, previous[1], segment


@lru_cache(maxsize=1)
def _version():
    package = Path(_core('atd_vehicle_calibration', 'motion').__file__)
    return [_digest(p, _signature(p)) for p in (Path(__file__), package, package.with_name('inputs.py'),
                                                 Path(localization.__file__))]


def _index(store, bag, source_id):
    digest = _bag_digest(store, bag)
    src = _source(store, bag, digest, source_id)
    key = hashlib.sha256(json.dumps([bag, source_id, src.signature, _version()]).encode()).hexdigest()
    index = confined(store.state_root, Path('manual-speed') / (key + '.sqlite'), exists=False)
    if not index.is_file():
        with store.lock(index):
            if not index.is_file():
                index.parent.mkdir(parents=True, exist_ok=True)
                with TemporaryDirectory(dir=index.parent) as working:
                    new = Path(working) / 'speed.sqlite'
                    with closing(sqlite3.connect(new)) as db:
                        db.execute('CREATE TABLE speed(t INTEGER PRIMARY KEY, end INTEGER, vx REAL, segment INTEGER)')
                        db.executemany('INSERT INTO speed VALUES (?,?,?,?)', _spans(_samples(src)))
                        if not db.execute('SELECT 1 FROM speed LIMIT 1').fetchone():
                            raise ValueError('この推定結果に有効な車体前後速度がありません')
                        db.commit()
                    verified = _source(store, bag, _bag_digest(store, bag), source_id)
                    if src.signature != verified.signature:
                        raise ValueError('読込中に速度の出典が変更されました。再度選択してください')
                    os.replace(new, index)
    return index


def _read(store, bag, source_id):
    try:
        index = _index(store, bag, source_id)
        return closing(sqlite3.connect(index.as_uri() + '?mode=ro&immutable=1', uri=True))
    except (KeyError, TypeError, AttributeError, csv.Error, OSError, sqlite3.Error, OverflowError) as error:
        raise ValueError(f'速度の出典 {source_id} を確認できません。{error}') from error


def prepare(store, bag, source_id, bag_start_ns, duration_ns):
    """この走行時間内で少なくとも1件の有効な前後速度があることを確認する。"""
    with _read(store, bag, source_id) as db:
        row = db.execute('SELECT 1 FROM speed WHERE t<=? AND end>? LIMIT 1',
                         (bag_start_ns + duration_ns, bag_start_ns)).fetchone()
        if not row:
            raise ValueError('選択した bag の収録時間内に有効な cuVSLAM 前後速度がありません')


def spans(store, bag, source_id, bag_start_ns, duration_ns):
    """収録区間に重なる前後速度を相対 ns と連続区間 ID で順に返す。"""
    end = bag_start_ns + duration_ns + 1
    with _read(store, bag, source_id) as db:
        for t, until, vx, segment in db.execute(
                'SELECT t,end,vx,segment FROM speed WHERE t<? AND end>? ORDER BY t', (end, bag_start_ns)):
            yield max(0, t - bag_start_ns), min(end, until) - bag_start_ns, vx, segment


def window(store, bag, source_id, bag_start_ns, start_ns, stop_ns):
    """start_ns, stop_ns は bag 開始からの半開区間。出典の版ごとに解析結果を保存する。"""
    a, b = bag_start_ns + start_ns, bag_start_ns + stop_ns
    with _read(store, bag, source_id) as db:
        rows = db.execute('SELECT t,end,vx,segment FROM speed WHERE t >= '
                          'coalesce((SELECT max(t) FROM speed WHERE t<=?),0) AND t<? AND end>? ORDER BY t', (a,b,a))
        return [dict(start_s=(max(a,t)-bag_start_ns)/NS, end_s=(min(b,end)-bag_start_ns)/NS,
                     speed_mps=vx, segment=segment) for t, end, vx, segment in rows]

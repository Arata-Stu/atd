"""同じ bag を使った cuVSLAM と ORB-SLAM3 の観測姿勢を比較する。"""
import argparse
from bisect import bisect_left
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
from itertools import pairwise
import json
import math
import os
from pathlib import Path
from statistics import fmean, median
import sys
from uuid import uuid4

from .orb_gap_eval import run_dir
from .replay_gt_geometry import Alignment, Trace
from .replay_gt_source import find_ground_truth
from .replay_result import GAP_NS, compare

BACKENDS = ('cuvslam', 'orbslam3')
# cuVSLAM は左右の遅い方、ORB は左画像の撮影時刻を使う。
PAIR_TOLERANCE_NS = 5_000_000


def _sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


@dataclass
class Sample:
    stamp: int
    epoch: int
    error: tuple | None = None  # 平面位置 [m]・絶対方位差 [deg]
    pose: tuple | None = None   # map 座標の xyz, xyzw


def _load_run(root, run_id):
    matches = []
    for kind, filename in (('replay', 'summary.json'), ('localization', 'report.json')):
        folder = run_dir(root, kind, run_id)
        if (folder / filename).is_file():
            matches.append((kind, folder, json.loads((folder / filename).read_bytes())))
    if len(matches) != 1:
        raise ValueError(f'runs/replay または runs/localization に一意の run ID が必要です: {run_id}')
    return matches[0]


def _backend(kind, folder, meta):
    if meta.get('backend') in BACKENDS:
        return meta['backend']
    if 'backend' not in meta:  # 以前の比較 run には backend がない。
        if kind == 'replay' and 'slam_max_map_size' in meta:
            return 'cuvslam'
        if kind == 'localization':
            path = folder / 'hdmap.json'
            if _sha(path) == meta['hdmap_sha256']:
                source = json.loads(path.read_bytes())['source'].get('kind')
                if source in ('nvidia_offline', 'orbslam3'):
                    return 'cuvslam' if source == 'nvidia_offline' else 'orbslam3'
    raise ValueError(f'run のバックエンドを確認できません: {folder.name}')


def _replay(folder, meta):
    if meta['status'] != 'complete' or folder.name != meta['id']:
        raise ValueError(f'完走した GT 比較 run を指定してください: {folder.name}')
    series = []
    with (folder / 'samples.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            err = math.hypot(row['along_m'], row['lateral_m']), abs(row['heading_deg'])
            if not all(map(math.isfinite, err)):
                raise ValueError('GT 比較の誤差が非有限です')
            series.append(Sample(int(row['stamp_ns']), 0, err))
    if len(series) != meta['sample_count']:
        raise ValueError(f'比較 run の保存件数が集計と一致しません: {folder.name}')
    bounds = int(meta['gt_start_stamp_ns']), int(meta['gt_finish_stamp_ns'])
    return series, bounds, dict(alignment_source=meta.get('alignment_source', 'first_estimate'),
                                map_id=meta.get('map_id'), slam_max_map_size=meta.get('slam_max_map_size'))


def _fixed(folder, meta):
    if meta['outcome'] not in ('tracked_to_end', 'partial_tracking', 'no_tracking'):
        raise ValueError(f'再生が完了していません: {folder.name} ({meta["outcome"]})')
    stereo = meta['stereo']
    series = []
    with (folder / 'trajectory.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            pose = tuple(float(row[key]) for key in ('x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw'))
            if not all(map(math.isfinite, pose)):
                raise ValueError('追跡姿勢に非有限値があります')
            sample = Sample(int(row['stamp_ns']), int(row['epoch']), pose=pose)
            if series and series[-1].stamp == sample.stamp:
                # 再捕捉した epoch が直前と同時刻の場合は新しい方を採用する。
                series[-1] = sample
            else:
                series.append(sample)
    # TrackingReport records a transition on state or epoch change; a missed loss message
    # can therefore give adjacent tracking entries from two distinct epochs.
    acquisitions = sum(row['state'] == 'tracking' for row in meta['state_changes'])
    details = dict(map_id=meta['map_id'], map_sha256=meta['map_sha256'], outcome=meta['outcome'],
                   stereo_pairs=stereo['stereo_pairs'], reacquisitions=max(0, acquisitions - 1))
    return series, (stereo['first_stamp_ns'], stereo['last_stamp_ns']), details


def _coverage(series, start, end):
    deltas = [b.stamp - a.stamp for a, b in pairwise(series)]
    covered = sum(dt for (a, b), dt in zip(pairwise(series), deltas) if a.epoch == b.epoch and dt <= GAP_NS)
    return dict(observations=len(series), observed_time_fraction=covered / (end - start),
                initial_gap_s=(series[0].stamp-start) / 1e9 if series else None,
                final_gap_s=(end-series[-1].stamp) / 1e9 if series else None,
                internal_gaps_over_threshold=sum(dt > GAP_NS for dt in deltas),
                longest_internal_gap_s=max(deltas) / 1e9 if deltas else None)


def _nearest(stamps, target):
    i = bisect_left(stamps, target)
    return min(range(max(0, i-1), min(i+1, len(stamps))), key=lambda k: abs(stamps[k] - target))


def _paired(a, b):
    if not a or not b:
        return []
    left, right = [row.stamp for row in a], [row.stamp for row in b]
    matches = []
    for i, stamp in enumerate(left):
        j = _nearest(right, stamp)
        if abs(right[j]-stamp) <= PAIR_TOLERANCE_NS and _nearest(left, right[j]) == i:
            matches.append((a[i], b[j]))
    return matches


def _accuracy(errors):
    if not errors:
        return None
    pos, yaw = (sorted(values) for values in zip(*errors))
    near_rank = math.ceil(len(pos)*.95) - 1
    return dict(samples=len(pos), xy_median_m=median(pos), xy_p95_m=pos[near_rank],
                xy_rmse_m=math.sqrt(fmean(x*x for x in pos)), abs_yaw_median_deg=median(yaw),
                abs_yaw_p95_deg=yaw[near_rank])


def _relative_errors(pairs, trace):
    anchors = []
    for pair in pairs:
        truth = [trace.at(row.stamp) for row in pair]
        if any(p is None for p in truth):
            continue
        if not anchors:
            anchors = [Alignment.from_anchor(p, row.pose[:3], row.pose[3:]) for row, p in zip(pair, truth)]
            # 初回の誤差は位置合わせでゼロになる。
            continue
        for row, p, alignment in zip(pair, truth, anchors):
            result = compare(alignment, p, row.pose[:3], row.pose[3:])
            row.error = (math.hypot(result['along_m'], result['lateral_m']), abs(result['heading_deg']))


def compare_runs(root, cu_id, orb_id, record_root=None):
    runs = [_load_run(root, run_id) for run_id in (cu_id, orb_id)]
    if runs[0][0] != runs[1][0]:
        raise ValueError('2本とも同じ検証手順（atd replay または validate_bag）で作成してください')
    kind = runs[0][0]
    for backend, (_, folder, meta) in zip(BACKENDS, runs):
        if _backend(kind, folder, meta) != backend:
            raise ValueError('cuVSLAM、ORB-SLAM3 の順で各1本の run ID を指定してください')
    cu, orb = (meta for _, _, meta in runs)
    same_hash = cu.get('metadata_sha256') and cu['metadata_sha256'] == orb.get('metadata_sha256')
    if kind == 'replay':
        if cu.get('mode', 'mapping') != 'mapping' or orb['mode'] != 'mapping':
            raise ValueError('地図作成の比較には両方で --map を省き、固定地図の比較には validate_bag を使ってください')
        if record_root is not None:
            raise ValueError('--record-root は validate_bag の固定地図比較で使用します')
        if (not same_hash and (cu['bag'] != orb['bag'] or cu.get('metadata_sha256') and orb.get('metadata_sha256'))
                or _sha(runs[0][1] / 'reference.json') != _sha(runs[1][1] / 'reference.json')):
            raise ValueError('評価 bag または Gazebo 正解軌跡が一致しません')
        if (cu['rate'] != orb['rate'] or cu['tracking'] != orb['tracking']
                or cu.get('start_offset_s', 0) != orb.get('start_offset_s', 0)
                or cu.get('alignment_source', 'first_estimate') != orb['alignment_source']):
            raise ValueError('再生倍率・入力モード・開始位置・位置合わせ方法を揃えてください')
        mode, rate, tracking = 'mapping', cu['rate'], cu['tracking']
        accuracy_frame = 'replay_aligned_to_first_estimate'
        read = _replay
    else:
        if (not same_hash or cu['source_metadata_sha256'] != orb['source_metadata_sha256']
                or cu['replay_rate'] != orb['replay_rate'] or set(cu['inputs']) != set(orb['inputs'])):
            raise ValueError('評価 bag、地図作成 bag、入力トピック、再生倍率を揃えてください')
        mode, rate, tracking = 'fixed_map', cu['replay_rate'], 'vo'
        accuracy_frame = None
        read = _fixed

    loaded = [read(folder, meta) for _, folder, meta in runs]
    start, end = max(r[1][0] for r in loaded), min(r[1][1] for r in loaded)
    if end <= start:
        raise ValueError('両 run が共通の評価区間を持ちません')
    report = dict(schema='atd.vslam_benchmark.v1', mode=mode, bag=cu['bag'], rate=rate, tracking=tracking,
                  metadata_sha256=cu.get('metadata_sha256') or orb.get('metadata_sha256'),
                  source_metadata_sha256=cu.get('source_metadata_sha256'), pose_source='backend_observations',
                  window_ns=[start, end], gap_threshold_s=GAP_NS / 1e9,
                  pairing_tolerance_s=PAIR_TOLERANCE_NS / 1e9, backends={})
    series = []
    for backend, (_, folder, _), (samples, _, details) in zip(BACKENDS, runs, loaded):
        if any(b.stamp <= a.stamp for a, b in pairwise(samples)):
            raise ValueError(f'観測時刻が単調増加していません: {folder.name}')
        samples = [row for row in samples if start <= row.stamp <= end]
        series.append(samples)
        report['backends'][backend] = dict(run_id=folder.name, **details, **_coverage(samples, start, end))

    pairs = _paired(*series)
    if kind == 'localization' and record_root is not None:
        bag = (record_root / cu['bag']).resolve()
        if not bag.is_relative_to(record_root.resolve()) or _sha(bag / 'metadata.yaml') != cu['metadata_sha256']:
            raise ValueError('指定した評価 bag の metadata.yaml が run と一致しません')
        gt = find_ground_truth(bag, root)
        if not gt:
            raise ValueError('指定した bag に対応する Gazebo 正解軌跡がありません')
        _relative_errors(pairs, Trace.from_file(gt))
        accuracy_frame = 'aligned_at_first_common_observation'
    report.update(accuracy_frame=accuracy_frame, paired_observations=len(pairs))
    for i, backend in enumerate(BACKENDS):
        report['backends'][backend]['paired_accuracy'] = _accuracy(
            [pair[i].error for pair in pairs if pair[i].error is not None])
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cuvslam_run')
    parser.add_argument('orbslam3_run')
    parser.add_argument('--results-root', type=Path, default=Path(os.environ.get('ATD_REPLAY_RESULTS_ROOT', '/data/results')))
    parser.add_argument('--record-root', type=Path, help='固定地図の Gazebo 相対誤差も評価する bag ルート')
    args = parser.parse_args(argv)
    try:
        report = compare_runs(args.results_root, args.cuvslam_run, args.orbslam3_run, args.record_root)
        name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid4().hex[:12]
        path = args.results_root / 'runs' / 'vslam_benchmark' / name / 'report.json'
        payload = json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + '\n'
        path.parent.mkdir(parents=True)
        path.write_text(payload, encoding='utf-8')
    except (KeyError, OSError, ValueError) as exc:
        print(f'比較できません: {exc}', file=sys.stderr)
        return 1
    print(f"共通評価区間 {(report['window_ns'][1]-report['window_ns'][0])/1e9:.3f} 秒、"
          f"±5 ms で対応した観測 {report['paired_observations']} 組")
    for backend, result in report['backends'].items():
        print(f"{backend}  観測カバー率 {result['observed_time_fraction']:.1%}  "
              f"0.25 s 超の姿勢間隔 {result['internal_gaps_over_threshold']} 回", end='')
        if result['paired_accuracy']:
            err = result['paired_accuracy']
            print(f"  共通観測の位置 RMSE {err['xy_rmse_m']:.3f} m・方位誤差中央値 {err['abs_yaw_median_deg']:.2f}°", end='')
        print()
    print(f'比較結果: {path}')
    return 0

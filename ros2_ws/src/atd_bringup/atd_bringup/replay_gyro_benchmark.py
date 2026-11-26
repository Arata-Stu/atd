"""同じ固定地図試験の ORB 観測とジャイロ補完を、別に再生した cuVSLAM と比較する。"""
import csv
from itertools import pairwise
import json
import math
from pathlib import Path

from atd_localization.orb_assets import orb_map_metadata

from .orb_gap_eval import run_dir
from .replay_benchmark import PAIR_TOLERANCE_NS, Sample, _accuracy, _backend, _coverage, _fixed, _paired, _sha
from .replay_gt_geometry import Alignment, Trace
from .replay_gt_source import find_ground_truth
from .replay_inputs import STEREO_TOPICS
from .replay_result import GAP_NS, compare


def _orb_series(folder, meta):
    epochs = {}
    tracking_entries = 0
    previous_state = None
    with (folder / 'status.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            state = row['state'], int(row['epoch'])
            if row['state'] == 'tracking':
                tracking_entries += state != previous_state
                stamp = int(row['stamp_ns'])
                if epochs.setdefault(stamp, state[1]) != state[1]:
                    epochs[stamp] = None
            previous_state = state

    def pose(row):
        result = tuple(float(row[k]) for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))
        if not all(map(math.isfinite, result)):
            raise ValueError('ORB の姿勢に非有限値があります')
        return result

    raw, continuous, predicted = [], [], set()
    with (folder / 'trajectory.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            stamp = int(row['image_stamp_ns'])
            epoch = epochs.get(stamp)
            # DDS で status が欠けた姿勢は残し、隣の姿勢への追跡継続だけを未確認とする。
            raw.append(Sample(stamp, epoch, pose=pose(row)))
    if len(raw) != meta['trajectory_samples']:
        raise ValueError('ORB 観測の保存件数が summary.json と一致しません')

    if (meta.get('gyro_bridge') or {}).get('report') != 'gyro_bridge_summary.json':
        raise ValueError('--gyro-bridge on で完走した ORB 固定地図 run を指定してください')
    bridge = json.loads((folder / 'gyro_bridge_summary.json').read_bytes())
    with (folder / 'gyro_trajectory.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            stamp = int(row['stamp_ns'])
            if row['source'] == 'predicted':
                predicted.add(stamp)
            elif row['source'] != 'observed':
                raise ValueError(f"補完軌跡に未知の出力種別があります: {row['source']}")
            continuous.append(Sample(stamp, 0, pose=pose(row)))
    if len(predicted) != bridge['predicted'] or len(continuous) - len(predicted) != bridge['observed']:
        raise ValueError('ジャイロ補完の保存件数が gyro_bridge_summary.json と一致しません')
    return raw, continuous, predicted, bridge, max(0, tracking_entries-1)


def _window(rows, start, end):
    if any(b.stamp <= a.stamp for a, b in pairwise(rows)):
        raise ValueError('姿勢の計測時刻が単調増加していません')
    return [row for row in rows if start <= row.stamp <= end]


def _output_stats(rows, start, end, *, visual):
    info = _coverage(rows, start, end)
    duration = sum(b.stamp - a.stamp for a, b in pairwise(rows) if b.stamp - a.stamp <= GAP_NS)
    info['output_time_fraction'] = duration / (end - start)
    if not visual:
        del info['observed_time_fraction']
    return info


def _score(rows, trace, alignment, anchor_stamp):
    for row in rows:
        if row.stamp <= anchor_stamp:
            continue
        truth = trace.at(row.stamp)
        if truth is not None:
            err = compare(alignment, truth, row.pose[:3], row.pose[3:])
            row.error = math.hypot(err['along_m'], err['lateral_m']), abs(err['heading_deg'])


def compare_gyro_run(root, cu_id, orb_id, record_root=None):
    """同一走行・同一倍率の検証結果。3出力に共通する画像と、補完中の出力時刻を集計する。"""
    root = Path(root)
    cu_dir = run_dir(root, 'localization', cu_id)
    orb_dir = run_dir(root, 'orbslam3', orb_id)
    cu = json.loads((cu_dir / 'report.json').read_bytes())
    orb = json.loads((orb_dir / 'summary.json').read_bytes())
    if _backend('localization', cu_dir, cu) != 'cuvslam':
        raise ValueError('最初の run には cuVSLAM 地図の validate_bag 結果を指定してください')
    if (orb.get('backend'), orb.get('mode'), orb.get('status'), orb.get('tracking')) != (
            'orbslam3', 'localization', 'complete', 'vo'):
        raise ValueError('ステレオ固定地図を使って完走した ORB run を指定してください')
    source_hash = orb.get('source_metadata_sha256')
    if not source_hash:
        asset = orb_map_metadata(root, orb['map_id'])
        if asset['map_sha256'] != orb['map_sha256']:
            raise ValueError('旧 ORB run が参照した地図と現在の地図の SHA-256 が異なります')
        source_hash = asset['source_metadata_sha256']
    if not cu.get('metadata_sha256') or cu['metadata_sha256'] != orb['metadata_sha256']:
        raise ValueError('評価 bag の metadata.yaml が一致しません')
    if not source_hash or source_hash != cu['source_metadata_sha256']:
        raise ValueError('地図作成 bag の metadata.yaml が一致しません')
    if (orb['rate'] != cu['replay_rate'] or orb['start_offset_s'] != 0
            or set(cu['inputs']) != set(STEREO_TOPICS)):
        raise ValueError('再生倍率、開始位置、ステレオ入力を揃えてください')

    cu_rows, cu_bounds, cu_info = _fixed(cu_dir, cu)
    raw, continuous, predicted, bridge, raw_reacquisitions = _orb_series(orb_dir, orb)
    start = max(cu_bounds[0], orb['first_input_stamp_ns'])
    end = min(cu_bounds[1], orb['last_input_stamp_ns'])
    if start >= end:
        raise ValueError('3方式が共通の評価区間を持ちません')
    cu_rows, raw, continuous = [_window(rows, start, end) for rows in (cu_rows, raw, continuous)]
    cu_info.update(run_id=cu_id, **_output_stats(cu_rows, start, end, visual=True))
    raw_info = dict(run_id=orb_id, map_id=orb['map_id'], map_sha256=orb['map_sha256'],
                    stereo_pairs=orb['input_pairs'], reacquisitions=raw_reacquisitions,
                    missing_epoch_samples=sum(row.epoch is None for row in raw),
                    **_output_stats(raw, start, end, visual=True))
    predicted_rows = [row for row in continuous if row.stamp in predicted]
    gyro_info = dict(run_id=orb_id, map_id=orb['map_id'], map_sha256=orb['map_sha256'],
                     observed_samples=len(continuous)-len(predicted_rows), predicted_samples=len(predicted_rows),
                     bridge_summary=bridge, **_output_stats(continuous, start, end, visual=False))

    # ORB の2出力は同じ実行なので、同一画像時刻で結合する。予測時刻に近い別の画像では代用しない。
    pairs = _paired(cu_rows, raw)
    by_stamp = {row.stamp: row for row in continuous if row.stamp not in predicted}
    triples = [(a, b, by_stamp[b.stamp]) for a, b in pairs if b.stamp in by_stamp]
    anchor = None
    if record_root is not None:
        record_root = Path(record_root).resolve()
        bag = (record_root / cu['bag']).resolve()
        if not bag.is_relative_to(record_root) or _sha(bag / 'metadata.yaml') != cu['metadata_sha256']:
            raise ValueError('指定した評価 bag の metadata.yaml が run と一致しません')
        path = find_ground_truth(bag, root)
        if path is None:
            raise ValueError('指定した bag の Gazebo 正解軌跡がありません')
        trace = Trace.from_file(path)
        for a, b in pairs:
            origins = trace.at(a.stamp), trace.at(b.stamp)
            if None in origins:
                continue
            cu_map = Alignment.from_anchor(origins[0], a.pose[:3], a.pose[3:])
            orb_map = Alignment.from_anchor(origins[1], b.pose[:3], b.pose[3:])
            _score(cu_rows, trace, cu_map, a.stamp)
            _score(raw, trace, orb_map, b.stamp)
            _score(continuous, trace, orb_map, b.stamp)
            anchor = dict(cuvslam=a.stamp, orbslam3=b.stamp)
            break

    scored_triples = [triple for triple in triples if all(row.error is not None for row in triple)]
    for i, (rows, info) in enumerate(((cu_rows, cu_info), (raw, raw_info), (continuous, gyro_info))):
        info['paired_accuracy'] = _accuracy([triple[i].error for triple in scored_triples])
        info['output_accuracy'] = _accuracy([row.error for row in rows if row.error is not None])
    gyro_info['predicted_accuracy'] = _accuracy([row.error for row in predicted_rows if row.error is not None])
    return dict(schema='atd.vslam_gyro_benchmark.v1', mode='fixed_map', bag=cu['bag'],
                rate=cu['replay_rate'], tracking='vo', metadata_sha256=cu['metadata_sha256'],
                source_metadata_sha256=source_hash, window_ns=[start, end], gap_threshold_s=GAP_NS / 1e9,
                pairing_tolerance_s=PAIR_TOLERANCE_NS / 1e9, paired_observations=len(triples),
                accuracy_frame='aligned_at_first_common_observation' if anchor else None, anchor_stamp_ns=anchor,
                backends=dict(cuvslam=cu_info, orbslam3=raw_info, orbslam3_gyro=gyro_info))

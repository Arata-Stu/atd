"""同じ固定地図試験の ORB 観測とジャイロ補完を、別に再生した cuVSLAM と比較する。"""
import csv
import json
from pathlib import Path

from .orb_gap_eval import run_dir
from .replay_benchmark import (
    PAIR_TOLERANCE_NS, Sample, _accuracy, _backend, _fixed, _paired, _sensor_input_transports,
)
from .replay_fixed_benchmark import (
    _check_orb_conditions, _dds_profiles, _ground_truth, _orb_observations, _orb_pose, _output_stats, _score, _window,
)
from .replay_gt_geometry import Alignment
from .replay_result import GAP_NS


def _orb_series(folder, meta):
    raw, reacquisitions = _orb_observations(folder, meta)
    continuous, predicted = [], set()
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
            continuous.append(Sample(stamp, 0, pose=_orb_pose(row)))
    if len(predicted) != bridge['predicted'] or len(continuous) - len(predicted) != bridge['observed']:
        raise ValueError('ジャイロ補完の保存件数が gyro_bridge_summary.json と一致しません')
    return raw, continuous, predicted, bridge, reacquisitions


def compare_gyro_run(root, cu_id, orb_id, record_root=None):
    """同一走行・同一倍率の検証結果。3出力に共通する画像と、補完中の出力時刻を集計する。"""
    root = Path(root)
    cu_dir = run_dir(root, 'localization', cu_id)
    orb_dir = run_dir(root, 'orbslam3', orb_id)
    cu = json.loads((cu_dir / 'report.json').read_bytes())
    orb = json.loads((orb_dir / 'summary.json').read_bytes())
    if _backend('localization', cu_dir, cu) != 'cuvslam':
        raise ValueError('最初の run には cuVSLAM 地図の validate_bag 結果を指定してください')
    source_hash = _check_orb_conditions(root, cu, orb, 'vo')

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
        trace = _ground_truth(root, record_root, cu)
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
                dds_profiles=_dds_profiles(cu, orb), sensor_input_transports=_sensor_input_transports(cu, orb),
                backends=dict(cuvslam=cu_info, orbslam3=raw_info, orbslam3_gyro=gyro_info))

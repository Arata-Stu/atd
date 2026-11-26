"""同じ走行で cuVSLAM・固定地図のステレオ ORB・固定地図に合わせた ORB VIO を比較する。"""
import csv
from itertools import pairwise
import math
from pathlib import Path

from atd_localization.orb_assets import orb_map_metadata

from .replay_benchmark import (
    PAIR_TOLERANCE_NS, Sample, _accuracy, _backend, _coverage, _fixed, _paired, _sensor_input_transports, _sha,
)
from .replay_gt_geometry import Alignment, Trace
from .replay_gt_source import find_ground_truth
from .replay_inputs import STEREO_TOPICS
from .replay_result import GAP_NS, compare


def _orb_pose(row):
    result = tuple(float(row[k]) for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))
    if not all(map(math.isfinite, result)):
        raise ValueError('ORB の姿勢に非有限値があります')
    return result


def _orb_observations(folder, meta):
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

    raw = []
    with (folder / 'trajectory.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            stamp = int(row['image_stamp_ns'])
            epoch = epochs.get(stamp)
            # DDS で status が欠けた姿勢は残し、隣の姿勢への追跡継続だけを未確認とする。
            raw.append(Sample(stamp, epoch, pose=_orb_pose(row)))
    if len(raw) != meta['trajectory_samples']:
        raise ValueError('ORB 観測の保存件数が summary.json と一致しません')

    return raw, max(0, tracking_entries - 1)


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


def _dds_profiles(cu, orb, vio=None):
    runs = {'cuvslam': cu, 'orbslam3': orb}
    if vio is not None:
        runs['orbslam3_vio'] = vio
    profiles = {}
    keys = ('rmw_implementation', 'sha256', 'discovery_range')
    for name, run in runs.items():
        profile = run.get('dds_profile')
        if not isinstance(profile, dict):
            profile = {'path': profile}  # 旧 run はパスだけ、または未記録。
        profiles[name] = {key: profile.get(key) for key in (*keys, 'path')}
    differences = [key for key in keys if len({p[key] for p in profiles.values() if p[key]}) > 1]
    warning = None
    if differences:
        comparison = 'different'
        warning = '比較対象の再生時の DDS 設定が異なります（' + '、'.join(differences) + '）。'
    elif all(p[key] for p in profiles.values() for key in keys):
        comparison = 'match'
    else:
        comparison = 'unknown'
    return dict(**profiles, comparison=comparison, warning=warning)


def _check_orb_conditions(root, cu, orb, tracking):
    if (orb.get('backend'), orb.get('mode'), orb.get('status'), orb.get('tracking')) != (
            'orbslam3', 'localization', 'complete', tracking):
        raise ValueError(f'固定地図で完走した ORB の {tracking.upper()} run を指定してください')
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

    return source_hash


def _ground_truth(root, record_root, cu):
    record_root = Path(record_root).resolve()
    bag = (record_root / cu['bag']).resolve()
    if not bag.is_relative_to(record_root) or _sha(bag / 'metadata.yaml') != cu['metadata_sha256']:
        raise ValueError('指定した評価 bag の metadata.yaml が run と一致しません')
    path = find_ground_truth(bag, root)
    if path is None:
        raise ValueError('指定した bag の Gazebo 正解軌跡がありません')
    return Trace.from_file(path)


def compare_fixed_runs(root, runs, record_root=None):
    """各方式を個別再生し、同じ画像時刻で保存地図上の姿勢を評価する。"""
    if [kind for kind, _, _ in runs] != ['localization', 'orbslam3', 'orbslam3']:
        raise ValueError('cuVSLAM の validate_bag、固定地図 ORB の VO、固定地図 ORB の VIO の順に指定してください')
    (_, cu_dir, cu), (_, stereo_dir, stereo), (_, vio_dir, vio) = runs
    if _backend('localization', cu_dir, cu) != 'cuvslam' or cu.get('tracking', 'vo') != 'vo':
        raise ValueError('最初の run には cuVSLAM ステレオ VO の保存地図検証を指定してください')
    for meta, tracking in ((stereo, 'vo'), (vio, 'vio')):
        _check_orb_conditions(root, cu, meta, tracking)
        if (meta.get('gyro_bridge') or {}).get('report'):
            raise ValueError('この比較には --gyro-bridge off で再生した ORB run を指定してください')
    if vio.get('localization_method') != 'fixed_stereo_atlas_vio':
        raise ValueError('3本目は固定ステレオ地図に位置合わせした VIO run が必要です')
    if (not stereo.get('map_id') or not stereo.get('map_sha256') or
            (stereo['map_id'], stereo['map_sha256']) != (vio['map_id'], vio['map_sha256'])):
        raise ValueError('ORB の VO と VIO は同一の固定地図（ID・SHA-256）を指定してください')

    cu_rows, cu_bounds, cu_info = _fixed(cu_dir, cu)
    if (cu_info['outcome'] == 'no_tracking') != (not cu_rows):
        raise ValueError('cuVSLAM の追跡結果と保存姿勢数が一致しません')
    series = [cu_rows]
    details = [dict(cu_info, run_id=cu_dir.name, tracking='vo')]
    start = max(cu_bounds[0], stereo['first_input_stamp_ns'], vio['first_input_stamp_ns'])
    end = min(cu_bounds[1], stereo['last_input_stamp_ns'], vio['last_input_stamp_ns'])
    if start >= end:
        raise ValueError('3方式が共通の評価区間を持ちません')
    for folder, meta in ((stereo_dir, stereo), (vio_dir, vio)):
        rows, reacquisitions = _orb_observations(folder, meta)
        # Old completed runs always required at least one valid map pose.
        # Only an explicit no_tracking result certifies EOF without one.
        outcome = meta.get('outcome', 'tracking_observed')
        if outcome != ('tracking_observed' if rows else 'no_tracking'):
            raise ValueError(f'ORB の追跡結果と保存姿勢数が一致しません: {folder.name}')
        series.append(rows)
        details.append(dict(outcome=outcome, no_tracking_reason=meta.get('no_tracking_reason'), run_id=folder.name,
                            map_id=meta['map_id'], map_sha256=meta['map_sha256'],
                            tracking=meta['tracking'], stereo_pairs=meta['input_pairs'],
                            processed_frames_observed=meta.get('processed_frames_observed'),
                            imu_frequency_hz=meta.get('imu_frequency_hz'), reacquisitions=reacquisitions))
    details[2].update(local_vio_samples=vio.get('local_vio_samples'),
                      map_matches_observed=vio.get('map_matches_observed'),
                      first_map_match_stamp_ns=vio.get('first_map_match_stamp_ns'))
    series = [_window(rows, start, end) for rows in series]
    for rows, info in zip(series, details):
        info.update(_output_stats(rows, start, end, visual=info['tracking'] == 'vo'))
        info['missing_epoch_samples'] = sum(row.epoch is None for row in rows)

    # Both ORB runs stamp poses with the original left image time. The cuVSLAM
    # stamp can differ slightly on the same stereo pair.
    vio_at = {row.stamp: row for row in series[2]}
    triples = [(cu_row, raw, vio_at[raw.stamp]) for cu_row, raw in _paired(*series[:2]) if raw.stamp in vio_at]
    anchor = None
    if record_root is not None:
        trace = _ground_truth(root, record_root, cu)
        for cu_row, raw, _ in triples:
            cu_gt, orb_gt = trace.at(cu_row.stamp), trace.at(raw.stamp)
            if cu_gt is None or orb_gt is None:
                continue
            cu_map = Alignment.from_anchor(cu_gt, cu_row.pose[:3], cu_row.pose[3:])
            orb_map = Alignment.from_anchor(orb_gt, raw.pose[:3], raw.pose[3:])
            _score(series[0], trace, cu_map, cu_row.stamp)
            # Both ORB variants report in the same fixed atlas. An independent
            # fit to the VIO run would hide an initial localization offset.
            for rows in series[1:]:
                _score(rows, trace, orb_map, raw.stamp)
            anchor = dict(cuvslam=cu_row.stamp, orbslam3=raw.stamp)
            break
    scored = [triple for triple in triples if all(row.error is not None for row in triple)]
    for index, (rows, info) in enumerate(zip(series, details)):
        info['paired_accuracy'] = _accuracy([triple[index].error for triple in scored])
        info['output_accuracy'] = _accuracy([row.error for row in rows if row.error is not None])
    return dict(schema='atd.vslam_fixed_benchmark.v1', mode='fixed_map', bag=cu['bag'], rate=cu['replay_rate'],
                metadata_sha256=cu['metadata_sha256'], source_metadata_sha256=cu['source_metadata_sha256'],
                window_ns=[start, end], gap_threshold_s=GAP_NS / 1e9, pairing_tolerance_s=PAIR_TOLERANCE_NS / 1e9,
                paired_observations=len(triples), anchor_stamp_ns=anchor,
                accuracy_frame='first_common_observation_per_map' if anchor else None,
                dds_profiles=_dds_profiles(cu, stereo, vio),
                sensor_input_transports=_sensor_input_transports(cu, stereo, vio),
                backends=dict(zip(('cuvslam', 'orbslam3', 'orbslam3_vio'), details)))

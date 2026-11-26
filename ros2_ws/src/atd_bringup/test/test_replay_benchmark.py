import csv
import hashlib
import json

import pytest

from atd_bringup import replay_benchmark as bench
from atd_bringup.replay_gt_source import store_ground_truth

NS = 1_000_000_000


def put(path, payload):
    path.write_text(json.dumps(payload))


def replay(root, name, backend, times):
    folder = root / 'runs/replay' / name
    folder.mkdir(parents=True)
    (folder / 'reference.json').write_text('[]')
    put(folder / 'summary.json', dict(id=name, status='complete', sample_count=len(times), backend=backend,
        mode='mapping', metadata_sha256='a'*64, bag='/bag', rate=1, tracking='vo', alignment_source='first_estimate',
        gt_start_stamp_ns=0, gt_finish_stamp_ns=NS))
    with (folder / 'samples.jsonl').open('w') as stream:
        for t in times:
            print(json.dumps(dict(stamp_ns=str(round(t*NS)), along_m=t, lateral_m=0, heading_deg=-10*t)), file=stream)
    return folder


def local(root, name, backend, times, *, evaluation_hash='a'*64):
    folder = root / 'runs/localization' / name
    folder.mkdir(parents=True)
    changes = [dict(state='localizing'), dict(state='tracking'), dict(state='lost'), dict(state='tracking')]
    put(folder / 'report.json', dict(backend=backend, map_id=name, map_sha256=name*64, bag='bag',
        metadata_sha256=evaluation_hash, source_metadata_sha256='b'*64, inputs=['/left', '/right'],
        replay_rate=1, outcome='tracked_to_end' if times else 'no_tracking',
        stereo=dict(first_stamp_ns=0, last_stamp_ns=NS, stereo_pairs=101),
        state_changes=changes if times else [dict(state='lost')]))
    with (folder / 'trajectory.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('stamp_ns', 'epoch', 'x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw'))
        for index, t in enumerate(times):
            # Completely different origins in each backend's map.
            writer.writerow((round(t*NS), 1 if index < 2 else 2, t+(90 if backend == 'orbslam3' else 31),
                             0, 0, 0, 0, 0, 1))
    return folder


def test_mapping_coverage_and_error_compare_the_same_observations(tmp_path, capsys):
    replay(tmp_path, 'cu', 'cuvslam', [0, .1, .2, .4, 1])
    replay(tmp_path, 'orb', 'orbslam3', [.102, .2, .399, .65, .8])
    result = bench.compare_runs(tmp_path, 'cu', 'orb')
    assert result['paired_observations'] == 3
    cu, orb = result['backends'].values()
    assert cu['observed_time_fraction'] == pytest.approx(.4)
    assert cu['internal_gaps_over_threshold'] == 1
    assert cu['paired_accuracy']['xy_median_m'] == .2
    assert cu['paired_accuracy']['abs_yaw_p95_deg'] == 4
    assert orb['initial_gap_s'] == .102
    assert orb['final_gap_s'] == .2
    assert bench.main(['cu', 'orb', '--results-root', str(tmp_path)]) == 0
    path, = tmp_path.glob('runs/vslam_benchmark/*/report.json')
    assert json.loads(path.read_text()) == result
    assert '共通観測' in capsys.readouterr().out


@pytest.mark.parametrize('field,value', [('mode', 'localization'), ('rate', .5), ('tracking', 'vio'),
                                         ('metadata_sha256', 'x'), ('start_offset_s', 1), ('status', 'running')])
def test_reject_mismatched_conditions(tmp_path, field, value):
    replay(tmp_path, 'cu', 'cuvslam', [0, 1])
    path = replay(tmp_path, 'orb', 'orbslam3', [0, 1]) / 'summary.json'
    put(path, {**json.loads(path.read_bytes()), field: value})
    with pytest.raises(ValueError):
        bench.compare_runs(tmp_path, 'cu', 'orb')


def test_previous_replay_checks_source_trace_and_bag_path(tmp_path):
    cu = replay(tmp_path, 'cu', 'cuvslam', [])
    orb = replay(tmp_path, 'orb', 'orbslam3', [])
    path = cu / 'summary.json'
    meta = json.loads(path.read_bytes())
    for key in ('metadata_sha256', 'alignment_source', 'backend', 'mode'):
        del meta[key]
    put(path, dict(meta, slam_max_map_size=0))
    assert bench.compare_runs(tmp_path, 'cu', 'orb')['mode'] == 'mapping'
    (orb / 'reference.json').write_text('other')
    with pytest.raises(ValueError, match='正解軌跡'):
        bench.compare_runs(tmp_path, 'cu', 'orb')


def test_fixed_maps_common_interval_count_reacquisition_and_no_tracking(tmp_path):
    cu = local(tmp_path, 'cu', 'cuvslam', [.1, .2, .22, .3])
    orb = local(tmp_path, 'orb', 'orbslam3', [])
    # Legacy validation output can identify the backend from its immutable map snapshot.
    path = cu / 'report.json'
    meta = json.loads(path.read_bytes())
    del meta['backend']
    # DDS can miss a brief loss notification while the next tracking update carries its new epoch.
    meta['state_changes'] = [dict(state='tracking', epoch=e) for e in (0, 1)]
    put(cu / 'hdmap.json', {'source': {'kind': 'nvidia_offline'}})
    put(path, dict(meta, hdmap_sha256=hashlib.sha256((cu / 'hdmap.json').read_bytes()).hexdigest()))
    result = bench.compare_runs(tmp_path, 'cu', 'orb')
    first, second = result['backends'].values()
    assert first['observed_time_fraction'] == pytest.approx(.18)  # don't bridge epoch change
    assert first['reacquisitions'] == 1
    assert second['observed_time_fraction'] == 0
    assert second['initial_gap_s'] is None
    assert result['accuracy_frame'] is None
    assert second['paired_accuracy'] is None
    path = orb / 'report.json'
    put(path, {**json.loads(path.read_bytes()), 'outcome': 'failed'})
    with pytest.raises(ValueError, match='完了していません'):
        bench.compare_runs(tmp_path, 'cu', 'orb')


def test_fixed_map_relative_gt_anchor_shared_and_excluded(tmp_path):
    bag = tmp_path / 'bags/bag'
    bag.mkdir(parents=True)
    (bag / 'metadata.yaml').write_text('ok')
    (bag / 'data.mcap').write_bytes(b'abc')
    tr = tmp_path / 'trace'
    tr.write_text('\n'.join(json.dumps(dict(sim_time_s=t, x_m=t, y_m=0, yaw_rad=0)) for t in (0, .5, 1)))
    store_ground_truth(bag, tr, {'data.mcap': 3})
    digest = hashlib.sha256(b'ok').hexdigest()
    local(tmp_path, 'cu', 'cuvslam', [0, .1, .2, .6], evaluation_hash=digest)
    local(tmp_path, 'orb', 'orbslam3', [.102, .2, .4, .6], evaluation_hash=digest)
    result = bench.compare_runs(tmp_path, 'cu', 'orb', bag.parent)
    assert result['paired_observations'] == 3
    assert result['accuracy_frame'] == 'aligned_at_first_common_observation'
    for backend in result['backends'].values():
        assert backend['paired_accuracy']['samples'] == 2
        assert backend['paired_accuracy']['xy_rmse_m'] < 1e-12
    (bag / 'metadata.yaml').write_text('other')
    with pytest.raises(ValueError, match='metadata.yaml'):
        bench.compare_runs(tmp_path, 'cu', 'orb', bag.parent)


def test_fixed_evaluation_requires_same_mapping_bag(tmp_path):
    local(tmp_path, 'cu', 'cuvslam', [])
    path = local(tmp_path, 'orb', 'orbslam3', [.2, .6]) / 'report.json'
    put(path, {**json.loads(path.read_text()), 'source_metadata_sha256': 'other'})
    with pytest.raises(ValueError, match='地図作成 bag'):
        bench.compare_runs(tmp_path, 'cu', 'orb')

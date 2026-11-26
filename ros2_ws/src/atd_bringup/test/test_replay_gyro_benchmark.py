import csv
import hashlib
import json
import math

import pytest

from atd_bringup.orb_gap_bridge import TRAJECTORY_FIELDS
from atd_bringup import replay_benchmark
from atd_bringup.replay_gt_source import store_ground_truth
from atd_bringup.replay_gyro_benchmark import compare_gyro_run
from atd_bringup.replay_inputs import IMU_TOPIC, STEREO_TOPICS
from atd_localization.replay_dds import replay_dds_profile


OBSERVED_MS = (0, 100, 500, 600)
POSE_COLUMNS = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')


def stamp(ms):
    return 1_000_000_000 + ms * 1_000_000


def pose(ms, origin, *, predicted=False):
    # 車体は直線上を 1 m/s で移動する。途絶中の予測だけに既知の横・方位誤差を与える。
    half_yaw = math.radians(7.5) if predicted else 0.
    return (origin + ms / 1000, .5 if predicted else 0., 0., 0., 0., math.sin(half_yaw), math.cos(half_yaw))


def write_csv(path, header, rows):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def fixture_runs(root, cu_ms=OBSERVED_MS, *, late_recovery=False):
    bag = root / 'records/test-laps'
    bag.mkdir(parents=True)
    (bag / 'metadata.yaml').write_text('evaluation')
    (bag / 'data.mcap').write_bytes(b'data')
    trace = root / 'trace.jsonl'
    trace.write_text('\n'.join(json.dumps(dict(sim_time_s=stamp(t)/1e9, x_m=t/1000, y_m=0., yaw_rad=0.))
                               for t in (0, 600)))
    store_ground_truth(bag, trace, {'data.mcap': 4})
    digest = hashlib.sha256((bag / 'metadata.yaml').read_bytes()).hexdigest()
    source = dict(source_metadata_sha256='b'*64, source_bag='mapping-lap', metadata_sha256=digest)

    cu = root / 'runs/localization/cu'
    cu.mkdir(parents=True)
    cu_meta = dict(**source, backend='cuvslam', bag=bag.name, map_id='cu-map', map_sha256='c'*64,
                   inputs=STEREO_TOPICS, replay_rate=1., outcome='tracked_to_end', pose_samples=len(cu_ms),
                   state_changes=[dict(state='tracking', epoch=1)],
                   stereo=dict(first_stamp_ns=stamp(0), last_stamp_ns=stamp(600), stereo_pairs=7))
    (cu / 'report.json').write_text(json.dumps(cu_meta))
    write_csv(cu/'trajectory.csv', ('stamp_ns', 'epoch', 'x_m', 'y_m', 'z_m', *POSE_COLUMNS[3:]),
              [(stamp(t), 1, *pose(t, 35)) for t in cu_ms])

    orb = root / 'runs/orbslam3/orb'
    orb.mkdir(parents=True)
    bridge = 'gyro_bridge_summary.json'
    meta = dict(**source, backend='orbslam3', mode='localization', tracking='vo', status='complete',
                map_id='orb-map', map_sha256='d'*64, bag=str(bag), rate=1., start_offset_s=0.,
                input_pairs=7, first_input_stamp_ns=stamp(0), last_input_stamp_ns=stamp(600),
                camera_fps=10., trajectory_samples=len(OBSERVED_MS), last_processed_stamp_ns=stamp(600),
                gyro_bridge=dict(topic=IMU_TOPIC, report=bridge))
    (orb / 'summary.json').write_text(json.dumps(meta))
    write_csv(orb/'trajectory.csv', ('image_stamp_ns', *POSE_COLUMNS),
              [(stamp(t), *pose(t, -17)) for t in OBSERVED_MS])
    write_csv(orb/'status.csv', ('stamp_ns', 'state', 'epoch', 'reason'),
              [(stamp(t), 'lost' if t == 200 else 'tracking', 1 if t < 200 else 2, '')
               for t in (0, 100, 200, 500, 600)])
    rows = []
    for t in range(0, 601, 100):
        predicted = t not in OBSERVED_MS or (late_recovery and t == 500)
        rows.append((stamp(t), 'predicted' if predicted else 'observed', (t-100)/1000 if predicted else 0.,
                     *pose(t, -17, predicted=predicted)))
    write_csv(orb/'gyro_trajectory.csv', TRAJECTORY_FIELDS, rows)
    predicted_count = sum(row[1] == 'predicted' for row in rows)
    (orb / bridge).write_text(json.dumps(dict(imu_topic=IMU_TOPIC, observed=len(rows)-predicted_count,
        predicted=predicted_count,
        counts=dict(gyro_samples=241), reacquisitions=1, measured_jumps=1, unavailable={},
        position_jump_m=dict(median=.5, max=.5), abs_yaw_jump_deg=dict(median=15., max=15.))))
    return bag.parent, orb


def test_predictions_increase_output_coverage_and_are_scored_during_the_gap(tmp_path, capsys):
    records, _ = fixture_runs(tmp_path)
    result = compare_gyro_run(tmp_path, 'cu', 'orb', records)
    assert result['dds_profiles']['comparison'] == 'unknown'
    streams = result['backends']
    raw, gyro = streams['orbslam3'], streams['orbslam3_gyro']
    assert raw['output_time_fraction'] == pytest.approx(1/3)
    assert gyro['output_time_fraction'] == pytest.approx(1.)
    assert gyro['predicted_samples'] == 3
    assert gyro['predicted_accuracy']['samples'] == 3
    assert gyro['predicted_accuracy']['xy_rmse_m'] == pytest.approx(.5)
    assert gyro['predicted_accuracy']['abs_yaw_median_deg'] == pytest.approx(15.)
    # At every common backend observation both maps are exact; none of the predictions occurred there.
    for stream in streams.values():
        assert stream['paired_accuracy']['xy_rmse_m'] == pytest.approx(0., abs=1e-12)
    assert gyro['bridge_summary']['measured_jumps'] == 1
    assert replay_benchmark.main(['cu', 'orb', '--results-root', str(tmp_path), '--record-root', str(records)]) == 0
    saved, = tmp_path.glob('runs/vslam_benchmark/*/report.json')
    assert json.loads(saved.read_text()) == result
    assert 'orbslam3_gyro' in capsys.readouterr().out


def test_recorded_dds_content_and_rmw_are_compared_across_runs(tmp_path, monkeypatch, capsys):
    _, orb = fixture_runs(tmp_path)
    profile = tmp_path / 'fastdds.xml'
    monkeypatch.setenv('FASTDDS_DEFAULT_PROFILES_FILE', str(profile))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SYSTEM_DEFAULT')
    cu_meta = tmp_path / 'runs/localization/cu/report.json'
    orb_meta = orb / 'summary.json'
    for path, xml in ((cu_meta, b'udp'), (orb_meta, b'shm')):
        profile.write_bytes(xml)
        recorded = replay_dds_profile()
        assert recorded['path'] == str(profile)
        assert recorded['discovery_range'] == 'SYSTEM_DEFAULT'
        assert recorded['sha256'] == hashlib.sha256(xml).hexdigest()
        path.write_text(json.dumps({**json.loads(path.read_text()), 'dds_profile': recorded}))
    assert replay_benchmark.main(['cu', 'orb', '--results-root', str(tmp_path)]) == 0
    saved, = tmp_path.glob('runs/vslam_benchmark/*/report.json')
    dds = json.loads(saved.read_text())['dds_profiles']
    assert dds['comparison'] == 'different'
    assert dds['cuvslam']['sha256'] != dds['orbslam3']['sha256']
    assert dds['warning'] in capsys.readouterr().err

    # 同じ設定を別パスに配置した場合も、実装と内容から照合する。
    for path in (cu_meta, orb_meta):
        meta = json.loads(path.read_text())
        meta['dds_profile'] = dict(rmw_implementation='rmw_fastrtps_cpp', path=str(path.parent / 'copy.xml'),
                                   sha256=recorded['sha256'], discovery_range='SYSTEM_DEFAULT')
        path.write_text(json.dumps(meta))
    assert compare_gyro_run(tmp_path, 'cu', 'orb')['dds_profiles']['comparison'] == 'match'
    meta['dds_profile']['discovery_range'] = 'LOCALHOST'
    orb_meta.write_text(json.dumps(meta))
    assert compare_gyro_run(tmp_path, 'cu', 'orb')['dds_profiles']['comparison'] == 'different'
    del meta['dds_profile']['discovery_range']
    orb_meta.write_text(json.dumps(meta))
    assert compare_gyro_run(tmp_path, 'cu', 'orb')['dds_profiles']['comparison'] == 'unknown'
    meta['dds_profile']['rmw_implementation'] = 'rmw_cyclonedds_cpp'
    orb_meta.write_text(json.dumps(meta))
    assert compare_gyro_run(tmp_path, 'cu', 'orb')['dds_profiles']['comparison'] == 'different'
    profile.unlink()
    assert replay_dds_profile()['sha256'] is None


@pytest.mark.parametrize('field,value', [('source_metadata_sha256', 'a'*64), ('metadata_sha256', 'e'*64),
                                         ('rate', .5), ('start_offset_s', .1)])
def test_refuses_different_mapping_evaluation_or_playback(tmp_path, field, value):
    records, orb = fixture_runs(tmp_path)
    path = orb / 'summary.json'
    path.write_text(json.dumps({**json.loads(path.read_text()), field: value}))
    with pytest.raises(ValueError):
        compare_gyro_run(tmp_path, 'cu', 'orb', records)


def test_no_shared_observation_still_reports_coverage_without_relative_accuracy(tmp_path):
    records, _ = fixture_runs(tmp_path, cu_ms=(30, 130, 530))
    streams = compare_gyro_run(tmp_path, 'cu', 'orb', records)['backends']
    assert streams['orbslam3_gyro']['output_time_fraction'] == pytest.approx(1.)
    assert all(item['paired_accuracy'] is None for item in streams.values())
    assert streams['orbslam3_gyro']['predicted_accuracy'] is None


def test_late_raw_observation_at_a_predicted_stamp_does_not_inflate_paired_accuracy(tmp_path):
    records, _ = fixture_runs(tmp_path, late_recovery=True)
    result = compare_gyro_run(tmp_path, 'cu', 'orb', records)
    assert result['paired_observations'] == 3  # 0, 100, 600 ms
    for item in result['backends'].values():
        assert item['paired_accuracy']['samples'] == 2  # 最初の時刻は座標合わせに使う
        assert item['paired_accuracy']['xy_rmse_m'] == pytest.approx(0., abs=1e-12)
    forecast = result['backends']['orbslam3_gyro']['predicted_accuracy']
    assert forecast['samples'] == 4
    assert forecast['xy_rmse_m'] == pytest.approx(.5)


def test_old_orb_run_uses_provenance_only_from_its_matching_saved_atlas(tmp_path, monkeypatch):
    _, orb = fixture_runs(tmp_path)
    path = orb / 'summary.json'
    summary = json.loads(path.read_text())
    metadata = {'source_metadata_sha256': summary.pop('source_metadata_sha256'),
                'map_sha256': summary['map_sha256']}
    path.write_text(json.dumps(summary))
    def load(root, map_id):
        assert root == tmp_path and map_id == 'orb-map'
        return metadata
    monkeypatch.setattr('atd_bringup.replay_gyro_benchmark.orb_map_metadata', load)
    assert compare_gyro_run(tmp_path, 'cu', 'orb')['source_metadata_sha256'] == metadata['source_metadata_sha256']
    metadata['map_sha256'] = '0'*64
    with pytest.raises(ValueError, match='地図'):
        compare_gyro_run(tmp_path, 'cu', 'orb')

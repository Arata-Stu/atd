"""ROS で実際に保存される区間集計と、UI が受け取る軌跡・喪失位置の整合性。"""
import csv
import json
from pathlib import Path

import pytest

from atd_api import localization_results as view
from atd_api.store import Store, atomic_json

NS = 10**9
START = 1_810_000_000 * NS
FIELDS = 'stamp_ns epoch x_m y_m z_m qx qy qz qw speed_mps yaw_rate_rps'.split()


@pytest.fixture
def run(tmp_path, monkeypatch):
    pkg = Path(__file__).resolve().parents[4] / 'ros2_ws/src/atd_localization'
    monkeypatch.syspath_prepend(str(pkg))
    from atd_localization.replay_report import TrackingReport

    store = Store(tmp_path / 'record', tmp_path / 'results')
    store.create_map('course')
    m = store.folder('course')
    atomic_json(m / 'vgl.json', dict(status='complete', cuvslam_sha256={'map.mdb': 'c'*64}))
    ref = store.live_reference('course')
    folder = store.result_root / 'runs/localization/trial'
    folder.mkdir(parents=True)
    original = (m / 'hdmap/map.json').read_bytes()
    header = dict(schema='atd.localization.validation.v1', map_id='course', bag='other/run', source_bag='mapping/run',
                  map_sha256=ref['map_sha256'], hdmap_sha256=ref['hdmap_sha256'], metadata_sha256='a'*64,
                  source_metadata_sha256='b'*64, frame='map', body_frame='base_link')

    def save(events, *, stride=.5, end=6):
        stats = TrackingReport(dict(first_stamp_ns=START, last_stamp_ns=START+round(end*NS)), max_gap_s=stride)
        with (folder / 'trajectory.csv').open('w', newline='') as file:
            out = csv.writer(file)
            out.writerow(FIELDS)
            for state, epoch, t in events:
                stamp = START + round(t*NS) if t is not None else 0
                pose = (t, epoch, 0, 0, 0, 0, 1, 1, 0)
                if stats.observe('session', epoch, state, state+' reason', stamp, pose if state == 'tracking' else None):
                    out.writerow([stamp, epoch, *pose])
        atomic_json(folder / 'report.json', dict(header, trajectory='trajectory.csv', **stats.summary()))

    return store, folder, original, save


def test_interruption_evidence_uses_state_order_and_separate_observed_endpoints(run):
    store, _, _, save = run
    save([('lost', 0, None), ('tracking', 0, 1), ('tracking', 0, 1.4),
          # The ROS notification time exceeds the sensor time in the next tracking message.
          ('stale', 1, 2.03), ('lost', 1, 2.1), ('tracking', 1, 2), ('tracking', 1, 2.5),
          ('tracking', 1, 4), ('tracking', 1, 4.4), ('stale', 2, 4.8)])
    data = view.load_result(store, 'trial')
    assert data['outcome'] == 'partial_tracking'
    assert [x['kind'] for x in data['gaps']] == ['lost', 'gap', 'stale']
    assert data['gaps'][0] == dict(kind='lost', reason='lost reason', start_s=1.4, end_s=2,
                                  before=dict(time_s=1.4, point=[1.4, 0]), after=dict(time_s=2, point=[2, 1]))
    assert data['gaps'][2]['before']['point'] == [4.4, 1] and data['gaps'][2]['after'] is None
    assert data['gaps'][2]['end_s'] == 6
    assert [s['points'] for s in data['segments']] == [[[1, 0], [1.4, 0]], [[2, 1], [2.5, 1]], [[4, 1], [4.4, 1]]]
    assert data['states'][0]['time_s'] == 0 and data['first_tracking_delay_s'] == 1


def test_preserves_original_raceline_or_withholds_newer_map(run):
    store, folder, original, save = run
    save([('tracking', 0, 5.5), ('tracking', 0, 6)])
    assert view.load_result(store, 'trial')['map_issue'] is None
    atomic_json(store.folder('course') / 'vgl.json', dict(status='complete', cuvslam_sha256={'map.mdb':'f'*64}))
    assert view.load_result(store, 'trial')['document'] is None
    store.save_map('course', dict(store.load_map('course'), title='edited'))
    (folder / 'hdmap.json').write_bytes(original)
    data = view.load_result(store, 'trial')
    assert data['document'] == json.loads(original) and data['map_issue'] is None
    (folder / 'hdmap.json').write_bytes(original+b' ')
    data = view.load_result(store, 'trial')
    assert data['document'] is None and 'SHA256' in data['map_issue']
    assert data['segments'][0]['points'] == [[5.5, 0], [6, 0]]


def test_no_tracking_failed_cancelled_and_directory_isolation(run):
    store, folder, _, save = run
    save([('waiting_hint', 0, None), ('lost', 0, 3)])
    data = view.load_result(store, 'trial')
    assert data['pose_samples'] == 0 and data['outcome'] == 'no_tracking'
    assert not data['segments'] and not data['gaps'] and data['states'][-1]['state'] == 'lost'
    report = json.loads((folder / 'report.json').read_text())
    (folder / 'trajectory.csv').unlink()
    for outcome in ('failed','cancelled'):
        atomic_json(folder / 'report.json', dict(report, outcome=outcome, error='input'))
        data = view.load_result(store, 'trial')
        assert data['pose_samples'] is None and data['error'] == 'input'
    (folder.parent/'run-alias').symlink_to(folder)
    (folder.parent/'in-progress').mkdir()
    assert [r['id'] for r in view.list_runs(store)] == ['trial']
    assert view.list_runs(store, 'other') == []
    with pytest.raises(ValueError):
        view.load_result(store, '../trial')
    with pytest.raises(ValueError):
        view.load_result(store, 'run-alias')


def test_downsampling_preserves_each_boundary_and_checks_every_csv_row(run, monkeypatch):
    store, folder, _, save = run
    monkeypatch.setattr(view, 'SAMPLE_BUDGET', 3)
    save([('tracking', 0, n/10) for n in range(10)] + [('tracking', 0, 4), ('tracking', 0, 4.3)])
    data = view.load_result(store, 'trial')
    assert data['pose_samples'] == 12 and data['displayed_samples'] < 12
    assert data['segments'][0]['points'][0] == [0,0] and data['segments'][0]['points'][-1] == [.9,0]
    assert data['segments'][1]['points'] == [[4,0],[4.3,0]]
    csv_file = folder/'trajectory.csv'
    lines=csv_file.read_text().splitlines()
    row = lines[3].split(',')
    row[2] = 'nan'
    lines[3] = ','.join(row)
    csv_file.write_text('\n'.join(lines)+'\n')
    with pytest.raises(ValueError, match='軌跡 CSV'):
        view.load_result(store, 'trial')


def test_duplicate_first_pose_after_interruption_is_not_a_recovery_position(run):
    store, _, _, save = run
    save([('tracking', 0, 1.), ('tracking', 0, 1.3), ('stale', 0, 1.4), ('tracking', 0, 1.3),
          ('tracking', 0, 1.6), ('tracking', 0, 2), ('lost', 0, 2.1), ('tracking', 0, 2)])
    data = view.load_result(store, 'trial')
    assert [item['kind'] for item in data['gaps']] == ['stale','lost']
    assert data['gaps'][0]['after'] == dict(time_s=1.6, point=[1.6, 0])
    assert data['gaps'][1]['after'] is None


def test_single_observed_pose_at_end_has_no_invented_loss(run):
    store, _, _, save = run
    save([('waiting_hint',0,None), ('tracking',0,6.)])
    result = view.load_result(store,'trial')
    assert result['outcome'] == 'partial_tracking' and result['tracking_time_fraction'] == 0.
    assert result['segments'][0]['points'] == [[6., 0]]
    assert result['gaps'] == []

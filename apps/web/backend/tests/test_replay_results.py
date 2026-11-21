import fcntl
import json
from pathlib import Path

from fastapi.testclient import TestClient
import pytest

from atd_api.main import create_app
from atd_api.store import atomic_json


@pytest.fixture
def saved(tmp_path):
    results = tmp_path / 'results'
    folder = results / 'runs/replay/monza-two-laps'
    folder.mkdir(parents=True)
    api = TestClient(create_app(tmp_path / 'record', results, tmp_path / 'static'))
    summary = dict(version=1, id=folder.name, bag='gazebo-monza-test', created_at='2026-11-21T10:40:46Z',
                   circuit='monza', status='running', sample_count=3, laps=2, lap_length_m=579.20,
                   rate=0.5, reason=None)
    points = [dict(time_s=n, distance_m=distance, station_m=station, lap=lap, gt_x_m=x, gt_y_m=y)
              for n, (distance, station, lap, x, y) in enumerate([
                  (523.9, 523.9, 1, 1.2, 0.1), (580.5, 1.3, 2, 3.4, .2), (583.4, 4.2, 2, 5., .3)])]
    rows = [dict(**p, est_x_m=p['gt_x_m']+.5, est_y_m=p['gt_y_m'], along_m=.1234567890123,
                 lateral_m=8.75 if i == 1 else -.1, heading_deg=-3., segment=min(i, 1),
                 stamp_ns=str(10**16+i)) for i, p in enumerate(points)]
    atomic_json(folder / 'summary.json', summary)
    atomic_json(folder / 'reference.json', points)
    (folder / 'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    return folder, api, summary, rows


def test_running_snapshot_preserves_precision_gaps_and_ignores_uncommitted_tail(saved):
    folder, api, summary, rows = saved
    # すでに flush され未確定の4行目と、書きかけの5行目。
    with (folder / 'samples.jsonl').open('ab') as stream:
        stream.write((json.dumps(rows[-1])+'\n').encode() + b'{"stamp_ns":')
    assert api.get('/api/replay/runs').json() == {'items': [summary]}
    reply = api.get('/api/replay/runs/' + folder.name)
    assert reply.status_code == 200
    data = reply.json()
    assert data['samples'] == rows
    assert [row['lap'] for row in data['reference']] == [1, 2, 2]
    assert data['reference'][1]['x_m'] == 3.4
    assert [s['id'] for s in data['sections']] == ['post_ascari', 'home']
    assert data['sections'][1]['start_m'] > data['sections'][1]['end_m']


def test_truncated_committed_data_is_rejected(saved):
    folder, api, summary, _ = saved
    atomic_json(folder / 'summary.json', dict(summary, sample_count=4))
    reply = api.get('/api/replay/runs/' + folder.name)
    assert reply.status_code == 400 and 'サンプル数' in reply.json()['detail']


@pytest.mark.parametrize('limit', [0, 300])
def test_map_limit_in_run_list_and_detail(saved, limit):
    folder, api, summary, _ = saved
    atomic_json(folder / 'summary.json', dict(summary, slam_max_map_size=limit))
    assert api.get('/api/replay/runs').json()['items'][0]['slam_max_map_size'] == limit
    assert api.get('/api/replay/runs/' + folder.name).json()['slam_max_map_size'] == limit


def test_interrupted_writer_keeps_last_committed_snapshot_without_modifying_archive(saved):
    folder, api, summary, rows = saved
    atomic_json(folder / 'summary.json', dict(summary, writer_lock='samples.jsonl'))
    with (folder / 'samples.jsonl').open('ab') as writer:
        fcntl.flock(writer, fcntl.LOCK_EX | fcntl.LOCK_NB)
        writer.write(b'{')
        assert api.get('/api/replay/runs').json()['items'][0]['status'] == 'running'
    data = api.get('/api/replay/runs/' + folder.name).json()
    assert data['status'] == 'partial' and 'チェックポイント' in data['reason']
    assert data['samples'] == rows
    assert json.loads((folder / 'summary.json').read_text())['status'] == 'running'


def test_concurrent_normal_close_preserves_completed_status(saved, monkeypatch):
    folder, api, summary, _ = saved
    stamped = dict(summary, writer_lock='samples.jsonl')
    atomic_json(folder / 'summary.json', stamped)
    flock = fcntl.flock

    def finalize_then_probe(file, flags):
        atomic_json(folder / 'summary.json', dict(stamped, status='complete'))
        return flock(file, flags)

    monkeypatch.setattr(fcntl, 'flock', finalize_then_probe)
    assert api.get('/api/replay/runs/' + folder.name).json()['status'] == 'complete'


def test_unknown_course_or_layout_withholds_sections_and_uses_measured_distance(saved):
    folder, api, summary, rows = saved
    for row in rows:
        row.update(station_m=None, lap=None)
    (folder / 'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    atomic_json(folder / 'reference.json', rows)
    for circuit, length in ((None, None), ('spa', 579.2), ('monza', 700.)):
        atomic_json(folder / 'summary.json', dict(summary, circuit=circuit, lap_length_m=length,
                                                  status='partial', reason='再生を中断しました'))
        data = api.get('/api/replay/runs/' + folder.name).json()
        assert not data['sections'] and data['status'] == 'partial'
        for series in ('reference', 'samples'):
            assert all(row['station_m'] == row['distance_m'] and row['lap'] is None for row in data[series])


def test_unavailable_run_and_symlinks(saved):
    folder, api, _, _ = saved
    (folder.parent / 'alias').symlink_to(folder)
    assert api.get('/api/replay/runs/missing').status_code == 404
    assert api.get('/api/replay/runs/alias').status_code == 400
    assert [run['id'] for run in api.get('/api/replay/runs').json()['items']] == [folder.name]
    for name in ('reference.json', 'samples.jsonl', 'summary.json'):
        path = folder / name
        moved = path.rename(folder.parent / name)
        path.symlink_to(moved)
        assert api.get('/api/replay/runs/' + folder.name).status_code == 400
        path.unlink()
        moved.rename(path)


def test_real_recorder_to_ui_and_partial_result(tmp_path, monkeypatch):
    source = Path(__file__).resolve().parents[4] / 'ros2_ws/src'
    for pkg in ('atd_bringup', 'atd_mapping'):
        monkeypatch.syspath_prepend(str(source / pkg))
    from atd_bringup.replay_gt_geometry import Alignment, PlanarPose, Trace
    from atd_bringup.replay_result import ReplayResult

    results = tmp_path / 'results'
    api = TestClient(create_app(tmp_path / 'record', results, tmp_path / 'static'))
    trace = Trace((0, 1_000_000_000, 2_000_000_000),
                  tuple(PlanarPose(x, 2., 0.) for x in (0., 1., 2.)))
    recorder = ReplayResult(results, 'manual-selection', .5, trace, {})
    url = '/api/replay/runs/' + recorder.directory.name
    assert api.get(url).json()['samples'] == []
    q = (0., 0., 0., 1.)
    alignment = Alignment.from_anchor(trace.poses[0], (0., 2., 0.), q)
    recorder.record(0, trace.poses[0], alignment, (0., 2., 0.), q)
    recorder.record(1_000_000_000, trace.poses[1], alignment, (1.5, 1., 0.), q)
    recorder.close(KeyboardInterrupt())
    result = api.get(url).json()
    assert result['status'] == 'partial' and result['sample_count'] == 2
    assert result['anchor_stamp_ns'] == '0' and result['gap_threshold_s'] == .25
    assert result['samples'][1]['along_m'] == .5 and result['samples'][1]['lateral_m'] == -1.
    assert result['samples'][1]['segment'] != result['samples'][0]['segment']
    assert result['reference'][-1]['x_m'] == 2. and result['reference'][-1]['lap'] is None

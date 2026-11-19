"""配布地図から編集コピー、独立 worker、比較用レポートまでの Notebook 経路。"""
import csv
import io
import time
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from atd_api import simulation_benchmark as benchmark
from atd_api.jobs import ACTIVE
from atd_api.main import create_app
from atd_api.store import read_json
from atd_processing.geometry import settings as generation_settings


@pytest.fixture
def client(tmp_path):
    with TestClient(create_app(tmp_path / 'record', tmp_path / 'results')) as client:
        yield client


def test_shipped_catalog_matches_hdmap_bundles(client):
    response = client.get('/api/simulation/courses')
    assert response.status_code == 200, response.text
    data = response.json()
    tracks = data['items']
    assert len(tracks) == len({t['folder'] for t in tracks}) == len({t['id'] for t in tracks}) == 23
    assert all(t['editable_id'] is None and t['sources'] for t in tracks)
    assert {t['id'] for t in tracks if not t['included']} == {'ims', 'moscowraceway', 'oschersleben'}
    assert next(t for t in tracks if t['id'] == 'ims')['hosted_f1']
    assert len(data['upstream']['revision']) == 40
    maps = benchmark.SAMPLES / 'maps'
    assert {p.name for p in maps.iterdir() if p.is_dir()} == {t['id'] for t in tracks if t['included']}
    for entry in tracks:
        if not entry['included']:
            continue
        folder = maps / entry['id']
        doc = read_json(folder / 'hdmap/map.json')
        assert doc['closed'] and doc['frame'] == 'map'
        assert len(doc['left']) >= 3 and len(doc['right']) >= 3
        for kind, method in [('centerline', 'region_equal_distance_v1'), ('raceline', 'tph_mincurv')]:
            line = doc['lines'][kind]
            saved = read_json(folder / 'lines' / (kind + '.json'))
            assert line['method'] == method
            assert line['settings'] == generation_settings({})
            assert len(line['profile']) == len(line['points']) >= 3
            assert (saved['points'], saved['profile']) == (line['points'], line['profile'])


def test_editable_copy_preserves_original_and_existing_work(client):
    endpoint = '/api/simulation/courses/monza/edit'
    original = benchmark.SAMPLES / 'maps/monza/hdmap/map.json'
    before = original.read_bytes()
    first = client.post(endpoint)
    assert first.status_code == 200, first.text
    map_id = first.json()['map_id']
    doc = client.get('/api/maps/' + map_id).json()
    assert doc['source']['benchmark_course'] == 'monza'
    assert doc['lines'] == read_json(original)['lines']
    copy = client.app.state.store.folder(map_id)
    assert all(read_json(copy / 'lines' / (kind + '.json'))['map_id'] == map_id
               for kind in ('centerline', 'raceline'))
    edit = client.put('/api/maps/' + map_id, json={'revision': doc['revision'], 'title': '試験用の変更'})
    assert edit.status_code == 200, edit.text
    assert client.post(endpoint).json() == first.json()
    assert client.get('/api/maps/' + map_id).json()['title'] == '試験用の変更'
    assert original.read_bytes() == before
    entries = client.get('/api/simulation/courses').json()['items']
    assert next(t for t in entries if t['id'] == 'monza')['editable_id'] == map_id


def test_unrelated_existing_map_is_not_replaced(client):
    key = 'f1tenth-monza'
    assert client.post('/api/maps', json={'id': key, 'title': '手作業の地図'}).status_code == 200
    assert client.post('/api/simulation/courses/monza/edit').status_code == 400
    assert client.get('/api/maps/' + key).json()['title'] == '手作業の地図'


def test_trace_uses_working_map_and_requires_new_line_after_boundary_edit(client):
    map_id = client.post('/api/simulation/courses/monza/edit').json()['map_id']
    url = '/api/simulation/courses/monza/compare'
    opts = dict(source='workspace', line='centerline', laps=1, trial=1)
    response = client.post(url, json=opts)
    assert response.status_code == 200, response.text
    trace = response.json()
    assert trace['course']['id'] == map_id
    assert all(run['samples'] for run in trace['runs'])
    doc = client.get('/api/maps/' + map_id).json()
    boundary = doc['left']
    # 同じ輪郭の始点だけを入れ替える。ジオメトリー編集が従属ラインを無効化する。
    saved = client.put('/api/maps/' + map_id, json={'revision': doc['revision'], 'left': boundary[1:] + boundary[:1]})
    assert saved.status_code == 200, saved.text
    assert not saved.json()['lines']
    assert client.post(url, json=opts).status_code == 400


def test_worker_rejects_inputs_changed_after_acceptance(client):
    key = client.post('/api/simulation/courses/monza/edit').json()['map_id']
    store = client.app.state.store
    payload = benchmark.prepare(store, benchmark.Batch(course_ids=['monza'], source='workspace', laps=1, trials=1))
    doc = store.load_map(key)
    line = doc['lines']['raceline']
    # 境界を保ったまま再生成した速度プロファイルだけを変更する。
    line['speeds'][0] *= .9
    line['profile'][0][5] = line['speeds'][0]
    store.save_line(key, doc['revision'], 'raceline', line)
    task = SimpleNamespace(key='changed', check=lambda: None, log=lambda _message: None)
    with pytest.raises(ValueError, match='更新'):
        benchmark.execute(store, task, payload)
    assert not (store.result_root / benchmark.REPORTS / task.key / 'report.json').exists()


def test_api_job_publishes_paired_json_and_csv(client):
    opts = dict(course_ids=['monza'], line='centerline', laps=1, trials=1, settings={'steering_delay_ms': 25})
    started = client.post('/api/simulation/benchmarks', json=opts)
    assert started.status_code == 200, started.text
    key = started.json()['id']
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        job = client.get('/api/jobs/' + key).json()
        if job['status'] not in ACTIVE:
            break
        time.sleep(.05)
    assert job['status'] == 'succeeded', client.get('/api/jobs/' + key + '/log').text
    assert job['result']['report_id'] == key
    root = '/api/simulation/benchmarks/' + key
    report = client.get(root).json()
    assert report['options']['settings']['steering_delay_ms'] == 25
    assert report['options']['source'] == 'sample'
    assert report['upstream']['revision'] == benchmark.catalog()['upstream']['revision']
    assert report['implementation_sha256']
    course, = report['courses']
    assert course['course_id'] == 'monza' and course['line'] == 'centerline'
    assert course['course_sha256'] == report['options']['course_sha256']['monza']
    assert key in [r['id'] for r in client.get('/api/simulation/benchmarks').json()['items']]
    csv_response = client.get(root + '/summary.csv')
    assert csv_response.status_code == 200 and 'text/csv' in csv_response.headers['content-type']
    rows = list(csv.DictReader(io.StringIO(csv_response.text)))
    assert [r['mode'] for r in rows] == ['fixed', 'adaptive']
    for result, row in zip(course['results'], rows, strict=True):
        run, = result['runs']
        assert row['course_sha256'] == course['course_sha256'] and row['course_id'] == 'monza'
        assert row['line'] == 'centerline' and row['mode'] == result['mode']
        assert int(row['trials']) == result['trials'] == 1
        assert int(row['finishes']) == result['finishes'] == (run['outcome'] == 'finished')
        assert float(row['finish_rate']) == result['finish_rate']
        assert int(row['collisions']) == result['collisions'] == (run['outcome'] == 'collision')
        if run['lap_times_s']:
            assert float(row['mean_lap_time_s']) == run['lap_times_s'][0] > 0
        else:
            assert not row['mean_lap_time_s'] and result['mean_lap_time_s'] is None


def test_bundled_baseline_covers_shipped_courses_and_matches_geometry(client):
    assert client.get('/api/simulation/benchmarks').json()['baseline']
    report = client.get('/api/simulation/benchmarks/baseline').json()
    included = set(benchmark.included())
    assert {row['course_id'] for row in report['courses']} == included
    assert len(report['courses']) == len(included)
    assert set(report['options']['course_ids']) == included
    for course in report['courses']:
        key = course['course_id']
        doc = read_json(benchmark.SAMPLES / 'maps' / key / 'hdmap/map.json')
        assert benchmark.signature(doc, course['line']) == course['course_sha256']
        assert course['course_sha256'] == report['options']['course_sha256'][key]
        assert [r['mode'] for r in course['results']] == ['fixed', 'adaptive']
        assert all(len(r['runs']) == report['options']['trials'] for r in course['results'])


@pytest.mark.parametrize(('body', 'status'), [
    ({'course_ids': ['ims']}, 400),
    ({'course_ids': ['missing']}, 400),
    ({'course_ids': ['monza', 'monza']}, 400),
    ({'course_ids': ['monza'], 'source': 'workspace'}, 400),
    ({'course_ids': ['monza'], 'trials': 0}, 422),
    ({'course_ids': [], 'laps': 1}, 422),
])
def test_invalid_request_creates_no_job(client, body, status):
    response = client.post('/api/simulation/benchmarks', json=body)
    assert response.status_code == status, response.text
    assert client.get('/api/jobs').json()['items'] == []

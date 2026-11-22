"""保存した地図から独立 worker、比較用レポートまでの Notebook 経路。"""
import csv
import io
import math
import time
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from atd_api import simulation_benchmark as benchmark
from atd_api.jobs import ACTIVE
from atd_api.main import create_app
from atd_api.sample_map import ODAIBA_COURSE, ODAIBA_MAP, odaiba_document
from atd_api.store import atomic_json, read_json
from atd_processing.geometry import generate


@pytest.fixture
def client(tmp_path):
    with TestClient(create_app(tmp_path / 'record', tmp_path / 'results')) as client:
        yield client


@pytest.fixture
def course(client):
    store = client.app.state.store
    doc = store.create_map('short-loop', '短い周回テスト')

    def circle(radius):
        return [[radius * math.cos(i * math.tau / 32), radius * math.sin(i * math.tau / 32)] for i in range(32)]

    doc = store.save_map(doc['id'], dict(revision=doc['revision'], left=circle(2.), right=circle(3.)))
    line = generate(doc, 'centerline', {})
    return store.save_line(doc['id'], doc['revision'], 'centerline', line)['id']


def test_catalog_lists_saved_closed_maps_with_profiles(client, course):
    store = client.app.state.store
    store.create_map('empty')
    valid = store.load_map(course)
    for key, changes in [('open-map', dict(closed=False)),
                         ('missing-profile', dict(lines={'centerline': {'points': [[1, 1]]}})),
                         ('previous-sample', dict(source={'kind': 'f1tenth_benchmark'}))]:
        target = store.create_map(key)
        atomic_json(store.folder(key) / 'hdmap/map.json', dict(valid, **changes, id=target['id']))
        assert client.post(f'/api/simulation/courses/{key}/compare', json={'line': 'centerline'}).status_code == 400
    response = client.get('/api/simulation/courses')
    sample, saved = response.json()['items']
    assert sample['id'] == ODAIBA_COURSE and sample['lines'] == ['raceline', 'centerline']
    assert saved == dict(id=course, name='短い周回テスト', lines=['centerline'])
    assert store.load_map('previous-sample')['lines'] == valid['lines']


def test_saved_result_readable_without_original_course(client):
    assert [item['id'] for item in client.get('/api/simulation/courses').json()['items']] == [ODAIBA_COURSE]
    assert client.get('/api/simulation/benchmarks').json() == {'items': []}
    report = dict(id='previous', schema='atd.simulation_benchmark.v1',
                  options={'source': 'sample'}, courses=[])
    store = client.app.state.store
    atomic_json(store.result_root / benchmark.REPORTS / 'previous/report.json', report)
    assert client.get('/api/simulation/benchmarks').json()['items'][0]['id'] == 'previous'
    assert client.get('/api/simulation/benchmarks/previous').json() == report


def test_trace_uses_saved_map_and_requires_new_line_after_boundary_edit(client, course):
    map_id = course
    url = '/api/simulation/courses/' + course + '/compare'
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


def test_odaiba_edit_copy_does_not_shadow_or_overwrite_workspace_maps(client):
    store = client.app.state.store
    raw = ODAIBA_MAP.read_bytes()
    existing = client.post('/api/maps', json={'id': 'odaiba', 'sample': 'odaiba', 'title': '編集したお台場'}).json()
    items = client.get('/api/simulation/courses').json()['items']
    assert [item['id'] for item in items] == [ODAIBA_COURSE, 'odaiba']
    assert items[1]['name'] == '編集したお台場'
    assert client.post('/api/maps', json={'id': 'odaiba', 'sample': 'odaiba'}).status_code == 400
    assert store.load_map('odaiba') == existing
    key = 'practice-odaiba'
    response = client.post('/api/maps', json={'id': key, 'sample': 'odaiba'})
    assert response.status_code == 200, response.text
    doc = response.json()
    original = odaiba_document()
    assert doc['id'] == key and doc['source'] == original['source']
    for field in ('left', 'right', 'lines'):
        assert doc[field] == original[field]
    for kind, line in doc['lines'].items():
        assert read_json(store.folder(key) / 'lines' / (kind + '.json')) == dict(line, map_id=key, revision=doc['revision'])
    opts = benchmark.Batch(course_ids=[ODAIBA_COURSE, 'odaiba'])
    both = benchmark.prepare(store, opts)
    assert both['source'] == 'mixed' and len(set(both['course_sha256'].values())) == 1
    boundary = doc['left']
    store.save_map(key, dict(revision=doc['revision'], left=boundary[1:] + boundary[:1]))
    assert not store.load_map(key)['lines']
    assert ODAIBA_MAP.read_bytes() == raw


def test_odaiba_comparison_uses_bundled_raceline(client):
    response = client.post(f'/api/simulation/courses/{ODAIBA_COURSE}/compare', json={'laps': 1, 'source': 'sample'})
    assert response.status_code == 200, response.text
    data = response.json()
    assert data['course']['id'] == ODAIBA_COURSE
    assert len(data['course']['points']) == len(odaiba_document()['lines']['raceline']['points'])
    assert {run['mode'] for run in data['runs'] if run['samples']} == {'fixed', 'adaptive'}
    assert client.app.state.store.list_maps() == []


def test_worker_rejects_inputs_changed_after_acceptance(client, course):
    key = course
    store = client.app.state.store
    payload = benchmark.prepare(store, benchmark.Batch(course_ids=[key], line='centerline', laps=1, trials=1))
    doc = store.load_map(key)
    line = doc['lines']['centerline']
    # 境界を保ったまま再生成した速度プロファイルだけを変更する。
    line['speeds'][0] *= .9
    line['profile'][0][5] = line['speeds'][0]
    store.save_line(key, doc['revision'], 'centerline', line)
    task = SimpleNamespace(key='changed', check=lambda: None, log=lambda _message: None)
    with pytest.raises(ValueError, match='更新'):
        benchmark.execute(store, task, payload)
    assert not (store.result_root / benchmark.REPORTS / task.key / 'report.json').exists()


def test_api_job_publishes_paired_json_and_csv(client, course):
    map_id = course
    opts = dict(course_ids=[map_id], line='centerline', laps=1, trials=1, settings={'steering_delay_ms': 25})
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
    assert report['options']['source'] == 'workspace'
    assert report['implementation_sha256']
    course, = report['courses']
    assert course['course_id'] == map_id and course['line'] == 'centerline'
    assert course['course_sha256'] == report['options']['course_sha256'][map_id]
    assert key in [r['id'] for r in client.get('/api/simulation/benchmarks').json()['items']]
    csv_response = client.get(root + '/summary.csv')
    assert csv_response.status_code == 200 and 'text/csv' in csv_response.headers['content-type']
    rows = list(csv.DictReader(io.StringIO(csv_response.text)))
    assert [r['mode'] for r in rows] == ['fixed', 'adaptive']
    for result, row in zip(course['results'], rows, strict=True):
        run, = result['runs']
        assert row['course_sha256'] == course['course_sha256'] and row['course_id'] == map_id
        assert row['line'] == 'centerline' and row['mode'] == result['mode']
        assert int(row['trials']) == result['trials'] == 1
        assert int(row['finishes']) == result['finishes'] == (run['outcome'] == 'finished')
        assert float(row['finish_rate']) == result['finish_rate']
        assert int(row['collisions']) == result['collisions'] == (run['outcome'] == 'collision')
        if run['lap_times_s']:
            assert float(row['mean_lap_time_s']) == run['lap_times_s'][0] > 0
        else:
            assert not row['mean_lap_time_s'] and result['mean_lap_time_s'] is None


@pytest.mark.parametrize(('body', 'status'), [
    ({'course_ids': ['missing'], 'source': 'sample'}, 400),
    ({'course_ids': ['missing']}, 400),
    ({'course_ids': ['short-loop', 'short-loop']}, 400),
    ({'course_ids': ['short-loop'], 'trials': 0}, 422),
    ({'course_ids': [], 'laps': 1}, 422),
])
def test_invalid_request_creates_no_job(client, body, status):
    response = client.post('/api/simulation/benchmarks', json=body)
    assert response.status_code == status, response.text
    assert client.get('/api/jobs').json()['items'] == []

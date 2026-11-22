import time

from fastapi.testclient import TestClient
import pytest

from atd_api.main import create_app
from atd_api.store import atomic_json, read_json


@pytest.fixture
def library(tmp_path):
    app = create_app(tmp_path / 'record', tmp_path / 'results')
    store = app.state.store
    document = store.create_map('track')
    folder = store.folder('track')
    line = dict(method='manual', points=[[0, 0], [1, 0]], speeds=[1, 1])
    source = dict(kind='nvidia_offline', pointcloud_status='ready', capture_relative='hdmap/captures/a')
    document.update(left=[[0, 1]], right=[[0, -1]], custom=[[0, 0]], lines=dict(custom=line),
                    source=source, raceline_calibration=dict(run_id='cal'))
    atomic_json(folder / 'hdmap/map.json', document)
    atomic_json(folder / 'lines/custom.json', dict(map_id='track', revision=1, **line))
    atomic_json(folder / 'exports/r1/manifest.json', dict(map_id='track'))
    atomic_json(folder / 'exports/r1/hdmap.yaml', document)
    atomic_json(folder / 'nvidia/test.json', {'sample': True})
    atomic_json(folder / 'vgl.json', {'transaction_id': 'vgl'})
    capture = folder / source['capture_relative']
    atomic_json(capture / 'manifest.json', dict(map_id='track', vgl_transaction='vgl'))
    atomic_json(capture / 'cloud.json', dict(points=[[1, 2, 3]], provenance=dict(map_id='track')))
    atomic_json(capture / 'snapshot.json', dict(provenance=dict(map_id='track')))
    with TestClient(app) as client:
        yield client, store, folder


def delete(client, scope, revision=1):
    return client.request('DELETE', '/api/maps/track', json=dict(scope=scope, revision=revision))


def wait_job(client, response):
    assert response.status_code == 200, response.text
    job = response.json()
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        job = client.get('/api/jobs/' + job['id']).json()
        if job['status'] not in ('queued', 'starting', 'running'):
            return job
        time.sleep(.03)
    pytest.fail('地図複製の worker が終了しませんでした')


@pytest.mark.parametrize('scope', ['map', 'hdmap'])
def test_duplicate_via_worker(library, scope):
    client, store, original = library
    params = dict(destination='copy', scope=scope, revision=1)
    job = wait_job(client, client.post('/api/maps/track/duplicate', json=params))
    assert job['status'] == 'succeeded', job['message']
    clone = store.folder('copy')
    document = store.load_map('copy')
    assert document['id'] == document['title'] == 'copy' and document['revision'] > 1
    assert read_json(clone / 'lines/custom.json')['map_id'] == 'copy'
    assert read_json(original / 'lines/custom.json')['map_id'] == 'track'
    assert client.get('/api/maps/copy/cloud').json()['points'] == [[1, 2, 3]]
    assert store.load_map('track')['revision'] == 1
    if scope == 'map':
        assert (clone / 'nvidia/test.json').is_file()
        assert (clone / 'vgl.json').read_bytes() == (original / 'vgl.json').read_bytes()
        assert read_json(clone / 'hdmap/captures/a/manifest.json')['map_id'] == 'copy'
        assert read_json(clone / 'hdmap/captures/a/snapshot.json')['provenance']['map_id'] == 'track'
        assert read_json(clone / 'exports/r1/hdmap.yaml')['id'] == 'copy'
    else:
        assert document['source']['kind'] == 'hdmap_copy'
        assert not (clone / 'nvidia').exists()
        assert not (clone / 'hdmap/captures').exists()
        assert not (clone / 'exports').exists()
    assert client.post('/api/maps/track/duplicate', json=params).status_code == 400


def test_delete_respects_scope_revision_and_jobs(library):
    client, store, folder = library
    other = store.create_map('other')
    assert delete(client, 'hdmap', 3).status_code == 400
    job = client.app.state.jobs.root / 'work.json'
    atomic_json(job, dict(id='work', status='running', resources=['map:track'], title='処理中', started=time.time()))
    assert delete(client, 'map').status_code == 400
    assert client.post('/api/maps/track/duplicate', json=dict(scope='map', destination='copy', revision=1)).status_code == 400
    job.unlink()
    assert delete(client, 'hdmap').is_success
    doc = store.load_map('track')
    assert doc['revision'] == 2 and not (doc['left'] or doc['right'] or doc['custom'] or doc['lines'])
    assert 'raceline_calibration' not in doc
    assert not (folder / 'lines').exists() and not (folder / 'exports').exists()
    assert (folder / 'vgl.json').is_file() and (folder / 'nvidia/test.json').is_file()
    assert client.get('/api/maps/track/cloud').json()['points'] == [[1, 2, 3]]
    assert delete(client, 'map', 2).is_success
    assert not folder.exists() and store.list_maps() == [other]


def test_symlink_copy_fails_without_publishing(library, tmp_path):
    client, store, folder = library
    outside = tmp_path / 'private'
    outside.write_text('outside')
    (folder / 'link').symlink_to(outside)
    job = wait_job(client, client.post('/api/maps/track/duplicate', json=dict(scope='map', destination='copy', revision=1)))
    assert job['status'] == 'failed'
    assert not (store.maps_root / 'copy').exists()
    assert delete(client, 'map').is_success and outside.read_text() == 'outside'


def test_gazebo_without_hdmap(library):
    client, store, _ = library
    raw = store.maps_root / 'gazebo-monza'
    manifest = dict(map_id=raw.name, source='gazebo_live_cuvslam', run='runs/gazebo_monza/source')
    atomic_json(raw / 'mapping.json', manifest)
    db = raw / 'nvidia/cuvslam_map'
    db.mkdir(parents=True)
    (db / 'map.mdb').write_bytes(b'map-data')
    # 未登録のフォルダーや未完成の地図を管理対象に混ぜない。
    invalid = store.maps_root / 'incomplete'
    atomic_json(invalid / 'mapping.json', dict(manifest, map_id='incomplete'))
    catalog = client.get('/api/maps').json()
    assert [entry['id'] for entry in catalog['items']] == ['track']
    assert [entry['id'] for entry in catalog['gazebo_maps']] == [raw.name]
    signature = catalog['gazebo_maps'][0]['gazebo_manifest_sha256']
    url = '/api/maps/' + raw.name
    params = dict(scope='map', gazebo_manifest_sha256=signature)
    assert client.request('DELETE', url, json=dict(params, scope='hdmap')).status_code == 400
    assert client.request('DELETE', url, json=dict(params, gazebo_manifest_sha256='0'*64)).status_code == 400
    copied = wait_job(client, client.post(url + '/duplicate', json=dict(params, destination='g-copy')))
    assert copied['status'] == 'succeeded', copied['message']
    clone = store.folder('g-copy')
    assert not (clone / 'hdmap').exists()
    assert read_json(clone / 'mapping.json') == dict(manifest, map_id='g-copy')
    assert (clone / 'nvidia/cuvslam_map/map.mdb').read_bytes() == b'map-data'
    assert client.request('DELETE', url, json=params).is_success
    assert not raw.exists()
    assert [entry['id'] for entry in client.get('/api/maps').json()['gazebo_maps']] == ['g-copy']


def test_replaced_maps_root_cannot_delete_external_data(library, tmp_path):
    client, store, _ = library
    outside = tmp_path / 'outside'
    store.maps_root.rename(outside)
    store.maps_root.symlink_to(outside, target_is_directory=True)
    assert delete(client, 'map').status_code == 400
    assert client.post('/api/maps/track/duplicate', json=dict(scope='map', destination='copy', revision=1)).status_code == 400
    assert (outside / 'track/hdmap/map.json').is_file()

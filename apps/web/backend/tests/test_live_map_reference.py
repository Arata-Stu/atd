import hashlib
import json

from fastapi.testclient import TestClient
from atd_api.main import create_app
from atd_api.store import atomic_json


def test_live_reference_compares_transferred_bytes_and_vgl_map_content(tmp_path):
    app = create_app(tmp_path / 'rec', tmp_path / 'out')
    store = app.state.store
    store.create_map('course')
    root = store.folder('course')
    with TestClient(app) as client:
        url = '/api/maps/course/live-reference'
        ref = client.get(url).json()
        assert ref['map_sha256'] is None
        hashes = {'track.mdb': 'a' * 64, 'tiles/': 'directory'}
        atomic_json(root / 'vgl.json', dict(status='complete', cuvslam_sha256=hashes))
        ref = client.get(url).json()
        assert ref['document'] == store.load_map('course')
        assert ref['hdmap_sha256'] == hashlib.sha256((root/'hdmap/map.json').read_bytes()).hexdigest()
        assert ref['map_sha256'] == hashlib.sha256(json.dumps(hashes, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
        store.save_map('course', {**ref['document'], 'title':'updated'})
        updated = client.get(url).json()
        assert ref['map_sha256'] == updated['map_sha256'] and ref['hdmap_sha256'] != updated['hdmap_sha256']

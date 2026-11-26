"""実 API と独立 worker を使い、ROS 実行先だけをテスト用 executable に置き換える。"""
from hashlib import sha256
import os
from pathlib import Path
import time

from fastapi.testclient import TestClient
import pytest

from atd_api import localization_validation as validation
from atd_api.main import create_app
from atd_api.store import atomic_json

FAKE_VALIDATOR = '''#!/usr/bin/env python3
import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import signal
import sys
import time

p = argparse.ArgumentParser()
for option in ('run-id','map','bag','rate','domain','results-root','record-root','engine-dir','map-correction-source','vgl-frequency'):
    p.add_argument('--' + option)
args = p.parse_args()
assert os.environ['ATD_REPLAY_DDS'] == 'udp'
root = Path(args.results_root)
run = root / 'runs/localization' / args.run_id
run.mkdir(parents=True)
if os.environ.get('ATD_TEST_WAIT'):
    def terminate(*args):
        (run / 'cleaned').touch()
        sys.exit(130)
    signal.signal(signal.SIGTERM, terminate)
    (run / 'ready').touch()
    time.sleep(60)
folder = root / 'maps' / args.map
doc = (folder / 'hdmap/map.json').read_bytes()
source = json.loads(doc)['source']
fingerprint = json.loads((folder / 'vgl.json').read_text())['cuvslam_sha256']
bag_hash = sha256((Path(args.record_root) / args.bag / 'metadata.yaml').read_bytes()).hexdigest()
report = dict(schema='atd.localization.validation.v1', map_id=args.map, bag=args.bag, hdmap_sha256=sha256(doc).hexdigest(),
    map_sha256=sha256(json.dumps(fingerprint, sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
    metadata_sha256=bag_hash, source_bag=source['bag'], source_metadata_sha256=source['metadata_sha256'],
    frame='map', body_frame='base_link', ros_domain_id=int(args.domain), replay_rate=float(args.rate),
    outcome=os.environ.get('ATD_TEST_OUTCOME', 'partial_tracking'))
if args.map_correction_source == 'vgl':
    report.update(map_correction_source='vgl', vgl_frequency_hz=float(args.vgl_frequency or '1'))
(run / 'report.json').write_text(json.dumps(report))
sys.exit(0 if report['outcome'] == 'tracked_to_end' else 2)
'''


@pytest.fixture
def api(tmp_path, monkeypatch):
    monkeypatch.delenv('ATD_REPLAY_DDS', raising=False)
    record, results = tmp_path / 'record', tmp_path / 'results'
    for name, data in [('day1/lap', b'mapping'), ('copy/lap', b'mapping'), ('day2/lap', b'validation')]:
        bag = record / 'rosbags' / name
        bag.mkdir(parents=True)
        (bag / 'metadata.yaml').write_bytes(data)
    app = create_app(record, results)
    store = app.state.store
    doc = store.create_map('circuit')
    source = dict(kind='nvidia_offline', bag='day1/lap', metadata_sha256=sha256(b'mapping').hexdigest(),
                  inputs=dict(base_frame='base_link'))
    root = store.folder('circuit')
    doc.update(source=source, lines=dict(raceline=dict(points=[[0., 0.], [1., 0.]])))
    atomic_json(root / 'hdmap/map.json', doc)
    build = dict(status='complete', asset_id='asset', export_id='test', bundle_sha256='b'*64)
    atomic_json(root / 'vgl.json', dict(build, cuvslam_sha256={'file.mdb':'f'*64}))
    atomic_json(results / 'models/vgl/asset/engines/my-target/build1/build.json', build)
    store.create_map('manual')
    (tmp_path / 'bin').mkdir()
    prefix = tmp_path / 'localization'
    executable = prefix / 'lib/atd_localization/validate_bag'
    executable.parent.mkdir(parents=True)
    executable.write_text(FAKE_VALIDATOR)
    executable.chmod(0o755)
    shell = tmp_path / 'bin/bash'
    shell.write_text('#!/bin/sh\nif [ "$3" = atd-ros-worker ]; then shift 3; exec "$@"; fi\n' +
                     'printf "%s\\n" "$ATD_TEST_PREFIX"\n')
    shell.chmod(0o755)
    monkeypatch.setenv('ATD_TEST_PREFIX', str(prefix))
    monkeypatch.setenv('PATH', str(shell.parent) + os.pathsep + os.environ['PATH'])
    validation._probe.cache_clear()
    with TestClient(app) as client:
        yield client
    validation._probe.cache_clear()


def test_options_and_preflight(api):
    options = api.get('/api/localization/options', params={'map_id':'circuit'}).json()
    assert options['available'] and options['engine_dirs'] == ['models/vgl/asset/engines/my-target/build1']
    circuit, manual = options['maps']
    assert circuit['ready'] and circuit['source_bag'] == 'day1/lap' and circuit['has_raceline']
    assert not manual['ready'] and manual['issue']
    assert {r['id']:r['eligible'] for r in options['bags']} == {'day1/lap':False,'copy/lap':False,'day2/lap':True}
    for override in [{'map_id':'manual'}, {'revision':2}, {'bag':'../outside'}, {'bag':'copy/lap'},
                     {'engine_dir':'models/vgl/unrelated'}, {'vgl_frequency':1.0}]:
        body = dict(map_id='circuit', revision=1, bag='day2/lap') | override
        assert api.post('/api/localization/runs', json=body).status_code == 400


@pytest.mark.parametrize('outcome,mode,frequency', [
    ('tracked_to_end', {}, None), ('partial_tracking', {'map_correction_source': 'vgl'}, 1.0),
    ('no_tracking', {'map_correction_source': 'vgl', 'vgl_frequency': 0.0}, 0.0),
])
def test_job_result_belongs_to_this_run(api, monkeypatch, outcome, mode, frequency):
    monkeypatch.setenv('ATD_TEST_OUTCOME', outcome)
    response = api.post('/api/localization/runs', json=dict(map_id='circuit', revision=1, bag='day2/lap', rate=.5, **mode))
    assert response.status_code == 200, response.text
    job = response.json()
    assert 'ros-domain:95' in job['resources']
    api.app.state.jobs.children[job['id']].wait(timeout=15)
    done = api.get('/api/jobs/' + job['id']).json()
    assert done['status'] == 'succeeded', api.get('/api/jobs/' + job['id'] + '/log').json()
    assert done['result'] == dict(run_id=job['id'], map_id='circuit', bag='day2/lap', outcome=outcome)
    history = api.get('/api/localization/runs').json()['items']
    assert history[0]['map_correction_source'] == mode.get('map_correction_source', 'cuvslam')
    assert history[0]['vgl_frequency_hz'] == frequency


def test_cancel_waits_for_cleanup_and_releases_resources(api, monkeypatch):
    monkeypatch.setenv('ATD_TEST_WAIT','1')
    body = dict(map_id='circuit', revision=1, bag='day2/lap')
    job = api.post('/api/localization/runs', json=body).json()['id']
    folder = api.app.state.store.result_root / 'runs/localization' / job
    deadline = time.monotonic() + 10
    while not (folder / 'ready').exists():
        assert time.monotonic() < deadline, api.get('/api/jobs/' + job + '/log').json()
        time.sleep(.025)
    assert api.post('/api/localization/runs', json=body).status_code == 400
    assert api.post('/api/jobs/' + job + '/cancel').json()['cancel_requested']
    api.app.state.jobs.children[job].wait(timeout=10)
    assert (folder / 'cleaned').exists()
    assert api.get('/api/jobs/' + job).json()['status'] == 'cancelled'
    api.app.state.jobs.require_available(['ros-domain:95'])

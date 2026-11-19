"""校正の実際の時系列形式から候補、採用、Raceline の速度割り当てまでを通す。"""
import csv
import math
import os
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from atd_api.main import create_app
from atd_api.store import atomic_json


@pytest.fixture
def api(tmp_path, monkeypatch):
    repo = Path(__file__).resolve().parents[4]
    monkeypatch.syspath_prepend(str(repo / 'ros2_ws/src/atd_vehicle_calibration'))
    from atd_vehicle_calibration.analysis import analyze

    app = create_app(tmp_path / 'record', tmp_path / 'results')
    store = app.state.store
    run = store.result_root / 'runs/vehicle_calibration/trial'
    run.mkdir(parents=True)
    # Two repetitions of straight plateaus, acceleration/deceleration, and turns.
    states = [(1100, 0.7, 0., 0.), (1250, 1.2, 0., 0.), (1400, 1.7, 0., 0.),
              (1500, .55, .6, 0.), (1000, 1.65, -.5, 0.),
              (1120, .8, 0., .64), (1120, .8, 0., -.72)]
    pico, motion = [], []
    for episode, (throttle, initial_speed, accel, yaw) in enumerate(states * 3):
        for sample in range(41):
            t = sample * .05
            stamp = 100. + 3 * episode + t
            steering = 992 if yaw == 0 else 700 if yaw < 0 else 1250
            pico.append(dict(stamp=stamp, segment=0, instance_id='test', serial='pico', steering_raw=steering,
                             throttle_raw=throttle, rc_valid=True, manual_valid=True, active_path=1))
            motion.append(dict(stamp=stamp, segment=0, velocity_body=[initial_speed + accel*t, 0., 0.],
                               acceleration_body=[accel, 0., 0.], yaw_rate_rps=yaw))
    rows, analysis = analyze(dict(pico=pico, imu=[]), dict(samples=motion))
    with (run / 'timeseries.csv').open('w', newline='') as out:
        writer = csv.DictWriter(out, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    atomic_json(run / 'summary.json', dict(schema_version=1, source=dict(bag=str(store.record_root / 'day/test'),
                pose_frame='base_link', metadata_sha256='a' * 64, poses_sha256='b' * 64), analysis=analysis,
                timeseries_samples=len(rows), eligible_samples=sum(row['valid'] for row in rows)))
    (run / 'response.png').write_bytes(b'fixture')
    with TestClient(app) as client:
        yield client


ROOT = '/api/calibration/runs/trial/'
MAP = '/api/maps/course'


def test_reviewed_speed_table_is_reusable_and_bound_to_its_source(api, monkeypatch):
    import json
    from atd_api import jetson_bridge, live_input
    from atd_api.speed_tables import TABLES
    from atd_api.store import read_json
    digest = api.get(ROOT + 'candidates').json()['proposal_sha256']
    selection = dict(run_id='trial', proposal_sha256=digest)
    assert api.post('/api/speed/tables', json=dict(selection, proposal_sha256='f'*64)).status_code == 400
    adopted = api.post('/api/speed/tables', json=selection)
    assert adopted.status_code == 200, adopted.text
    assert api.post('/api/speed/tables', json=selection).json() == adopted.json()
    assert api.get('/api/speed/tables').json()['items'] == [adopted.json()['table']]
    sent = []

    def send(connection, path, method, body):
        sent.append((path, method, body))
        return dict(saved=True, proposal_sha256=body['table']['proposal_sha256'], run_id=body['table']['run_id'])

    monkeypatch.setattr(live_input, 'bridge_request', send)
    selected = dict(instance_id='jetson-a', proposal_sha256=digest)
    reply = api.put('/api/live/speed-table', json=selected)
    assert reply.status_code == 200 and reply.json()['proposal_sha256'] == digest
    route, method, body = sent.pop()
    assert (route, method) == ('/live/speed-table', 'PUT') and repr(route) in jetson_bridge.REMOTE
    assert body['instance_id'] == selected['instance_id']
    assert body['table']['points'] == [{k: point[k] for k in ('speed_mps', 'throttle_raw')}
                                       for point in api.get(ROOT + 'feedforward.json',
                                                            params={'proposal_sha256': digest}).json()['points']]
    path = api.app.state.store.result_root / TABLES / (digest + '.json')
    saved = read_json(path)
    saved['table']['points'][0]['throttle_raw'] += 1
    path.write_text(json.dumps(saved))
    assert api.put('/api/live/speed-table', json=selected).status_code == 400
    assert not sent
    listing = api.get('/api/speed/tables').json()
    assert not listing['items'] and listing['issues']


def adopt(api, fields):
    proposal = api.get(ROOT + 'candidates')
    assert proposal.status_code == 200, proposal.text
    data = proposal.json()
    revision = api.get(MAP).json()['revision']
    request = dict(revision=revision, run_id='trial', proposal_sha256=data['proposal_sha256'], fields=fields)
    result = api.post(MAP + '/raceline-calibration', json=request)
    assert result.status_code == 200, result.text
    return data, result.json(), request


def test_review_export_and_select_against_original_report(api):
    api.post('/api/maps', json={'id': 'course'})
    data, doc, request = adopt(api, ['accel', 'decel'])
    assert doc['raceline_calibration']['limits'] == {key: data['limits'][key]['value'] for key in request['fields']}
    assert data['bag'] == 'day/test'
    download = api.get(ROOT + 'feedforward.json', params={'proposal_sha256': data['proposal_sha256']})
    assert download.status_code == 200 and 'attachment' in download.headers['content-disposition']
    table = download.json()
    assert table['schema'] == 'atd.throttle_feedforward.v1' and table['output_unit'] == 'sbus_raw_ch2'
    assert [(point['speed_mps'], point['throttle_raw']) for point in table['points']] == [
        (.7, 1100), (1.2, 1250), (1.7, 1400)]

    path = api.app.state.store.result_root / 'runs/vehicle_calibration/trial/timeseries.csv'
    with path.open('a') as stream:
        stream.write('\n')
    # Even changes that leave the numerical result intact require a fresh review.
    rejected = api.post(MAP + '/raceline-calibration', json=dict(request, revision=doc['revision']))
    assert rejected.status_code == 400 and '再読込' in rejected.json()['detail']
    assert api.get(MAP).json() == doc
    assert api.get(ROOT + 'feedforward.json', params={'proposal_sha256': data['proposal_sha256']}).status_code == 400


def test_real_raceline_uses_reviewed_limits(api, monkeypatch):
    python = os.getenv('ATD_TEST_RACELINE_PYTHON')
    if not python:
        pytest.skip('実 QP helper は ATD_TEST_RACELINE_PYTHON 指定時に確認')
    monkeypatch.setenv('ATD_RACELINE_PYTHON', python)
    document = api.post('/api/maps', json={'id': 'course'}).json()

    def ring(radius):
        return [[radius * math.cos(i * math.tau / 36), radius * math.sin(i * math.tau / 36)] for i in range(36)]

    saved = api.put(MAP, json=dict(document, left=ring(3.), right=ring(5.)))
    assert saved.status_code == 200, saved.text
    data, doc, _ = adopt(api, ['curvature_limit', 'lateral_accel', 'accel', 'decel'])
    expected = {key: candidate['value'] for key, candidate in data['limits'].items()}
    for kind in ['centerline', 'raceline']:
        request = dict(revision=doc['revision'], kind=kind, settings=dict(max_speed=4.))
        start = api.post(MAP + '/generate', json=request)
        assert start.status_code == 200, start.text
        job_id = start.json()['id']
        api.app.state.jobs.children[job_id].wait(timeout=60)
        state = api.get('/api/jobs/' + job_id).json()
        assert state['status'] == 'succeeded', api.get(f'/api/jobs/{job_id}/log').text
        doc = api.get(MAP).json()
    line = doc['lines']['raceline']
    assert all(line['settings'][key] == value for key, value in expected.items())
    for _, _, _, _, curvature, speed, acceleration in line['profile']:
        assert abs(curvature) <= expected['curvature_limit'] * 1.01
        assert speed**2 * abs(curvature) <= expected['lateral_accel'] + 1e-8
        assert -expected['decel'] - 1e-8 <= acceleration <= expected['accel'] + 1e-8

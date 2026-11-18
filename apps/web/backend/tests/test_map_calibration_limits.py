import math
import os
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from atd_api.exports import export_map
from atd_api.main import create_app
from atd_api.store import Store, atomic_json
from atd_processing.geometry import generate


def adoption(**limits):
    return dict(run_id='test-drive', bag='course/drive', proposal_sha256='4' * 64, limits=limits)


def test_map_revision_selection_replacement_and_export(tmp_path):
    store = Store(tmp_path / 'record', tmp_path / 'results')
    store.create_map('course')
    existing = dict(points=[[0, 0]], speeds=[0], settings={'decel': .85})
    store.save_line('course', 1, 'raceline', existing)
    first = adoption(accel=.5)
    saved = store.apply_raceline_calibration('course', 2, first)
    assert saved['revision'] == 3
    assert saved['lines']['raceline'] == existing
    assert saved == store.apply_raceline_calibration('course', 3, first)
    with pytest.raises(ValueError, match='revision'):
        store.apply_raceline_calibration('course', 2, None)
    with pytest.raises(ValueError, match='再生成'):
        export_map(store, 'course')

    # Unrelated GUI saves cannot override server-verified calibration.
    saved = store.save_map('course', dict(revision=3, title='new title', raceline_calibration={}))
    assert saved['raceline_calibration']['limits'] == {'accel': .5}
    for limits in [{'foo': 2}, {'max_speed': 2}, {}, {'decel': 0}, {'accel': float('inf')}]:
        with pytest.raises(ValueError):
            store.apply_raceline_calibration('course', 4, adoption(**limits))
    saved = store.apply_raceline_calibration('course', 4, adoption(curvature_limit=1.2))
    assert saved['raceline_calibration']['limits'] == {'curvature_limit': 1.2}
    assert store.load_map('course') == saved
    cleared = store.apply_raceline_calibration('course', 5, None)
    assert 'raceline_calibration' not in cleared
    assert cleared['lines']['raceline'] == existing
    assert export_map(store, 'course')['revision'] == 6


@pytest.mark.parametrize('method', ['mincurv', 'mincurv_iqp'])
def test_generated_geometry_and_speed_profile_use_selected_limits(tmp_path, method, monkeypatch):
    helper = os.environ.get('ATD_TEST_RACELINE_PYTHON')
    if not helper or not Path(helper).is_file():
        pytest.skip('固定版 Raceline helper の Python が必要です')
    monkeypatch.setenv('ATD_RACELINE_PYTHON', helper)
    app = create_app(tmp_path / 'record', tmp_path / 'results')
    store = app.state.store
    doc = store.create_map('circuit')
    def ellipse(a, b):
        return [[a*math.cos(i*math.tau/60), b*math.sin(i*math.tau/60)] for i in range(60)]
    doc = store.save_map('circuit', dict(doc, left=ellipse(5.5, 2.5), right=ellipse(6.5, 3.5)))
    limits = dict(curvature_limit=1.1, lateral_accel=.5, accel=.15, decel=.2)
    doc = store.apply_raceline_calibration('circuit', doc['revision'], adoption(**limits))
    center = generate(doc, 'centerline', {'spacing': .25, 'accel': .9})
    assert center['settings']['accel'] == .9 and 'raceline_calibration' not in center
    doc = store.save_line('circuit', doc['revision'], 'centerline', center)
    request = dict(spacing=.25, optimizer=method, max_speed=5, curvature_limit=.01, accel=5, decel=5)
    with TestClient(app) as api:
        response = api.post('/api/maps/circuit/generate',
                            json=dict(kind='raceline', revision=doc['revision'], settings=request))
        assert response.status_code == 200, response.text
        job_id = response.json()['id']
        app.state.jobs.children[job_id].wait(timeout=90)
        assert api.get('/api/jobs/' + job_id).json()['status'] == 'succeeded', api.get('/api/jobs/' + job_id + '/log').text
        saved = api.get('/api/maps/circuit').json()
    line = saved['lines']['raceline']
    assert line['method'] == 'tph_' + method
    assert line['raceline_calibration'] == doc['raceline_calibration']
    assert all(line['settings'][k] == v for k, v in limits.items())
    assert line['settings']['max_speed'] == 5
    assert max(abs(row[4]) for row in line['profile']) <= limits['curvature_limit'] * 1.01
    assert max(abs(row[4]) * row[5]**2 for row in line['profile']) <= limits['lateral_accel'] + 1e-7
    acceleration = [row[6] for row in line['profile']]
    assert max(acceleration) == pytest.approx(limits['accel'])
    assert min(acceleration) == pytest.approx(-limits['decel'])
    assert export_map(store, 'circuit')['revision'] == saved['revision']
    atomic_json(store.folder('circuit') / 'lines/raceline.json', dict(points=[], speeds=[]))
    with pytest.raises(ValueError, match='再生成'):
        export_map(store, 'circuit')

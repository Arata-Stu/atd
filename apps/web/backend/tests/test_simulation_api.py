"""空の Notebook 環境からサンプルコースの比較と入力検査を実行する。"""
from fastapi.testclient import TestClient
import pytest

from atd_api.main import create_app


@pytest.fixture
def api(tmp_path):
    with TestClient(create_app(tmp_path / 'record', tmp_path / 'results')) as client:
        yield client


def test_simulation_on_empty_notebook(api):
    response = api.post('/api/simulation/compare', json={'steering_delay_ms': 75})
    assert response.status_code == 200, response.text
    data = response.json()
    assert data['settings']['steering_delay_ms'] == 75
    assert data['settings']['fixed_lookahead_m'] == .65
    assert data['course']['length_m'] > 0
    assert data['model']
    assert [run['mode'] for run in data['runs']] == ['fixed', 'adaptive']
    for run in data['runs']:
        assert run['completed']
        assert run['samples'][-1]['progress_m'] >= data['course']['length_m']
        assert run['metrics']['rms_lateral_error_m'] >= 0


@pytest.mark.parametrize('body', [
    {'fixed_lookahead_m': True},
    {'throttle_delay_ms': None},
    {'steering_tau_s': '0.12'},
    {'min_lookahead_m': 1.6, 'max_lookahead_m': .5},
    {'throttle_delay_ms': 1001},
    {'duration_s': 1e9},
])
def test_simulation_rejects_invalid_settings(api, body):
    response = api.post('/api/simulation/compare', json=body)
    assert response.status_code == 400
    assert response.json()['detail']


@pytest.mark.parametrize('body', ['null', '[]'])
def test_simulation_requires_object(api, body):
    response = api.post('/api/simulation/compare', content=body, headers={'Content-Type': 'application/json'})
    assert response.status_code == 422

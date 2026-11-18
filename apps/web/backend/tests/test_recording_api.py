from types import SimpleNamespace
from unittest.mock import patch

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

from atd_api.jetson_bridge import BridgeUnavailable
from atd_api.live_input import router


@pytest.fixture
def client():
    app = FastAPI()
    app.state.store = SimpleNamespace(connection=lambda: {'host': 'jetson.local', 'user': 'atd'})
    app.include_router(router, prefix='/api')
    return TestClient(app)


def test_status_get_forwards_once_and_keeps_recording_report(client):
    payload = {'connected': True, 'status': {'schema_version': 1, 'stamp': 100., 'age_s': .2, 'stale': False,
               'state': 'recording', 'bag_path': '/bag', 'elapsed_s': 2., 'last_result': None},
               'freshness_s': 2.5, 'availability': {'status': 'fresh'}}
    with patch('atd_api.live_input.bridge_request', return_value=payload) as bridge:
        response = client.get('/api/recording/status')
    assert response.status_code == 200
    assert response.json() == payload
    assert bridge.call_count == 1
    assert bridge.call_args.args[1:] == ('/recording/status', 'GET', None)


def test_unreceived_manager_and_disconnected_bridge_are_distinct(client):
    with patch('atd_api.live_input.bridge_request', return_value={
            'connected': True, 'status': None, 'availability': {'status': 'missing'}}):
        missing = client.get('/api/recording/status').json()
    assert missing['connected'] is True and missing['status'] is None
    with patch('atd_api.live_input.bridge_request', side_effect=BridgeUnavailable('offline')):
        disconnected = client.get('/api/recording/status').json()
    assert disconnected['connected'] is False and disconnected['status'] is None
    assert disconnected['availability'] == {'bridge': 'disconnected', 'status': 'missing'}
    assert not any(key in disconnected for key in ('recording', 'ready', 'last_result'))


def test_stale_recording_state_is_history_not_faked_idle(client):
    payload = {'connected': True, 'status': {'stamp': 100., 'age_s': 8., 'stale': True, 'state': 'recording'},
               'availability': {'status': 'stale'}}
    with patch('atd_api.live_input.bridge_request', return_value=payload):
        value = client.get('/api/recording/status').json()
    assert value['status']['state'] == 'recording'
    assert value['status']['stale'] is True


def test_recording_http_control_is_not_exposed(client):
    with patch('atd_api.live_input.bridge_request') as bridge:
        assert client.post('/api/recording/status', json={}).status_code == 405
        assert client.put('/api/recording/status', json={}).status_code == 405
        for operation in ('start', 'stop', 'restart'):
            assert client.post('/api/recording/' + operation, json={}).status_code == 404
        bridge.assert_not_called()

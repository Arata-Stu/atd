"""プロポ監視・役割設定のNotebook API契約。実USB/SSH/ROSは使わない。"""
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


@pytest.mark.parametrize(('path', 'method', 'body'), [
    ('/propo/status', 'GET', None), ('/propo/profile', 'GET', None),
    ('/propo/profile', 'PUT', {'profile': {}, 'revision': 'r', 'instance_id': 'target'}),
    ('/propo/configure', 'POST', {'enabled': True, 'instance_id': 'target'}),
    ('/propo/configure', 'POST', {'enabled': False, 'instance_id': 'target'}),
    ('/propo/reload', 'POST', {'instance_id': 'target'}),
])
def test_only_monitor_and_role_operations_are_forwarded(client, path, method, body):
    with patch('atd_api.live_input.bridge_request', return_value={'connected': True}) as bridge:
        response = client.request(method, '/api' + path, json=body)
    assert response.status_code == 200
    assert bridge.call_count == 1
    assert bridge.call_args.args[1:] == (path, method, body)


@pytest.mark.parametrize('enabled', [None, 1, 0, 'true', {}, []])
def test_configuration_requires_boolean(client, enabled):
    with patch('atd_api.live_input.bridge_request') as bridge:
        response = client.post('/api/propo/configure', json={'enabled': enabled})
    assert response.status_code == 422
    bridge.assert_not_called()


def test_disconnect_has_missing_board_and_mutation_is_visible_failure(client):
    with patch('atd_api.live_input.bridge_request', side_effect=BridgeUnavailable('disconnected')):
        response = client.get('/api/propo/status')
        mutation = client.post('/api/propo/reload', json={'instance_id': 'target'})
    value = response.json()
    assert response.status_code == 200
    assert value['connected'] is False and value['ready'] is False
    assert value['board'] is None and value['input'] is None
    assert value['rc_valid'] is False
    assert mutation.status_code == 503


def test_api_exposes_no_actuator_or_firmware_output(client):
    with patch('atd_api.live_input.bridge_request') as bridge:
        for path in ('command', 'drive', 'bootsel'):
            response = client.post('/api/propo/' + path, json={'throttle': 1})
            assert response.status_code == 404
    bridge.assert_not_called()

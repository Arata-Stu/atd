"""Notebook API forwarding contract; no SSH, ROS or Bluetooth I/O."""
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
    ('/teleop/status', 'GET', None), ('/teleop/profile', 'GET', None),
    ('/teleop/profile', 'PUT', {'profile': {}, 'revision': 'expected', 'instance_id': 'test_bridge'}),
    ('/teleop/configure', 'POST', {'enabled': True, 'instance_id': 'test_bridge'}),
    ('/teleop/configure', 'POST', {'enabled': False, 'instance_id': 'test_bridge'}),
    ('/teleop/reload', 'POST', {'instance_id': 'test_bridge'}),
])
def test_teleop_routes_only_forward_configuration(client, path, method, body):
    with patch('atd_api.live_input.bridge_request', return_value={'connected': True}) as bridge:
        response = client.request(method, '/api' + path, json=body)
    assert response.status_code == 200
    assert bridge.call_count == 1
    assert bridge.call_args.args[1:] == (path, method, body)


@pytest.mark.parametrize('value', [None, 0, 1, 'true', [], {}])
def test_configure_requires_explicit_boolean(client, value):
    with patch('atd_api.live_input.bridge_request') as bridge:
        response = client.post('/api/teleop/configure', json={'enabled': value})
    assert response.status_code == 422
    bridge.assert_not_called()


def test_disconnected_teleop_is_missing_and_mutation_failure_is_visible(client):
    with patch('atd_api.live_input.bridge_request', side_effect=BridgeUnavailable('disconnected')) as bridge:
        status = client.get('/api/teleop/status')
        mutation = client.post('/api/teleop/reload', json={'instance_id': 'test_bridge'})
    assert status.status_code == 200
    assert status.json()['connected'] is False
    assert status.json()['ready'] is False
    assert status.json()['status'] is None
    assert mutation.status_code == 503
    assert bridge.call_count == 2


def test_api_has_no_drive_command_route(client):
    with patch('atd_api.live_input.bridge_request') as bridge:
        response = client.post('/api/teleop/command', json={'throttle': 1})
    assert response.status_code == 404
    bridge.assert_not_called()

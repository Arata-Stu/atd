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
    ('/vehicle/status', 'GET', None), ('/vehicle/profile', 'GET', None),
    ('/vehicle/profile', 'PUT', {'profile': {}, 'revision': 'r', 'instance_id': 'target'}),
])
def test_layout_routes_forward_once(client, path, method, body):
    with patch('atd_api.live_input.bridge_request', return_value={'connected': True}) as bridge:
        response = client.request(method, '/api' + path, json=body)
    assert response.status_code == 200
    assert bridge.call_args.args[1:] == (path, method, body)
    assert bridge.call_count == 1


def test_disconnected_node_not_fake_applied_and_no_live_tf_endpoint(client):
    with patch('atd_api.live_input.bridge_request', side_effect=BridgeUnavailable('offline')):
        assert client.get('/api/vehicle/status').json()['status'] is None
        assert client.put('/api/vehicle/profile', json={}).status_code == 503
    with patch('atd_api.live_input.bridge_request') as bridge:
        for endpoint in ('apply', 'restart', 'reload', 'drive'):
            assert client.post('/api/vehicle/' + endpoint, json={}).status_code == 404
        bridge.assert_not_called()

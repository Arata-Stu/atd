"""Notebook → Jetson bridge dispatch → queued plan. No serial port is opened."""
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

from atd_api import live_input
from atd_api.jetson_bridge import BridgeUnavailable


@pytest.fixture
def session(tmp_path, monkeypatch):
    pytest.importorskip('yaml', reason='Jetson bridge の依存がある環境で結合検証する')
    src = Path(__file__).resolve().parents[4] / 'ros2_ws/src'
    for package in ('atd_esc_trials', 'atd_joy', 'atd_teleop', 'atd_rc_input',
                    'atd_vehicle_description', 'atd_web_bridge'):
        monkeypatch.syspath_prepend(str(src / package))
    from atd_esc_trials.store import TrialStore
    path = src / 'atd_web_bridge/test/test_teleop_routes.py'
    spec = importlib.util.spec_from_file_location('stub_ros', path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    kind = helper.load_bridge_module().BridgeNode
    node = object.__new__(kind)
    node.settings = {'esc_trials_enabled': True}
    node.identity = {'instance_id': 'this-jetson'}
    node.operation_lock = threading.RLock()
    node.esc_trials = TrialStore(tmp_path)
    app = FastAPI()
    app.state.store = SimpleNamespace(connection=lambda: {})
    app.include_router(live_input.router, prefix='/api')
    def request(_connection, path, method, body):
        try:
            return dict(**node.dispatch(path, method, body or {}), connected=True, identity=node.identity)
        except ValueError as exc:
            raise BridgeUnavailable(str(exc)) from exc
    monkeypatch.setattr(live_input, 'bridge_request', request)
    return TestClient(app), node


def test_remote_edits_only_queue_local_plan(session):
    client, node = session
    payload = {'steps': [{'action': 'forward', 'duration_s': .4, 'level': .1}], 'instance_id': 'this-jetson'}
    assert client.get('/api/esc/status').json()['trial'] is None
    node.settings['esc_trials_enabled'] = False
    assert not client.post('/api/esc/prepare', json=payload).is_success
    node.settings['esc_trials_enabled'] = True
    assert not client.post('/api/esc/prepare', json=dict(payload, instance_id='former')).is_success
    assert not client.post('/api/esc/prepare', json=dict(payload, steps=[dict(payload['steps'][0],action='reverse')])).is_success
    trial = client.post('/api/esc/prepare', json=payload).json()['trial']
    assert trial['status'] == 'queued' and trial['pwm_us'] is None
    assert not client.post('/api/esc/prepare', json=payload).is_success
    for route in ('start','claim','command','confirm','throttle'):
        assert client.post('/api/esc/' + route, json={'id': trial['id']}).status_code == 404
        with pytest.raises(ValueError, match='Unsupported'):
            node.dispatch('/esc/' + route, 'POST', {})
    assert client.post('/api/esc/cancel', json={'id':trial['id']}).json()['trial']['status'] == 'cancelled'
    assert client.get('/api/esc/status').json()['trial']['state_history'][-1]['status'] == 'cancelled'

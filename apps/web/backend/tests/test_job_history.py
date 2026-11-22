import fcntl
import time

import pytest
from fastapi.testclient import TestClient

from atd_api.main import create_app
from atd_api.store import atomic_json


@pytest.fixture
def client(tmp_path):
    with TestClient(create_app(tmp_path / 'record', tmp_path / 'results')) as client:
        yield client


def add_job(jobs, key, status, started, resources=()):
    atomic_json(jobs.root / (key + '.json'),
                dict(id=key, title=key, status=status, started=started, resources=list(resources)))


def test_history_snapshot_includes_old_jobs_and_preserves_outputs(client):
    jobs = client.app.state.jobs
    for i in range(202):
        add_job(jobs, f'job{i}', 'succeeded', i)
    add_job(jobs, 'active', 'running', time.time())
    add_job(jobs, 'process', 'orphaned', 1, ['map:example'])
    snapshot = client.get('/api/jobs/history').json()
    assert len(client.get('/api/jobs').json()['items']) == 200
    assert len(snapshot['items']) == 202
    assert snapshot['protected'] == 2
    with pytest.raises(ValueError, match='使用中'):
        jobs.require_available(['map:example'])

    add_job(jobs, 'completed-later', 'succeeded', time.time())
    for suffix in ['.log', '.cancel', '.lease', '.request.json']:
        (jobs.root / ('job1' + suffix)).touch()
    artifact = jobs.store.result_root / 'runs' / 'calibration' / 'job1' / 'report.json'
    atomic_json(artifact, {'ok': True})

    ids = [e['id'] for e in snapshot['items']]
    assert client.post('/api/jobs/history/delete', json={'ids': ids}).json() == {'deleted': 202}
    assert client.post('/api/jobs/history/delete', json={'ids': ids}).json() == {'deleted': 0}
    assert not any(jobs.root.glob('job1.*'))
    assert artifact.exists()
    assert {s['id'] for s in jobs.list()} == {'active', 'process', 'completed-later'}


def test_reject_mixed_active_or_invalid_selection_before_unlink(client):
    jobs = client.app.state.jobs
    add_job(jobs, 'done', 'failed', 10)
    add_job(jobs, 'busy', 'starting', time.time())
    for key in ['busy', '../escape']:
        response = client.post('/api/jobs/history/delete', json={'ids': ['done', key]})
        assert response.status_code == 400
        assert (jobs.root / 'done.json').exists()
    with (jobs.root / 'done.lease').open('a') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX)
        assert client.post('/api/jobs/history/delete', json={'ids': ['done']}).status_code == 400
    assert client.post('/api/jobs/history/delete', json={'ids': ['done', 'done']}).json() == {'deleted': 1}

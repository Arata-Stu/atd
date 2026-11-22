import time

from fastapi.testclient import TestClient

from atd_api.main import create_app
from atd_api.store import atomic_json


def test_delete_calibration_and_benchmark_with_active_reference(tmp_path):
    app = create_app(tmp_path/'bags', tmp_path/'results')
    store, jobs = app.state.store, app.state.jobs
    root = store.result_root / 'runs'
    folder = root / 'vehicle_calibration/calib'
    atomic_json(folder/'summary.json', dict(schema_version=1, source=dict(bag='lap', metadata_sha256='abc'),
                                            analysis={}, eligible_samples=1, timeseries_samples=1))
    (folder/'response.png').write_bytes(b'plot')
    (folder/'timeseries.csv').write_text('measurements')
    stored_table = root/'vehicle/feedforward/adopted.json'
    atomic_json(stored_table, {})
    benchmark = root/'simulation_benchmarks/test'
    atomic_json(benchmark/'report.json', dict(schema='atd.simulation_benchmark.v1', id='test'))
    atomic_json(jobs.root/'reader.request.json', dict(payload=dict(reference_run='calib')))
    state = dict(id='reader', action='calibration', status='running', started=time.time(), resources=[])
    atomic_json(jobs.root/'reader.json', state)
    with TestClient(app) as api:
        route = '/api/analysis-results/'
        assert api.delete(route+'benchmark/missing').status_code == 404
        assert api.delete(route+'benchmark/test').is_success and not benchmark.exists()
        assert api.delete(route+'calibration/calib').status_code == 409 and folder.exists()
        atomic_json(jobs.root/'reader.json', dict(state, status='succeeded'))
        assert api.delete(route+'calibration/calib').is_success and not folder.exists()
        assert api.get('/api/calibration/options').json()['runs'] == []
        assert stored_table.is_file() and (jobs.root/'reader.json').is_file()

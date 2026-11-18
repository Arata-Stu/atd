"""Notebook の校正ジョブ。ROS/GPU 実行だけを置き換え、保存境界と再利用を検査する。"""
from hashlib import sha256
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from atd_api import calibration
from atd_api.jobs import Cancelled, Jobs, Task
from atd_api.store import Store, atomic_json
from atd_api.worker import execute


def write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)


@pytest.fixture
def store(tmp_path):
    store = Store(tmp_path / 'bags', tmp_path / 'results')
    for name in ['a/run', 'b/run']:
        write(store.record_root / name / 'metadata.yaml', name.encode())
    return store


def make_map(store, key, bag, digest=None):
    folder = store.maps_root / key
    metadata = (store.record_root / bag / 'metadata.yaml').read_bytes()
    atomic_json(folder / 'mapping.json', dict(source=dict(kind='nvidia_offline', bag=bag,
                metadata_sha256=digest or sha256(metadata).hexdigest(), inputs=dict(base_frame='base_link'))))
    write(folder / calibration.POSE_FILE, b'0 0 0 0 0 0 0 1\n')


def request(store, **args):
    return calibration.payload(store, dict(bag='a/run', pose_frame='camera_link') | args)


def task_with_result(eligible=3):
    task = Mock(key='example', state={})

    def run(argv, **kwargs):
        if '--output' not in argv:
            return 0
        output = Path(argv[argv.index('--output') + 1])
        bag = Path(argv[argv.index('--bag') + 1])
        write(output / 'response.png', b'png')
        write(output / 'timeseries.csv', b'time,velocity')
        atomic_json(output / 'summary.json', dict(schema_version=1, analysis={}, source=dict(
            bag=str(bag), poses=argv[argv.index('--poses') + 1], pose_frame=argv[argv.index('--pose-frame') + 1],
            metadata_sha256=sha256((bag / 'metadata.yaml').read_bytes()).hexdigest()),
            eligible_samples=eligible, timeseries_samples=4))
        return 0 if eligible else 2
    task.run.side_effect = run
    return task


def test_discover_and_revalidate_generation_source(store):
    make_map(store, 'valid', 'a/run')
    make_map(store, 'other', 'b/run', sha256(b'a/run').hexdigest())
    make_map(store, 'outdated', 'a/run', sha256(b'old').hexdigest())
    make_map(store, 'empty', 'a/run')
    write(store.maps_root / 'empty' / calibration.POSE_FILE, b'')
    make_map(store, 'symlink', 'a/run')
    linked = store.maps_root / 'symlink' / calibration.POSE_FILE
    linked.unlink()
    linked.symlink_to(store.maps_root / 'valid' / calibration.POSE_FILE)
    candidates = calibration.options(store, 'a/run')['poses']
    assert [pose['map_id'] for pose in candidates] == ['valid']
    assert candidates[0]['base_frame'] == 'base_link'
    with pytest.raises(ValueError):
        request(store, map_id='other')
    payload = request(store)
    assert calibration.resources(payload) == ['record:a/run', 'map:valid']
    make_map(store, 'another-valid', 'a/run')
    with pytest.raises(ValueError, match='複数'):
        request(store)
    assert request(store, map_id='another-valid')['map_id'] == 'another-valid'
    write(store.record_root / 'a/run/metadata.yaml', b'changed')
    task = task_with_result()
    with pytest.raises(ValueError, match='選び直し'):
        execute(store, task, 'calibration', payload)
    task.run.assert_not_called()


@pytest.mark.parametrize('eligible, outcome', [(3, 'ok'), (0, 'no_eligible_samples')])
def test_generate_reuse_and_report(store, monkeypatch, eligible, outcome):
    task = task_with_result(eligible)
    def build(store, task, inputs):
        make_map(store, inputs['id'], inputs['bag'])
    build = Mock(side_effect=build)
    monkeypatch.setattr(calibration.offline, 'map_build', build)
    job = request(store, wheelbase_m=0.257)
    assert job['generate'] and 'offline-mapping' in calibration.resources(job)
    result = execute(store, task, 'calibration', job)
    assert result['outcome'] == outcome and result['pose_reused'] is False
    assert '--require-stereo' in task.run.call_args_list[1].args[0]
    assert task.run.call_args.args[0][-2:] == ['--wheelbase-m', '0.257']
    assert task.run.call_args.kwargs['allowed_exit_codes'] == (0, 2)
    assert calibration.options(store, 'a/run')['runs'][0]['bag'] == 'a/run'
    assert calibration.artifact(store, 'example', 'response.png').read_bytes() == b'png'
    task = task_with_result()
    task.key = 'second'
    reused = execute(store, task, 'calibration', request(store))
    assert reused['pose_reused'] and build.call_count == 1
    assert '--require-stereo' not in task.run.call_args_list[1].args[0]
    with pytest.raises(ValueError, match='別の走行'):
        request(store, reference_run='example')
    if eligible:
        another = request(store, bag='b/run', reference_run='example')
        task.key = 'validation'
        execute(store, task, 'calibration', another)
        assert task.run.call_args.args[0][-1] == str(calibration.artifact(store, 'example', 'summary.json'))


def test_invalid_or_cancelled_jobs_do_not_publish(store, monkeypatch):
    job = request(store)
    task = task_with_result()
    build = Mock()
    monkeypatch.setattr(calibration.offline, 'map_build', build)
    task.run.side_effect = ValueError('package missing')
    with pytest.raises(ValueError, match='scripts/build.sh'):
        execute(store, task, 'calibration', job)
    task.run.side_effect = [0, 3]
    with pytest.raises(ValueError, match='bag の事前検査'):
        execute(store, task, 'calibration', job)
    build.assert_not_called()
    make_map(store, 'valid', 'a/run')
    job = request(store)
    task = task_with_result()
    task.check.side_effect = [None, None, None, Cancelled()]
    with pytest.raises(Cancelled):
        execute(store, task, 'calibration', job)
    assert calibration.options(store)['runs'] == []
    assert not (store.result_root / calibration.RUNS / task.key).exists()
    task = task_with_result()
    task.run.side_effect = [0, 0, 2]
    with pytest.raises(ValueError):
        execute(store, task, 'calibration', job)
    assert calibration.options(store)['runs'] == []


def test_artifact_whitelist_and_path_boundaries(store):
    make_map(store, 'valid', 'a/run')
    execute(store, task_with_result(), 'calibration', request(store))
    for run, name in [('example', '../summary.json'), ('../../maps', 'summary.json'), ('example', 'metadata.yaml')]:
        with pytest.raises(ValueError):
            calibration.artifact(store, run, name)
    for args in [dict(bag='../bags/a/run'), dict(map_id='/tmp'), dict(pose_frame=''), dict(wheelbase_m=float('nan'))]:
        with pytest.raises(ValueError):
            request(store, **args)
    output = store.result_root / calibration.RUNS / 'example'
    (output / 'response.png').unlink()
    (output / 'response.png').symlink_to(output / 'timeseries.csv')
    with pytest.raises(ValueError):
        calibration.artifact(store, 'example', 'response.png')
    assert calibration.options(store)['runs'] == []


def test_ros_child_uses_overlay_environment_without_web_venv(store):
    jobs = Jobs(store)
    atomic_json(jobs.root / 'job.json', dict(id='job', status='starting'))
    task = Task(jobs, 'job')
    child = Mock(pid=123, returncode=2)
    child.poll.return_value = 2
    env = dict(PATH='/web/bin/:/usr/bin', VIRTUAL_ENV='/web', PYTHONPATH='/web/src', ROS_DOMAIN_ID='93')
    with patch('atd_api.jobs.identity', return_value='p'), patch('atd_api.jobs.subprocess.Popen', return_value=child) as spawn:
        assert task.run(['ros2', 'run', 'atd_vehicle_calibration', 'analyze'], env=env, allowed_exit_codes=(0, 2)) == 2
        child_env = spawn.call_args.kwargs['env']
        assert child_env == dict(PATH='/usr/bin', ROS_DOMAIN_ID='93')
        with pytest.raises(ValueError, match='終了コード 2'):
            task.run(['ros2', '--help'], env=env)
    assert env['VIRTUAL_ENV'] == '/web'

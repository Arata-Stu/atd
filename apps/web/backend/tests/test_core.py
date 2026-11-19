import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import pytest
from fastapi.testclient import TestClient

from atd_api.main import create_app
from atd_api.store import Store, atomic_json, confined
from atd_api.jobs import Jobs, Task, Cancelled
from atd_api.transfers import REMOTE, connection, inventory, relative
from atd_processing.geometry import generate


def ring(radius, count=24):
    return [[radius * math.cos(i * 2 * math.pi / count), radius * math.sin(i * 2 * math.pi / count)] for i in range(count)]


@pytest.fixture
def client(tmp_path):
    app = create_app(tmp_path / 'record', tmp_path / 'results')
    with TestClient(app) as client:
        yield client


def test_geometry_clearance_speed_and_rejection():
    doc = dict(left=ring(3), right=ring(1), custom=ring(2), closed=True)
    for kind in ('centerline', 'custom'):
        output = generate(doc, kind, {'spacing': 0.2})
        assert output['length'] > 10
        assert len(output['points']) == len(output['speeds'])
        assert max(output['speeds']) <= 1.5
        for i, a in enumerate(output['points']):
            j = (i + 1) % len(output['points'])
            distance = math.dist(a, output['points'][j])
            assert output['speeds'][j] ** 2 <= output['speeds'][i] ** 2 + 2 * 1.5 * distance + 1e-7
    with pytest.raises(ValueError, match='領域外|余裕|境界'):
        generate(dict(doc, custom=ring(4)), 'custom', {})
    with pytest.raises(ValueError, match='自己交差'):
        generate(dict(doc, left=[[0, 0], [3, 3], [0, 3], [3, 0]]), 'centerline', {})


def test_open_path_endpoints_and_acceleration():
    out = generate(dict(left=[[0, 1], [10, 1]], right=[[0, -1], [10, -1]], custom=[[0, 0], [10, 0]], closed=False), 'custom', {})
    assert out['points'][0] == [0, 0]
    assert out['points'][-1] == [10, 0]
    assert out['speeds'][0] == out['speeds'][-1] == 0


def test_api_map_revision_job_persistence(client):
    created = client.post('/api/maps', json={'id': 'circle'}).json()
    assert created['revision'] == 1
    document = dict(created, left=ring(3), right=ring(1), custom=ring(2))
    saved = client.put('/api/maps/circle', json=document).json()
    assert saved['revision'] == 2
    assert client.put('/api/maps/circle', json=document).status_code == 400
    result = client.post('/api/maps/circle/generate', json={'kind': 'centerline', 'revision': 2, 'settings': {}})
    assert result.status_code == 200, result.text
    key = result.json()['id']
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        state = client.get('/api/jobs/' + key).json()
        if state['status'] not in ('starting', 'running'):
            break
        time.sleep(0.1)
    assert state['status'] == 'succeeded', client.get(f'/api/jobs/{key}/log').text
    document = client.get('/api/maps/circle').json()
    assert document['revision'] == 3
    assert 'centerline' in document['lines']
    reopened = Jobs(client.app.state.store)
    assert reopened.get(key)['status'] == 'succeeded'
    saved = client.put('/api/maps/circle', json=dict(document, left=ring(3.1))).json()
    assert saved['lines'] == {}


def test_regenerated_centerline_invalidates_dependent_raceline(client):
    store = client.app.state.store
    document = store.create_map('line-dependency')
    document = store.save_line(document['id'], document['revision'], 'centerline', {'points': []})
    document = store.save_line(document['id'], document['revision'], 'raceline', {'points': []})
    document = store.save_line(document['id'], document['revision'], 'custom', {'points': []})
    document = store.save_line(document['id'], document['revision'], 'centerline', {'points': [[1, 2]]})
    assert 'raceline' not in document['lines']
    assert document['lines']['centerline']['points'] == [[1, 2]]
    assert 'custom' in document['lines']


def test_api_disconnect_security_and_cloud(client):
    assert client.get('/api/live/status').json()['connected'] is False
    assert client.get('/api/health', headers={'Host': 'evil.example'}).status_code == 403
    assert client.post('/api/maps', json={'id': 'unsafe'}, headers={'Origin': 'https://evil.example'}).status_code == 403
    assert client.post('/api/maps', json={'id': '../unsafe'}).status_code == 400
    assert client.post('/api/maps', json={'id': 'safe', 'shell': 'echo nope'}).status_code == 422
    doc = client.post('/api/maps', json={'id': 'cloud'}).json()
    assert client.post('/api/maps/cloud/cloud', json={'revision': 1, 'frame': 'odom', 'points': [[1, 2, 3]]}).status_code == 400
    assert client.post('/api/maps/cloud/cloud', json={'revision': 1, 'frame': 'map', 'points': [[1, 2, 3]]}).status_code == 200
    assert client.get('/api/maps/cloud/cloud').json()['source_count'] == 1


def test_transfer_defaults_and_saved_settings(client):
    defaults = client.get('/api/connection').json()
    assert defaults == dict(host='', user='atd', port=22, bridge_port=8790,
                            record_root='/home/atd/workspaces/atd/data/record/rosbags',
                            map_root='/home/atd/workspaces/atd/data/results/maps')
    custom = dict(host='jetson.local', user='driver', port=2222, bridge_port=8791,
                  record_root='/mnt/usb/records', map_root='/mnt/usb/maps')
    assert client.put('/api/connection', json=custom).json() == custom
    assert client.get('/api/connection').json() == custom


def test_transfer_uses_the_connection_reviewed_in_the_ui(client, monkeypatch):
    usb = dict(client.get('/api/connection').json(), host='192.168.55.1')
    hotspot = dict(usb, host='10.42.0.1')
    assert client.put('/api/connection', json=usb).status_code == 200
    assert client.put('/api/connection', json=hotspot).status_code == 200
    monkeypatch.setattr('atd_api.transfers.request', lambda config, *_: {'host': config['host']})
    monkeypatch.setattr(client.app.state.jobs, 'start', lambda _, payload, *args: payload)
    client.app.state.store.create_map('track')
    for route, data in [('browse', {'relative': ''}), ('pull', {'relative': 'run'}), ('push', {'id': 'track'})]:
        url = '/api/transfer/' + route
        assert client.post(url, json=dict(data, expected_connection=usb)).status_code == 409
        for body in (dict(data, expected_connection=hotspot), data):
            response = client.post(url, json=body)
            assert response.status_code == 200
            if route == 'browse':
                assert response.json()['host'] == hotspot['host']
            else:
                assert response.json()['connection'] == hotspot


def test_transfer_paths_and_hash_publish(tmp_path):
    for name in ('../out', '/out', 'foo;pwd', 'foo$HOME', '-oProxyCommand=anything'):
        with pytest.raises(ValueError):
            relative(name)
    with pytest.raises(ValueError):
        connection(dict(host='x;whoami', user='admin', port=22, record_root='/bags', map_root='/maps'))
    root = tmp_path / 'remote'
    stage = root / '.atd-upload-test'
    source = stage / 'bundle'
    source.mkdir(parents=True)
    (source / 'file').write_bytes(b'data')
    expected = inventory(source)
    request = dict(action='publish', root=str(root), relative=stage.name, destination='map', files=expected)
    wrong = dict(expected)
    wrong['file'] = dict(expected['file'], sha256='0' * 64)
    failed = subprocess.run([sys.executable, '-c', REMOTE], input=json.dumps(dict(request, files=wrong)), text=True, capture_output=True)
    assert failed.returncode != 0
    assert source.exists()
    complete = subprocess.run([sys.executable, '-c', REMOTE], input=json.dumps(request), text=True, capture_output=True)
    assert complete.returncode == 0, complete.stdout
    assert (root / 'map/file').read_bytes() == b'data'
    assert not stage.exists()


def test_symlink_escape_and_tamper(tmp_path):
    root = tmp_path / 'root'
    root.mkdir()
    (root / 'bad').symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError):
        confined(root, 'bad/file', False)
    with pytest.raises(ValueError):
        inventory(root)


def test_cancel_stops_external_child(tmp_path):
    import threading
    store = Store(tmp_path / 'bags', tmp_path / 'results')
    jobs = Jobs(store)
    key = 'c' * 32
    atomic_json(jobs.root / (key + '.json'), dict(id=key, status='running', title='test', resources=[], started=time.time()))
    task = Task(jobs, key)
    timer = threading.Timer(0.25, lambda: atomic_json(jobs.root / (key + '.cancel'), {}))
    timer.start()
    try:
        with pytest.raises(Cancelled):
            task.run([sys.executable, '-c', 'import time; time.sleep(20)'])
        assert task.state['process_pid'] is None
    finally:
        timer.join()


def test_export_same_revision_and_missing_ros_is_error(client, monkeypatch):
    doc = client.post('/api/maps', json={'id': 'export'}).json()
    result = client.post('/api/maps/export/export', json={})
    assert result.status_code == 200
    location = client.app.state.store.result_root / result.json()['relative']
    assert json.loads((location / 'manifest.json').read_text())['revision'] == doc['revision']
    assert json.loads((location / 'hdmap.yaml').read_text())['id'] == 'export'
    assert client.post('/api/maps/export/export', json={}).json() == result.json()
    monkeypatch.setattr('atd_api.offline.shutil.which', lambda _name: None)
    bag = client.app.state.store.record_root / 'input'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}')
    response = client.post('/api/maps/build', json={'id': 'mapping', 'bag': 'input'})
    assert response.status_code == 400
    assert not (client.app.state.store.maps_root / 'mapping').exists()


@pytest.mark.parametrize('odom', [b'0.0 0 0 0 0 0 0 1\n', b'', None], ids=['trajectory', 'empty', 'missing'])
def test_official_mapping_staging_and_artifact_check(tmp_path, monkeypatch, odom):
    from atd_api.offline import map_build
    store = Store(tmp_path / 'bags', tmp_path / 'results')
    bag = store.record_root / 'bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text('record')
    monkeypatch.setattr('atd_api.offline.shutil.which', lambda _name: '/opt/ros/lyrical/bin/ros2')

    class Work:
        key = 'a' * 32
        def check(self): pass
        def log(self, text): pass
        def run(self, argv, env=None):
            assert argv[:4] == ['ros2', 'run', 'isaac_mapping_ros', 'create_map_offline.py']
            assert argv[argv.index('--override') + 1] == 'cuvslam.cfg_planar=false'
            assert '--base_link_name=camera_link' in argv
            assert '--use_cusfm=False' in argv and '--use_cuvslam_opencv_compat=True' in argv
            assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
            output = Path(next(a.split('=', 1)[1] for a in argv if a.startswith('--base_output_folder=')))
            run = output / 'run1'
            (run / 'cuvslam_map').mkdir(parents=True)
            (run / 'cuvslam_map/map.mdb').write_bytes(b'not-empty')
            (run / 'map_frames/rectified').mkdir(parents=True)
            (run / 'map_frames/rectified/frames_meta.json').write_text('{}')
            (run / 'poses/runs').mkdir(parents=True)
            (run / 'poses/runs/odom_poses.tum').write_bytes(b'other')
            if odom is not None:
                (run / 'poses/odom_poses.tum').write_bytes(odom)
            (output / 'latest').symlink_to('run1', target_is_directory=True)

    result = map_build(store, Work(), {'id': 'built', 'bag': 'bag', 'base_frame': 'camera_link'})
    assert result['cuvslam_ready'] is True and result['pointcloud_ready'] is False
    assert (store.maps_root / 'built/nvidia/cuvslam_map/map.mdb').read_bytes() == b'not-empty'
    assert store.load_map('built')['source']['pointcloud_status'] == 'pending'
    assert store.load_map('built')['source']['mapping_options']['ground_constraint'] is False
    source = json.loads((store.maps_root / 'built/mapping.json').read_text())['source']
    if odom:
        assert source['odom_poses'] == dict(path='nvidia/poses/odom_poses.tum', frame='camera_link',
                                            sha256=hashlib.sha256(odom).hexdigest())
    else:
        assert 'odom_poses' not in source


def test_hash_rpc_cancellation_terminates_ssh_process(tmp_path, monkeypatch):
    import threading
    import atd_api.transfers as transfers
    store = Store(tmp_path / 'bags', tmp_path / 'results')
    jobs = Jobs(store)
    key = 'e' * 32
    atomic_json(jobs.root / (key + '.json'), dict(id=key, status='running', title='hash', resources=[], started=time.time()))
    task = Task(jobs, key)
    original = subprocess.Popen
    processes = []
    def slow_ssh(_argv, **kwargs):
        process = original([sys.executable, '-c', 'import time; time.sleep(20)'], **kwargs)
        processes.append(process)
        return process
    monkeypatch.setattr(transfers.subprocess, 'Popen', slow_ssh)
    timer = threading.Timer(0.2, lambda: atomic_json(jobs.root / (key + '.cancel'), {}))
    timer.start()
    try:
        with pytest.raises(Cancelled):
            transfers.request(dict(host='jetson', user='admin', port=22, record_root='/bags', map_root='/maps'),
                              'inventory', '/bags', 'bag', task=task)
        assert all(p.poll() is not None for p in processes)
    finally:
        timer.join()


@pytest.mark.parametrize('completed', ['succeeded', 'cancelled'])
def test_finished_state_survives_lease_release_race(tmp_path, monkeypatch, completed):
    import atd_api.jobs as module
    store = Store(tmp_path / 'bags', tmp_path / 'results')
    jobs = Jobs(store)
    key = 'f' * 32
    file = jobs.root / (key + '.json')
    active = dict(id=key, status='running', resources=[], started=time.time() - 30, process_pid=None)
    final = dict(active, status=completed, finished=time.time())
    atomic_json(file, active)
    original_read = module.read_json
    calls = 0
    def finish_after_first_read(path):
        nonlocal calls
        calls += 1
        result = original_read(path)
        if calls == 1:
            # worker が最初の read 後に完了を保存し、lease を解放した瞬間を再現。
            atomic_json(file, final)
        return result
    monkeypatch.setattr(module, 'read_json', finish_after_first_read)
    assert jobs.get(key)['status'] == completed
    assert original_read(file)['status'] == completed
    assert calls == 2


def test_spaces_unicode_scp_pull_and_reserve_response_loss(tmp_path, monkeypatch):
    import shutil
    import atd_api.transfers as module
    store = Store(tmp_path / 'PC 記録', tmp_path / 'PC 結果')
    remote_records = tmp_path / 'Jetson 記録'
    remote_maps = tmp_path / 'Jetson 地図'
    remote_maps.mkdir()
    bag_relative = '2026 11/走行 テスト1'
    remote_bag = remote_records / bag_relative
    remote_bag.mkdir(parents=True)
    (remote_bag / 'metadata.yaml').write_text('record')
    (remote_bag / 'sensor data.mcap').write_bytes(b'bag payload')
    config = dict(host='jetson', user='admin', port=22, record_root=str(remote_records), map_root=str(remote_maps))
    assert module.connection(config)['record_root'] == str(remote_records)
    assert module.relative(bag_relative) == bag_relative
    calls = []
    def local_rpc(_raw, action, root, relative_path='', task=None, **extra):
        calls.append((action, relative_path))
        request = dict(action=action, root=root, relative=relative_path, **extra)
        done = subprocess.run([sys.executable, '-c', module.REMOTE], input=json.dumps(request), text=True, capture_output=True)
        assert done.returncode == 0, done.stdout
        if action == 'reserve':
            raise ValueError('mkdir 成功後に SSH 応答を失いました')
        return json.loads(done.stdout)
    monkeypatch.setattr(module, 'request', local_rpc)
    class Work:
        def check(self): pass
        def log(self, text): pass
        def run(self, argv):
            assert argv[0] == 'scp' and '-O' not in argv
            assert argv[-2] == 'admin@jetson:' + str(remote_records / bag_relative)
            shutil.copytree(Path(argv[-2].split(':', 1)[1]), Path(argv[-1]))
    result = module.pull(store, Work(), config, bag_relative)
    assert result['verified'] is True
    assert (store.record_root / bag_relative / 'sensor data.mcap').read_bytes() == b'bag payload'
    store.create_map('map')
    with pytest.raises(ValueError, match='応答を失い'):
        module.push(store, Work(), config, 'map')
    assert calls[-1][0] == 'cleanup'
    assert not list(remote_maps.iterdir())


@pytest.mark.parametrize('value', ['../out', '階層/../out', 'foo\x00bar', 'foo\nbar', '-option', 'foo*', 'foo?', 'foo[0]', 'foo$HOME', 'foo;pwd', 'foo`pwd`', 'foo|pwd', 'foo\\bar'])
def test_scp_unicode_validation_still_rejects_unsafe_paths(value):
    from atd_api.transfers import relative
    with pytest.raises(ValueError):
        relative(value)


@pytest.mark.parametrize('crash_after', [1, 2])
def test_vgl_partial_publish_hard_exit_is_recoverable(tmp_path, crash_after):
    from atd_processing.vgl_transaction import check_preparable, recover, publish, JOURNAL
    folder = tmp_path / 'map'
    (folder / 'nvidia').mkdir(parents=True)
    stage = folder / 'stage'
    config, output = stage / 'config', stage / 'output'
    config.mkdir(parents=True)
    output.mkdir()
    (config / 'settings').write_text('configuration')
    (output / 'database').write_bytes(b'vgl data')
    script = '''
import os,sys
from pathlib import Path
from atd_processing.vgl_transaction import publish
original=os.rename
calls=0
def crash(source,destination):
    global calls
    original(source,destination)
    calls+=1
    if calls==int(sys.argv[2]): os._exit(44)
os.rename=crash
folder=Path(sys.argv[1])
publish(folder,folder/'stage/config',folder/'stage/output',{'runtime_model_dir':'models/test'})
'''
    done = subprocess.run([sys.executable, '-c', script, str(folder), str(crash_after)])
    assert done.returncode == 44
    assert (folder / JOURNAL).exists()
    assert not (folder / 'vgl.json').exists()
    check_preparable(folder)  # API は未完了の自ジョブ journal を読み取って再試行を許可。
    recover(folder)
    assert not (folder / 'vgl_runtime_config').exists()
    assert not (folder / 'nvidia/cuvgl_map').exists()
    assert not (folder / JOURNAL).exists()
    for directory in (config, output):
        directory.mkdir(exist_ok=True)
        (directory / 'retry').write_text('new data')
    publish(folder, config, output, {'runtime_model_dir': 'models/retry'})
    assert json.loads((folder / 'vgl.json').read_text())['status'] == 'complete'
    recover(folder)  # 完了済み成果物は保持。
    assert (folder / 'nvidia/cuvgl_map/retry').is_file()


def test_vgl_marker_commit_survives_exit_before_journal_cleanup(tmp_path):
    from atd_processing.vgl_transaction import recover, JOURNAL
    folder = tmp_path / 'map'
    (folder / 'nvidia').mkdir(parents=True)
    for name in ('config', 'output'):
        (folder / name).mkdir()
        (folder / name / 'data').write_text(name)
    script = '''
import os,sys
from pathlib import Path
from atd_processing import vgl_transaction as m
original=Path.unlink
def crash(path,*args,**kwargs):
    if path.name==m.JOURNAL: os._exit(45)
    return original(path,*args,**kwargs)
Path.unlink=crash
folder=Path(sys.argv[1])
m.publish(folder,folder/'config',folder/'output',{})
'''
    done = subprocess.run([sys.executable, '-c', script, str(folder)])
    assert done.returncode == 45
    assert (folder / JOURNAL).exists()
    recover(folder)
    assert not (folder / JOURNAL).exists()
    assert (folder / 'nvidia/cuvgl_map/data').read_text() == 'output'
    assert (folder / 'vgl_runtime_config/data').read_text() == 'config'
    assert json.loads((folder / 'vgl.json').read_text())['status'] == 'complete'

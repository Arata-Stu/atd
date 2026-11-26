"""保存地図点群取得の準備検査・作業コピー・公開境界。ROS/GPU は fake のみ。"""
import json
import os
from pathlib import Path
import shutil
from unittest.mock import Mock

import pytest
from fastapi.testclient import TestClient

from atd_api import capture
from atd_api.main import create_app
from atd_api.offline import DEFAULT_INPUTS
from atd_api.store import Store, atomic_json, read_json
from atd_processing.vgl_transaction import fingerprint
from tools.vgl.bundle import make_manifest, validate_bundle


def write(path, value=b'fixture'):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(value)
    return path


def write_export(results, asset_id, export_id, width=424, height=240):
    path = results / 'models/vgl' / asset_id / 'exported' / export_id
    for name in ('models/aliked_lightglue/aliked.onnx', 'models/aliked_lightglue/lightglue.onnx',
                 'configs/isaac/keypoint_creation_config.pb.txt', 'configs/isaac/matching_task_worker_config.pb.txt'):
        write(path / name)
    make_manifest(path, asset_id, export_id, dict(family='aliked_lightglue', mode='package', width=width,
                  height=height, top_k=2048), {'fixture': True})
    return path


def test_vgl_export_catalog_reads_the_configured_results_before_map_creation(tmp_path, monkeypatch):
    from tools.vgl import bundle

    results = tmp_path / 'notebook-results'
    monkeypatch.setenv('ATD_WEB_RESULT_ROOT', str(results))
    client = TestClient(create_app(record_root=tmp_path / 'records'))
    endpoint = '/api/vgl/exports'
    assert client.get(endpoint).json() == dict(items=[], result_root=str(results))

    export = write_export(results, 'aliked_lightglue', 'd455_640x480', 640, 480)
    shutil.copytree(export, export.parent / '.partial-new-export')
    shutil.copytree(export, export.parent / 'wrong-manifest-id')
    write(export.parent / 'unfinished/models/aliked_lightglue/aliked.onnx')
    external = write_export(tmp_path / 'external', 'elsewhere', 'bundle')
    (results / 'models/vgl/elsewhere').symlink_to(external.parents[1], target_is_directory=True)
    (export.parent / 'external-export').symlink_to(external, target_is_directory=True)

    digest = Mock(wraps=bundle.sha256)
    monkeypatch.setattr(bundle, 'sha256', digest)
    expected = dict(asset_id='aliked_lightglue', export_id='d455_640x480', width=640, height=480,
                    model_dir='models/vgl/aliked_lightglue/exported/d455_640x480')
    assert client.get(endpoint).json() == dict(items=[expected], result_root=str(results))
    assert digest.called
    digest.reset_mock()
    assert client.get(endpoint).json()['items'] == [expected]
    digest.assert_not_called()

    # A same-size edit after a successful listing must invalidate its cached digest.
    (export / 'models/aliked_lightglue/aliked.onnx').write_bytes(b'changed')
    assert client.get(endpoint).json()['items'] == []
    assert digest.called


@pytest.fixture
def prepared(tmp_path, monkeypatch):
    store = Store(tmp_path / 'record/rosbags', tmp_path / 'results')
    doc = store.create_map('course')
    folder = store.folder('course')
    bag = store.record_root / 'session/bag'
    metadata = write(bag / 'metadata.yaml', b'recorded source')
    write(folder / 'nvidia/cuvslam_map/map.mdb', b'original map')
    write(folder / 'nvidia/map_frames/rectified/frames_meta.json', b'{}')
    write(folder / 'nvidia/cuvgl_map/bow_index.pb')
    write(folder / 'nvidia/cuvgl_map/keyframes/frames_meta.json', b'{}')
    write(folder / 'nvidia/cuvgl_map/vocabulary/vocabulary.pb')
    for name in ('keypoint_creation_config.pb.txt', 'localizer_config.pb.txt', 'matching_config.pb.txt', 'query_map_config.pb.txt'):
        write(folder / 'vgl_runtime_config' / name)
    bundle = write_export(store.result_root, 'example', 'source')
    models = store.result_root / 'models/vgl/example/engines/notebook/run'
    extractor = write(models / 'aliked_lightglue/aliked_424x240.engine')
    matcher = write(models / 'aliked_lightglue/lightglue_aliked_2048.engine')
    write(models / 'aliked_lightglue/aliked.onnx')
    bundle_hash = capture.digest(bundle / 'manifest.json')
    atomic_json(models / 'build.json', dict(status='complete', asset_id='example', export_id='source',
        bundle_sha256=bundle_hash, engines={'extractor': {'sha256': capture.digest(extractor)},
                                         'matcher': {'sha256': capture.digest(matcher)}}))
    atomic_json(folder / 'vgl.json', dict(status='complete', transaction_id='fixture', asset_id='example',
        export_id='source', model_export=str(bundle.relative_to(store.result_root)),
        runtime_model_dir=str(models.relative_to(store.result_root)), input_shape=[1, 3, 240, 424],
        bundle_sha256=bundle_hash, cuvslam_sha256=fingerprint(folder / 'nvidia/cuvslam_map'),
        frames_meta_sha256=capture.digest(folder / 'nvidia/map_frames/rectified/frames_meta.json'),
        cuvgl_sha256=fingerprint(folder / 'nvidia/cuvgl_map'), config_sha256=fingerprint(folder / 'vgl_runtime_config')))
    doc['source'] = dict(kind='nvidia_offline', bag='session/bag', pointcloud_status='pending',
                         metadata_sha256=capture.digest(metadata), inputs=DEFAULT_INPUTS,
                         mapping_options={'ground_constraint': False})
    atomic_json(folder / 'hdmap/map.json', doc)
    monkeypatch.setattr(capture, 'collector_available', lambda: True)
    monkeypatch.setattr(capture.shutil, 'which', lambda _: '/fake/ros2')
    return store


def test_preflight_selects_matching_assets_and_rejects_changes(prepared):
    result = capture.preflight(prepared, 'course')
    assert result['ready'] is True, result
    assert result['bag'] == result['source_bag'] == 'session/bag'
    assert result['model_dir'] == 'models/vgl/example/engines/notebook/run'
    assert result['input_shape'] == [1, 3, 240, 424]
    assert not Path(result['model_dir']).is_absolute()
    write(prepared.folder('course') / 'nvidia/cuvslam_map/map.mdb', b'tampered')
    assert capture.preflight(prepared, 'course')['ready'] is False


def test_source_bag_ground_constraint_and_engine_checks(prepared):
    bag = prepared.record_root / 'other'
    write(bag / 'metadata.yaml', b'another recording')
    assert not capture.preflight(prepared, 'course', bag='other')['ready']
    doc = prepared.load_map('course')
    del doc['source']['mapping_options']
    atomic_json(prepared.folder('course') / 'hdmap/map.json', doc)
    assert any('平面拘束' in issue for issue in capture.preflight(prepared, 'course')['issues'])
    doc['source']['mapping_options'] = {'ground_constraint': False}
    atomic_json(prepared.folder('course') / 'hdmap/map.json', doc)
    write(prepared.result_root / 'models/vgl/example/engines/notebook/run/aliked_lightglue/aliked_424x240.engine', b'changed')
    assert any('engine' in issue for issue in capture.preflight(prepared, 'course')['issues'])


@pytest.mark.parametrize('relative', [
    'nvidia/cuvgl_map/keyframes/frames_meta.json', 'nvidia/cuvgl_map/bow_index.pb',
    'nvidia/cuvgl_map/vocabulary/vocabulary.pb', 'vgl_runtime_config/keypoint_creation_config.pb.txt',
    'vgl_runtime_config/localizer_config.pb.txt', 'vgl_runtime_config/matching_config.pb.txt',
    'vgl_runtime_config/query_map_config.pb.txt',
])
def test_preflight_rejects_structural_gaps_even_if_marker_hashes_match(prepared, relative):
    folder = prepared.folder('course')
    (folder / relative).unlink()
    profile = read_json(folder / 'vgl.json')
    profile.update(cuvgl_sha256=fingerprint(folder / 'nvidia/cuvgl_map'),
                   config_sha256=fingerprint(folder / 'vgl_runtime_config'))
    atomic_json(folder / 'vgl.json', profile)
    result = capture.preflight(prepared, 'course')
    assert result['ready'] is False
    assert result['vgl_ready'] is False


def test_preflight_requires_runtime_onnx_and_checks_its_export_identity(prepared):
    runtime = prepared.result_root / 'models/vgl/example/engines/notebook/run/aliked_lightglue/aliked.onnx'
    runtime.unlink()
    assert not capture.preflight(prepared, 'course')['ready']
    write(runtime, b'another model')
    result = capture.preflight(prepared, 'course')
    assert not result['ready']
    assert any('ONNX' in issue and 'export' in issue for issue in result['issues'])


@pytest.mark.parametrize('outcome', ['complete', 'incomplete_map', 'matcher_failure'])
def test_vgl_publication_and_capture_readiness_share_runtime_requirements(prepared, monkeypatch, outcome):
    from atd_processing import vgl_prepare
    folder = prepared.folder('course')
    original_map = fingerprint(folder / 'nvidia/cuvslam_map')
    model_dir = 'models/vgl/example/exported/source'
    export = prepared.result_root / model_dir
    original_export = validate_bundle(export)
    original_engine = prepared.result_root / 'models/vgl/example/engines/notebook/run'
    previous_build = fingerprint(original_engine)
    builder_inputs = []
    (folder / 'vgl.json').unlink()
    shutil.rmtree(folder / 'nvidia/cuvgl_map')
    shutil.rmtree(folder / 'vgl_runtime_config')
    share = prepared.result_root / 'fixture-sdk/share'
    for name in ('keypoint_creation_config.pb.txt', 'localizer_config.pb.txt', 'matching_config.pb.txt', 'query_map_config.pb.txt'):
        write(share / 'configs/isaac' / name)
    monkeypatch.setattr(vgl_prepare, 'gpu_identity', lambda: {'gpu_uuid': 'fixture'})
    monkeypatch.setattr(vgl_prepare, 'package_paths', lambda: (share.parent, share))
    monkeypatch.setattr(vgl_prepare, 'inspect_engine', lambda path, *args: {'sha256': capture.digest(path)})
    monkeypatch.setattr(vgl_prepare.subprocess, 'check_output', lambda *args, **kwargs:
        '--map_folder --raw_image_folder --config_folder_path --model_dir --binary_folder_path --extract_feature --build_bow_index')

    def run(command):
        command = list(map(str, command))
        if Path(command[0]).name.startswith('export_'):
            output = Path(command[command.index('--output_model_dir') + 1])
            models = Path(command[command.index('--model_dir') + 1])
            builder_inputs.append(models)
            name = 'aliked_424x240.engine' if Path(command[0]).name == 'export_extractor_engine' else 'lightglue_aliked_2048.engine'
            # The real exporter writes the engine to both its input model directory and the requested output.
            cached = write(models / 'aliked_lightglue' / name, name.encode())
            write(output / 'aliked_lightglue' / name, cached.read_bytes())
            input_onnx = models / 'aliked_lightglue/aliked.onnx'
            assert input_onnx.read_bytes() == (export / 'models/aliked_lightglue/aliked.onnx').read_bytes()
            assert not input_onnx.samefile(export / 'models/aliked_lightglue/aliked.onnx')
            if outcome == 'matcher_failure' and Path(command[0]).name == 'export_lightglue_engine':
                raise RuntimeError('SDK matcher failure')
        else:
            output = Path(next(arg.split('=', 1)[1] for arg in command if arg.startswith('--map_folder=')))
            write(output / 'debug.log')  # 非空directoryだけでは完成と扱わない。
            if outcome == 'complete':
                write(output / 'keyframes/frames_meta.json', b'{}')
                write(output / 'bow_index.pb')
                write(output / 'vocabulary/vocabulary.pb')
    monkeypatch.setattr(vgl_prepare, 'run', run)
    arguments = (prepared.result_root, 'course', model_dir)
    if outcome == 'complete':
        vgl_prepare.prepare(*arguments)
        result = capture.preflight(prepared, 'course')
        assert result['ready'], result
        generated = prepared.result_root / result['model_dir']
        assert len(list(generated.rglob('*.engine'))) == 2
        assert list(generated.rglob('*.onnx')) == [generated / 'aliked_lightglue/aliked.onnx']
    else:
        error, message = (RuntimeError, 'SDK matcher failure') if outcome == 'matcher_failure' else (ValueError, 'VGL')
        with pytest.raises(error, match=message):
            vgl_prepare.prepare(*arguments)
        assert not (folder / 'vgl.json').exists()
        assert not (folder / 'nvidia/cuvgl_map').exists()
        assert not (folder / 'vgl_runtime_config').exists()
        if outcome == 'matcher_failure':
            failed, = (prepared.result_root / 'models/vgl/example/engines/notebook-fixture').iterdir()
            assert (failed / 'build-failed.json').is_file()
            assert not list(failed.rglob('*.onnx'))
    assert len(builder_inputs) == 2
    assert all(not path.exists() for path in builder_inputs)
    assert validate_bundle(export) == original_export
    assert fingerprint(original_engine) == previous_build
    assert fingerprint(folder / 'nvidia/cuvslam_map') == original_map


def snapshot():
    return dict(schema='atd.snapshot.v1', frame='map', points=[[1, 2, 3], [4, 5, 6]],
                trajectory=[[0, 0, 0]], provenance=dict(map_id='course', localized_in_existing_map=True,
                cloud_tail_age_s=2.0, cloud_within_tail_tolerance=False))


class Work:
    key = 'fixture-capture'
    def log(self, text): pass
    def check(self): pass
    def run(self, argv, ros=False):
        assert ros and argv[0] == 'env'
        command = argv.index('ros2')
        env = dict(option.split('=', 1) for option in argv[1:command])
        assert argv[command:-1] == ['ros2', 'run', 'atd_mapping', 'capture_snapshot', '--job']
        job = json.loads(Path(argv[-1]).read_text())
        assert job['workflow']['ground_constraint'] is False
        assert env['ROS_DOMAIN_ID'] == '94'
        assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SYSTEM_DEFAULT'
        assert env['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
        assert env['ATD_REPLAY_DDS'] == os.environ.get('ATD_REPLAY_DDS', 'udp')
        assert 'FASTDDS_DEFAULT_PROFILES_FILE' not in env
        assert '.capture-' in job['map_dir']
        assert (Path(job['map_dir']) / 'map.mdb').read_bytes() == b'original map'
        # collector は作業コピーを変更しても元地図を変更しない。
        write(Path(job['map_dir']) / 'map.mdb', b'work copy only')
        atomic_json(Path(job['output']) / 'snapshot.json', snapshot())


@pytest.mark.parametrize('dds', [None, 'shm'])
def test_capture_commits_only_verified_snapshot_and_auto_cloud(prepared, monkeypatch, dds):
    if dds is None:
        monkeypatch.delenv('ATD_REPLAY_DDS', raising=False)
    else:
        monkeypatch.setenv('ATD_REPLAY_DDS', dds)
    result = capture.capture(prepared, Work(), {'id': 'course', 'revision': 1})
    assert result == dict(map='course', revision=2, pointcloud_ready=True, point_count=2)
    doc = prepared.load_map('course')
    assert doc['source']['pointcloud_status'] == 'ready'
    assert (prepared.folder('course') / 'nvidia/cuvslam_map/map.mdb').read_bytes() == b'original map'
    saved = prepared.folder('course') / doc['source']['capture_relative']
    assert read_json(saved / 'snapshot.json')['points'] == [[1, 2, 3], [4, 5, 6]]
    assert read_json(saved / 'cloud.json')['provenance']['cloud_tail_age_s'] == 2.0
    app = create_app(prepared.record_root.parent, prepared.result_root)
    with TestClient(app) as client:
        cloud = client.get('/api/maps/course/cloud').json()
        assert cloud['source_count'] == 2 and cloud['status'] == 'ready'
        assert cloud['trajectory'] == [[0, 0, 0]]
        assert client.get('/api/maps/course/capture').json()['ready'] is False
    assert not list(prepared.maps_root.glob('.capture-*'))


def test_capture_failure_preserves_pending_map_and_all_source_assets(prepared, monkeypatch):
    original = capture.atomic_json
    def fail_commit(path, data):
        if Path(path) == prepared.folder('course') / 'hdmap/map.json':
            raise OSError('simulated commit failure')
        return original(path, data)
    monkeypatch.setattr(capture, 'atomic_json', fail_commit)
    with pytest.raises(OSError, match='commit'):
        capture.capture(prepared, Work(), {'id': 'course', 'revision': 1})
    assert prepared.load_map('course')['source']['pointcloud_status'] == 'pending'
    assert not list((prepared.folder('course') / 'hdmap/captures').iterdir())
    assert not list(prepared.maps_root.glob('.capture-*'))
    assert (prepared.folder('course') / 'nvidia/cuvslam_map/map.mdb').read_bytes() == b'original map'


def test_capture_api_guards_pending_edit_and_enqueues_same_ui(prepared):
    app = create_app(prepared.record_root.parent, prepared.result_root)
    app.state.jobs.start = Mock(return_value={'id': 'capture-job', 'status': 'starting'})
    with TestClient(app) as client:
        assert client.get('/api/maps/course/capture').json()['ready'] is True
        body = dict(prepared.load_map('course'), left=[[0, 0], [1, 0], [1, 1]])
        assert client.put('/api/maps/course', json=body).status_code == 400
        assert client.post('/api/maps/course/cloud', json={'revision': 1, 'frame': 'map', 'points': []}).status_code == 400
        assert prepared.load_map('course')['source']['pointcloud_status'] == 'pending'
        assert client.post('/api/maps/course/capture', json={'revision': 0}).status_code == 422
        assert client.post('/api/maps/course/capture', json={'revision': True}).status_code == 422
        assert client.post('/api/maps/course/capture', json={'revision': 99}).status_code == 400
        assert client.post('/api/maps/course/capture', json={'revision': 1}).json()['id'] == 'capture-job'
    assert app.state.jobs.start.call_args.args[0] == 'capture'
    assert 'offline-mapping' in app.state.jobs.start.call_args.args[2]


def test_normalize_requires_localization_and_keeps_cloud_update_age():
    result = capture.normalize(snapshot(), 'course')
    assert result['source_count'] == 2
    assert result['provenance']['cloud_within_tail_tolerance'] is False
    for changed in (dict(frame='odom'), dict(points=[[True, 0, 0]]), dict(points=[]),
                    dict(provenance={'localized_in_existing_map': False, 'map_id': 'course'})):
        with pytest.raises(ValueError):
            capture.normalize(dict(snapshot(), **changed), 'course')

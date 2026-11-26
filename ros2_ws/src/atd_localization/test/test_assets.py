import hashlib
import json
from pathlib import Path
import shutil

import pytest

from atd_localization.assets import map_metadata, prepare_runtime
from atd_mapping.capture_config import CONFIG_FILES, component_configuration, digest

HW = dict(architecture='aarch64', gpu_uuid='local-device', tensorrt_version='test')
INPUTS = dict(base_frame='base_link', left_image='/left', right_image='/right', left_info='/li',
              right_info='/ri', left_frame='left_optical', right_frame='right_optical')


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(data, dict):
        data = json.dumps(data)
    path.write_text(data)
    return path


def inventory(root):
    return {p.relative_to(root).as_posix() + ('/' if p.is_dir() else ''): 'directory' if p.is_dir() else digest(p)
            for p in root.rglob('*')}


@pytest.fixture
def deployment(tmp_path):
    folder = tmp_path / 'maps/course'
    model = tmp_path / 'models/vgl/aliked'
    local = model / 'engines/orin/summer'
    notebook = model / 'engines/notebook/first'
    source = dict(kind='nvidia_offline', inputs=INPUTS, bag='day1/mapping', metadata_sha256='b'*64,
                  mapping_options=dict(ground_constraint=False))
    write(folder / 'hdmap/map.json', dict(id='course', source=source, frame='map'))
    slam = folder / 'nvidia/cuvslam_map'
    write(slam / 'landmarks.mdb', '123')
    vgl = folder / 'nvidia/cuvgl_map'
    for name in ['keyframes/frames_meta.json', 'bow_index.pb', 'vocabulary/index']:
        write(vgl / name, name)
    frame_file = write(folder / 'nvidia/map_frames/rectified/frames_meta.json', 'stored frames')
    for name in CONFIG_FILES:
        write(folder / 'vgl_runtime_config' / name, name)
    onnx = write(local / 'aliked_lightglue/aliked.onnx', 'onnx')
    manifest = write(model / 'exported/base/manifest.json', dict(schema_version=1, kind='atd.vgl.export', asset_id='aliked', export_id='base',
        model={'height': 480, 'width': 640}, files={'models/aliked_lightglue/aliked.onnx': dict(sha256=digest(onnx))}))
    hashes = {}
    for kind, name in [('extractor', 'aliked_640.engine'), ('matcher', 'lightglue_aliked_640.engine')]:
        hashes[kind] = dict(sha256=digest(write(onnx.parent / name, kind)))
    report = dict(schema_version=1, status='complete', asset_id='aliked', export_id='base', bundle_sha256=digest(manifest),
                  engines=hashes, hardware=HW, target_id='orin', build_id='summer')
    write(local / 'build.json', report)
    write(notebook / 'build.json', dict(report, hardware=dict(HW, gpu_uuid='other'),
                                        target_id='notebook', build_id='first'))
    write(folder / 'vgl.json', dict(status='complete', transaction_id='vg1', asset_id='aliked', export_id='base',
        runtime_model_dir=str(notebook.relative_to(tmp_path)), input_shape=[1, 3, 480, 640],
        bundle_sha256=digest(manifest), cuvslam_sha256=inventory(slam), cuvgl_sha256=inventory(vgl),
        frames_meta_sha256=digest(frame_file), config_sha256=inventory(folder / 'vgl_runtime_config')))
    return tmp_path, folder, local


def prepare(deployment, explicit=''):
    root, _, _ = deployment

    def inspect(binaries, shape):
        assert set(binaries) == {'extractor', 'matcher'} and shape == [1, 3, 480, 640]

    return prepare_runtime(root, 'course', explicit, root / 'work', hardware=HW, inspect=inspect)


def test_validated_target_and_tf_owner_for_both_clocks(deployment):
    root, folder, local = deployment
    result = prepare(deployment)
    assert result['model_dir'] == local
    assert result['hdmap_sha256'] == digest(folder / 'hdmap/map.json')
    assert result['source_bag'] == 'day1/mapping'
    assert result['map_sha256'] == hashlib.sha256(json.dumps(
        inventory(folder/'nvidia/cuvslam_map'), separators=(',', ':'), sort_keys=True).encode()).hexdigest()
    assert result['map_dir'] == root / 'work/cuvslam_map'
    assert (result['map_dir'] / 'landmarks.mdb').read_bytes() == b'123'
    for simulated, landmarks in ((True, True), (True, False), (False, False)):
        resolved = component_configuration(Path(__file__).resolve().parents[2] / 'atd_mapping/config',
                result['workflow'], result['map_dir'], result['map_root'], result['model_dir'], result['config_dir'],
                namespace='/localization', use_sim_time=simulated, capture_visualization=landmarks)
        slam, vgl = resolved['parameters']['cuvslam'], resolved['parameters']['vgl']
        assert slam['base_frame'] == vgl['base_frame'] == 'base_link'
        assert slam['use_sim_time'] is simulated and vgl['use_sim_time'] is simulated
        assert slam['slam_max_map_size'] == 300
        assert slam['publish_map_to_odom_tf'] and slam['publish_odom_to_base_tf']
        assert not vgl['publish_map_to_base_tf'] and not vgl['publish_map_to_odom_tf']
        assert slam['enable_slam_visualization'] is landmarks
        assert slam['enable_landmarks_view'] is landmarks
        assert not slam['enable_observations_view'] and slam['save_map_folder_path'] == ''
        assert resolved['hint_topic'] == '/localization/pose_hint'
        assert ('visual_localization/pose', resolved['hint_topic']) in resolved['remaps']['vgl']
        assert ('visual_slam/initial_pose', resolved['hint_topic']) in resolved['remaps']['cuvslam']
        assert slam['enable_request_hint']
        assert ('visual_slam/trigger_hint', '/localization/request_hint') in resolved['remaps']['cuvslam']
        assert ('visual_localization/trigger_localization', '/localization/request_hint') in resolved['remaps']['vgl']


@pytest.mark.parametrize('changed', ['nvidia/cuvslam_map/landmarks.mdb', 'nvidia/cuvgl_map/bow_index.pb',
                          'nvidia/map_frames/rectified/frames_meta.json', 'vgl_runtime_config/localizer_config.pb.txt'])
def test_map_or_runtime_config_changed(deployment, changed):
    write(deployment[1] / changed, 'changed')
    with pytest.raises(ValueError):
        prepare(deployment)


def second_build(local):
    other = local.parent / 'winter'
    shutil.copytree(local, other)
    report = json.loads((other / 'build.json').read_bytes())
    write(other / 'build.json', dict(report, build_id='winter'))
    return other


def test_multiple_candidates_need_explicit_choice(deployment):
    root, _, local = deployment
    second_build(local)
    with pytest.raises(ValueError, match='複数'):
        prepare(deployment)
    assert prepare(deployment, str(local.relative_to(root)))['model_dir'] == local


@pytest.mark.parametrize('selection', ['recorded', 'explicit', 'corrupt'])
def test_map_recorded_build_with_multiple_compatible_engines(deployment, selection):
    root, folder, local = deployment
    recorded = second_build(local)
    profile = folder / 'vgl.json'
    write(profile, dict(json.loads(profile.read_bytes()), runtime_model_dir=recorded.relative_to(root).as_posix()))
    if selection == 'corrupt':
        write(recorded / 'aliked_lightglue/aliked_640.engine', 'changed')
        with pytest.raises(ValueError, match='内容'):
            prepare(deployment)
    elif selection == 'explicit':
        assert prepare(deployment, local.relative_to(root).as_posix())['model_dir'] == local
    else:
        assert prepare(deployment)['model_dir'] == recorded


def test_reject_mismatched_gpu_and_corrupt_engine(deployment):
    root, _, local = deployment
    with pytest.raises(ValueError, match='GPU'):
        prepare(deployment, 'models/vgl/aliked/engines/notebook/first')
    write(local / 'aliked_lightglue/lightglue_aliked_640.engine', 'other')
    with pytest.raises(ValueError, match='内容'):
        prepare(deployment)


def test_map_identity_capture_and_frame_fail_closed(deployment):
    root, folder, _ = deployment
    document_path = folder/'hdmap/map.json'
    doc = json.loads(document_path.read_bytes())
    with pytest.raises(ValueError):
        map_metadata(root, '../course')
    doc['source']['inputs']['base_frame'] = 'camera_link'
    write(document_path, doc)
    with pytest.raises(ValueError, match='base_link'):
        map_metadata(root, 'course')
    doc['source']['inputs']['base_frame'] = 'base_link'
    doc['source'].update(pointcloud_status='ready', capture_relative='hdmap/captures/test')
    write(document_path, doc)
    write(folder/'hdmap/captures/test/manifest.json', dict(schema='atd.capture.v1', map_id='other'))
    with pytest.raises(ValueError, match='点群'):
        map_metadata(root, 'course')

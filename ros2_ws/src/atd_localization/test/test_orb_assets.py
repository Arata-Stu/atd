import json
from pathlib import Path

import pytest

from atd_localization.assets import map_metadata, verified_hdmap
from atd_localization.orb_assets import map_workspace, publish_map


def complete(stage, root):
    (stage / 'orbslam3/atlas.osa').write_bytes(b'fixed atlas')
    (stage / 'orbslam3/camera.yml').write_bytes(b'frozen camera frame')
    (stage / 'orbslam3/cloud.json').write_text('{"frame":"map","points":[[1,2,3]]}')
    return publish_map(stage, root, 'course', 'day1/mapping', 'c' * 64, 90., image_size=(640, 480))


def test_publish_then_load_the_same_coordinates(tmp_path):
    with map_workspace(tmp_path, 'course') as stage:
        complete(stage, tmp_path)
    meta = map_metadata(tmp_path, 'course')
    assert meta['backend'] == 'orbslam3' and meta['source_bag'] == 'day1/mapping'
    assert meta['input_shape'] == [1, 1, 480, 640]
    assert Path(meta['atlas_path']).read_bytes() == b'fixed atlas'
    hdmap = json.loads(verified_hdmap(tmp_path, meta))
    assert hdmap['source']['map_sha256'] == meta['map_sha256'] and hdmap['lines'] == {}
    assert hdmap['source']['pointcloud_status'] == 'ready'
    assert json.loads((tmp_path / 'maps/course/hdmap/cloud.json').read_bytes())['points'] == [[1, 2, 3]]
    with pytest.raises(FileExistsError), map_workspace(tmp_path, 'course'):
        pass


@pytest.mark.parametrize('name', ['atlas.osa', 'camera.yml', 'gazebo_alignment.json'])
def test_reject_changed_asset(tmp_path, name):
    with map_workspace(tmp_path, 'course') as stage:
        (stage / 'orbslam3/gazebo_alignment.json').write_bytes(b'{}')
        folder = complete(stage, tmp_path)
    (folder / 'orbslam3' / name).write_bytes(b'changed')
    with pytest.raises(ValueError, match='内容'):
        map_metadata(tmp_path, 'course')


def test_interrupted_build_and_competing_writer(tmp_path):
    with pytest.raises(ValueError), map_workspace(tmp_path, 'course') as incomplete:
        publish_map(incomplete, tmp_path, 'course', 'bag', 'a' * 64, 30, image_size=(640, 480))
    assert not (tmp_path / 'maps/course').exists()
    with map_workspace(tmp_path, 'course') as first, map_workspace(tmp_path, 'course') as second:
        target = complete(first, tmp_path)
        with pytest.raises(FileExistsError):
            complete(second, tmp_path)
    assert target.exists() and list((tmp_path / 'maps').iterdir()) == [target]


def test_refuse_hdmap_from_another_atlas(tmp_path):
    with map_workspace(tmp_path, 'course') as stage:
        path = complete(stage, tmp_path) / 'hdmap/map.json'
    doc = json.loads(path.read_bytes())
    doc['source']['map_sha256'] = 'a' * 64
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match='出典'):
        map_metadata(tmp_path, 'course')


def test_refuse_replaced_editor_cloud(tmp_path):
    with map_workspace(tmp_path, 'course') as stage:
        cloud = complete(stage, tmp_path) / 'hdmap/cloud.json'
    cloud.write_bytes(b'{}')
    with pytest.raises(ValueError, match='背景点群'):
        map_metadata(tmp_path, 'course')

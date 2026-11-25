"""Publish and reopen one stereo ORB Atlas in the coordinates of its original vehicle."""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import time

from atd_mapping.capture_config import digest, require_file
from .assets import _identifier, _object, _path, _sha


FILES = ('atlas.osa', 'camera.yml')
ALIGNMENT = 'gazebo_alignment.json'
D455 = dict(left_image='/camera/camera/infra1/image_rect_raw', right_image='/camera/camera/infra2/image_rect_raw',
            left_info='/camera/camera/infra1/camera_info', right_info='/camera/camera/infra2/camera_info',
            left_frame='camera_infra1_optical_frame', right_frame='camera_infra2_optical_frame',
            base_frame='base_link', tf_static='/tf_static')


def _folder(results_root, map_id):
    return _path(Path(results_root).resolve(), Path('maps') / _identifier(map_id))


def is_orb_map(results_root, map_id):
    return _path(_folder(results_root, map_id), 'orbslam3').is_dir()


def _check_recording(bag, sha, fps, size, tracking):
    if (not isinstance(bag, str) or not Path(bag).parts or Path(bag).is_absolute()
            or '..' in Path(bag).parts or not _sha(sha)):
        raise ValueError('ORB-SLAM3 地図作成 bag の識別情報が不正です')
    if (type(fps) not in (int, float) or not math.isfinite(fps) or fps <= 0
            or not isinstance(size, (list, tuple)) or len(size) != 2
            or any(type(n) is not int or n <= 0 for n in size) or tracking != 'vo'):
        raise ValueError('ORB-SLAM3 の保存地図には実測した画像周期・サイズと stereo VO が必要です')


def _map_hash(files):
    # A camera calibration and its frozen map transform give the Atlas its ATD coordinates.
    identity = {name: files[name] for name in FILES}
    return hashlib.sha256(json.dumps(identity, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


@contextmanager
def map_workspace(results_root, map_id):
    target = _folder(results_root, map_id)
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise FileExistsError(f'同名の地図があります: {target}')
    with TemporaryDirectory(dir=target.parent, prefix='.orb-build-') as temp:
        stage = Path(temp) / 'map'
        (stage / 'orbslam3').mkdir(parents=True)
        yield stage


def publish_map(stage, results_root, map_id, bag_id, bag_metadata_sha256, camera_fps, tracking='vo', *, image_size):
    _check_recording(bag_id, bag_metadata_sha256, camera_fps, image_size, tracking)
    target = _folder(results_root, map_id)
    stage = Path(stage)
    orb = stage / 'orbslam3'
    files = {name: digest(require_file(_path(orb, name))) for name in FILES}
    if _path(orb, ALIGNMENT).exists():
        files[ALIGNMENT] = digest(require_file(orb / ALIGNMENT))
    cloud = require_file(_path(orb, 'cloud.json'))
    source = dict(kind='orbslam3', frame='map', bag=bag_id, metadata_sha256=bag_metadata_sha256,
                  inputs=D455, map_sha256=_map_hash(files), pointcloud_status='ready')
    manifest = dict(schema='atd.orbslam3.map.v1', files=files, cloud_sha256=digest(cloud), source_bag=bag_id,
                    source_metadata_sha256=bag_metadata_sha256, camera_fps=camera_fps,
                    image_size=list(image_size), tracking=tracking)
    hdmap = dict(schema='atd.hdmap.v1', id=map_id, title=map_id, revision=1, frame='map',
                 left=[], right=[], custom=[], lines={}, closed=True, source=source, updated=time.time())
    (stage / 'hdmap').mkdir()
    cloud.rename(stage / 'hdmap/cloud.json')
    for path, value in ((orb / 'manifest.json', manifest), (stage / 'hdmap/map.json', hdmap)):
        with path.open('x', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write('\n')
    # The Web editor also holds Store.lock('maps') while creating or copying a map ID.
    locks = _path(Path(results_root).resolve(), 'runs/web')
    locks.mkdir(parents=True, exist_ok=True)
    catalog_lock = _path(locks, hashlib.sha256(b'maps').hexdigest() + '.lock')
    with catalog_lock.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if target.exists():
            raise FileExistsError(f'同名の地図があります: {target}')
        stage.rename(target)
    return target


def orb_map_metadata(results_root, map_id):
    folder = _folder(results_root, map_id)
    manifest, _ = _object(_path(folder, 'orbslam3/manifest.json'))
    doc, hdmap_sha = _object(_path(folder, 'hdmap/map.json'))
    source = doc.get('source')
    files = manifest.get('files')
    if (manifest.get('schema') != 'atd.orbslam3.map.v1' or doc.get('frame') != 'map'
            or doc.get('id') != map_id or not isinstance(source, dict) or source.get('kind') != 'orbslam3'
            or source.get('inputs') != D455 or not isinstance(files, dict)
            or set(files) not in (set(FILES), {*FILES, ALIGNMENT})):
        raise ValueError('選択した ORB-SLAM3 地図と manifest・HDMap の対応が不正です')
    bag, sha = manifest.get('source_bag'), manifest.get('source_metadata_sha256')
    fps, size, tracking = manifest.get('camera_fps'), manifest.get('image_size'), manifest.get('tracking')
    _check_recording(bag, sha, fps, size, tracking)
    orb = _path(folder, 'orbslam3')
    if (_path(orb, ALIGNMENT).exists() != (ALIGNMENT in files)
            or any(not _sha(value) or digest(require_file(_path(orb, name))) != value for name, value in files.items())):
        raise ValueError('ORB-SLAM3 の Atlas・校正・評価用座標変換の内容が保存時と一致しません')
    if manifest.get('cloud_sha256') or source.get('pointcloud_status') == 'ready':
        if (source.get('pointcloud_status') != 'ready' or
                digest(require_file(_path(folder, 'hdmap/cloud.json'))) != manifest.get('cloud_sha256')):
            raise ValueError('ORB-SLAM3 の背景点群が Atlas 保存時の内容と一致しません')
    map_sha = _map_hash(files)
    if (source.get('bag'), source.get('metadata_sha256'), source.get('map_sha256')) != (bag, sha, map_sha):
        raise ValueError('HDMap と ORB-SLAM3 地図の出典が一致しません')
    return dict(backend='orbslam3', map_id=map_id, map_sha256=map_sha, hdmap_sha256=hdmap_sha,
                atlas_path=str(orb / 'atlas.osa'), camera_fps=fps, tracking=tracking, workflow=dict(D455),
                input_shape=[1, 1, size[1], size[0]], source_bag=bag, source_metadata_sha256=sha)

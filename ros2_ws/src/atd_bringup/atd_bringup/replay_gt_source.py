"""Gazebo の測定済み走行軌跡と、同じ試行で確定した rosbag を対応付ける。"""

import hashlib
import json
from pathlib import Path
import re
import shutil


TRACE = 'rig_motion.jsonl'
SIDECAR = 'atd_ground_truth'
LEGACY_NAME = re.compile(
    r'gazebo-(?P<circuit>[a-z0-9_]+)-(?P<run>[0-9]{8}T[0-9]{6}Z-bag-(?:shm|udp)-[A-Za-z0-9]{6})')
BAG_NAME = re.compile(
    r'gazebo-(?P<circuit>[a-z0-9_]+)-\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}_'
    r'\d+x\d+-\d+Hz_imu-(?:off|\d+Hz)'
    r'(?:_(?P<laps>[1-9]\d*)laps?_\d+(?:\.\d+)?(?:e-?\d+)?mps)?(?:-\d+)?')


def _sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def store_ground_truth(bag: Path, trace: Path, mcap_sizes: dict, *, course=None):
    """公開直前の bag に Gazebo の post-update trace と照合用の記録を保存する。"""
    if trace.stat().st_size == 0:
        raise ValueError(f'Gazebo の走行軌跡が空です: {trace}')
    destination = bag / SIDECAR
    destination.mkdir(exist_ok=True)
    shutil.copyfile(trace, destination / TRACE)
    manifest = {
        'version': 1,
        'metadata_sha256': _sha256(bag / 'metadata.yaml'),
        'trace_sha256': _sha256(destination / TRACE),
        'file_bytes': mcap_sizes,
    }
    if course is not None:
        manifest['course'] = course
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')


def _read_object(path):
    value = json.loads(path.read_bytes())
    if not isinstance(value, dict):
        raise ValueError(f'オブジェクト形式の JSON が必要です: {path}')
    return value


def _check_mcap_sizes(bag, expected):
    actual = {p.name: p.stat().st_size for p in bag.glob('*.mcap') if p.is_file()}
    if not actual or actual != expected:
        raise ValueError(f'記録時と MCAP のファイル名またはサイズが異なります: {bag}')


def matching_manifest(bag):
    """同梱した小さな管理情報を bag と照合する。画像・走行軌跡は読み込まない。"""
    manifest = _read_object(bag / SIDECAR / 'manifest.json')
    if manifest.get('version') != 1 or manifest.get('metadata_sha256') != _sha256(bag / 'metadata.yaml'):
        raise ValueError(f'bag と同梱した Gazebo 記録情報の照合に失敗しました: {bag}')
    _check_mcap_sizes(bag, manifest.get('file_bytes'))
    return manifest


def matching_run(bag, results_root):
    match = BAG_NAME.fullmatch(bag.name) or LEGACY_NAME.fullmatch(bag.name)
    if not match:
        return None
    circuit = match['circuit']
    run = match.groupdict().get('run', bag.name)
    runs = results_root / 'runs'
    directory = (runs / 'gazebo_monza' if circuit == 'monza' else runs / 'gazebo' / circuit) / run
    if not directory.exists():
        return None
    report = _read_object(directory / 'run.json')
    if (report.get('mode') != 'bag' or report.get('status') != 'complete'
            or report.get('circuit') != circuit or report.get('scene') != 'scene/scene.json'
            or report.get('bag_path') != f'/data/record/rosbags/{bag.name}'):
        raise ValueError(f'Gazebo の記録結果と選択した bag が一致しません: {directory}')
    audit = report.get('bag_audit')
    _check_mcap_sizes(bag, audit.get('file_bytes') if isinstance(audit, dict) else None)
    return directory, report


def find_ground_truth(bag: Path, results_root: Path = Path('/data/results')) -> Path | None:
    """bag 内の GT を優先し、旧 bag はコースと記録 ID が示す単一の保存先を調べる。"""
    try:
        sidecar = bag / SIDECAR
        if sidecar.exists():
            manifest = matching_manifest(bag)
            trace = sidecar / TRACE
            if manifest.get('trace_sha256') != _sha256(trace):
                raise ValueError(f'同梱した Gazebo 軌跡の照合に失敗しました: {trace}')
            return trace

        match = matching_run(bag, results_root)
        if match is None:
            return None
        directory, _ = match
        trace = directory / 'scene' / TRACE
        if not trace.is_file() or trace.stat().st_size == 0:
            raise ValueError(f'Gazebo の走行軌跡を読み取れません: {trace}')
        return trace
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f'{bag.name} の Gazebo 走行記録を読み取れません: {exc}') from exc

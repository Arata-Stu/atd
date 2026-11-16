"""VGL の可搬入力 bundle。GPU / ROS / 外部 Python package を必要としない。"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tempfile
from datetime import datetime, timezone


KIND = 'atd.vgl.export'
SCHEMA_VERSION = 1
REQUIRED_FILES = (
    'models/aliked_lightglue/aliked.onnx',
    'configs/isaac/keypoint_creation_config.pb.txt',
    'configs/isaac/matching_task_worker_config.pb.txt',
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='seconds').replace('+00:00', 'Z')


def validate_id(value: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}', value):
        raise ValueError('ID は英数字で始まる96文字以内の英数字・_・-・. で指定してください')
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write((json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + '\n').encode())
            stream.flush()
            os.fchmod(stream.fileno(), 0o644)
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)


def _relative_path(name: str) -> PurePosixPath:
    if (not isinstance(name, str) or not name or '\\' in name
            or any(ord(char) < 32 for char in name)):
        raise ValueError('bundle 内のファイル名が不正です')
    path = PurePosixPath(name)
    if path.is_absolute() or any(part in ('', '.', '..') for part in name.split('/')):
        raise ValueError('bundle の相対パスが不正です: ' + name)
    if path.parts[0] not in ('models', 'configs'):
        raise ValueError('bundle は models/ と configs/ の入力ファイルに限定します: ' + name)
    return path


def inventory(root: Path) -> dict:
    root = Path(root)
    if root.is_symlink() or not root.is_dir():
        raise ValueError('bundle は通常のディレクトリを指定してください: ' + str(root))
    files = {}
    for current, directories, names in os.walk(root, followlinks=False):
        for name in directories:
            entry = Path(current) / name
            if entry.is_symlink():
                raise ValueError('bundle 内に symlink は使えません: ' + str(entry))
        for name in sorted(names):
            entry = Path(current) / name
            relative = entry.relative_to(root).as_posix()
            if not stat.S_ISREG(entry.lstat().st_mode):
                raise ValueError('bundle 内に通常ファイル以外があります: ' + relative)
            if relative == 'manifest.json':
                continue
            _relative_path(relative)
            files[relative] = {'sha256': sha256(entry), 'size': entry.stat().st_size}
    return files


def validate_bundle(path: Path) -> dict:
    path = Path(path)
    manifest_path = path / 'manifest.json'
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise ValueError('bundle の manifest.json がありません: ' + str(path))
    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, UnicodeError, ValueError) as error:
        raise ValueError('manifest を読み込めません: ' + str(error)) from error
    if (not isinstance(manifest, dict) or type(manifest.get('schema_version')) is not int
            or manifest['schema_version'] != SCHEMA_VERSION or manifest.get('kind') != KIND):
        raise ValueError('対応しない VGL bundle manifest です')
    validate_id(manifest.get('asset_id'))
    validate_id(manifest.get('export_id'))
    files = manifest.get('files')
    if not isinstance(files, dict) or not files:
        raise ValueError('manifest の files が空または不正です')
    for name, value in files.items():
        _relative_path(name)
        if (not isinstance(value, dict) or not isinstance(value.get('size'), int)
                or isinstance(value.get('size'), bool) or value['size'] < 0
                or not isinstance(value.get('sha256'), str)
                or not re.fullmatch(r'[a-f0-9]{64}', value['sha256'])):
            raise ValueError('manifest の hash / size が不正です: ' + name)
    for name in REQUIRED_FILES:
        if name not in files or files[name]['size'] == 0:
            raise ValueError('必要なモデル・設定がありません: ' + name)
    if not any(name.startswith('models/aliked_lightglue/lightglue') and name.endswith('.onnx')
               and value['size'] > 0 for name, value in files.items()):
        raise ValueError('LightGlue ONNX がありません')
    model = manifest.get('model')
    if (not isinstance(model, dict) or model.get('family') != 'aliked_lightglue'
            or model.get('mode') not in ('aliked', 'package')
            or any(not isinstance(model.get(key), int) or isinstance(model.get(key), bool)
                   or model[key] <= 0 for key in ('width', 'height', 'top_k'))
            or model['top_k'] != 2048):
        raise ValueError('対応しない model 契約です')
    actual = inventory(path)
    if actual != files:
        missing, extra = sorted(files.keys() - actual.keys()), sorted(actual.keys() - files.keys())
        changed = sorted(name for name in files.keys() & actual.keys() if files[name] != actual[name])
        raise ValueError(f'bundle 検証失敗: missing={missing}, extra={extra}, changed={changed}')
    return manifest


def make_manifest(path: Path, asset_id: str, export_id: str, model: dict, source: dict) -> dict:
    manifest = {
        'schema_version': SCHEMA_VERSION, 'kind': KIND,
        'asset_id': validate_id(asset_id), 'export_id': validate_id(export_id),
        'created_at': utc_now(), 'model': model, 'source': source,
        'files': inventory(path),
    }
    write_json(Path(path) / 'manifest.json', manifest)
    return validate_bundle(path)


def model_root(results: Path, asset_id: str) -> Path:
    results = Path(results).resolve()
    parent = results / 'models/vgl'
    root = parent / validate_id(asset_id)
    # 共有保存先の既存 symlink を使って別の場所へ書き込まない。
    for path in (results / 'models', parent, root):
        if path.is_symlink():
            raise ValueError('モデル保存先に symlink は使えません: ' + str(path))
    return root


def export_path(results: Path, asset_id: str, export_id: str) -> Path:
    root = model_root(results, asset_id) / 'exported'
    if root.is_symlink():
        raise ValueError('exported 保存先に symlink は使えません')
    return root / validate_id(export_id)


def copy_inputs(source: Path, destination: Path) -> None:
    """インストール済み package のモデルと設定を可搬化。GPU engine は含めない。"""
    source, destination = Path(source).resolve(strict=True), Path(destination)
    for section in ('models', 'configs'):
        source_section = source / section
        if not source_section.is_dir():
            raise ValueError('source は models/ と configs/ を含む package share を指定してください')
        for current, directories, names in os.walk(source_section, followlinks=False):
            relative = Path(current).relative_to(source_section)
            for name in directories:
                if (Path(current) / name).is_symlink():
                    raise ValueError('入力ディレクトリの symlink は展開対象を確認してください: ' + str(Path(current) / name))
            target = destination / section / relative
            target.mkdir(parents=True, exist_ok=True)
            for name in names:
                path = Path(current) / name
                if path.suffix.lower() in ('.engine', '.plan', '.so', '.a', '.pyc', '.pth', '.pt'):
                    continue
                resolved = path.resolve(strict=True)
                if not resolved.is_file() or not resolved.is_relative_to(source_section.resolve()):
                    raise ValueError('入力ファイルが source の外側を参照しています: ' + str(path))
                shutil.copyfile(resolved, target / name)

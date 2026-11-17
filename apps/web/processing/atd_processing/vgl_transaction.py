"""VGL の複数成果物を完了 marker と journal で公開・復旧する。ROS/GPU 不要。"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import uuid

JOURNAL = '.vgl-transaction.json'
MARKER = 'vgl.json'
ARTIFACTS = ('vgl_runtime_config', 'nvidia/cuvgl_map')


def _write(path, value):
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def _path(folder, relative):
    result = folder / relative
    current = result
    while current != folder:
        if current.is_symlink():
            raise ValueError('VGL 成果物に symlink は扱えません')
        current = current.parent
    return result


def fingerprint(path):
    result = {}
    if path.is_symlink() or not path.is_dir():
        raise ValueError('VGL 成果物が通常のディレクトリではありません')
    for parent, directories, files in os.walk(path, followlinks=False):
        for name in directories + files:
            entry = Path(parent) / name
            if entry.is_symlink():
                raise ValueError('VGL 成果物に symlink は扱えません')
            if entry.is_file():
                with entry.open('rb') as stream:
                    result[entry.relative_to(path).as_posix()] = hashlib.file_digest(stream, 'sha256').hexdigest()
            elif entry.is_dir():
                result[entry.relative_to(path).as_posix() + '/'] = 'directory'
            else:
                raise ValueError('VGL 成果物に特殊ファイルは扱えません')
    return result


def _read_journal(folder):
    path = _path(folder, JOURNAL)
    journal = json.loads(path.read_text())
    if (journal.get('schema') != 'atd.vgl.transaction.v1' or not isinstance(journal.get('token'), str)
            or set(journal.get('artifacts', {})) != set(ARTIFACTS)):
        raise ValueError('VGL 公開 journal が不正です。診断データを確認してください')
    return journal


def check_preparable(folder):
    """API の read-only 検査。稼働 worker の公開途中を変更しない。"""
    folder = Path(folder)
    if _path(folder, MARKER).exists():
        raise ValueError('VGL 完了 marker が既にあります。既存の成果物を確認してください')
    if _path(folder, JOURNAL).exists():
        _read_journal(folder)
        return  # 自ジョブの未完了公開は、資源所有後の worker が復旧する。
    if any(_path(folder, name).exists() for name in ARTIFACTS):
        raise ValueError('VGL 成果物がありますが完了 marker / journal がありません。既存データを確認してください')


def recover(folder):
    """map 資源を所有する worker のみ実行。所有 hash が一致する未完了公開だけを戻す。"""
    folder = Path(folder)
    if not _path(folder, JOURNAL).exists():
        return
    journal = _read_journal(folder)
    marker = _path(folder, MARKER)
    if marker.exists():
        profile = json.loads(marker.read_text())
        if profile.get('status') != 'complete' or profile.get('transaction_id') != journal['token']:
            raise ValueError('VGL 完了 marker と公開 journal が不一致です。自動削除しません')
        for name, expected in journal['artifacts'].items():
            if fingerprint(_path(folder, name)) != expected:
                raise ValueError('VGL 完成成果物が変更されています。自動削除しません')
        _path(folder, JOURNAL).unlink()  # marker 書込後の終了なら完成物を保持する。
        return
    # 削除を始める前に全対象を検査し、第三者の変更を部分削除しない。
    existing = []
    for name, expected in journal['artifacts'].items():
        path = _path(folder, name)
        if path.exists():
            if fingerprint(path) != expected:
                raise ValueError('未完成の VGL 成果物が変更されています。診断データを確認してください')
            existing.append(path)
    for path in existing:
        shutil.rmtree(path)
    _path(folder, JOURNAL).unlink()


def publish(folder, config, output, profile):
    folder = Path(folder)
    recover(folder)
    check_preparable(folder)
    artifacts = dict(zip(ARTIFACTS, (fingerprint(config), fingerprint(output))))
    token = uuid.uuid4().hex
    _write(_path(folder, JOURNAL), dict(schema='atd.vgl.transaction.v1', token=token, artifacts=artifacts))
    try:
        os.rename(config, _path(folder, ARTIFACTS[0]))
        os.rename(output, _path(folder, ARTIFACTS[1]))
        _write(_path(folder, MARKER), dict(profile, status='complete', transaction_id=token))
        _path(folder, JOURNAL).unlink()
    except BaseException:
        recover(folder)
        raise

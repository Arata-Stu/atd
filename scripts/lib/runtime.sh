#!/usr/bin/env bash
# build・再接続・診断で共通の起動設定 revision。
atd_runtime_revision() {
    python3 - "$1" <<'PY'
import hashlib
import os
from pathlib import Path
import sys

root = Path(sys.argv[1])
paths = ['docker/Dockerfile.atd', 'docker/dependencies.apt', 'docker/dev.dockerargs']
for directory in ('docker/scripts', 'docker/middleware_profiles', 'docker/python', 'docker/isaac_ros'):
    for current, directories, files in os.walk(root / directory):
        directories[:] = sorted(name for name in directories
                                if name not in ('.venv', '.atd-python', '__pycache__'))
        paths.extend(str((Path(current) / name).relative_to(root))
                     for name in sorted(files) if not name.endswith('.pyc'))
for current, directories, files in os.walk(root / 'tools/vgl'):
    directories[:] = sorted(name for name in directories if name != '__pycache__')
    paths.extend(str((Path(current) / name).relative_to(root))
                 for name in sorted(files) if name.endswith('.py'))
digest = hashlib.sha256()
for relative in paths:
    try:
        content = (root / relative).read_bytes()
    except OSError as error:
        sys.exit('エラー: 起動設定を読めません: ' + str(error))
    digest.update(relative.encode() + b'\0' + content + b'\0')
print(digest.hexdigest())
PY
}

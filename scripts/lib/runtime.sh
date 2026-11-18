#!/usr/bin/env bash
# イメージに固定する入力と、起動済みコンテナの検査に使う入力を分ける。
atd_image_revision() { _atd_revision "$1" image; }
atd_runtime_revision() { _atd_revision "$1" runtime; }

_atd_revision() {
    python3 - "$1" "$2" <<'PY'
import hashlib
import os
from pathlib import Path
import sys

root = Path(sys.argv[1])
paths = [
    'docker/Dockerfile.atd', 'docker/dependencies.apt',
    'docker/scripts/install-dependencies.sh', 'docker/scripts/configure-dds.sh',
    'docker/middleware_profiles/fastdds.xml',
    'docker/scripts/atd-entrypoint.sh', 'docker/scripts/atd-dispatch.sh',
]
directories_to_hash = []
if sys.argv[2] == 'runtime':
    # atd-python と共通ライブラリは root hook が gosu admin で実行する ROS 環境同期にも使う。
    paths.extend([
        'docker/Dockerfile.realsense455', 'docker/dev.dockerargs',
        'docker/scripts/10-atd-runtime.sh', 'docker/scripts/configure-serial-access.sh',
        'docker/scripts/atd-python.sh', 'docker/scripts/lib/python-environment.sh',
    ])
    directories_to_hash = ['docker/python', 'docker/isaac_ros', 'tools/vgl']
for directory in directories_to_hash:
    for current, directories, files in os.walk(root / directory):
        directories[:] = sorted(name for name in directories
                                if name not in ('.venv', '.atd-python', '__pycache__'))
        paths.extend(str((Path(current) / name).relative_to(root))
                     for name in sorted(files) if not name.endswith('.pyc')
                     and (directory != 'tools/vgl' or name.endswith('.py')))
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

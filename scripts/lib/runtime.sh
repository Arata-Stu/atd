#!/usr/bin/env bash
# イメージに固定する入力と、起動済みコンテナの検査に使う入力を分ける。
atd_image_revision() { _atd_revision "$1" "${2:-image}"; }
atd_runtime_revision() { _atd_revision "$1" runtime; }

_atd_revision() {
    python3 - "$1" "$2" <<'PY'
import hashlib
from pathlib import Path
import sys

root = Path(sys.argv[1])
paths = [
    'docker/Dockerfile.atd', 'docker/dependencies.apt',
    'docker/scripts/install-dependencies.sh',
    'docker/scripts/atd-entrypoint.sh', 'docker/scripts/atd-dispatch.sh',
]
if sys.argv[2] == 'runtime':
    # 起動時に適用する設定。手動で呼ぶコマンドや用途別 venv は checkout の次回実行で更新する。
    paths.extend([
        'docker/Dockerfile.realsense455', 'docker/dev.dockerargs',
        'docker/scripts/10-atd-runtime.sh', 'docker/scripts/configure-serial-access.sh',
        'docker/scripts/configure-dds.sh', 'docker/middleware_profiles/fastdds.xml',
        'docker/scripts/atd-python.sh', 'docker/scripts/lib/python-environment.sh',
        'docker/python/ros/pyproject.toml', 'docker/python/ros/uv.lock',
        'docker/isaac_ros/build_image_layers.yaml', 'docker/isaac_ros/common-config.sh',
        'docker/isaac_ros/config.yaml',
    ])
if sys.argv[2] == 'orbslam3':
    paths.extend(str(path.relative_to(root)) for path in sorted((root / 'docker/orbslam3').rglob('*'))
                 if path.is_file())
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

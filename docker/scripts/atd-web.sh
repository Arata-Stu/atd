#!/usr/bin/env bash
# Notebook のコンテナ内だけで利用する Web UI 入口。ブラウザはホストで開く。
set -euo pipefail
fail() { printf 'ATD Web: %s\n' "$*" >&2; exit 1; }
usage() {
    cat <<'EOF'
使い方: atd-web [serve|build|dev|check] [--port 8766] [--dry-run]
  serve  必要な場合だけ frontend を build して API と一緒に配信（既定）
  build  frontend の依存同期・型検査・build
  dev    API と Vite の開発サーバーを起動（UI: 127.0.0.1:5173）
  check  runtime・source・lock・ROS package の導入状態を確認
Notebook の起動済み ATD Docker 専用。Jetson では起動しない。
EOF
}
task_action=serve
task_port=8766
task_dry_run=false
while (($#)); do
    case "$1" in
        serve|build|dev|check) task_action="$1"; shift ;;
        --port) (($# >= 2)) || fail '--port の値が必要です'; task_port="$2"; shift 2 ;;
        --dry-run) task_dry_run=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) fail "不明な引数: $1" ;;
    esac
done
[[ "$task_port" =~ ^[0-9]+$ && ${#task_port} -le 5 ]] || fail 'port は 1024〜65535 を指定してください'
((10#$task_port >= 1024 && 10#$task_port <= 65535)) || fail 'port は 1024〜65535 を指定してください'
[[ "$(uname -s)" = Linux && "$(uname -m)" = x86_64 ]] || fail 'Web UI は Notebook の Linux / x86_64 Docker 専用です'
task_project="${ATD_PROJECT_DIR:-/workspaces/atd}"
task_workspace="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
task_source="${task_project}/apps/web/frontend"
task_build="${task_workspace}/.atd-web/frontend"
[[ "$task_project" = /* && "$task_workspace" = /* ]] || fail 'project / workspace は絶対パスを指定してください'
if $task_dry_run; then
    printf '[dry-run] action=%s\nsource=%s\nbuild=%s\nAPI=http://127.0.0.1:%s\n' "$task_action" "$task_source" "$task_build" "$task_port"
    exit 0
fi
[[ -r "$task_source/package.json" && -r "$task_source/pnpm-lock.yaml" ]] || fail 'frontend の source / lock がありません'
for task_command in python3 uv node pnpm rsync; do command -v "$task_command" >/dev/null || fail "$task_command がありません。scripts/dev.sh --build で再構築してください"; done
if [[ "$task_action" = check ]]; then
    node --version
    pnpm --version
    atd-python web check
    atd-python raceline check
    (
        set +u  # ROS setup は nounset を前提としていない。
        source /opt/ros/lyrical/setup.bash
        if [[ -f "$task_workspace/install/setup.bash" ]]; then source "$task_workspace/install/setup.bash"; fi
        set -u
        task_failed=false
        for task_package in isaac_mapping_ros isaac_ros_cuvslam isaac_ros_visual_mapping isaac_ros_visual_global_localization atd_mapping atd_vehicle_calibration atd_localization; do
            if ros2 pkg prefix "$task_package" >/dev/null 2>&1; then printf '[OK] %s\n' "$task_package"; else printf '[FAIL] %s 未導入 / 未build\n' "$task_package"; task_failed=true; fi
        done
        ! $task_failed
    )
    exit $?
fi
[[ -d "$task_workspace" && -w "$task_workspace" ]] || fail 'workspace に書き込めません'
mkdir -p "$task_build"
# node_modules / dist は workspace 内の Web ビルド用ディレクトリに置く。
rsync -a --delete --exclude=node_modules --exclude=dist --exclude=.source-hash "$task_source/" "$task_build/"
task_fingerprint="$(python3 - "$task_source" <<'PY'
import hashlib
from pathlib import Path
import sys
root = Path(sys.argv[1])
digest = hashlib.sha256()
for path in sorted(root.rglob('*')):
    if not path.is_file() or any(part in ('node_modules', 'dist', '.git') for part in path.relative_to(root).parts):
        continue
    digest.update(str(path.relative_to(root)).encode() + b'\0' + path.read_bytes() + b'\0')
print(digest.hexdigest())
PY
)"
task_previous=''
[[ ! -r "$task_build/.source-hash" ]] || task_previous="$(cat "$task_build/.source-hash")"
if [[ "$task_fingerprint" != "$task_previous" || ! -r "$task_build/dist/index.html" || "$task_action" = build || "$task_action" = dev ]]; then
    (cd "$task_build" && pnpm install --frozen-lockfile && pnpm build)
    printf '%s\n' "$task_fingerprint" > "$task_build/.source-hash"
fi
[[ "$task_action" != build ]] || exit 0
# 曲率最適化を Web / ROS の Python 依存へ混在させない。初回のみ取得し以後 lock に同期する。
if [[ -z "${ATD_RACELINE_PYTHON:-}" ]]; then
    atd-python raceline sync
    export ATD_RACELINE_PYTHON="${task_workspace}/.atd-python/$(uname -m)/raceline/bin/python"
fi
[[ -f "$ATD_RACELINE_PYTHON" && -x "$ATD_RACELINE_PYTHON" ]] || fail '最小曲率 helper 用 Python がありません。raceline 環境を確認してください'
export ISAAC_ROS_WS="$task_workspace" ATD_WEB_RECORD_ROOT=/data/record ATD_WEB_RESULT_ROOT=/data/results
export ATD_WEB_STATIC_DIR="${task_build}/dist" PYTHONDONTWRITEBYTECODE=1
task_python_path="${task_project}/apps/web/backend/src:${task_project}/apps/web/processing"
# web venv は ROS の PYTHONPATH を引き継がない。公式 ROS CLI は別 process から起動する。
task_server=(atd-python web run -- env "PYTHONPATH=$task_python_path" python -m uvicorn atd_api.main:create_app --factory --host 127.0.0.1 --port "$task_port")
printf 'ATD Web: http://127.0.0.1:%s （Notebook のブラウザで開く）\n' "$task_port"
if [[ "$task_action" = dev ]]; then
    "${task_server[@]}" & task_api_pid=$!
    trap 'kill "$task_api_pid" 2>/dev/null || true' EXIT INT TERM
    (cd "$task_build" && ATD_API_TARGET="http://127.0.0.1:$task_port" pnpm dev --port 5173 --strictPort)
else
    exec "${task_server[@]}"
fi

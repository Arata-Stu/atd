#!/usr/bin/env bash
# Notebook のコンテナ内だけで利用する Web UI 入口。ブラウザはホストで開く。
set -euo pipefail
fail() { printf 'ATD Web: %s\n' "$*" >&2; exit 1; }
usage() {
    cat <<'EOF'
使い方: atd-web [serve|prepare|build|dev|check] [--port 8766] [--dry-run]
  serve  準備済みの UI と API をオフラインで起動（既定）
  prepare  Web・Raceline の依存を同期し、UI をビルド（初回・更新時にオンラインで実行）
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
        serve|prepare|build|dev|check) task_action="$1"; shift ;;
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
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/python-environment.sh"
task_system_python="${ATD_SYSTEM_PYTHON:-/usr/bin/python3}"
task_venvs="${task_workspace}/.atd-python/$(uname -m)"
task_web_venv="$task_venvs/web"
needs_preparation() { fail "$1。オンラインで atd-web prepare を実行してください"; }

# context は atd-python と同じ定義・venv を解決する。変数はこの subshell 内に閉じる。
web_python_environment() (
    atd_python_context "$1"
    task_stamp="$task_venv/.atd-web-lock"
    task_hash="$("$task_python" -I -c '
import hashlib, pathlib, sys
contents = (pathlib.Path(arg).read_bytes() for arg in sys.argv[1:])
print(hashlib.sha256(b"\0".join(contents)).hexdigest())
' "$task_project/pyproject.toml" "$task_project/uv.lock")"
    if [[ "$task_action" = prepare || "$task_action" = dev ]]; then
        if [[ -f "$task_stamp" ]]; then : > "$task_stamp"; fi
        atd-python "$task_profile" sync
        [[ -f "$task_venv/bin/python" && -x "$task_venv/bin/python" ]] || fail "$task_profile venv の Python がありません"
        printf '%s\n' "$task_hash" > "$task_stamp"
    else
        [[ -r "$task_stamp" && "$(cat "$task_stamp")" = "$task_hash" ]] || needs_preparation "$task_profile の Python 環境が未準備か依存定義が更新されています"
        atd_python_check || needs_preparation "$task_profile の Python 環境を確認できません"
    fi
)
if [[ "$task_action" = check ]]; then
    node --version
    pnpm --version
    (
        set +u  # ROS setup は nounset を前提としていない。
        source /opt/ros/lyrical/setup.bash
        if [[ -f "$task_workspace/install/setup.bash" ]]; then source "$task_workspace/install/setup.bash"; fi
        set -u
        task_failed=false
        for task_package in isaac_mapping_ros isaac_ros_cuvslam isaac_ros_visual_mapping isaac_ros_visual_global_localization atd_mapping atd_vehicle_calibration atd_localization atd_control atd_web_bridge; do
            if ros2 pkg prefix "$task_package" >/dev/null 2>&1; then printf '[OK] %s\n' "$task_package"; else printf '[FAIL] %s 未導入 / 未build\n' "$task_package"; task_failed=true; fi
        done
        ! $task_failed
    )
fi
task_fingerprint="$("$task_system_python" -I - "$task_source" <<'PY'
import hashlib
import os
from pathlib import Path
import sys
root = Path(sys.argv[1])
digest = hashlib.sha256()
files = []
for directory, children, names in os.walk(root):
    children[:] = [c for c in children if c not in ('node_modules', 'dist', '.git')]
    files.extend(Path(directory) / name for name in names)
for path in sorted(files):
    if path.is_file():
        digest.update(str(path.relative_to(root)).encode() + b'\0' + path.read_bytes() + b'\0')
print(digest.hexdigest())
PY
)"
if [[ "$task_action" = serve || "$task_action" = check ]]; then
    [[ -r "$task_build/dist/index.html" && -r "$task_build/.source-hash" && "$(cat "$task_build/.source-hash")" = "$task_fingerprint" ]] || needs_preparation 'UI が未ビルドかソースが更新されています'
else
    for task_command in node pnpm rsync; do command -v "$task_command" >/dev/null || fail "$task_command がありません。scripts/dev.sh --build で再構築してください"; done
    mkdir -p "$task_build"
    : > "$task_build/.source-hash"
    rsync -a --delete --exclude=node_modules --exclude=dist --exclude=.source-hash "$task_source/" "$task_build/"
    (cd "$task_build" && pnpm install --frozen-lockfile && pnpm build)
    [[ -r "$task_build/dist/index.html" ]] || fail 'frontend の build 成果物がありません'
    printf '%s\n' "$task_fingerprint" > "$task_build/.source-hash"
fi
[[ "$task_action" != build ]] || exit 0
web_python_environment web
if [[ -z "${ATD_RACELINE_PYTHON:-}" ]]; then
    web_python_environment raceline
    export ATD_RACELINE_PYTHON="$task_venvs/raceline/bin/python"
fi
[[ -f "$ATD_RACELINE_PYTHON" && -x "$ATD_RACELINE_PYTHON" ]] || fail '最小曲率 helper 用 Python がありません。raceline 環境を確認してください'
if [[ "$task_action" = check || "$task_action" = prepare ]]; then
    printf '[OK] Web UI の準備が完了しました。atd-web で起動できます。\n'
    exit 0
fi
export ISAAC_ROS_WS="$task_workspace" ATD_WEB_RECORD_ROOT=/data/record ATD_WEB_RESULT_ROOT=/data/results
export ATD_WEB_STATIC_DIR="${task_build}/dist" PYTHONDONTWRITEBYTECODE=1
# 子の ROS ジョブが Web venv を PATH から外せるよう、環境名も実行する Python に合わせる。
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    PATH="$("$task_system_python" -I -c 'import os,sys; old=os.path.normpath(sys.argv[1]+"/bin"); print(":".join(p for p in sys.argv[2].split(":") if os.path.normpath(p) != old))' "$VIRTUAL_ENV" "$PATH")"
fi
unset PYTHONHOME PYTHONSTARTUP __PYVENV_LAUNCHER__ UV_MANAGED_PYTHON
export VIRTUAL_ENV="$task_web_venv" PATH="${task_web_venv}/bin:$PATH"
export PYTHONNOUSERSITE=1 BASH_ENV=/dev/null ENV=/dev/null
export PYTHONPATH="${task_project}/apps/web/backend/src:${task_project}/apps/web/processing"
task_server=("$task_web_venv/bin/python" -m uvicorn atd_api.main:create_app --factory --host 127.0.0.1 --port "$task_port")
printf 'ATD Web: http://127.0.0.1:%s （Notebook のブラウザで開く）\n' "$task_port"
if [[ "$task_action" = dev ]]; then
    "${task_server[@]}" & task_api_pid=$!
    trap 'kill "$task_api_pid" 2>/dev/null || true' EXIT INT TERM
    (cd "$task_build" && ATD_API_TARGET="http://127.0.0.1:$task_port" pnpm dev --port 5173 --strictPort)
else
    exec "${task_server[@]}"
fi

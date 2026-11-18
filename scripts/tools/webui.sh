#!/usr/bin/env bash
# ホスト用の補助入口。通常は Notebook の Docker 内で atd-web を実行する。
set -euo pipefail
task_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
if [[ "${1:-}" = --help || "${1:-}" = -h ]]; then
    printf '使い方: scripts/tools/webui.sh [serve|build|dev|check] [--port 8766] [--dry-run]\nホストから既存コンテナへ起動を委譲する補助入口です。通常は scripts/dev.sh で入室後、Docker 内で atd-web を実行します。Jetson では UI を起動しません。\n'
    exit 0
fi
source "$task_root/scripts/lib/detect_host.sh"
if [[ "$(detect_atd_host)" = jetson ]]; then
    printf 'ATD Web: UI は Notebook 専用です。Jetson では ROS bridge のみを起動してください。\n' >&2
    exit 1
fi
task_dry_run=false
task_arguments=(web run -- /usr/local/bin/atd-web)
for task_argument in "$@"; do
    if [[ "$task_argument" = --dry-run ]]; then task_dry_run=true; else task_arguments+=("$task_argument"); fi
done
if $task_dry_run; then
    exec bash "$task_root/scripts/tools/python.sh" --host pc --dry-run "${task_arguments[@]}"
fi
exec bash "$task_root/scripts/tools/python.sh" --host pc "${task_arguments[@]}"

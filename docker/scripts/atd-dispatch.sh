#!/usr/bin/env bash
# /usr/local/bin/atd-* から、同名の checkout 内コマンドを呼ぶ。
set -euo pipefail
if [[ -z ${ATD_RUNTIME_DIR:-} ]]; then
    if [[ -d /workspaces/atd ]]; then
        ATD_RUNTIME_DIR=/workspaces/atd/docker
    else
        ATD_RUNTIME_DIR="${ATD_IMAGE_DIR:-/opt/atd}"
    fi
fi
export ATD_RUNTIME_DIR
task_script="$ATD_RUNTIME_DIR/scripts/${0##*/}.sh"
if [[ "$ATD_RUNTIME_DIR" != /* || ! -f "$task_script" ]]; then
    printf 'ATD: コンテナ内のコマンドが見つかりません: %s\n' "$task_script" >&2
    exit 1
fi
exec /bin/bash "$task_script" "$@"

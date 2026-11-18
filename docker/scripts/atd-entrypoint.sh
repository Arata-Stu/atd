#!/usr/bin/env bash
# Isaac ROS の entrypoint と同じシェルで DDS 環境と起動引数を更新する。
if [[ -z ${ATD_RUNTIME_DIR:-} ]]; then
    if [[ -d /workspaces/atd ]]; then
        ATD_RUNTIME_DIR=/workspaces/atd/docker
    else
        ATD_RUNTIME_DIR="${ATD_IMAGE_DIR:-/opt/atd}"
    fi
fi
export ATD_RUNTIME_DIR
if [[ "$ATD_RUNTIME_DIR" != /* || ! -f "$ATD_RUNTIME_DIR/scripts/10-atd-runtime.sh" ]]; then
    printf 'ATD runtime: 起動 hook が見つかりません: %s/scripts/10-atd-runtime.sh\n' "$ATD_RUNTIME_DIR" >&2
    exit 1
fi
source "$ATD_RUNTIME_DIR/scripts/10-atd-runtime.sh" || exit $?

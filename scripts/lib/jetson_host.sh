#!/usr/bin/env bash
# Jetson ホストの設定を操作するコマンドで共有する。
source "$(dirname -- "${BASH_SOURCE[0]}")/detect_host.sh"

atd_require_jetson_host() {
    if [[ "$(uname -s)" != Linux || "$(uname -m)" != aarch64 || "$(detect_atd_host)" != jetson ]]; then
        printf 'Jetson のホスト端末で実行してください。\n' >&2
        return 1
    fi
    if [[ -e /.dockerenv || -e /run/.containerenv ]] || systemd-detect-virt --quiet --container; then
        printf 'この操作はコンテナの外の Jetson ホストで実行してください。\n' >&2
        return 1
    fi
}

atd_run_as_root() {
    if ((EUID == 0)); then
        "$@"
    elif command -v sudo >/dev/null; then
        sudo -- "$@"
    else
        printf 'この操作には管理者権限が必要です。Jetson ホストの sudo を確認してください。\n' >&2
        return 1
    fi
}

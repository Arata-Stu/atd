#!/usr/bin/env bash
# Docker build は --install、起動時は --check（既定）で導入状態を確認する。
set -euo pipefail

fail() {
    printf 'ATD dependencies: %s\n' "$*" >&2
    exit 1
}

[[ "${ROS_DISTRO:-}" == lyrical ]] || fail 'ROS_DISTRO=lyrical が必要です'
task_action=--check
case "${1:-}" in
    --install|--check) task_action="$1"; shift ;;
esac
if [[ "$task_action" == --install ]]; then
    [[ "$(id -u)" == 0 ]] || fail 'apt 依存の導入は Docker build で root として実行してください'
fi
(($# <= 1)) || fail '使い方: install-dependencies.sh [--check|--install] [dependencies.apt]'
task_script_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
task_manifest="${1:-${task_script_dir}/../dependencies.apt}"
[[ -f "$task_manifest" && -r "$task_manifest" ]] || fail "依存一覧を読めません: $task_manifest"

task_packages=()
while IFS= read -r task_line || [[ -n "$task_line" ]]; do
    task_line="${task_line%%#*}"
    read -r task_package task_extra <<< "$task_line"
    [[ -n "$task_package" ]] || continue
    [[ -z "$task_extra" && "$task_package" =~ ^[a-z0-9][a-z0-9+.-]*$ ]] \
        || fail "依存一覧は 1 行 1 パッケージで記載してください: $task_line"
    case "$task_package" in
        *event-camera*|*eventcamera*|*openeb*|*metavision*|*silky*|*prophesee*)
            fail "利用しないセンサ関連の依存は追加できません: $task_package"
            ;;
    esac
    task_packages+=("$task_package")
done < "$task_manifest"
((${#task_packages[@]} > 0)) || fail '依存一覧が空です'

task_missing=()
for task_package in "${task_packages[@]}"; do
    task_status="$(dpkg-query -W -f='${Status}' "$task_package" 2>/dev/null)" || task_status=''
    if [[ "$task_status" != 'install ok installed' ]]; then
        task_missing+=("$task_package")
    fi
done

if ((${#task_missing[@]} == 0)); then
    printf 'ATD dependencies: 導入済み（追加インストールなし）\n'
    exit 0
fi
if [[ "$task_action" == --check ]]; then
    fail "イメージに不足しているパッケージ: ${task_missing[*]}。ホストで ./scripts/dev.sh --build を実行してください"
fi

printf 'ATD dependencies: 不足する %d パッケージを導入します\n' "${#task_missing[@]}"
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends "${task_missing[@]}"
printf 'ATD dependencies: 導入完了\n'

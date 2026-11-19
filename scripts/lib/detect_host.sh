#!/usr/bin/env bash
# ホスト用スクリプトで共有する機種判定。引数は検証時に使うファイルシステムのルート。
# CPU アーキテクチャだけでは Jetson と判定しない。
detect_atd_host() {
    local task_root="${1:-/}" task_path task_value
    task_root="${task_root%/}"
    for task_path in /proc/device-tree/model /sys/firmware/devicetree/base/model; do
        if [[ -r "${task_root}${task_path}" ]]; then
            if task_value="$(tr -d '\000' < "${task_root}${task_path}" 2>/dev/null)"; then
                case "$task_value" in
                    *[Jj][Ee][Tt][Ss][Oo][Nn]*) printf 'jetson\n'; return 0 ;;
                esac
            fi
        fi
    done
    for task_path in /proc/device-tree/compatible /sys/firmware/devicetree/base/compatible; do
        if [[ -r "${task_root}${task_path}" ]]; then
            if task_value="$(tr '\000' '\n' < "${task_root}${task_path}" 2>/dev/null)"; then
                case "$task_value" in
                    *nvidia,tegra*) printf 'jetson\n'; return 0 ;;
                esac
            fi
        fi
    done
    if [[ -r "${task_root}/etc/nv_tegra_release" && -s "${task_root}/etc/nv_tegra_release" ]]; then
        printf 'jetson\n'
        return 0
    fi
    printf 'pc\n'
}

atd_in_container() {
    [[ "${ATD_RUNTIME_API:-}" = 1 || -e /.dockerenv || -e /run/.containerenv ]] ||
        { command -v systemd-detect-virt >/dev/null && systemd-detect-virt --quiet --container; }
}

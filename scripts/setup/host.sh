#!/usr/bin/env bash
# Ubuntu ホストだけを準備する。ROS / D455 SDK / DDS は既存の Docker 層で管理する。
# 根拠: docs.docker.com/engine/install/ubuntu/
#       docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
#       nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html
set -euo pipefail

fail() { printf 'セットアップ: %s\n' "$*" >&2; exit 1; }
task_host=''
task_action=''
task_dry_run=false
while (($#)); do
    case "$1" in
        --host|--action)
            (($# >= 2)) || fail "$1 の値が必要です"
            if [[ "$1" == --host ]]; then task_host="$2"; else task_action="$2"; fi
            shift 2 ;;
        --dry-run) task_dry_run=true; shift ;;
        *) fail "不明な引数: $1" ;;
    esac
done
case "$task_host" in pc|jetson) ;; *) fail '--host pc|jetson を指定してください' ;; esac
case "$task_action" in tools|docker|runtime|isaac|devices|check) ;; *) fail '--action tools|docker|runtime|isaac|devices|check を指定してください' ;; esac

# fixture 用 prefix。通常は Linux ホストの /etc を参照する。
task_root="${ATD_SETUP_ROOT:-/}"
[[ "$task_root" = /* ]] || fail 'ATD_SETUP_ROOT は絶対パスにしてください'
task_root="${task_root%/}"
task_os_file="${task_root}/etc/os-release"
task_os_id='unknown'
task_os_version='unknown'
if [[ -r "$task_os_file" ]]; then
    task_os_id="$(. "$task_os_file"; printf '%s' "${ID:-unknown}")"
    task_os_version="$(. "$task_os_file"; printf '%s' "${VERSION_ID:-unknown}")"
fi

show_command() { printf '+' >&2; printf ' %q' "$@" >&2; printf '\n' >&2; }
run() { show_command "$@"; if ! $task_dry_run; then "$@"; fi; }
privileged() {
    if [[ "$(id -u)" == 0 ]]; then run "$@"; else run sudo "$@"; fi
}
installed() { [[ "$(dpkg-query -W -f='${Status}' "$1" 2>/dev/null || true)" == 'install ok installed' ]]; }
repository_configured() {
    local task_url="$1" task_source
    for task_source in "${task_root}/etc/apt/sources.list" "${task_root}/etc/apt/sources.list.d/"*.list "${task_root}/etc/apt/sources.list.d/"*.sources; do
        [[ -r "$task_source" ]] || continue
        if grep -E '^[[:space:]]*(deb |URIs:)' "$task_source" | grep -qF "$task_url"; then return 0; fi
    done
    return 1
}
install_missing() {
    local task_package
    local -a task_missing=()
    for task_package in "$@"; do
        if ! installed "$task_package"; then task_missing+=("$task_package"); fi
    done
    if ((${#task_missing[@]})); then
        privileged apt-get update
        privileged env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${task_missing[@]}"
    else
        printf '必要なパッケージは導入済みです。\n'
    fi
}
write_config() {
    local task_path="$1" task_content="$2" task_temp
    if [[ -r "$task_path" ]] && [[ "$(cat "$task_path")" == "$task_content" ]]; then return; fi
    if $task_dry_run; then
        printf '[dry-run] 設定を配置: %s\n%s\n' "$task_path" "$task_content"
        return
    fi
    task_temp="$(mktemp)"
    printf '%s\n' "$task_content" > "$task_temp"
    if ! privileged install -D -m 0644 "$task_temp" "$task_path"; then
        rm -f -- "$task_temp"
        return 1
    fi
    rm -f -- "$task_temp"
}
download_key() {
    local task_url="$1" task_path="$2" task_temp task_key
    if [[ -s "$task_path" ]]; then return; fi
    if $task_dry_run; then
        printf '[dry-run] 公式署名鍵: %s -> %s\n' "$task_url" "$task_path"
        return
    fi
    task_temp="$(mktemp -d)"
    task_key="${task_temp}/key"
    if ! curl -fsSL "$task_url" -o "${task_temp}/download" \
        || ! gpg --batch --yes --dearmor -o "$task_key" "${task_temp}/download" \
        || ! privileged install -D -m 0644 "$task_key" "$task_path"; then
        rm -rf -- "$task_temp"
        return 1
    fi
    rm -rf -- "$task_temp"
}
add_group() {
    local task_group="$1" task_user
    task_user="${SUDO_USER:-$(id -un)}"
    [[ "$task_user" != root ]] || return 0
    getent group "$task_group" >/dev/null || return 0
    if ! id -nG "$task_user" | tr ' ' '\n' | grep -qxF "$task_group"; then
        privileged usermod -aG "$task_group" "$task_user"
        printf '%s グループを追加しました。反映にはログアウト・再ログインまたは SSH 再接続が必要です。\n' "$task_group"
    fi
}

if [[ "$task_action" == check ]]; then
    printf 'host: %s\nOS: %s %s\nkernel: %s\narch: %s\n' "$task_host" "$task_os_id" "$task_os_version" "$(uname -s)" "$(uname -m)"
    for task_command in docker nvidia-ctk isaac-ros fzf rsync bluetoothctl; do
        if command -v "$task_command" >/dev/null 2>&1; then
            printf '[導入済み] %s\n' "$task_command"
        else
            printf '[未導入] %s\n' "$task_command"
        fi
    done
    if [[ -r "${task_root}/etc/nv_tegra_release" ]]; then cat "${task_root}/etc/nv_tegra_release"; fi
    if command -v nvidia-smi >/dev/null 2>&1; then nvidia-smi || true; fi
    if command -v docker >/dev/null 2>&1; then docker info --format '{{json .Runtimes}}' || true; fi
    if command -v isaac-ros >/dev/null 2>&1; then isaac-ros status --output json || true; fi
    df -h "${task_root:-/}" || true
    if [[ "$(uname -s)" != Linux || "$task_os_id" != ubuntu || "$task_os_version" != 24.04 ]]; then
        printf '[要確認] 自動導入の対象は Ubuntu 24.04 ホストです。\n'
    fi
    exit 0
fi

[[ "$(uname -s)" == Linux && "$task_os_id" == ubuntu && "$task_os_version" == 24.04 ]] \
    || fail '自動導入は Ubuntu 24.04 の Linux ホストで実行してください（--action check は読取り専用）'
if [[ "$task_host" == pc && "$(uname -m)" != x86_64 ]]; then
    fail 'PC の自動導入は x86_64 を対象とします'
fi
if [[ "$task_host" == jetson && "$(uname -m)" != aarch64 ]]; then
    fail 'Jetson の自動導入は aarch64 を対象とします'
fi

case "$task_action" in
    tools)
        task_packages=(ca-certificates curl gnupg git git-lfs python3 fzf rsync openssh-client usbutils)
        if [[ "$task_host" == jetson ]]; then task_packages+=(bluez openssh-server); fi
        install_missing "${task_packages[@]}"
        ;;
    docker)
        if command -v docker >/dev/null 2>&1; then
            printf '既存の Docker を使用します。再インストールは行いません。\n'
        else
            for task_conflict in docker.io docker-compose docker-compose-v2 docker-doc docker-buildx podman-docker containerd runc; do
                if installed "$task_conflict"; then fail "既存の $task_conflict を確認してください。自動削除・Docker CE への置換は行いません"; fi
            done
            install_missing ca-certificates curl gnupg
            if ! repository_configured https://download.docker.com/linux/ubuntu; then
                download_key https://download.docker.com/linux/ubuntu/gpg "${task_root}/etc/apt/keyrings/atd-docker.gpg"
                task_arch="$(dpkg --print-architecture)"
                write_config "${task_root}/etc/apt/sources.list.d/atd-docker.sources" "Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: noble
Components: stable
Architectures: ${task_arch}
Signed-By: ${task_root}/etc/apt/keyrings/atd-docker.gpg"
            fi
            install_missing docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
            privileged systemctl enable --now docker
        fi
        add_group docker
        ;;
    runtime)
        command -v docker >/dev/null 2>&1 || fail '先に Docker の準備を実行してください'
        task_runtimes="$(docker info --format '{{json .Runtimes}}' 2>/dev/null || true)"
        if [[ -z "$task_runtimes" ]] && ! $task_dry_run; then
            task_runtimes="$(privileged docker info --format '{{json .Runtimes}}' 2>/dev/null || true)"
        fi
        if [[ "$task_runtimes" == *'"nvidia"'* ]]; then
            printf 'NVIDIA runtime は登録済みです。設定・Docker 再起動を省略します。\n'
            exit 0
        fi
        # 設定変更やサービス再起動の前に、稼働中の全コンテナを確認する。
        if ! $task_dry_run; then
            if ! task_running="$(privileged docker ps -q)"; then
                if systemctl is-active --quiet docker; then fail 'Docker の実行状態を取得できません'; fi
                task_running=''
            fi
            [[ -z "$task_running" ]] || fail 'コンテナが稼働中です。処理終了後に runtime の準備を再実行してください（自動停止・再起動はしません）'
        fi
        if ! command -v nvidia-ctk >/dev/null 2>&1; then
            install_missing ca-certificates curl gnupg
            # 公式 stable/deb repository のみを使用する。experimental は追加しない。
            if ! repository_configured https://nvidia.github.io/libnvidia-container/stable/deb; then
                download_key https://nvidia.github.io/libnvidia-container/gpgkey "${task_root}/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg"
                if $task_dry_run; then
                    printf '[dry-run] 公式 stable repository を配置: %s\n' https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list
                else
                    task_list="$(curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list)"
                    task_list="$(printf '%s\n' "$task_list" | sed "s#deb https://#deb [signed-by=${task_root}/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g")"
                    write_config "${task_root}/etc/apt/sources.list.d/atd-nvidia-container-toolkit.list" "$task_list"
                fi
            fi
            install_missing nvidia-container-toolkit
        fi
        privileged nvidia-ctk runtime configure --runtime=docker --set-as-default
        privileged systemctl enable docker
        privileged systemctl restart docker
        ;;
    isaac)
        if command -v isaac-ros >/dev/null 2>&1; then
            printf '既存の Isaac ROS CLI を確認します。\n'
        else
            install_missing ca-certificates curl gnupg
            task_suite=noble
            if [[ "$task_host" == jetson ]]; then task_suite=noble-jetpack; fi
            if ! repository_configured https://isaac.download.nvidia.com/isaac-ros/release-5.0; then
                download_key https://isaac.download.nvidia.com/isaac-ros/repos.key "${task_root}/usr/share/keyrings/nvidia-isaac-ros.gpg"
                write_config "${task_root}/etc/apt/sources.list.d/atd-isaac-ros.list" \
                    "deb [signed-by=${task_root}/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-5.0 ${task_suite} main"
            fi
            install_missing isaac-ros-cli
            if $task_dry_run; then
                privileged isaac-ros init docker
                exit 0
            fi
        fi
        if ! task_status="$(isaac-ros status --output json)"; then
            fail 'CLI の状態を取得できません。既存設定を確認してください。初期化の上書きは行いません'
        fi
        task_mode="$(python3 -c 'import json,sys; value=json.load(sys.stdin).get("mode"); print("uninitialized" if value is None else value)' <<< "$task_status")" \
            || fail 'CLI の状態 JSON を解釈できません'
        case "$task_mode" in
            docker) printf 'Isaac ROS CLI は Docker モードで初期化済みです。\n' ;;
            uninitialized|none|'') privileged isaac-ros init docker ;;
            *) fail "Isaac ROS CLI は ${task_mode} モードです。切替による他プロジェクトへの影響を確認後、sudo isaac-ros init docker を明示実行してください" ;;
        esac
        ;;
    devices)
        install_missing ca-certificates curl usbutils
        task_rule="${task_root}/etc/udev/rules.d/99-realsense-libusb.rules"
        if [[ -s "$task_rule" ]]; then
            printf '既存の RealSense udev ルールを維持します: %s\n' "$task_rule"
        elif $task_dry_run; then
            printf '[dry-run] RealSense v2.56.3 の公式 udev ルールを配置: %s\n' "$task_rule"
        else
            task_temp="$(mktemp)"
            if ! curl -fsSL https://raw.githubusercontent.com/realsenseai/librealsense/v2.56.3/config/99-realsense-libusb.rules -o "$task_temp" \
                || ! privileged install -D -m 0644 "$task_temp" "$task_rule"; then
                rm -f -- "$task_temp"
                exit 1
            fi
            rm -f -- "$task_temp"
            privileged udevadm control --reload-rules
        fi
        printf 'D455 はルール反映後に抜き差ししてください。firmware の更新は行いません。\n'
        if [[ "$task_host" == jetson ]]; then
            install_missing bluez
            privileged systemctl enable --now bluetooth
            printf 'DualSense のペアリングは Ubuntu の Bluetooth 設定または bluetoothctl で行ってください。\n'
        fi
        for task_group in plugdev input dialout; do add_group "$task_group"; done
        ;;
esac

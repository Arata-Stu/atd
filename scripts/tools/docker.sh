#!/usr/bin/env bash
# 読取り専用のセットアップ診断と、ホスト別の結果保存。
set -uo pipefail

usage() {
    cat <<'EOF'
使い方: scripts/tools/docker.sh [--host notebook|pc|jetson] [--action check|logs|start|build] [--dry-run]

引数なし / --check: 設定を OK / FAIL / SKIP で確認し、結果を保存する
--logs / --latest: 保存済み結果を表示（対話端末では fzf で選択）
--start: 確認成功後に dev.sh で入室・再接続
--build: 確認成功後に dev.sh --build で build・入室
--dry-run: 予定だけ表示し、診断コマンド・保存・起動を行わない

結果: logs/docker/<pc|jetson>/*.log（ATD_LOG_DIR で logs/docker 部分を変更）
失敗項目があれば終了値は 1。未起動のコンテナは SKIP。
診断で sudo・インストール・GPU コンテナのダウンロードは行わない。
EOF
}
fail() { printf 'エラー: %s\n' "$*" >&2; return 1; }

task_action=check
task_host=''
task_dry_run=false
task_latest=false
while (($#)); do
    case "$1" in
        --host|--action)
            (($# >= 2)) && [[ -n "$2" ]] || { fail "$1 の値が必要です"; exit 1; }
            if [[ "$1" == --host ]]; then task_host="$2"; else task_action="$2"; fi
            shift 2 ;;
        --check) task_action=check; shift ;;
        --logs) task_action=logs; shift ;;
        --latest) task_action=logs; task_latest=true; shift ;;
        --start) task_action=start; shift ;;
        --build) task_action=build; shift ;;
        --dry-run) task_dry_run=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) fail "不明な引数: $1"; exit 1 ;;
    esac
done
[[ "$task_host" != notebook ]] || task_host=pc
case "$task_host" in ''|pc|jetson) ;; *) fail '--host は notebook、pc または jetson を指定してください'; exit 1 ;; esac
case "$task_action" in check|logs|start|build) ;; *) fail "不明な action: $task_action"; exit 1 ;; esac
task_project_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)" || exit 1
if [[ -z "$task_host" ]]; then
    source "${task_project_dir}/scripts/lib/detect_host.sh" || exit 1
    task_host="$(detect_atd_host "${ATD_SETUP_ROOT:-/}")" || exit 1
fi
task_log_dir="${ATD_LOG_DIR:-${task_project_dir}/logs/docker}/${task_host}"
[[ "$task_log_dir" = /* ]] || { fail 'ATD_LOG_DIR は絶対パスを指定してください'; exit 1; }
if $task_dry_run; then
    printf '[dry-run] host: %s\naction: %s\n結果保存先: %s\n' "$task_host" "$task_action" "$task_log_dir"
    printf 'OS / architecture / workspace / storage / Docker / NVIDIA runtime / Isaac CLI / DDS / container を確認予定。\n'
    if [[ "$task_action" == start || "$task_action" == build ]]; then printf '確認成功後に dev.sh を実行予定。\n'; fi
    exit 0
fi

show_logs() {
    local task_file task_selected=''
    local -a task_files=()
    for task_file in "$task_log_dir"/*.log; do
        [[ -f "$task_file" ]] && task_files+=("$task_file")
    done
    ((${#task_files[@]})) || { fail "保存済みの診断結果がありません: $task_log_dir"; return 1; }
    if [[ -t 0 && -t 1 ]] && ! $task_latest && command -v fzf >/dev/null 2>&1; then
        task_selected="$(printf '%s\n' "${task_files[@]}" | sort -r | fzf --height=75% --border \
            --prompt='ATD Docker logs > ' --header="${task_host}: 診断結果を選択")" || return 0
        # picker の出力を shell として実行せず、列挙したファイルだけ開く。
        for task_file in "${task_files[@]}"; do
            if [[ "$task_selected" == "$task_file" ]]; then cat -- "$task_file"; return $?; fi
        done
        fail '選択した結果がありません'; return 1
    fi
    task_selected="$(printf '%s\n' "${task_files[@]}" | sort -r | head -n 1)"
    printf '最新の結果: %s\n' "$task_selected"
    cat -- "$task_selected"
}
if [[ "$task_action" == logs ]]; then show_logs; exit $?; fi

task_ok=0
task_fail=0
task_skip=0
report() {
    local task_state="$1" task_label="$2" task_message="$3"
    case "$task_state" in
        OK) task_ok=$((task_ok + 1)) ;;
        FAIL) task_fail=$((task_fail + 1)) ;;
        SKIP) task_skip=$((task_skip + 1)) ;;
    esac
    printf '[%-4s] %-18s %s\n' "$task_state" "$task_label" "$task_message"
}

storage_directory_ok() {
    local task_path="$1"
    [[ "$task_path" == /* && -d "$task_path" && -r "$task_path" && -w "$task_path" && -x "$task_path" ]] || return 1
    case "$task_path" in *[\$\`\"\\,]*|*$'\n'*|*$'\r'*) return 1 ;; esac
    task_path="$(CDPATH= cd -- "$task_path" && pwd -P)" || return 1
    case "$task_path" in *[\$\`\"\\,]*|*$'\n'*|*$'\r'*) return 1 ;; esac
    ATD_CHECKED_STORAGE_PATH="$task_path"
}

workspace_metadata_ok() {
    local task_workspace="$1" task_parent
    task_parent="$(dirname -- "$task_workspace")"
    cmp -s "${task_project_dir}/docker/isaac_ros/config.yaml" "$task_workspace/.isaac-ros-cli/config.yaml" \
        && cmp -s "${task_project_dir}/docker/isaac_ros/common-config.sh" "$task_workspace/scripts/.isaac_ros_common-config" \
        && cmp -s "${task_project_dir}/docker/isaac_ros/build_image_layers.yaml" "$task_parent/scripts/.build_image_layers.yaml" || return 1
    if [[ -e "$task_parent/scripts/.isaac_ros_common-config" || -L "$task_parent/scripts/.isaac_ros_common-config" ]]; then
        cmp -s "${task_project_dir}/docker/isaac_ros/common-config.sh" "$task_parent/scripts/.isaac_ros_common-config" || return 1
    fi
}

run_checks() {
    local task_root="${ATD_SETUP_ROOT:-/}" task_os_id=unknown task_os_version=unknown task_arch task_kernel
    local task_workspace='' task_info='' task_cli='' task_value='' task_storage task_record='' task_result=''
    local task_docker_ok=false task_python_ok=false task_inspect task_revision task_ready task_container task_source_revision
    task_root="${task_root%/}"
    printf 'ATD Docker setup check\nUTC: %s\nhost: %s\nproject: %s\n' "$(date -u '+%Y-%m-%d %H:%M:%S')" "$task_host" "$task_project_dir"
    printf 'この診断は設定・起動準備の確認です。CUDA 演算・D455・Joy・実走行は検証しません。\n\n'
    if [[ -r "${task_root}/etc/os-release" ]]; then
        task_os_id="$(. "${task_root}/etc/os-release"; printf '%s' "${ID:-unknown}")"
        task_os_version="$(. "${task_root}/etc/os-release"; printf '%s' "${VERSION_ID:-unknown}")"
    fi
    task_kernel="$(uname -s)"
    task_arch="$(uname -m)"
    if [[ "$task_kernel" == Linux && "$task_os_id" == ubuntu && "$task_os_version" == 24.04 ]]; then
        report OK OS 'Ubuntu 24.04 / Linux'
    else report FAIL OS "${task_kernel} / ${task_os_id} ${task_os_version}（対象: Ubuntu 24.04）"; fi
    if [[ "$task_host" == pc && "$task_arch" == x86_64 ]] || [[ "$task_host" == jetson && "$task_arch" == aarch64 ]]; then
        report OK architecture "$task_arch"
    else report FAIL architecture "${task_host} に対して ${task_arch} は対象外"; fi
    if [[ "$task_host" == jetson ]]; then
        if [[ "$(detect_atd_host "${task_root:-/}")" == jetson ]]; then report OK Jetson 'Jetson / Tegra marker あり'
        else report FAIL Jetson 'Jetson / Tegra marker がありません'; fi
    fi
    if command -v git >/dev/null 2>&1 && git lfs version >/dev/null 2>&1; then report OK git-lfs '導入済み'
    else report FAIL git-lfs '未導入: setup.sh の tools を実行してください'; fi
    if command -v python3 >/dev/null 2>&1; then task_python_ok=true; report OK python3 '導入済み'
    else report FAIL python3 '未導入: setup.sh の tools を実行してください'; fi
    if [[ -r "${task_project_dir}/scripts/lib/workspace.sh" ]] \
        && source "${task_project_dir}/scripts/lib/workspace.sh" \
        && atd_load_workspace "$task_project_dir" "$task_host" \
        && atd_validate_workspace_path "$ATD_RESOLVED_ROS_WS"; then
        task_workspace="$ATD_RESOLVED_ROS_WS"
        if [[ -d "$task_workspace" ]]; then task_workspace="$(CDPATH= cd -- "$task_workspace" && pwd -P)"; fi
        if atd_validate_workspace_path "$task_workspace" && [[ -d "$task_workspace/src" && -r "$task_workspace" && -w "$task_workspace" && -x "$task_workspace" ]]; then
            report OK workspace "$task_workspace"
        else report FAIL workspace "未準備または読書き不可: $task_workspace（setup.sh の workspace を実行）"; fi
        if workspace_metadata_ok "$task_workspace"; then report OK workspace-CLI 'ATD の CLI metadata と一致'
        else report FAIL workspace-CLI 'CLI metadata が不足または衝突: setup.sh の workspace を実行してください'; fi
    else report FAIL workspace 'workspace 設定を読み込めません'; fi
    task_storage="${ATD_STORAGE_CONFIG:-${task_project_dir}/config/storage.env}"
    if [[ -f "$task_storage" ]]; then
        if ! [[ -r "$task_storage" ]] || ! source "$task_storage"; then report FAIL storage "保存先設定を読めません: $task_storage"; fi
    elif [[ -n "${ATD_STORAGE_CONFIG:-}" ]]; then report FAIL storage "指定した保存先設定がありません: $task_storage"; fi
    if [[ "$task_host" == pc ]]; then
        task_record="${ATD_RECORD_DIR:-${ATD_PC_RECORD_DIR:-}}"
        task_result="${ATD_RESULT_DIR:-${ATD_PC_RESULT_DIR:-}}"
    else
        task_record="${ATD_RECORD_DIR:-${ATD_JETSON_RECORD_DIR:-}}"
        task_result="${ATD_RESULT_DIR:-${ATD_JETSON_RESULT_DIR:-}}"
    fi
    if storage_directory_ok "$task_record"; then
        report OK record "$ATD_CHECKED_STORAGE_PATH"
    else report FAIL record '未設定・不存在・読書き不可・不正パス: setup.sh の storage を実行してください'; fi
    if storage_directory_ok "$task_result"; then
        report OK results "$ATD_CHECKED_STORAGE_PATH"
    else report FAIL results '未設定・不存在・読書き不可・不正パス: setup.sh の storage を実行してください'; fi
    if command -v docker >/dev/null 2>&1; then
        report OK Docker-CLI '導入済み'
        if task_info="$(docker info --format '{{json .Runtimes}}' 2>/dev/null)"; then
            task_docker_ok=true; report OK Docker-daemon '現在のユーザーで接続可能'
            if $task_python_ok && printf '%s' "$task_info" | python3 -c 'import json,sys; sys.exit(0 if "nvidia" in json.load(sys.stdin) else 1)' 2>/dev/null; then
                report OK NVIDIA-runtime 'Docker に nvidia runtime 登録済み（GPU 演算は未確認）'
            else report FAIL NVIDIA-runtime 'nvidia runtime を確認できません: setup.sh の runtime を実行してください'; fi
        else report FAIL Docker-daemon '接続不可: サービス・docker グループを確認し、必要なら再ログインしてください'; report SKIP NVIDIA-runtime 'Docker に接続できないため'; fi
    else report FAIL Docker-CLI '未導入: setup.sh の docker を実行してください'; report SKIP Docker-daemon 'Docker CLI 未導入'; report SKIP NVIDIA-runtime 'Docker CLI 未導入'; fi
    if command -v isaac-ros >/dev/null 2>&1; then
        if ! task_cli="$(atd_check_isaac_cli 2>&1)"; then
            report FAIL Isaac-CLI "$task_cli"
        elif task_cli="$(isaac-ros status --output json 2>/dev/null)" && $task_python_ok \
            && printf '%s' "$task_cli" | python3 -c 'import json,sys; sys.exit(0 if json.load(sys.stdin).get("mode") == "docker" else 1)' 2>/dev/null; then
            report OK Isaac-CLI 'Docker mode'
        else report FAIL Isaac-CLI 'Docker mode を確認できません: setup.sh の isaac を実行してください'; fi
    else report FAIL Isaac-CLI '未導入: setup.sh の isaac を実行してください'; fi
    if $task_python_ok && python3 - "${task_project_dir}/docker/middleware_profiles/fastdds.xml" <<'PY'
import sys
import xml.etree.ElementTree as ET
try:
    root = ET.parse(sys.argv[1]).getroot()
    ns = {'d': 'http://www.eprosima.com'}
    participant = root.find('.//d:participant[@is_default_profile="true"]', ns)
    assert participant is not None
    assert participant.findtext('./d:rtps/d:useBuiltinTransports', namespaces=ns) == 'false'
    transports = {t.findtext('d:transport_id', namespaces=ns): t
                  for t in root.findall('.//d:transport_descriptor', ns)}
    for identifier in participant.findall('./d:rtps/d:userTransports/d:transport_id', ns):
        transport = transports[identifier.text]
        assert transport.findtext('d:type', namespaces=ns) == 'UDPv4'
        assert [e.text for e in transport.findall('./d:interfaceWhiteList/d:address', ns)] == ['127.0.0.1']
    assert participant.findall('./d:rtps/d:userTransports/d:transport_id', ns)
    assert all(e.text == '127.0.0.1' for e in root.findall('.//d:address', ns))
except (OSError, ET.ParseError, AssertionError, KeyError):
    sys.exit(1)
PY
    then report OK DDS-source 'Fast DDS / UDPv4 loopback 設定（通信実測は別途）'
    else report FAIL DDS-source 'Docker 用 Fast DDS 設定を確認できません'; fi
    task_container="atd_${task_host}_dev"
    if $task_docker_ok && $task_python_ok; then
        if task_inspect="$(docker container inspect "$task_container" 2>/dev/null)"; then
            task_value="$(printf '%s' "$task_inspect" | python3 -c 'import json,sys; print("running" if json.load(sys.stdin)[0]["State"]["Running"] else "stopped")' 2>/dev/null)"
            if [[ "$task_value" == running ]]; then
                task_revision="$(printf '%s' "$task_inspect" | python3 -c 'import json,sys; env=dict(e.split("=",1) for e in json.load(sys.stdin)[0].get("Config",{}).get("Env",[]) if "=" in e); print(env.get("ATD_RUNTIME_REV", ""))' 2>/dev/null)"
                task_source_revision=''
                if source "${task_project_dir}/scripts/lib/runtime.sh"; then
                    task_source_revision="$(atd_runtime_revision "$task_project_dir")" || task_source_revision=''
                fi
                if printf '%s' "$task_inspect" | python3 -c '
import json
import os
import sys
try:
    container = json.load(sys.stdin)[0]
    paths = list(sys.argv[1:5])
    destinations = ["/workspaces/isaac_ros-dev", "/workspaces/atd", "/data/record", "/data/results"]
    mounts = {mount["Destination"]: mount for mount in container.get("Mounts", [])}
    source_root = os.path.join(sys.argv[2], "ros2_ws", "src")
    source_destination = "/workspaces/isaac_ros-dev/src/atd"
    if os.path.realpath(source_root) == os.path.realpath(os.path.join(sys.argv[1], "src")):
        if source_destination in mounts:
            sys.exit("不要な入れ子の source mount があります")
    else:
        paths.append(source_root)
        destinations.append(source_destination)
    for destination, expected in zip(destinations, paths):
        mount = mounts.get(destination, {})
        if (not expected or mount.get("Type") != "bind" or mount.get("RW") is not True
                or os.path.realpath(mount.get("Source", "")) != os.path.realpath(expected)):
            sys.exit("mount が現在の設定と異なります: " + destination)
    environment = dict(item.split("=", 1) for item in container.get("Config", {}).get("Env", []) if "=" in item)
    required = {
        "ATD_RUNTIME_API": "1",
        "ATD_RUNTIME_REV": sys.argv[5],
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "FASTDDS_DEFAULT_PROFILES_FILE": "/etc/atd/fastdds.xml",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "SYSTEM_DEFAULT",
        "ROS_STATIC_PEERS": "",
        "RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
    }
    if not sys.argv[5] or any(environment.get(key) != value for key, value in required.items()):
        sys.exit("依存・DDS の環境変数が現在の設定と異なります")
except (ValueError, KeyError, TypeError, IndexError):
    sys.exit("container inspect の構成を解釈できません")
' "$task_workspace" "$task_project_dir" "$task_record" "$task_result" "$task_source_revision"; then
                    report OK container-config 'checkout / workspace / data mounts / DDS 環境変数が現在の設定と一致'
                else report FAIL container-config '設定変更前の構成または不一致。処理を終了後にコンテナを再作成してください'; fi
                if [[ -n "$task_revision" ]] && task_ready="$(docker exec "$task_container" cat /etc/atd/runtime.ready 2>/dev/null)" \
                    && [[ "$task_ready" == "$task_revision" ]] \
                    && [[ "$task_source_revision" == "$task_revision" ]]; then
                    report OK container-startup "$task_container: 現在の source と一致し、初期設定完了"
                else report FAIL container-startup "$task_container: 初期設定未完了・旧形式・source不一致。起動ログを確認し、処理を終了後に再作成してください"; fi
            elif [[ "$task_value" == stopped ]]; then report SKIP container-config "$task_container は停止中"; report SKIP container-startup "$task_container は停止中"
            else report FAIL container-config 'Docker inspect を解釈できません'; report FAIL container-startup 'Docker inspect を解釈できません'; fi
        else report SKIP container-config "$task_container は未作成"; report SKIP container-startup "$task_container は未作成"; fi
    else report SKIP container-config 'Docker / python3 を確認できないため'; report SKIP container-startup 'Docker / python3 を確認できないため'; fi
    printf '\nsetup: OK=%d FAIL=%d SKIP=%d\n' "$task_ok" "$task_fail" "$task_skip"
    ((task_fail == 0))
}

# source 済みでない手動 --host の場合にも Jetson marker を診断できるようにする。
source "${task_project_dir}/scripts/lib/detect_host.sh" || exit 1
source "${task_project_dir}/scripts/lib/isaac_cli.sh" || exit 1
umask 077
mkdir -p -- "$task_log_dir" || { fail "結果保存先を作れません: $task_log_dir"; exit 1; }
task_log="${task_log_dir}/$(date -u '+%Y%m%dT%H%M%SZ')-$$.log"
run_checks 2>&1 | tee "$task_log"
task_statuses=("${PIPESTATUS[@]}")
if [[ "${task_statuses[1]}" != 0 ]]; then fail "結果を保存できません: $task_log"; exit 1; fi
printf '診断結果: %s\n' "$task_log"
[[ "${task_statuses[0]}" == 0 ]] || exit 1
if [[ "$task_action" == start || "$task_action" == build ]]; then
    task_dev_command=(bash "${task_project_dir}/scripts/dev.sh" --host "$task_host")
    if [[ "$task_action" == build ]]; then task_dev_command+=(--build); fi
    "${task_dev_command[@]}"
    task_exit=$?
    printf '\ndev.sh (%s) 終了値: %d\n' "$task_action" "$task_exit" | tee -a "$task_log"
    printf 'CLI の終了値だけでは起動成功を判定しません。稼働中に別端末で scripts/tools/docker.sh を実行して確認してください。\n' | tee -a "$task_log"
    exit "$task_exit"
fi

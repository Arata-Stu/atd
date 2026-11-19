#!/usr/bin/env bash
# 初回セットアップの短い入口。ホスト変更は選択した段階でのみ行う。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: scripts/setup.sh [--host notebook|pc|jetson] [--action ACTION] [--dry-run]

引数なし: fzf の対話メニュー（未導入時は番号選択）
ACTION: menu, check, tools, launcher, docker, runtime, isaac, storage, workspace, devices,
        verify, build, start, web, all
  check       ホスト・GPU・Docker 等の状態を表示
  tools       ホストの基本ツール・fzf を準備し、atd コマンドを登録
  launcher    この checkout の atd コマンドを ~/.local/bin/atd に登録
  docker      Docker を準備（既存導入は維持）
  runtime     NVIDIA Container Toolkit / Docker runtime を準備
  isaac       apt 版 Isaac ROS CLI を準備・Docker モードを確認
  storage     記録・解析結果の保存先を設定
  workspace   ROS workspace と CLI 設定を準備・確認
  devices     D455 のホスト udev・Jetson の Bluetooth 等を準備
  verify      Docker / CLI / GPU runtime の起動を確認
  build       ATD イメージを build して入室
  start       ATD コンテナを起動、または再接続
  web         起動済み Notebook コンテナで Web UI をオフライン利用できるよう事前準備
  all         tools → docker → runtime → isaac → devices → storage → workspace → verify → build

--record-dir DIR / --result-dir DIR: storage の保存先を直接指定
--workspace DIR: workspace のパスを直接指定（build / start では今回のみ適用）
--dry-run: 実行予定のみ表示。sudo、通信、ファイル作成を行わない
Notebook（x86_64）/ Jetson は自動判定。notebook は pc の別名。
ROS workspace は対話的に指定し、Notebook / Jetson 別に config/workspace.env へ保存。
既定値は <clone先>/ros2_ws、データは <clone先>/data/{record,results}。
標準の clone 先は ~/workspaces/atd。Docker 内の workspace は /workspaces/isaac_ros-dev。
OS・GPU driver・JetPack の自動更新は行わない。
全体を sudo で起動せず、必要なホスト操作だけ内部で sudo を使用する。
手順: docs/setup/README.md
EOF
}

fail() { printf 'エラー: %s\n' "$*" >&2; return 1; }

task_action=menu
task_host=""
task_dry_run=false
task_record_override=""
task_result_override=""
task_workspace_override=""
while (($#)); do
    case "$1" in
        --host|--action|--record-dir|--result-dir|--workspace)
            (($# >= 2)) || { fail "$1 の値が必要です"; exit 1; }
            [[ -n "$2" ]] || { fail "$1 の値が空です"; exit 1; }
            case "$1" in
                --host) task_host="$2" ;;
                --action) task_action="$2" ;;
                --record-dir) task_record_override="$2" ;;
                --result-dir) task_result_override="$2" ;;
                --workspace) task_workspace_override="$2" ;;
            esac
            shift 2
            ;;
        --dry-run) task_dry_run=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) fail "不明な引数: $1"; exit 1 ;;
    esac
done
if [[ "$task_host" = notebook ]]; then task_host=pc; fi
case "$task_host" in ""|pc|jetson) ;; *) fail '--host は notebook、pc または jetson を指定してください'; exit 1 ;; esac
case "$task_action" in menu|check|tools|launcher|docker|runtime|isaac|storage|workspace|devices|verify|build|start|web|all) ;;
    *) fail "不明な action: $task_action"; exit 1 ;;
esac

task_project_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${task_project_dir}/scripts/lib/workspace.sh"
task_host_selection=auto
if [[ -n "$task_host" ]]; then
    task_host_selection=manual
else
    source "${task_project_dir}/scripts/lib/detect_host.sh"
    task_host="$(detect_atd_host)"
fi
printf 'ATD setup — host: %s (%s)\n' "$task_host" "$task_host_selection"
printf 'プロジェクト（clone先）: %s\n' "$task_project_dir"
if [[ "$task_host" = pc ]]; then
    printf 'profile: Notebook / x86_64 (amd64)\n'
else
    printf 'profile: Jetson / aarch64 (BSP を確認)\n'
fi

run_command() {
    printf '+'
    printf ' %q' "$@"
    printf '\n'
    if $task_dry_run; then return 0; fi
    "$@"
}

install_launcher() {
    local task_source="${task_project_dir}/scripts/atd" task_bin_dir="${HOME}/.local/bin"
    local task_link="${task_bin_dir}/atd" task_resolved
    if $task_dry_run; then
        printf 'atd コマンドを登録予定: %s -> %s\n' "$task_link" "$task_source"
        return 0
    fi
    [[ -f "$task_source" && -x "$task_source" ]] || { fail "実行可能なファイルが見つかりません: $task_source"; return 1; }
    if [[ -e "$task_link" || -L "$task_link" ]]; then
        [[ -L "$task_link" && "$task_link" -ef "$task_source" ]] || {
            fail "$task_link は使用中です。この checkout のメニューは $task_source で開けます"
            return 1
        }
    else
        mkdir -p -- "$task_bin_dir" || return $?
        ln -s -- "$task_source" "$task_bin_dir/" || return $?
    fi
    printf 'atd コマンド: %s -> %s\n' "$task_link" "$task_source"
    task_resolved="$(command -v atd || true)"
    if [[ ! "$task_resolved" -ef "$task_source" ]]; then
        if [[ -n "$task_resolved" ]]; then printf '現在 PATH で選ばれる atd: %s\n' "$task_resolved"; fi
        printf '%s\n' 'ホストで atd を呼び出すには、シェルで以下を実行してください。' \
            '  export PATH="$HOME/.local/bin:$PATH"'
    fi
}

validate_storage_path() {
    local task_path="$1"
    [[ "$task_path" = /* ]] || { fail "保存先は絶対パスで指定してください: $task_path"; return 1; }
    case "$task_path" in
        *[\$\`\"\\,]*|*$'\n'*|*$'\r'*) fail '保存先に $, バッククォート, 二重引用符, バックスラッシュ, カンマ, 改行は使えません'; return 1 ;;
    esac
}

configure_storage() {
    local task_config task_record task_result task_default_record task_default_result task_input
    task_config="${ATD_STORAGE_CONFIG:-${task_project_dir}/config/storage.env}"
    if $task_dry_run; then
        printf '保存先を設定予定: %s\nrecord: %s\nresults: %s\n' "$task_config" \
            "${task_record_override:-既存値または ${task_project_dir}/data/record}" \
            "${task_result_override:-既存値または ${task_project_dir}/data/results}"
        return 0
    fi
    command -v python3 >/dev/null 2>&1 || { fail '先に tools を実行して python3 を導入してください'; return 1; }
    if [[ -f "$task_config" ]]; then
        # dev.sh と同じユーザー管理のローカル Bash 設定。
        source "$task_config" || return $?
    fi
    if [[ "$task_host" = pc ]]; then
        task_default_record="${ATD_PC_RECORD_DIR:-${task_project_dir}/data/record}"
        task_default_result="${ATD_PC_RESULT_DIR:-${task_project_dir}/data/results}"
    else
        task_default_record="${ATD_JETSON_RECORD_DIR:-${task_project_dir}/data/record}"
        task_default_result="${ATD_JETSON_RESULT_DIR:-${task_project_dir}/data/results}"
    fi
    task_record="${task_record_override:-${ATD_RECORD_DIR:-$task_default_record}}"
    task_result="${task_result_override:-${ATD_RESULT_DIR:-$task_default_result}}"
    if [[ -t 0 && -z "$task_record_override" ]]; then
        read -r -p "record 保存先（rosbag）[$task_record]: " task_input || return 1
        task_record="${task_input:-$task_record}"
    fi
    if [[ -t 0 && -z "$task_result_override" ]]; then
        read -r -p "results 保存先（地図・モデル・解析）[$task_result]: " task_input || return 1
        task_result="${task_input:-$task_result}"
    fi
    case "$task_record" in '~/'*) task_record="${HOME}/${task_record:2}" ;; esac
    case "$task_result" in '~/'*) task_result="${HOME}/${task_result:2}" ;; esac
    validate_storage_path "$task_record" || return $?
    validate_storage_path "$task_result" || return $?
    mkdir -p -- "$task_record" "$task_result" "$(dirname -- "$task_config")" || return $?
    python3 - "$task_config" "${task_project_dir}/config/storage.env.example" "$task_host" "$task_record" "$task_result" <<'PY'
import os
from pathlib import Path
import re
import shlex
import sys
import tempfile
config, example = Path(sys.argv[1]), Path(sys.argv[2])
host, record, results = sys.argv[3], Path(sys.argv[4]).resolve(), Path(sys.argv[5]).resolve()
for path in (record, results):
    if not os.access(path, os.R_OK | os.W_OK | os.X_OK):
        sys.exit('保存先を読み書きできません: ' + str(path))
content = config.read_text() if config.exists() else example.read_text()
for suffix, value in [('RECORD_DIR', record), ('RESULT_DIR', results)]:
    key = 'ATD_' + host.upper() + '_' + suffix
    pattern = r'^(?:export\s+)?' + key + r'\s*=.*$'
    replacement = key + '=' + shlex.quote(str(value))
    if re.search(pattern, content, re.MULTILINE):
        content = re.sub(pattern, lambda match: replacement, content, flags=re.MULTILINE)
    else:
        content = content.rstrip('\n') + '\n' + replacement + '\n'
content = content.rstrip('\n') + '\n'
if not config.exists() or config.read_text() != content:
    with tempfile.NamedTemporaryFile(dir=config.parent, delete=False) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(content.encode())
            stream.flush()
            os.fchmod(stream.fileno(), 0o600)
            os.replace(temporary, config)
        finally:
            temporary.unlink(missing_ok=True)
os.chmod(config, 0o600)
print('設定: ' + str(config))
print('record: ' + str(record))
print('results: ' + str(results))
PY
}

resolve_setup_workspace() {
    if $task_dry_run; then
        # dry-run ではユーザーのローカル Bash 設定を source しない。
        ATD_RESOLVED_ROS_WS="${task_workspace_override:-${ATD_ROS_WS:-${task_project_dir}/ros2_ws}}"
    else
        atd_load_workspace "$task_project_dir" "$task_host" || return $?
        ATD_RESOLVED_ROS_WS="${task_workspace_override:-$ATD_RESOLVED_ROS_WS}"
    fi
    case "$ATD_RESOLVED_ROS_WS" in '~/'*) ATD_RESOLVED_ROS_WS="${HOME}/${ATD_RESOLVED_ROS_WS:2}" ;; esac
}

show_workspace() {
    local task_display_workspace="${1:-}"
    if [[ -z "$task_display_workspace" ]]; then
        resolve_setup_workspace || return $?
        task_display_workspace="$ATD_RESOLVED_ROS_WS"
    fi
    printf 'ROS workspace（ホスト）: %s\n' "$task_display_workspace"
    printf 'ROS workspace（Docker）: /workspaces/isaac_ros-dev\n'
    if [[ "${task_display_workspace}/src" -ef "${task_project_dir}/ros2_ws/src" ]]; then
        printf 'ROS source（編集元）: %s/ros2_ws/src -> /workspaces/isaac_ros-dev/src (read-write)\n' "$task_project_dir"
    else
        printf 'ROS source（編集元）: %s/ros2_ws/src -> /workspaces/isaac_ros-dev/src/atd (read-write)\n' "$task_project_dir"
    fi
    printf 'ATD checkout（Docker）: /workspaces/atd (read-write)\n'
    printf 'workspace 設定（%s）: %s\n' "$task_host" "${ATD_WORKSPACE_CONFIG:-${task_project_dir}/config/workspace.env}"
    printf 'ISAAC_ROS_WS は dev.sh が設定します。ホストの bashrc への手動 export は不要です。\n'
}

prepare_workspace() {
    local task_workspace task_input
    resolve_setup_workspace || return $?
    task_workspace="$ATD_RESOLVED_ROS_WS"
    if $task_dry_run; then
        show_workspace || return $?
        if [[ "${task_workspace}/src" -ef "${task_project_dir}/ros2_ws/src" ]]; then
            run_command mkdir -p -- "${task_workspace}/src"
        else
            run_command mkdir -p -- "${task_workspace}/src/atd"
        fi
        printf 'workspace のパスを対話的に指定し、選択ホストの設定と CLI メタデータを保存する予定です。\n'
        return 0
    fi
    if [[ -t 0 && -z "$task_workspace_override" ]]; then
        read -r -p "ROS workspace（${task_host}）[$task_workspace]: " task_input || return 1
        task_workspace="${task_input:-$task_workspace}"
    fi
    case "$task_workspace" in '~/'*) task_workspace="${HOME}/${task_workspace:2}" ;; esac
    atd_validate_workspace_path "$task_workspace" || return $?
    atd_prepare_workspace "$task_project_dir" "$task_workspace" || return $?
    atd_save_workspace "$task_project_dir" "$task_host" "$task_workspace" || return $?
    show_workspace "$task_workspace" || return $?
    printf 'workspace を準備しました。Notebook / Jetson の build・install は共有しません。\n'
}

verify_environment() {
    local task_status task_mode
    if $task_dry_run; then
        run_command docker info
        run_command isaac-ros status --output json
        run_command docker run --rm --gpus all ubuntu:24.04 /bin/true
        return 0
    fi
    command -v docker >/dev/null 2>&1 || { fail '先に docker を実行してください'; return 1; }
    command -v isaac-ros >/dev/null 2>&1 || { fail '先に isaac を実行してください'; return 1; }
    source "${task_project_dir}/scripts/lib/isaac_cli.sh" || return 1
    atd_check_isaac_cli || return 1
    command -v python3 >/dev/null 2>&1 || { fail '先に tools を実行してください'; return 1; }
    if ! docker info --format 'Docker runtime: {{.DefaultRuntime}}'; then
        fail 'Docker に接続できません。サービスと docker グループの反映を確認し、必要なら再ログインしてください'
        return 1
    fi
    task_status="$(isaac-ros status --output json)" || { fail 'Isaac ROS CLI の状態を取得できません'; return 1; }
    task_mode="$(python3 -c 'import json,sys; print(json.load(sys.stdin)["mode"])' <<< "$task_status")" || return 1
    [[ "$task_mode" = docker ]] || { fail "CLI は Docker モードではありません: $task_mode"; return 1; }
    run_command docker run --rm --gpus all ubuntu:24.04 /bin/true || return $?
    printf 'GPU runtime でのコンテナ起動を確認しました。CUDA 演算・D455・Joy の確認は別途行います。\n'
}

run_action() {
    local task_selected="$1" task_step
    local -a task_dev_command
    printf '\n--- %s ---\n' "$task_selected"
    if ! $task_dry_run && [[ "$task_selected" != check && "$(id -u)" = 0 ]]; then
        fail 'setup.sh 全体を sudo で起動せず、通常ユーザーから実行してください'
        return 1
    fi
    case "$task_selected" in
        check)
            run_command bash "${task_project_dir}/scripts/setup/host.sh" --host "$task_host" --action check || return $?
            show_workspace
            ;;
        tools)
            run_command bash "${task_project_dir}/scripts/setup/host.sh" --host "$task_host" --action tools || return $?
            install_launcher
            ;;
        launcher) install_launcher ;;
        docker|runtime|isaac|devices)
            run_command bash "${task_project_dir}/scripts/setup/host.sh" --host "$task_host" --action "$task_selected"
            ;;
        storage) configure_storage ;;
        workspace) prepare_workspace ;;
        verify) verify_environment ;;
        web)
            [[ "$task_host" = pc ]] || { fail 'Web UI の事前準備は Notebook 専用です'; return 1; }
            run_command bash "${task_project_dir}/scripts/webui.sh" prepare
            ;;
        build|start)
            task_dev_command=(bash "${task_project_dir}/scripts/dev.sh")
            if [[ -n "$task_workspace_override" ]]; then
                task_dev_command=(env "ATD_ROS_WS=$task_workspace_override" "${task_dev_command[@]}")
            fi
            if [[ "$task_host_selection" = manual ]]; then task_dev_command+=(--host "$task_host"); fi
            if [[ "$task_selected" = build ]]; then task_dev_command+=(--build); fi
            run_command "${task_dev_command[@]}"
            ;;
        all)
            for task_step in tools docker runtime isaac devices storage workspace verify build; do
                run_action "$task_step" || return $?
            done
            ;;
    esac
}

choose_action() {
    local task_choices task_choice task_number task_index=0 task_label
    task_choices=$'all\t初回セットアップ一式\ncheck\t現在の環境を確認\ntools\t基本ツール・fzf を準備\ndocker\tDocker を準備\nruntime\tNVIDIA GPU runtime を準備\nisaac\tIsaac ROS CLI を準備\nstorage\t記録・解析の保存先を設定\nworkspace\tROS workspace のパス・設定を準備\ndevices\tD455・Bluetooth 等を準備\nverify\tDocker・CLI・GPU runtime を確認\nbuild\tATD を build して入室\nstart\tATD に入室・再接続'
    if [[ "$task_host" = pc ]]; then
        task_choices+=$'\nweb\tWeb UI のオフライン利用を準備'
    fi
    task_choices+=$'\nlauncher\tatd コマンドをホームに登録\nquit\t終了'
    if command -v fzf >/dev/null 2>&1; then
        task_choice="$(printf '%s\n' "$task_choices" | fzf --delimiter=$'\t' --with-nth=2 \
            --height=75% --border --prompt='ATD setup > ' --header="${task_host}: 実行する操作を選択")" || return 1
        printf '%s\n' "${task_choice%%$'\t'*}"
        return 0
    fi
    printf 'fzf 未導入のため番号で選択します。tools の実行後は fzf を使用します。\n' >&2
    while IFS=$'\t' read -r task_choice task_label; do
        task_index=$((task_index + 1))
        printf '%2d) %s\n' "$task_index" "$task_label" >&2
    done <<< "$task_choices"
    read -r -p '番号（空欄で終了）: ' task_number || return 1
    [[ -n "$task_number" ]] || return 1
    [[ "$task_number" =~ ^[0-9]+$ && ${#task_number} -le 2 ]] || return 1
    task_number=$((10#$task_number))
    ((task_number >= 1 && task_number <= task_index)) || return 1
    printf '%s\n' "$task_choices" | awk -F '\t' -v item="$task_number" 'NR==item {print $1}'
}

if [[ "$task_action" = menu ]]; then
    [[ -t 0 && -t 1 ]] || { fail '対話端末で起動するか、--action と --dry-run を指定してください'; exit 1; }
    while task_choice="$(choose_action)"; do
        [[ "$task_choice" != quit ]] || break
        if ! run_action "$task_choice"; then
            printf 'この段階で停止しました。上の内容を確認して、必要な操作から再実行してください。\n' >&2
        fi
    done
else
    run_action "$task_action"
fi

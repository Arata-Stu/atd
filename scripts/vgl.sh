#!/usr/bin/env bash
# VGL モデルの Notebook export → SCP → Jetson engine build の短い入口。
set -uo pipefail

usage() {
    cat <<'EOF'
使い方: scripts/vgl.sh [--host notebook|pc|jetson] [--dry-run] [ACTION] [OPTIONS]

引数なし: 操作後に戻る fzf メニュー（未導入時は番号選択）
ACTION: doctor, export, transfer, build, verify, list, logs
  doctor / export  Notebook の起動済み ATD Docker で確認・ONNX export
  transfer         export bundle を SCP 転送し、Jetson 側で検証して保存
  build            Jetson で TensorRT engine build（Notebook からは SSH 委譲）
  verify / list    このホストの Docker で資産を検証・一覧表示
  logs             保存した実行ログを選択（非対話端末では最新を表示）

OPTIONS: --asset-id ID --export-id ID --build-id ID --target-id ID
         --mode aliked|package --width N --height N --source-dir PATH
--dry-run: 予定だけ表示。ローカル設定の読込み・Docker/SSH/SCP・保存は行わない。
接続先: config/vgl.env（初回 transfer / Notebook build で対話保存）
結果先: config/storage.env のホスト別 RESULT_DIR（ATD_RESULT_DIR で上書き）
ログ: logs/vgl/<host>/。コンテナは自動起動しない。先に scripts/dev.sh で起動する。
EOF
}
fail() { printf 'VGL: %s\n' "$*" >&2; exit 1; }
task_host=''
task_action=''
task_asset=''
task_export=''
task_build=''
task_target=''
task_mode=''
task_width=''
task_height=''
task_source=''
task_dry_run=false
while (($#)); do
    case "$1" in
        --host|--asset-id|--export-id|--build-id|--target-id|--mode|--width|--height|--source-dir)
            (($# >= 2)) && [[ -n "$2" ]] || fail "$1 の値が必要です"
            case "$1" in
                --host) task_host="$2" ;; --asset-id) task_asset="$2" ;;
                --export-id) task_export="$2" ;; --build-id) task_build="$2" ;;
                --target-id) task_target="$2" ;; --mode) task_mode="$2" ;;
                --width) task_width="$2" ;; --height) task_height="$2" ;;
                --source-dir) task_source="$2" ;;
            esac
            shift 2 ;;
        --dry-run) task_dry_run=true; shift ;;
        -h|--help) usage; exit 0 ;;
        doctor|export|transfer|build|verify|list|logs)
            [[ -z "$task_action" ]] || fail 'ACTION は一つ指定してください'
            task_action="$1"; shift ;;
        *) fail "不明な引数: $1" ;;
    esac
done
[[ "$task_host" != notebook ]] || task_host=pc
case "$task_host" in ''|pc|jetson) ;; *) fail '--host は notebook、pc、jetson から指定してください' ;; esac
task_project="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)" || exit 1
if [[ -z "$task_host" ]]; then
    source "${task_project}/scripts/lib/detect_host.sh" || exit 1
    task_host="$(detect_atd_host)" || exit 1
fi
if $task_dry_run; then
    printf '[dry-run] host: %s\naction: %s\n' "$task_host" "${task_action:-対話メニュー}"
    printf 'export/doctor: Notebook Docker、transfer: SCP＋検証、build: Jetson Docker（PC は SSH 委譲）。\n'
    printf '指定: asset=%s export=%s build=%s target=%s mode=%s size=%sx%s source=%s\n' \
        "$task_asset" "$task_export" "$task_build" "$task_target" "$task_mode" "$task_width" "$task_height" "$task_source"
    exit 0
fi

pick() {
    local task_prompt="$1" task_items="$2" task_choice task_number task_count
    if command -v fzf >/dev/null 2>&1; then
        printf '%s\n' "$task_items" | fzf --height=75% --border --prompt="$task_prompt > "
        return $?
    fi
    task_count=0
    while IFS= read -r task_choice; do
        task_count=$((task_count + 1)); printf '%2d) %s\n' "$task_count" "$task_choice" >&2
    done <<< "$task_items"
    read -r -p "$task_prompt 番号（空欄で終了）: " task_number || return 1
    [[ "$task_number" =~ ^[0-9]+$ && ${#task_number} -le 3 ]] || return 1
    task_number=$((10#$task_number))
    ((task_number >= 1 && task_number <= task_count)) || return 1
    printf '%s\n' "$task_items" | awk -v n="$task_number" 'NR==n {print; exit}'
}

append_selected_arguments() {
    if [[ -n "$task_asset" ]]; then task_arguments+=(--asset-id "$task_asset"); fi
    if [[ -n "$task_export" ]]; then task_arguments+=(--export-id "$task_export"); fi
    if [[ -n "$task_build" ]]; then task_arguments+=(--build-id "$task_build"); fi
    if [[ -n "$task_target" ]]; then task_arguments+=(--target-id "$task_target"); fi
    if [[ -n "$task_mode" ]]; then task_arguments+=(--mode "$task_mode"); fi
    if [[ -n "$task_width" ]]; then task_arguments+=(--width "$task_width"); fi
    if [[ -n "$task_height" ]]; then task_arguments+=(--height "$task_height"); fi
    if [[ -n "$task_source" ]]; then task_arguments+=(--source-dir "$task_source"); fi
}

if [[ -z "$task_action" ]]; then
    [[ -t 0 && -t 1 ]] || fail '対話端末で起動するか ACTION を指定してください'
    task_arguments=(--host "$task_host")
    append_selected_arguments
    while task_choice="$(pick 'VGL' $'doctor   Notebook export 環境を確認\nexport   Notebook で ONNX を準備\ntransfer SCP で Jetson へ転送\nbuild    Jetson で engine build\nverify   保存済み資産を検証\nlist     資産一覧\nlogs     実行ログ\nquit     終了')"; do
        task_selected="${task_choice%% *}"
        case "$task_selected" in
            quit) break ;;
            doctor|export|transfer|build|verify|list|logs) ;;
            *) printf '一覧にある操作を選択してください。\n' >&2; continue ;;
        esac
        # 子は通常の直接実行と同じ経路を使い、TTY と終了値を保持する。
        bash "${task_project}/scripts/vgl.sh" "${task_arguments[@]}" "$task_selected"
        task_exit=$?
        if [[ "$task_exit" != 0 ]]; then
            printf 'VGL: %s は終了値 %s で停止しました。内容を確認し、メニューから再実行できます。\n' "$task_selected" "$task_exit" >&2
        fi
    done
    exit 0
fi

task_log_dir="${task_project}/logs/vgl/${task_host}"
if [[ "$task_action" == logs ]]; then
    task_latest=''
    task_items=''
    for task_path in "$task_log_dir"/*.log; do
        [[ -f "$task_path" ]] || continue
        task_items="${task_items}${task_path}"$'\n'
        if [[ -z "$task_latest" || "$task_path" -nt "$task_latest" ]]; then task_latest="$task_path"; fi
    done
    [[ -n "$task_latest" ]] || fail "保存済みログがありません: $task_log_dir"
    if [[ -t 0 && -t 1 ]]; then
        task_choice="$(pick 'VGL logs' "${task_items%$'\n'}")" || exit 0
        case "$task_choice" in "$task_log_dir"/*.log) ;; *) fail '列挙したログを選択してください' ;; esac
        # fzf 出力も検査し、別ディレクトリを含むパスを開かない。
        [[ "$(dirname -- "$task_choice")" == "$task_log_dir" && -f "$task_choice" ]] || fail 'ログがありません'
        cat -- "$task_choice"
    else printf '最新ログ: %s\n' "$task_latest"; cat -- "$task_latest"; fi
    exit $?
fi

command -v python3 >/dev/null 2>&1 || fail 'python3 が必要です。setup.sh の tools を実行してください'
task_storage="${ATD_STORAGE_CONFIG:-${task_project}/config/storage.env}"
if [[ -f "$task_storage" ]]; then source "$task_storage" || fail '保存先設定を読めません'
elif [[ -n "${ATD_STORAGE_CONFIG:-}" ]]; then fail "保存先設定がありません: $task_storage"; fi
if [[ "$task_host" == pc ]]; then task_results="${ATD_RESULT_DIR:-${ATD_PC_RESULT_DIR:-}}"
else task_results="${ATD_RESULT_DIR:-${ATD_JETSON_RESULT_DIR:-}}"; fi
[[ "$task_results" == /* && -d "$task_results" && -r "$task_results" && -w "$task_results" && -x "$task_results" ]] \
    || fail '結果保存先を設定し、読書き可能なディレクトリを作成してください（setup.sh の storage）'
task_results="$(CDPATH= cd -- "$task_results" && pwd -P)" || exit 1
case "$task_results" in *[\$\`\"\\,]*|*$'\n'*|*$'\r'*) fail '結果保存先に Docker mount で使用できない文字が含まれています' ;; esac

valid_id() { [[ "$1" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ && ${#1} -le 96 ]]; }
read_value() {
    local task_prompt="$1" task_default="$2" task_value
    [[ -t 0 ]] || fail "$task_prompt を引数で指定してください"
    read -r -p "$task_prompt [$task_default]: " task_value || return 1
    printf '%s\n' "${task_value:-$task_default}"
}
choose_existing() {
    local task_kind="$1" task_base="$2" task_path task_items=''
    [[ -t 0 ]] || fail "--${task_kind}-id を指定してください"
    for task_path in "$task_base"/*; do
        [[ -d "$task_path" ]] || continue
        valid_id "${task_path##*/}" || continue
        if [[ "$task_kind" == export && ! -f "$task_path/manifest.json" ]]; then continue; fi
        task_items="${task_items}${task_path##*/}"$'\n'
    done
    [[ -n "$task_items" ]] || fail "選択できる $task_kind がありません: $task_base"
    pick "$task_kind" "${task_items%$'\n'}"
}

case "$task_action" in
    export)
        [[ "$task_host" == pc ]] || fail 'モデル準備・export は Notebook で実行してください'
        if [[ -z "$task_asset" ]]; then task_asset="$(read_value asset-id aliked_lightglue)" || exit 1; fi
        if [[ -z "$task_export" ]]; then task_export="$(read_value export-id "$(date -u '+%Y%m%dT%H%M%SZ')")" || exit 1; fi
        if [[ -z "$task_mode" ]]; then
            if [[ -t 0 ]]; then
                task_choice="$(pick mode $'aliked  ALIKED を指定解像度で export\npackage 既存 ONNX を検証・可搬化')" || exit 1
                task_mode="${task_choice%% *}"
            else task_mode=aliked; fi
        fi
        # package は既存 ONNX の実 shape を使う。明示指定した場合だけ比較する。
        if [[ "$task_mode" == aliked ]]; then
            if [[ -t 0 ]]; then
                if [[ -z "$task_width" && -z "$task_height" ]]; then
                    task_choice="$(pick resolution $'848x480\n424x240\n640x480\n1280x720\n1920x1200\ncustom')" || exit 1
                    if [[ "$task_choice" != custom ]]; then
                        task_width="${task_choice%x*}"; task_height="${task_choice#*x}"
                    fi
                fi
                if [[ -z "$task_width" ]]; then task_width="$(read_value width 848)" || exit 1; fi
                if [[ -z "$task_height" ]]; then task_height="$(read_value height 480)" || exit 1; fi
            else task_width="${task_width:-848}"; task_height="${task_height:-480}"; fi
        fi
        ;;
    transfer|build|verify)
        if [[ -z "$task_asset" ]]; then task_asset="$(choose_existing asset "$task_results/models/vgl")" || exit 1; fi
        valid_id "$task_asset" || fail 'asset ID は96文字以内の英数字・._-で指定してください'
        if [[ -z "$task_export" ]]; then task_export="$(choose_existing export "$task_results/models/vgl/$task_asset/exported")" || exit 1; fi
        if [[ "$task_action" == build ]]; then
            if [[ -z "$task_target" ]]; then
                if [[ -t 0 ]]; then task_target="$(read_value target-id jetson_orin_nano)" || exit 1
                else task_target=jetson_orin_nano; fi
            fi
            if [[ -z "$task_build" && -t 0 ]]; then task_build="$(read_value build-id "$(date -u '+%Y%m%dT%H%M%SZ')")" || exit 1; fi
        fi
        ;;
esac
for task_id in "$task_asset" "$task_export" "$task_build" "$task_target"; do
    [[ -z "$task_id" ]] || valid_id "$task_id" || fail 'ID は96文字以内の英数字・._-で指定してください'
done
case "$task_mode" in ''|aliked|package) ;; *) fail '--mode は aliked または package です' ;; esac
for task_size in "$task_width" "$task_height"; do
    [[ -z "$task_size" || "$task_size" =~ ^[1-9][0-9]*$ ]] || fail 'width / height は正の整数です'
done

configure_remote() {
    local task_config="${ATD_VGL_CONFIG:-${task_project}/config/vgl.env}" task_save=false task_input task_key task_value
    if [[ -f "$task_config" ]]; then source "$task_config" || fail 'VGL 接続設定を読めません'; fi
    ATD_VGL_SSH_TARGET="${ATD_VGL_SSH_TARGET:-}"
    ATD_VGL_REMOTE_PROJECT="${ATD_VGL_REMOTE_PROJECT:-}"
    ATD_VGL_REMOTE_RESULT_DIR="${ATD_VGL_REMOTE_RESULT_DIR:-}"
    ATD_VGL_SSH_PORT="${ATD_VGL_SSH_PORT:-22}"
    for task_key in ATD_VGL_SSH_TARGET ATD_VGL_REMOTE_PROJECT ATD_VGL_REMOTE_RESULT_DIR; do
        case "$task_key" in
            ATD_VGL_SSH_TARGET) task_value="$ATD_VGL_SSH_TARGET" ;;
            ATD_VGL_REMOTE_PROJECT) task_value="$ATD_VGL_REMOTE_PROJECT" ;;
            ATD_VGL_REMOTE_RESULT_DIR) task_value="$ATD_VGL_REMOTE_RESULT_DIR" ;;
        esac
        if [[ -z "$task_value" ]]; then
            [[ -t 0 ]] || fail "$task_key が未設定です。対話端末で実行するか config/vgl.env を設定してください"
            read -r -p "$task_key: " task_input || return 1
            [[ -n "$task_input" ]] || fail "$task_key は空欄にできません"
            case "$task_key" in
                ATD_VGL_SSH_TARGET) ATD_VGL_SSH_TARGET="$task_input" ;;
                ATD_VGL_REMOTE_PROJECT) ATD_VGL_REMOTE_PROJECT="$task_input" ;;
                ATD_VGL_REMOTE_RESULT_DIR) ATD_VGL_REMOTE_RESULT_DIR="$task_input" ;;
            esac
            task_save=true
        fi
    done
    if $task_save; then
        read -r -p "SSH port [$ATD_VGL_SSH_PORT]: " task_input || return 1
        ATD_VGL_SSH_PORT="${task_input:-$ATD_VGL_SSH_PORT}"
    fi
    python3 - "$task_project" "$ATD_VGL_SSH_TARGET" "$ATD_VGL_REMOTE_PROJECT" "$ATD_VGL_REMOTE_RESULT_DIR" "$ATD_VGL_SSH_PORT" <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, sys.argv[1])
from tools.vgl.transfer import validate_target, validate_remote_path, validate_port
validate_target(sys.argv[2])
validate_remote_path(sys.argv[3])
validate_remote_path(sys.argv[4])
validate_port(sys.argv[5])
PY
    [[ "$?" == 0 ]] || return 1
    if $task_save; then
        python3 - "$task_config" "$ATD_VGL_SSH_TARGET" "$ATD_VGL_REMOTE_PROJECT" "$ATD_VGL_REMOTE_RESULT_DIR" "$ATD_VGL_SSH_PORT" <<'PY'
import os
from pathlib import Path
import re
import shlex
import sys
import tempfile
config = Path(sys.argv[1])
content = config.read_text() if config.exists() else '# VGL SSH connection (local only)\n'
keys = ('ATD_VGL_SSH_TARGET', 'ATD_VGL_REMOTE_PROJECT', 'ATD_VGL_REMOTE_RESULT_DIR', 'ATD_VGL_SSH_PORT')
for key, value in zip(keys, sys.argv[2:]):
    replacement = key + '=' + shlex.quote(value)
    pattern = r'^(?:export\s+)?' + key + r'\s*=.*$'
    if re.search(pattern, content, re.MULTILINE):
        content = re.sub(pattern, lambda _: replacement, content, flags=re.MULTILINE)
    else:
        content = content.rstrip('\n') + '\n' + replacement + '\n'
config.parent.mkdir(parents=True, exist_ok=True)
with tempfile.NamedTemporaryFile(dir=config.parent, delete=False) as stream:
    temporary = Path(stream.name)
    try:
        stream.write(content.encode())
        stream.flush()
        os.fchmod(stream.fileno(), 0o600)
        os.replace(temporary, config)
    finally:
        temporary.unlink(missing_ok=True)
print('VGL 接続設定: ' + str(config))
PY
    fi
}

run_logged() {
    local task_log
    umask 077
    mkdir -p -- "$task_log_dir" || return 1
    task_log="${task_log_dir}/$(date -u '+%Y%m%dT%H%M%SZ')-${task_action}-$$.log"
    printf 'VGL host=%s action=%s UTC=%s\n' "$task_host" "$task_action" "$(date -u '+%Y-%m-%d %H:%M:%S')" | tee "$task_log"
    "$@" 2>&1 | tee -a "$task_log"
    local -a task_status=("${PIPESTATUS[@]}")
    printf '終了値: %s\n' "${task_status[0]}" | tee -a "$task_log"
    printf 'ログ: %s\n' "$task_log"
    [[ "${task_status[1]}" == 0 ]] || return 1
    return "${task_status[0]}"
}

task_arguments=("$task_action")
append_selected_arguments

if [[ "$task_action" == transfer || ( "$task_action" == build && "$task_host" == pc ) ]]; then
    configure_remote || exit 1
    if [[ "$task_action" == transfer ]]; then
        run_logged python3 "$task_project/tools/vgl/transfer.py" send --bundle "$task_results/models/vgl/$task_asset/exported/$task_export" \
            --ssh-target "$ATD_VGL_SSH_TARGET" --port "$ATD_VGL_SSH_PORT" --remote-results "$ATD_VGL_REMOTE_RESULT_DIR"
    else
        # remote entrypoint が Jetson の起動済みコンテナ・保存先を再確認する。
        run_logged python3 "$task_project/tools/vgl/transfer.py" remote-build --ssh-target "$ATD_VGL_SSH_TARGET" \
            --port "$ATD_VGL_SSH_PORT" --remote-project "$ATD_VGL_REMOTE_PROJECT" -- "${task_arguments[@]:1}"
    fi
    exit $?
fi

command -v docker >/dev/null 2>&1 || fail 'Docker CLI がありません'
source "${task_project}/scripts/lib/runtime.sh" || exit 1
task_revision="$(atd_runtime_revision "$task_project")" || exit 1
task_container="atd_${task_host}_dev"
task_inspect="$(docker container inspect "$task_container" 2>/dev/null)" || fail "$task_container を先に dev.sh で起動してください"
printf '%s' "$task_inspect" | python3 -c '
import json, os, sys
try:
    container = json.load(sys.stdin)[0]
    if not container.get("State", {}).get("Running"):
        sys.exit("VGL: ATD コンテナは停止中です")
    env = dict(item.split("=", 1) for item in container.get("Config", {}).get("Env", []) if "=" in item)
    if env.get("ATD_RUNTIME_API") != "1" or env.get("ATD_RUNTIME_REV") != sys.argv[1]:
        sys.exit("VGL: コンテナの起動構成が古いか異なります。処理を終了後に dev.sh --build で再作成してください")
    mounts = {mount["Destination"]: mount for mount in container.get("Mounts", [])}
    for destination, source, writable in [("/workspaces/atd", sys.argv[2], False), ("/data/results", sys.argv[3], True)]:
        mount = mounts.get(destination, {})
        if mount.get("Type") != "bind" or mount.get("RW") is not writable or os.path.realpath(mount.get("Source", "")) != os.path.realpath(source):
            sys.exit("VGL: 起動済みコンテナの project / results mount が現在の設定と異なります")
except (ValueError, KeyError, TypeError, IndexError):
    sys.exit("VGL: Docker inspect を解釈できません")
' "$task_revision" "$task_project" "$task_results" || exit 1
task_ready="$(docker exec "$task_container" cat /etc/atd/runtime.ready 2>/dev/null)" || fail 'コンテナ初期設定が未完了です'
[[ "$task_ready" == "$task_revision" ]] || fail 'コンテナ初期設定が未完了または古い構成です'
task_command=(docker exec -i)
if [[ -t 0 && -t 1 ]]; then task_command+=(-t); fi
task_command+=(-u admin --workdir /workspaces/isaac_ros-dev "$task_container" /usr/local/bin/atd-shell /usr/local/bin/atd-vgl)
run_logged "${task_command[@]}" "${task_arguments[@]}"

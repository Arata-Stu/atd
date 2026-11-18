#!/usr/bin/env bash
# 起動構成を選び、既存 Docker 内の ROS 環境で package の launch を実行する。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: scripts/bringup.sh [--dry-run] [input|bridge] [name:=value ...]

引数なし: fzf で起動構成を選択（未導入時は番号選択）
  input      Jetson の Joy・Pico入力、役割処理、観測・設定bridge
  bridge     観測・設定bridgeのみ。入力・役割 node は別途起動する
  --list     現在の起動構成を表示
  --dry-run  起動せず予定を表示。構成省略時は input。Notebookでも利用可能

例: scripts/bringup.sh input
    scripts/bringup.sh input pico_serial:=PICO_SERIAL rc_profile:=/data/results/runs/input/propo.yaml
    scripts/bringup.sh bridge config:=/absolute/path/bridge.yaml
    scripts/bringup.sh --dry-run input propo:=false

起動済み Docker を使用します。先に scripts/dev.sh で起動してください。
現在の構成は Jetson 上で実行します。Notebook の Web UI は Docker 内の atd-web を使います。
基板への走行指令・bag 記録は開始しません。終了は Ctrl-C。
EOF
}
fail() { printf 'ATD Bringup: %s\n' "$*" >&2; exit 1; }

task_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
task_profile=''
task_dry_run=false
task_list=false
task_launch_arguments=()
while (($#)); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --dry-run) task_dry_run=true; shift ;;
        --list) task_list=true; shift ;;
        --) shift; task_launch_arguments+=("$@"); break ;;
        --*) fail "不明な引数: $1（--help を参照）" ;;
        *:=*) task_launch_arguments+=("$1"); shift ;;
        *)
            [[ -z "$task_profile" ]] || fail '起動構成は一つだけ指定してください'
            task_profile="$1"
            shift ;;
    esac
done
source "$task_root/scripts/lib/bringup.sh"
if $task_list; then atd_bringup_list; exit 0; fi
if [[ -z "$task_profile" ]]; then
    if $task_dry_run; then
        task_profile=input
    else
        [[ -t 0 && -t 1 ]] || fail '対話端末で起動するか、input / bridge を指定してください'
        task_profile="$(atd_bringup_choose)" || exit 0
        [[ "$task_profile" != quit ]] || exit 0
    fi
fi
task_target="$(atd_bringup_lookup "$task_profile")" || fail "未登録の起動構成: ${task_profile}（--list を参照）"
IFS=$'\t' read -r task_host task_package task_launch <<< "$task_target"
source "$task_root/scripts/lib/detect_host.sh"
if ! $task_dry_run && [[ "$(detect_atd_host)" != "$task_host" ]]; then
    fail "${task_profile} は ${task_host} 上で起動してください。Notebook の UI は Docker 内の atd-web を使います"
fi
task_command=(bash "$task_root/scripts/tools/python.sh" --host "$task_host")
if $task_dry_run; then task_command+=(--dry-run); fi
task_command+=(ros run -- ros2 launch "$task_package" "$task_launch")
if ((${#task_launch_arguments[@]})); then task_command+=("${task_launch_arguments[@]}"); fi
exec "${task_command[@]}"

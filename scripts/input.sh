#!/usr/bin/env bash
# Jetson の入力取得・操作要求・監視 bridge を起動する。ROS 環境は既存入口へ委譲する。
set -euo pipefail
task_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
task_dry_run=false
task_launch_arguments=(ros2 launch atd_web_bridge input.launch.py)
for task_argument in "$@"; do
    case "$task_argument" in
        -h|--help)
            printf '使い方: scripts/input.sh [--bridge-only] [--dry-run] [name:=value ...]\nJetson の起動済み Docker で Joy・Pico USB 入力・役割処理・loopback bridge を起動します。基板出力・bag 記録は開始しません。\nteleop:=false / propo:=false で各役割処理または Pico 入力を無効化できます。複数 Pico では pico_serial:=<ID> で選択します。\n設定ファイル: profile:=<Joy YAML> teleop_profile:=<teleop YAML> rc_profile:=<propo YAML>。Notebook は --dry-run のみ利用できます。\n'
            exit 0 ;;
        --dry-run) task_dry_run=true ;;
        --bridge-only) task_launch_arguments[3]=bridge.launch.py ;;
        --*) printf 'ATD Input: 不明な引数: %s\n' "$task_argument" >&2; exit 1 ;;
        *) task_launch_arguments+=("$task_argument") ;;
    esac
done
source "$task_root/scripts/detect_host.sh"
if [[ "$(detect_atd_host)" != jetson ]] && ! $task_dry_run; then
    printf 'ATD Input: 入力取得と ROS bridge は Jetson で起動してください。Notebook の UI は scripts/webui.sh を使います。\n' >&2
    exit 1
fi
task_command=(bash "$task_root/scripts/python.sh" --host jetson)
if $task_dry_run; then task_command+=(--dry-run); fi
task_command+=(ros run -- "${task_launch_arguments[@]}")
exec "${task_command[@]}"

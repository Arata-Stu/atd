#!/usr/bin/env bash
# 起動構成を選び、既存 Docker 内の ROS 環境で package の launch を実行する。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: scripts/bringup.sh [--dry-run] [--configure] [input|bridge|d455|manual] [name:=value ...]

引数なし: fzf で起動構成を選択（未導入時は番号選択）
  input      Jetson の Joy・Pico入力、入力記録待機、観測・設定bridge
  bridge     観測・設定bridgeのみ。入力・役割 node は別途起動する
  d455       D455・車両TF（bridgeなし）
  manual     D455・車両TF・input一式・地図用記録待機（bridgeは1つ）
  --configure D455設定TUIを明示して開く（対話端末専用）
  --list     現在の起動構成を表示
  --dry-run  起動せず予定を表示。構成省略時は input。Notebookでも利用可能

例: scripts/bringup.sh input
    scripts/bringup.sh input pico_serial:=PICO_SERIAL rc_profile:=/data/results/runs/input/propo.yaml
    scripts/bringup.sh bridge config:=/absolute/path/bridge.yaml
    scripts/bringup.sh --dry-run input propo:=false
    scripts/bringup.sh d455
    scripts/bringup.sh manual camera_serial:=D455_SERIAL enable_depth:=false
    scripts/bringup.sh manual recording_profile:=mapping
    scripts/bringup.sh manual recording_profile:=calibration
    scripts/bringup.sh input recording:=false

d455/manual は対話端末かつlaunch引数なしなら設定TUIへ進みます。
launch引数あり・非TTYでは保存設定/指定値で直接起動し、--configureで明示編集できます。
保存先: /data/results/runs/bringup/d455.yaml（camera_config:=絶対パスで変更）
既定要求: Infra左右640x480@90、RGB640x480@30、accel200Hz/gyro400Hz ON、Depth OFF。
対応可否はJetson上の実機SDK照会で検査し、未対応時に自動変更しません。
manual/inputは記録managerを起動しますが、録画はJoy・プロポの割当操作で開始します。
記録設定: recording_profile:=mapping|calibration|inputs、recording_config:=絶対YAMLパス。
calibration は車両モデル用の左右IR・CameraInfo・静的TF・加速度・角速度・Pico入力を必須にします。
manualの既定はmapping（左右IR必須）、inputはinputs（カメラ不要）。

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
task_configure=false
task_launch_arguments=()
while (($#)); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --dry-run) task_dry_run=true; shift ;;
        --list) task_list=true; shift ;;
        --configure) task_configure=true; shift ;;
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
        [[ -t 0 && -t 1 ]] || fail '対話端末で起動するか、input / bridge / d455 / manual を指定してください'
        task_profile="$(atd_bringup_choose)" || exit 0
        [[ "$task_profile" != quit ]] || exit 0
    fi
fi
task_target="$(atd_bringup_lookup "$task_profile")" || fail "未登録の起動構成: ${task_profile}（--list を参照）"
IFS=$'\t' read -r task_host task_package task_launch task_setup <<< "$task_target"
source "$task_root/scripts/lib/detect_host.sh"
if ! $task_dry_run && [[ "$(detect_atd_host)" != "$task_host" ]]; then
    fail "${task_profile} は ${task_host} 上で起動してください。Notebook の UI は Docker 内の atd-web を使います"
fi
task_use_setup=false
if $task_configure; then
    [[ "$task_setup" == camera ]] || fail '--configure は d455 / manual で使います'
    if ! $task_dry_run; then
        [[ -t 0 && -t 1 ]] || fail '--configure は対話端末で実行してください'
    fi
    task_use_setup=true
elif ! $task_dry_run && [[ "$task_setup" == camera && -t 0 && -t 1 ]] \
    && ((${#task_launch_arguments[@]} == 0)); then
    task_use_setup=true
fi
if $task_dry_run && [[ "$task_setup" == camera ]]; then
    printf '[dry-run] 未保存時の既定要求: infra1/2=640x480@90, RGB=640x480@30, accel=200Hz, gyro=400Hz, depth=OFF\n'
    printf '[dry-run] 保存先: /data/results/runs/bringup/d455.yaml。実機capabilityは未照会です。\n'
fi
task_command=(bash "$task_root/scripts/tools/python.sh" --host "$task_host")
if $task_dry_run; then task_command+=(--dry-run); fi
if $task_use_setup; then
    task_command+=(ros run -- ros2 run atd_sensors d455_setup --bringup-profile "$task_profile" --)
else
    task_command+=(ros run -- ros2 launch "$task_package" "$task_launch")
fi
if ((${#task_launch_arguments[@]})); then task_command+=("${task_launch_arguments[@]}"); fi
exec "${task_command[@]}"

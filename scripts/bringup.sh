#!/usr/bin/env bash
# 起動構成を選び、既存 Docker 内の ROS 環境で package の launch を実行する。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: scripts/bringup.sh [--dry-run] [--configure] [input|bridge|d455|joy-drive|manual|localization|autonomous|esc-trial|gazebo] [name:=value ...]

引数なし: fzf で起動構成を選択（未導入時は番号選択）
  input      Jetson の Joy・Pico入力、入力記録待機、観測・設定bridge
  bridge     観測・設定bridgeのみ。入力・役割 node は別途起動する
  d455       D455・車両TF（bridgeなし）
  joy-drive  DualSense で手動走行（CH3 切替、カメラなし）・入力記録待機
  manual     D455・車両TF・DualSense 手動走行・地図用記録待機
  localization  地図と参照ラインを選び、手動走行中の自己位置・前方軌道・操舵計算・速度 FF＋PID を監視
  autonomous    地図と速度付きの走行ラインを選び、CH3 で低速・前進限定の自動走行に切替
  esc-trial  ESC 試験用 bridge と Jetson の実行端末（port:=/dev/serial/by-id/...-if00、stop_source:=visual|localization）
  gazebo     Notebook で Gazebo の /clock と左右画像を使ったステレオ Visual Odometry（use_sim_time:=true）
  --configure D455設定TUIを明示して開く（対話端末専用）
  --list     現在の起動構成を表示
  --dry-run  起動せず予定を表示。構成省略時は input。Notebookでも利用可能

例: scripts/bringup.sh input
    scripts/bringup.sh joy-drive
    scripts/bringup.sh input pico_serial:=PICO_SERIAL rc_profile:=/data/results/runs/input/propo.yaml
    scripts/bringup.sh bridge config:=/absolute/path/bridge.yaml
    scripts/bringup.sh --dry-run input propo:=false
    scripts/bringup.sh d455
    scripts/bringup.sh manual camera_serial:=D455_SERIAL enable_depth:=false
    scripts/bringup.sh manual recording_profile:=mapping
    scripts/bringup.sh manual recording_profile:=calibration
    scripts/bringup.sh localization
    scripts/bringup.sh localization joy_drive:=true
    scripts/bringup.sh localization map_id:=course_a line:=centerline
    scripts/bringup.sh localization line:=custom
    scripts/bringup.sh localization lookahead_time_s:=0.5 max_lookahead_m:=2.0
    scripts/bringup.sh autonomous
    scripts/bringup.sh autonomous map_id:=course_a line:=centerline max_speed_mps:=0.4
    scripts/bringup.sh input recording:=false
    scripts/bringup.sh gazebo

d455/manual は対話端末かつlaunch引数なしなら設定TUIへ進みます。
launch引数あり・非TTYでは保存設定/指定値で直接起動し、--configureで明示編集できます。
保存先: /data/results/runs/bringup/d455.yaml（camera_config:=絶対パスで変更）
既定要求: Infra左右640x480@90、RGB640x480@30、accel200Hz/gyro400Hz ON、Depth OFF。
対応可否はJetson上の実機SDK照会で検査し、未対応時に自動変更しません。
joy-drive/manual は Jetson の USB 制御ポートで CH3 が HOST の時に DualSense の操作を Pico に送ります。
プロポ側では Pico が CH1/CH2 で運転します。manual を監視専用にするには joy_drive:=false を指定します。
localization は既定で監視専用です。DualSense 走行を併用するには joy_drive:=true を指定します。
autonomous の目標速度上限は max_speed_mps:=0.5（最大 1.0 m/s）。CH3 をプロポ側に戻すと Pico が手動操作を引き継ぎます。
自動走行用の実測校正は auto_calibration_path:=/data/results/runs/vehicle/auto_drive.yaml、速度 FF 表は feedforward_path で指定します。
Rev6 が 1 台であればポートは自動選択します。複数台の場合は pico_serial:= で対象を指定します。
制御ポートを明示するには手動走行で joy_drive_port、自動走行で auto_drive_port に CDC0 の by-id パスを指定します。
録画はJoy・プロポの割当操作で開始します。
記録設定: recording_profile:=mapping|calibration|inputs、recording_config:=絶対YAMLパス。
calibration は車両モデル用の左右IR・CameraInfo・静的TF・加速度・角速度・Pico入力を必須にします。
manualの既定はmapping（左右IR必須）、joy-drive/inputはinputs（カメラ不要）。

起動済み Docker を使用します。先に scripts/dev.sh で起動してください。
実車構成は Jetson、gazebo 構成は Linux x86_64 の Notebook で実行します。
gazebo には atd_pc_dev と、同じ Notebook 上の「atd gazebo serve」が必要です。
シミュレーション専用 ROS domain の既定値は 74。ATD_GAZEBO_ROS_DOMAIN_ID で両側に同じ値を指定できます。
終了は Ctrl-C。
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
source "$task_root/scripts/lib/detect_host.sh"
task_current_host="$(detect_atd_host)"
if [[ -z "$task_profile" ]]; then
    if $task_dry_run; then
        task_profile=input
    else
        [[ -t 0 && -t 1 ]] || fail '対話端末で起動するか、--list に表示される起動構成を指定してください'
        task_profile="$(atd_bringup_choose "$task_current_host")" || exit 0
        [[ "$task_profile" != quit ]] || exit 0
    fi
fi
task_target="$(atd_bringup_lookup "$task_profile")" || fail "未登録の起動構成: ${task_profile}（--list を参照）"
IFS=$'\t' read -r task_host task_package task_launch task_setup <<< "$task_target"
if ! $task_dry_run && [[ "$task_current_host" != "$task_host" ]]; then
    fail "${task_profile} は ${task_host} 上で起動してください"
fi
for task_arg in ${task_launch_arguments[@]+"${task_launch_arguments[@]}"}; do
    case "$task_arg" in
        use_sim_time:=[Tt][Rr][Uu][Ee]|use_sim_time:=1)
            [[ "$task_setup" == simulation ]] || fail 'use_sim_time:=true は Notebook の gazebo 構成で使用してください' ;;
    esac
done
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
if [[ "$task_setup" == simulation ]]; then
    if ! $task_dry_run && [[ "$(uname -s)/$(uname -m)" != Linux/x86_64 ]]; then
        fail 'gazebo は Linux x86_64 の Notebook で起動してください'
    fi
    task_domain="${ATD_GAZEBO_ROS_DOMAIN_ID:-74}"
    [[ "$task_domain" =~ ^(0|[1-9][0-9]{0,2})$ ]] && (( task_domain <= 232 )) || fail 'ROS domain ID は 0〜232 の整数で指定してください'
    task_clock_set=false
    for task_arg in ${task_launch_arguments[@]+"${task_launch_arguments[@]}"}; do
        case "$task_arg" in
            use_sim_time:=true) $task_clock_set && fail 'use_sim_time が重複しています'; task_clock_set=true ;;
            use_sim_time:=*) fail 'gazebo 構成は /clock を使います（use_sim_time:=true）' ;;
            *:=*) fail "gazebo 構成で未対応の launch 引数: $task_arg" ;;
        esac
    done
    task_command+=(ros run -- env "ROS_DOMAIN_ID=$task_domain"
        FASTDDS_DEFAULT_PROFILES_FILE=/etc/atd/fastdds.xml
        ros2 launch "$task_package" "$task_launch")
    if ! $task_clock_set; then task_command+=(use_sim_time:=true); fi
elif [[ "$task_setup" == trial ]]; then
    if ! $task_dry_run && [[ ! -t 0 || ! -t 1 ]]; then fail 'ESC 試験は Jetson の対話端末で起動してください'; fi
    task_command+=(ros run -- ros2 run "$task_package" "$task_launch")
elif [[ "$task_setup" == map ]]; then
    task_has_map=false
    task_has_line=false
    for task_arg in ${task_launch_arguments[@]+"${task_launch_arguments[@]}"}; do
        case "$task_arg" in
            map_id:=) fail 'map_id に地図 ID を指定してください' ;;
            map_id:=*) $task_has_map && fail 'map_id が重複しています'; task_has_map=true ;;
            line:=raceline|line:=centerline|line:=custom)
                $task_has_line && fail 'line が重複しています'; task_has_line=true ;;
            line:=*) fail 'line は raceline / centerline / custom から選択してください' ;;
            joy_drive:=true) [[ "$task_profile" != autonomous ]] || fail 'autonomous と joy_drive は同じ Pico 制御ポートを使うため同時起動できません' ;;
            --autonomous) fail '自動走行には autonomous 起動構成を指定してください' ;;
        esac
    done
    if ! $task_has_map || { [[ "$task_profile" == autonomous && -t 0 && -t 1 ]] && ! $task_has_line; }; then
        if ! $task_dry_run && [[ ! -t 0 || ! -t 1 ]]; then fail '非対話実行では map_id:=地図ID を指定してください'; fi
        task_command+=(ros run -- ros2 run atd_bringup choose_localization_map)
        if [[ "$task_profile" == autonomous ]]; then task_command+=(--autonomous); fi
    else
        task_command+=(ros run -- ros2 launch "$task_package" "$task_launch")
    fi
elif $task_use_setup; then
    task_command+=(ros run -- ros2 run atd_sensors d455_setup --bringup-profile "$task_profile" --)
else
    task_command+=(ros run -- ros2 launch "$task_package" "$task_launch")
fi
if ((${#task_launch_arguments[@]})); then task_command+=("${task_launch_arguments[@]}"); fi
exec "${task_command[@]}"

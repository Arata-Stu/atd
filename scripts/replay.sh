#!/usr/bin/env bash
# Notebook のステレオ bag を cuVSLAM と RViz で再生する。
set -euo pipefail

root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/scripts/lib/detect_host.sh"

domain="${ATD_REPLAY_ROS_DOMAIN_ID:-75}"
dry_run=false
command=(env "ROS_DOMAIN_ID=$domain"
    FASTDDS_DEFAULT_PROFILES_FILE=/workspaces/atd/docker/gazebo_monza/fastdds.xml
    ros2 run atd_bringup replay_vslam)
while (($#)); do
    case "$1" in
        --help|-h)
            cat <<'EOF'
使い方: atd replay [BAG] [--tracking vo|vio] [--headless] [--rate RATE] [--slam-max-map-size N] [--dry-run]

Linux Notebook の Isaac ROS コンテナで cuVSLAM と RViz を起動し、rosbag を再生します。
対話端末で引数なしの atd replay を実行すると、VO（左右画像）・VIO（左右画像＋統合 IMU）を選んでから bag を選びます。
BAG を省略すると対話端末で記録を選べます。直接指定する場合はコンテナ内の bag パスを使います。
Gazebo の正解軌跡がある記録では、青の正解と橙の cuVSLAM 推定を RViz に重ねます。
画像時刻ごとの誤差は /data/results/runs/replay/ に保存し、Notebook UI の「VSLAM 軌跡誤差」で確認できます。
--rate 0.5 は半速再生、--dry-run は起動コマンドの表示です。
--tracking vio は左右画像と /camera/camera/imu を使用します。既定の vo はステレオ画像で追跡します。
--headless は RViz を開かずに Gazebo の正解軌跡との誤差を保存し、再生の末尾で終了します。
--slam-max-map-size は SLAM 地図の姿勢数上限です。既定の 0 は無制限で、300 を指定すると従来の条件で比較できます。
再生専用の ROS_DOMAIN_ID は 75 です。ATD_REPLAY_ROS_DOMAIN_ID で変更できます。
ホストから使う場合は、先に atd dev で開発コンテナを起動してください。
EOF
            exit 0 ;;
        --dry-run) dry_run=true ;;
        --) command+=("$@"); break ;;
        *) command+=("$1") ;;
    esac
    shift
done

if [[ ! "$domain" =~ ^(0|[1-9][0-9]{0,2})$ ]] || (( domain > 232 )); then
    printf 'atd replay: ATD_REPLAY_ROS_DOMAIN_ID は 0〜232 の整数で指定してください。\n' >&2
    exit 2
fi
if ! "$dry_run" && [[ "$(uname -s)/$(uname -m)" != Linux/x86_64 || "$(detect_atd_host)" != pc ]]; then
    printf 'atd replay: Linux x86_64 の Notebook で実行してください。\n' >&2
    exit 1
fi

if atd_in_container; then
    # シェル入室後の build も overlay に取り込む。DDS の上書きはその後で行う。
    shell_entry="$root/docker/scripts/atd-shell.sh"
    if "$dry_run"; then
        printf '[dry-run]'
        printf ' %q' "$shell_entry" "${command[@]}"
        printf '\n'
        exit 0
    fi
    exec "$shell_entry" "${command[@]}"
fi
source "$root/scripts/lib/container.sh"
atd_container_exec "$root" pc "$dry_run" "${command[@]}"

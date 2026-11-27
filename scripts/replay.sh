#!/usr/bin/env bash
# Notebook のステレオ bag を cuVSLAM と RViz で再生する。
set -euo pipefail

root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/scripts/lib/detect_host.sh"

domain="${ATD_REPLAY_ROS_DOMAIN_ID:-75}"
dry_run=false
saved_map=false
map_id=
args=()
while (($#)); do
    case "$1" in
        --help|-h)
            cat <<'EOF'
使い方: atd replay [BAG] [--tracking vo|vio] [--headless] [--rate RATE] [--slam-max-map-size N] [--dry-run]

Linux Notebook の Isaac ROS コンテナで cuVSLAM と RViz を起動し、rosbag を再生します。
対話端末の atd replay では、VO・VIO による RViz 再生、または保存地図での自己位置検証を選べます。
BAG を省略すると対話端末で記録を選べます。直接指定する場合はコンテナ内の bag パスを使います。
RViz では cuVSLAM の直近の推定姿勢を橙の矢印で表示します。Gazebo の正解軌跡は青で重ねます。
画像時刻ごとの誤差は /data/results/runs/replay/ に保存し、Notebook UI の「VSLAM 軌跡誤差」で確認できます。
正解軌跡が2周以上ある周回記録では、再生末尾に cuVSLAM の姿勢グラフで周回間の閉路を照合します。
--rate 0.5 は半速再生、--dry-run は起動コマンドの表示です。
--tracking vio は左右画像と /camera/camera/imu を使用します。既定の vo はステレオ画像で追跡します。
--headless は RViz を開かずに Gazebo の正解軌跡との誤差を保存し、再生の末尾で終了します。
--slam-max-map-size は SLAM 地図の姿勢数上限です。既定の 0 は無制限で、300 を指定すると従来の条件で比較できます。
別プロセス間の DDS は Fast DDS 共有メモリが既定です。UDP 比較は ATD_REPLAY_DDS=udp atd replay を使います。
通常の再生は ROS domain 75（ATD_REPLAY_ROS_DOMAIN_ID で変更可能）、保存地図の検証は既定 95 です。
保存地図の検証: atd replay --validate-map MAP_ID --bag BAG --rate 1 [--map-correction-source cuvslam|vgl] [--rviz]
--select-map で保存地図を、--bag を省略すると別走行の bag を対話的に選びます。保存地図の検証は既定で等速です。
--rviz を付けると、保存地図の座標で推定姿勢と軌跡を表示しながら検証します。TUI では「走行中の表示」から選べます。
VGL 補正は --vgl-frequency で照合頻度を指定します（既定 1 Hz、0 は初回＋手動）。
--diagnose-frame-gaps で入力画像と処理済みフレームの間隔を調べられます。
ホストから使う場合は、先に atd dev で開発コンテナを起動してください。
EOF
            exit 0 ;;
        --dry-run) dry_run=true ;;
        --select-map|--validate-map)
            if "$saved_map"; then
                printf 'atd replay: --select-map または --validate-map のどちらかを一度だけ指定してください。\n' >&2
                exit 2
            fi
            saved_map=true
            if [[ "$1" = --validate-map ]]; then
                if [[ -z "${2:-}" || "$2" = -* ]]; then
                    printf 'atd replay: --validate-map に保存地図の ID を一つ指定してください。\n' >&2
                    exit 2
                fi
                map_id=$2
                shift
            fi ;;
        --) shift; args+=("$@"); break ;;
        *) args+=("$1") ;;
    esac
    shift
done

policy="${ATD_REPLAY_DDS:-shm}"
case "$policy" in
    udp|shm) ;;
    *) printf 'atd replay: ATD_REPLAY_DDS は udp または shm を指定してください。\n' >&2; exit 2 ;;
esac
if "$saved_map"; then
    command=(env "ATD_REPLAY_DDS=$policy" ros2 run atd_localization validate_bag)
    if [[ -n "$map_id" ]]; then command+=(--map "$map_id"); fi
else
    if [[ ! "$domain" =~ ^(0|[1-9][0-9]{0,2})$ ]] || (( domain > 232 )); then
        printf 'atd replay: ATD_REPLAY_ROS_DOMAIN_ID は 0〜232 の整数で指定してください。\n' >&2
        exit 2
    fi
    command=(env "ROS_DOMAIN_ID=$domain" "ATD_REPLAY_DDS=$policy" ros2 run atd_bringup replay_vslam)
fi
if ((${#args[@]})); then command+=("${args[@]}"); fi
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

#!/usr/bin/env bash
# Notebook のステレオ bag を cuVSLAM と RViz で再生する。
set -euo pipefail

root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/scripts/lib/detect_host.sh"

domain="${ATD_REPLAY_ROS_DOMAIN_ID:-75}"
dry_run=false
mode=
args=()
while (($#)); do
    case "$1" in
        --help|-h)
            cat <<'EOF'
使い方: atd replay [BAG] [--backend cuvslam|orbslam3] [--tracking vo|vio] [--headless] [--rate RATE] [--slam-max-map-size N] [--dry-run]

Linux Notebook の Isaac ROS コンテナで選択した SLAM と RViz を起動し、rosbag を再生します（既定 cuVSLAM）。
ORB-SLAM3 によるステレオ地図作成: atd replay BAG --backend orbslam3 --save-map MAP_ID --headless
保存地図で別走行の 15 秒地点から再定位: atd replay OTHER_BAG --backend orbslam3 --map MAP_ID --start-offset 15 --headless
固定地図でジャイロ補完と再取得時の段差を計測: atd replay BAG --backend orbslam3 --map MAP_ID --gyro-bridge on
cuVSLAM の保存地図で画像の途絶を診断: atd replay --validate-map MAP_ID --bag BAG --diagnose-frame-gaps
固定地図のステレオ VO では --gyro-bridge off で画像観測のみ、on で収録ジャイロによる途絶補完を選べます。
cuVSLAM と ORB-SLAM3 の同条件の run を比較: atd replay --compare CUV_RUN_ID ORB_RUN_ID
固定地図で cuVSLAM、ORB ステレオ、ORB Stereo-Inertial を比較: atd replay --compare CU_VALIDATION_ID ORB_STEREO_ID ORB_VIO_ID --record-root /data/record/rosbags
ORB 固定地図で VIO を地図座標に合わせる: atd replay BAG --backend orbslam3 --map MAP_ID --tracking vio --gyro-bridge off --headless
ORB-SLAM3 Stereo-Inertial を Gazebo bag で試す: atd replay BAG --backend orbslam3 --tracking vio --headless
実センサーの ORB-SLAM3 VIO には --imu-calibration IMU.yaml を追加します。
固定地図の姿勢途絶を収録済みジャイロで評価: atd replay --imu-gaps RUN_ID [--bag DIR] [--results-root DIR]
ORB-SLAM3 の状態と軌跡は /data/results/runs/orbslam3/ に保存します。
対話端末で引数なしの atd replay を実行すると、VO（左右画像）・VIO（左右画像＋統合 IMU）を選んでから bag を選びます。
BAG を省略すると対話端末で記録を選べます。直接指定する場合はコンテナ内の bag パスを使います。
RViz では cuVSLAM の直近の推定姿勢を橙の矢印で表示します。Gazebo の正解軌跡は青で重ねます。
画像時刻ごとの誤差は /data/results/runs/replay/ に保存し、Notebook UI の「VSLAM 軌跡誤差」で確認できます。
正解軌跡が2周以上ある周回記録では、再生末尾に cuVSLAM の姿勢グラフで周回間の閉路を照合します。
--rate 0.5 は半速再生、--dry-run は起動コマンドの表示です。
--tracking vio は左右画像と /camera/camera/imu を使用します。既定の vo はステレオ画像で追跡します。
--headless は RViz を開かずに Gazebo の正解軌跡との誤差を保存し、再生の末尾で終了します。
--slam-max-map-size は SLAM 地図の姿勢数上限です。既定の 0 は無制限で、300 を指定すると従来の条件で比較できます。
通常の再生には ROS domain 75（ATD_REPLAY_ROS_DOMAIN_ID で変更可能）、--validate-map の固定地図検証には 95（--domain で変更可能）を使います。
bag 再生の DDS は loopback UDP。以前の Gazebo bag の計測条件を再現する場合は ATD_REPLAY_DDS=shm を指定します。
ホストから使う場合は、先に atd dev で開発コンテナを起動してください。
EOF
            exit 0 ;;
        --dry-run) dry_run=true ;;
        --imu-gaps|--compare|--validate-map)
            if [[ -n "$mode" ]]; then
                printf 'atd replay: %s と %s は同時に指定できません。\n' "$mode" "$1" >&2
                exit 2
            fi
            mode=$1
            if [[ "$mode" = --validate-map ]]; then
                if [[ -z "${2:-}" || "$2" = -* ]]; then
                    printf 'atd replay: --validate-map に保存地図の ID を指定してください。\n' >&2
                    exit 2
                fi
                shift
                args+=(--map "$1")
            fi ;;
        --) args+=("$@"); break ;;
        *) args+=("$1") ;;
    esac
    shift
done
command=(env)
if [[ "$mode" != --validate-map ]]; then
    if [[ ! "$domain" =~ ^(0|[1-9][0-9]{0,2})$ ]] || (( domain > 232 )); then
        printf 'atd replay: ATD_REPLAY_ROS_DOMAIN_ID は 0〜232 の整数で指定してください。\n' >&2
        exit 2
    fi
    command+=("ROS_DOMAIN_ID=$domain")
fi
if [[ -z "$mode" || "$mode" = --validate-map ]]; then
    policy="${ATD_REPLAY_DDS:-udp}"
    case "$policy" in
        udp|shm) command+=("ATD_REPLAY_DDS=$policy") ;;
        *) printf 'atd replay: ATD_REPLAY_DDS は udp または shm を指定してください。\n' >&2; exit 2 ;;
    esac
fi
command+=(ros2 run)
case "$mode" in
    --validate-map) command+=(atd_localization validate_bag) ;;
    --imu-gaps) command+=(atd_bringup evaluate_orb_gyro) ;;
    --compare) command+=(atd_bringup compare_replays) ;;
    *) command+=(atd_bringup replay_vslam) ;;
esac
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

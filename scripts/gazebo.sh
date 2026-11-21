#!/usr/bin/env bash
# Gazebo は Notebook ホストの専用イメージで起動する。
set -euo pipefail
project="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$project/scripts/lib/detect_host.sh"

usage() {
    cat <<'HELP'
使い方: atd gazebo                        走行設定のメニューを開く
        atd gazebo 操作 [--course ID] [--speed m/s] [--laps 周回数] [オプション]
        ./scripts/gazebo.sh 操作 [オプション]
  list        選択できるコース ID を表示
  prepare     Gazebo 専用の Docker をビルドし、増分・ビルド中の最大ディスク使用量を記録
  run         動画を記録し、別セッションで配信を計測（スクリプトで操作を省略した場合の既定）
  video       左右の車載動画を保存（走行時間はコース・速度・周回数で決定）
  bag         指定周回のステレオ rosbag を record/rosbags に保存（既定1周）
  map         指定周回のステレオ画像で cuVSLAM 地図を保存（既定1周）。Notebook の起動済み atd_pc_dev を使用
  benchmark   録画を止めた状態で ROS 2 の受信を計測（既定 60 実時間秒）
  diagnose    同じ計測区間に Gazebo 側でも画像とシミュレーション時刻を観測（既定 20 実時間秒）
  serve       /clock・ステレオ画像・CameraInfo・固定 TF を配信（既定 Ctrl+C まで）
  --course ID         コースを選択（既定 monza、--circuit ID も使用可）
  --speed m/s         RC スケールの走行速度（シミュレーション時間）。省略時はコース既定
  --laps N            全周を N 周走行（正の整数）。run では動画が対象。serve は指定周回で終了
  --duration 秒       video・bag・map は1周または1区間の走行時間（--speed と排他）
                      benchmark・diagnose は実時間の計測時間（--speed と併用可）
  --route chicane|lap   Monza は既定で第1シケイン。他のコースは既定で1周。lap の連続配信は同方向に周回
  --wait-for-stereo   serve を一時停止状態で起動し、左右の画像と CameraInfo の購読を待って走行開始
  --dds shm|udp   Gazebo コンテナ内に Fast DDS 共有メモリを追加（shm、既定）、または従来の UDP のみ（udp）
  --camera-height m   地面から左右光学中心までの高さを指定
  --camera-pitch-deg 度   カメラの下向き角度を指定（既定 0）
  --dry-run           対象イメージ・保存先・実行コマンドを表示

Notebook の Linux x86_64 ホストで実行します。prepare はオンライン、走行時は準備済み
イメージを使います。Notebook の results/runs/gazebo/<コースID> に保存します（Monza は gazebo_monza）。
ROS_DOMAIN_ID はシミュレーション専用の 74。外部購読時は ATD_GAZEBO_ROS_DOMAIN_ID
で変更できます。containerd の独自の保存先は ATD_CONTAINERD_ROOT で計測に含めます。
HELP
}
fail() { printf 'gazebo: %s\n' "$*" >&2; exit 1; }
show_command() { printf '%q ' "$@"; printf '\n'; }

action=run
if [[ $# -gt 0 && "$1" != -* ]]; then action="$1"; shift; fi
catalog="$project/tools/gazebo_monza/circuits.py"
if [[ "$action" = list ]]; then
    [[ $# = 0 ]] || fail "list は引数なしで実行してください"
    exec python3 "$catalog"
fi
case "$action" in prepare|run|video|bag|map|benchmark|diagnose|serve) ;; *) fail "不明な操作: $action";; esac
dry_run=false
circuit=monza
route=
dds=shm
speed=
laps=
duration=
sim_args=()
while (( $# )); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --dry-run) dry_run=true; shift ;;
        --course|--circuit)
            [[ "$action" != prepare && -n "${2:-}" ]] || fail "走行時に $1 とコース ID を指定してください"
            circuit="$2"; shift 2 ;;
        --speed)
            [[ "$action" != prepare && -n "${2:-}" ]] || fail "走行時に --speed と速度（m/s）を指定してください"
            speed="$2"; shift 2 ;;
        --laps)
            case "$action" in run|video|bag|map|serve) ;; *) fail "--laps は run・video・bag・map・serve で指定してください" ;; esac
            [[ "${2:-}" =~ ^[1-9][0-9]*$ ]] || fail "--laps は1以上の整数で指定してください"
            laps="$2"; shift 2 ;;
        --wait-for-stereo)
            [[ "$action" = serve ]] || fail "$1 は serve で指定してください"
            sim_args+=("$1"); shift ;;
        --route)
            [[ "$action" != prepare && ( "${2:-}" = chicane || "${2:-}" = lap ) ]] || fail "走行時に --route chicane または --route lap を指定してください"
            route="$2"; shift 2 ;;
        --dds)
            [[ "$action" != prepare && ( "${2:-}" = shm || "${2:-}" = udp ) ]] || fail "走行時に --dds shm または --dds udp を指定してください"
            dds="$2"; shift 2 ;;
        --duration|--camera-height|--camera-pitch-deg)
            [[ "$action" != prepare ]] || fail "$1 は走行時に指定してください"
            [[ "$1" != --duration || "$action" != serve ]] || fail "serve の終了条件は --laps または Ctrl+C で指定してください"
            [[ -n "${2:-}" ]] || fail "$1 の値が必要です"
            if [[ "$1" = --duration ]]; then duration="$2"; fi
            sim_args+=("$1" "$2"); shift 2 ;;
        *) fail "不明な引数: $1";;
    esac
done
if [[ -n "$speed" ]]; then
    python3 -c 'import math, sys
try:
    speed = float(sys.argv[1])
except ValueError:
    sys.exit(1)
sys.exit(not (math.isfinite(speed) and speed > 0))' "$speed" || fail "--speed は有限の正の数（m/s）で指定してください"
    [[ -z "$duration" || "$action" = benchmark || "$action" = diagnose ]] ||
        fail "$action の走行時間は --speed または --duration の一方で指定してください"
fi
if [[ "$circuit" != monza ]]; then
    list="$(python3 "$catalog")"
    [[ $'\n'"$list"$'\n' = *$'\n'"$circuit"$'\n'* && "$circuit" =~ ^[a-z0-9]+$ ]] || fail "不明なコース: $circuit（atd gazebo list で一覧）"
    [[ "$route" != chicane ]] || fail "chicane は Monza 専用です"
fi
if [[ -n "$laps" || "$action" = map || "$action" = bag ]]; then
    [[ "$route" != chicane ]] || fail "周回走行では --route lap を使用してください"
    route=lap
fi
if [[ -n "$route" ]]; then sim_args+=(--route "$route"); fi
if [[ -n "$speed" ]]; then sim_args+=(--speed "$speed"); fi
storage="${ATD_STORAGE_CONFIG:-$project/config/storage.env}"
if [[ -f "$storage" ]]; then
    source "$storage"
elif [[ -n "${ATD_STORAGE_CONFIG:-}" ]]; then
    fail "ストレージ設定がありません: $storage"
fi
runs="${ATD_RESULT_DIR:-${ATD_PC_RESULT_DIR:-$project/data/results}}/runs"
results="$runs/gazebo_monza"
if [[ "$circuit" != monza ]]; then results="$runs/gazebo/$circuit"; fi
[[ "$results" = /* ]] || fail "results に絶対パスを指定してください"
record=
if [[ "$action" = bag ]]; then
    record="${ATD_RECORD_DIR:-${ATD_PC_RECORD_DIR:-$project/data/record}}"
    [[ "$record" = /* ]] || fail "record に絶対パスを指定してください"
fi
case "$results$project$record" in *','*|*'"'*|*$'\n'*) fail "Docker mount パスにカンマ・二重引用符・改行は使えません";; esac
domain="${ATD_GAZEBO_ROS_DOMAIN_ID:-74}"
[[ "$domain" =~ ^(0|[1-9][0-9]{0,2})$ ]] && (( domain <= 232 )) || fail "ROS domain ID は 0〜232 の整数で指定してください"
image=atd-gazebo-monza:jetty-lyrical
if ! $dry_run; then
    [[ "$(uname -s)/$(uname -m)" = Linux/x86_64 && "$(detect_atd_host)" = pc ]] ||
        fail "NVIDIA GPU を搭載した Notebook（Linux x86_64）のホストで実行してください"
    if atd_in_container; then fail "Gazebo 専用イメージを起動するため Notebook ホストで実行してください"; fi
    command -v docker >/dev/null || fail "docker が見つかりません"
    endpoint="${DOCKER_HOST:-$(docker context inspect --format '{{.Endpoints.docker.Host}}')}"
    [[ "$endpoint" = unix://* ]] || fail "バインドマウントとディスク計測には Notebook のローカル Docker を使用してください"
    mkdir -p -- "$results"
fi

new_output() {
    if $dry_run; then printf '%s/<run-id>-%s\n' "$results" "$1"; return; fi
    mktemp -d "$results/$(date -u +%Y%m%dT%H%M%SZ)-$1-XXXXXX"
}
if [[ "$action" = prepare ]]; then
    out="$(new_output prepare)"
    prepare=(python3 "$project/docker/gazebo_monza/build_storage.py" "$project/docker/gazebo_monza" "$image" "$out")
    printf '容量計測: %s\n' "$out"
    if $dry_run; then show_command "${prepare[@]}"; else "${prepare[@]}" 2>&1 | tee "$out/build.log"; fi
    exit
fi
if ! $dry_run; then
    docker image inspect "$image" >/dev/null || fail "先に ./scripts/gazebo.sh prepare を実行してください"
    if [[ "$action" = bag ]]; then mkdir -p -- "$record/rosbags"; fi
    # 同じ domain 内で複数の試行から同名 topic が混ざることを防ぐ。
    mkdir -p -- "$runs/gazebo_monza"
    exec 9>"$runs/gazebo_monza/.domain-$domain.lock"
    flock -n 9 || fail "同じ ROS domain の Gazebo が実行中です: $domain"
fi
run_mode() {
    local mode="$1" out size status bag_name profile=/workspaces/atd/docker/middleware_profiles/fastdds.xml
    out="$(new_output "$mode-$dds")"
    local -a command=(docker run --rm --init --pull=never --gpus all --network host
        --user "$(id -u):$(id -g)")
    if [[ "$dds" = shm ]]; then
        profile=/workspaces/atd/docker/gazebo_monza/fastdds.xml
        if [[ "$mode" = serve || "$mode" = map ]]; then
            local -a sim_ipc_args=()
            source "$project/scripts/lib/gazebo_ipc.sh"
            atd_gazebo_ipc_args "$dry_run"
            command+=("${sim_ipc_args[@]}")
        else
            command+=(--shm-size=128m)
        fi
    fi
    local map_container="atd-gazebo-map-$circuit-${out##*/}"
    if [[ "$mode" = map ]]; then command+=(--name "$map_container"); fi
    # PRIME / hybrid graphics 等で render node が存在するホストは既存グループを引き継ぐ。
    local gpu_device
    for gpu_device in /dev/dri/renderD*; do
        [[ -c "$gpu_device" ]] || continue
        command+=(--device "$gpu_device" --group-add "$(stat -c %g "$gpu_device")")
    done
    command+=(--tmpfs "/home/simulator:uid=$(id -u),gid=$(id -g),mode=0700"
        --env HOME=/home/simulator --env XDG_RUNTIME_DIR=/home/simulator
        --env GZ_SIM_SYSTEM_PLUGIN_PATH=/workspaces/atd/tools/gazebo_monza
        --env "FASTDDS_DEFAULT_PROFILES_FILE=$profile"
        --env NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
        --env "ROS_DOMAIN_ID=$domain" --env "GZ_PARTITION=atd_${circuit}_${out##*/}"
        --mount "type=bind,src=$project,dst=/workspaces/atd,readonly"
        --mount "type=bind,src=$out,dst=/output")
    if [[ "$mode" = bag ]]; then
        command+=(--mount "type=bind,src=$record,dst=/data/record")
    fi
    command+=("$image" python3 /workspaces/atd/tools/gazebo_monza/run.py --mode "$mode" --output /output --circuit "$circuit")
    if (( ${#sim_args[@]} )); then command+=("${sim_args[@]}"); fi
    if [[ -n "$laps" && "$mode" != benchmark ]]; then command+=(--laps "$laps"); fi
    if [[ "$mode" = bag ]]; then
        bag_name="gazebo-$circuit-${out##*/}"
        command+=(--bag-output "/data/record/rosbags/$bag_name")
        printf 'rosbag 保存先 %s/rosbags/%s\n' "$record" "$bag_name"
    fi
    printf '%s  DDS=%s  ROS_DOMAIN_ID=%s  保存先 %s\n' "$mode" "$dds" "$domain" "$out"
    if [[ "$mode" = map ]]; then
        local -a job=(python3 "$project/tools/gazebo_monza/map_job.py" host
            --output "$out" --results "${runs%/runs}" --circuit "$circuit" --domain "$domain"
            --profile "$profile" --container "$map_container")
        if $dry_run; then job+=(--dry-run); fi
        exec "${job[@]}" -- "${command[@]}"
    fi
    if $dry_run; then show_command "${command[@]}"; return; fi
    # Forward terminal and noninteractive stop signals directly via docker run.
    if [[ "$mode" = serve ]]; then exec "${command[@]}"; fi
    status=0
    "${command[@]}" || status=$?
    size="$(du -sk -- "$out" | cut -f1)"
    if [[ "$mode" = bag ]]; then
        printf '走行ログ %s KiB\n' "$size"
        if [[ "$status" = 0 ]]; then
            printf 'rosbag %s  %s/rosbags/%s\nWeb UI「地図とライン」で選ぶ記録データ: %s\n' \
                "$(du -sh -- "$record/rosbags/$bag_name" | cut -f1)" "$record" "$bag_name" "$bag_name"
        fi
    else
        printf 'この試行の生成物 %s KiB\n' "$size"
    fi
    printf '%s\n' "$size" > "$out/output-size-kib.txt"
    return "$status"
}
case "$action" in
    run) run_mode video; run_mode benchmark ;;
    *) run_mode "$action" ;;
esac

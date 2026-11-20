#!/usr/bin/env bash
# Gazebo は Notebook ホストの専用イメージで起動する。
set -euo pipefail
project="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$project/scripts/lib/detect_host.sh"

usage() {
    cat <<'HELP'
使い方: ./scripts/gazebo.sh prepare|run|video|benchmark [--duration 秒] [--camera-height m] [--camera-pitch-deg 度] [--dry-run]
  prepare     Gazebo 専用の Docker をビルドし、増分・ビルド中の最大ディスク使用量を記録
  run         動画を記録し、別セッションで配信を計測（引数なしの既定）
  video       左右の車載動画を保存（既定 10 シミュレーション秒）
  benchmark   録画を止めた状態で ROS 2 の受信を計測（既定 60 実時間秒）
  --camera-height m   地面から左右光学中心までの高さを指定
  --camera-pitch-deg 度   カメラの下向き角度を指定（既定 0）
  --dry-run           対象イメージ・保存先・実行コマンドを表示

Notebook の Linux x86_64 ホストで実行します。prepare はオンライン、走行時は準備済み
イメージを使います。保存先は Notebook の results/runs/gazebo_monza 配下です。
ROS_DOMAIN_ID はシミュレーション専用の 74。外部購読時は ATD_GAZEBO_ROS_DOMAIN_ID
で変更できます。containerd の独自の保存先は ATD_CONTAINERD_ROOT で計測に含めます。
HELP
}
fail() { printf 'gazebo: %s\n' "$*" >&2; exit 1; }
show_command() { printf '%q ' "$@"; printf '\n'; }

action=run
if [[ $# -gt 0 && "$1" != -* ]]; then action="$1"; shift; fi
case "$action" in prepare|run|video|benchmark) ;; *) fail "不明な操作: $action";; esac
dry_run=false
sim_args=()
while (( $# )); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --dry-run) dry_run=true; shift ;;
        --duration|--camera-height|--camera-pitch-deg)
            [[ "$action" != prepare ]] || fail "$1 は走行時に指定してください"
            [[ $# -ge 2 ]] || fail "$1 の値が必要です"
            sim_args+=("$1" "$2"); shift 2 ;;
        *) fail "不明な引数: $1";;
    esac
done
storage="${ATD_STORAGE_CONFIG:-$project/config/storage.env}"
if [[ -f "$storage" ]]; then
    source "$storage"
elif [[ -n "${ATD_STORAGE_CONFIG:-}" ]]; then
    fail "ストレージ設定がありません: $storage"
fi
results="${ATD_RESULT_DIR:-${ATD_PC_RESULT_DIR:-$project/data/results}}/runs/gazebo_monza"
[[ "$results" = /* ]] || fail "results に絶対パスを指定してください"
case "$results$project" in *','*|*'"'*|*$'\n'*) fail "Docker mount パスにカンマ・二重引用符・改行は使えません";; esac
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
    # 同じ domain 内で複数の試行から同名 topic が混ざることを防ぐ。
    exec 9>"$results/.domain-$domain.lock"
    flock -n 9 || fail "同じ ROS domain の Gazebo が実行中です: $domain"
fi
run_mode() {
    local mode="$1" out size status
    out="$(new_output "$mode")"
    local -a command=(docker run --rm --init --pull=never --gpus all --network host
        --user "$(id -u):$(id -g)")
    # PRIME / hybrid graphics 等で render node が存在するホストは既存グループを引き継ぐ。
    local gpu_device
    for gpu_device in /dev/dri/renderD*; do
        [[ -c "$gpu_device" ]] || continue
        command+=(--device "$gpu_device" --group-add "$(stat -c %g "$gpu_device")")
    done
    command+=(--tmpfs "/home/simulator:uid=$(id -u),gid=$(id -g),mode=0700"
        --env HOME=/home/simulator --env XDG_RUNTIME_DIR=/home/simulator
        --env GZ_SIM_SYSTEM_PLUGIN_PATH=/workspaces/atd/tools/gazebo_monza
        --env NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
        --env "ROS_DOMAIN_ID=$domain" --env "GZ_PARTITION=atd_monza_${out##*/}"
        --mount "type=bind,src=$project,dst=/workspaces/atd,readonly"
        --mount "type=bind,src=$out,dst=/output" "$image"
        python3 /workspaces/atd/tools/gazebo_monza/run.py --mode "$mode" --output /output)
    if (( ${#sim_args[@]} )); then command+=("${sim_args[@]}"); fi
    printf '%s  ROS_DOMAIN_ID=%s  保存先 %s\n' "$mode" "$domain" "$out"
    if $dry_run; then show_command "${command[@]}"; return; fi
    status=0
    "${command[@]}" || status=$?
    size="$(du -sk -- "$out" | cut -f1)"
    printf 'この試行の生成物 %s KiB\n' "$size"
    printf '%s\n' "$size" > "$out/output-size-kib.txt"
    return "$status"
}
case "$action" in
    run) run_mode video; run_mode benchmark ;;
    *) run_mode "$action" ;;
esac

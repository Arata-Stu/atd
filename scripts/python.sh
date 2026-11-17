#!/usr/bin/env bash
# 起動済み ATD コンテナで、用途別の uv 環境を選択・操作する短い入口。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: scripts/python.sh [--host notebook|pc|jetson] [--dry-run] [ros|train|vgl-export|web|raceline] [ACTION] [-- ARGS...]

引数なし: コンテナ内の対話メニューで Python 環境と操作を選択
ACTION: sync, check, shell, run, build（build は ros のみ）
ROS package の build は scripts/build.sh を標準の入口とする。build 操作は互換性のため維持する。
例: scripts/python.sh ros check
    scripts/python.sh train run -- python train.py

--host / --dry-run は用途名より前に指定する。
起動済みコンテナのみ使用する。先に scripts/dev.sh で起動する。
--dry-run は予定するコマンドだけを表示し、Docker や uv は実行しない。
EOF
}
fail() { printf 'エラー: %s\n' "$*" >&2; exit 1; }

task_host=''
task_dry_run=false
while (($#)); do
    case "$1" in
        --host)
            (($# >= 2)) && [[ -n "$2" ]] || fail '--host の値が必要です'
            task_host="$2"
            shift 2 ;;
        --dry-run) task_dry_run=true; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; break ;;
        --*) fail "不明な引数: $1（--help を参照）" ;;
        *) break ;;
    esac
done
[[ "$task_host" != notebook ]] || task_host=pc
case "$task_host" in ''|pc|jetson) ;; *) fail '--host は notebook、pc または jetson を指定してください' ;; esac

# コンテナ内では Docker socket を必要とせず、同じ入口を直接利用する。
if [[ -f /.dockerenv && -x /usr/local/bin/atd-python ]]; then
    if $task_dry_run; then
        printf '[dry-run] /usr/local/bin/atd-python'
        if (($#)); then printf ' %q' "$@"; fi
        printf '\n'
        exit 0
    fi
    exec /usr/local/bin/atd-python "$@"
fi

task_project_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ -z "$task_host" ]]; then
    source "${task_project_dir}/scripts/detect_host.sh"
    task_host="$(detect_atd_host)"
fi
task_container="atd_${task_host}_dev"
task_exec_args=(exec -i)
if [[ -t 0 && -t 1 ]]; then task_exec_args+=(-t); fi
task_exec_args+=(-u admin --workdir /workspaces/isaac_ros-dev "$task_container"
    /usr/local/bin/atd-shell /usr/local/bin/atd-python)
if $task_dry_run; then
    printf '[dry-run] host: %s\ncommand: docker' "$task_host"
    printf ' %q' "${task_exec_args[@]}"
    if (($#)); then printf ' %q' "$@"; fi
    printf '\n'
    exit 0
fi

command -v docker >/dev/null 2>&1 || fail 'Docker CLI がありません。scripts/setup.sh を実行してください'
command -v python3 >/dev/null 2>&1 || fail 'python3 が必要です。scripts/setup.sh の tools を実行してください'
source "${task_project_dir}/scripts/lib/runtime.sh"
task_revision="$(atd_runtime_revision "$task_project_dir")" || exit 1
task_inspect="$(docker container inspect "$task_container" 2>/dev/null)" \
    || fail "$task_container を確認できません。先に scripts/dev.sh で起動してください"
printf '%s' "$task_inspect" | python3 -c '
import json
import os
import sys
try:
    container = json.load(sys.stdin)[0]
    if not container.get("State", {}).get("Running"):
        sys.exit("エラー: コンテナは停止中です。先に scripts/dev.sh で起動してください")
    environment = dict(item.split("=", 1) for item in container.get("Config", {}).get("Env", []) if "=" in item)
    if environment.get("ATD_RUNTIME_API") != "1" or environment.get("ATD_RUNTIME_REV") != sys.argv[1]:
        sys.exit("エラー: コンテナの起動設定が古いか異なります。処理を終了後、scripts/dev.sh --build で再作成してください")
    mount = next((item for item in container.get("Mounts", []) if item.get("Destination") == "/workspaces/atd"), {})
    if (mount.get("Type") != "bind" or mount.get("RW") is not False
            or os.path.realpath(mount.get("Source", "")) != os.path.realpath(sys.argv[2])):
        sys.exit("エラー: 起動済みコンテナは別の ATD checkout を参照しています。scripts/dev.sh から構成を確認してください")
except (ValueError, TypeError, IndexError, KeyError):
    sys.exit("エラー: Docker inspect の結果を解釈できません")
' "$task_revision" "$task_project_dir" || exit 1
task_ready="$(docker exec "$task_container" cat /etc/atd/runtime.ready 2>/dev/null)" \
    || fail 'コンテナの初期設定完了を確認できません。起動ログを確認してください'
[[ "$task_ready" = "$task_revision" ]] || fail 'コンテナの初期設定が未完了、または現在の構成と異なります'

exec docker "${task_exec_args[@]}" "$@"

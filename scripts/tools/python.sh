#!/usr/bin/env bash
# 起動済み ATD コンテナで、用途別の uv 環境を選択・操作する短い入口。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: scripts/tools/python.sh [--host notebook|pc|jetson] [--dry-run] [ros|train|vgl-export|web|raceline] [ACTION] [-- ARGS...]

引数なし: コンテナ内の対話メニューで Python 環境と操作を選択
ACTION: sync, check, shell, run
ROS package の build は scripts/build.sh を使用する。
例: scripts/tools/python.sh ros check
    scripts/tools/python.sh train run -- python train.py

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
[[ "${2:-}" != build ]] || fail 'ROS package の build は scripts/build.sh を使用してください'

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

task_project_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "${task_project_dir}/scripts/lib/container.sh"
atd_container_exec "$task_project_dir" "$task_host" "$task_dry_run" /usr/local/bin/atd-python "$@"

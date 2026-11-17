#!/usr/bin/env bash
# ROS workspace の build 専用入口。コンテナ内の atd-build を直接起動する。
set -euo pipefail
task_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
task_host=''
task_dry_run=false
while (($#)); do
    case "$1" in
        -h|--help)
            cat <<'EOF'
使い方: scripts/build.sh [--host notebook|pc|jetson] [--dry-run] [colcon build の引数...]

例: scripts/build.sh
    scripts/build.sh --packages-up-to atd_web_bridge
    scripts/build.sh --dry-run --packages-select atd_teleop

PC / Jetson は自動判定します。--host / --dry-run は build 引数より前に指定します。
起動済み Docker を使用します。先に scripts/dev.sh で起動してください。
EOF
            exit 0 ;;
        --host)
            if (($# < 2)) || [[ -z "$2" || "$2" = --* ]]; then
                printf 'ATD Build: --host の値が必要です\n' >&2
                exit 1
            fi
            task_host="$2"
            shift 2 ;;
        --dry-run) task_dry_run=true; shift ;;
        --) shift; break ;;
        *) break ;;
    esac
done
[[ "$task_host" != notebook ]] || task_host=pc
case "$task_host" in ''|pc|jetson) ;; *) printf 'ATD Build: --host は notebook、pc または jetson を指定してください\n' >&2; exit 1 ;; esac

# Docker 内では socket を使わずビルド専用コマンドを直接実行する。
if [[ -f /.dockerenv ]]; then
    [[ -x /usr/local/bin/atd-build ]] || { printf 'ATD Build: atd-build がありません。Docker を再構築してください\n' >&2; exit 1; }
    if $task_dry_run; then
        printf '[dry-run] /usr/local/bin/atd-build'
        if (($#)); then printf ' %q' "$@"; fi
        printf '\n'
        exit 0
    fi
    exec /usr/local/bin/atd-build "$@"
fi
source "$task_root/scripts/lib/container.sh"
atd_container_exec "$task_root" "$task_host" "$task_dry_run" /usr/local/bin/atd-build "$@"

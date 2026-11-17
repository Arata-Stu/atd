#!/usr/bin/env bash
# ROS workspace の build 専用入口。環境準備とコンテナ選択は既存処理を使う。
set -euo pipefail
task_root="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
task_command=(bash "$task_root/scripts/python.sh")
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
            task_command+=(--host "$2")
            shift 2 ;;
        --dry-run) task_command+=(--dry-run); shift ;;
        --) shift; break ;;
        *) break ;;
    esac
done
exec "${task_command[@]}" ros build "$@"

#!/usr/bin/env bash
# ホストから一致する起動済み ATD コンテナでコマンドを実行する共通処理。
# source だけでは Docker や設定ファイルにアクセスしない。
atd_container_exec() {
    local task_project_dir="$1" task_host="$2" task_dry_run="$3" task_helper="$4"
    local task_container task_revision task_inspect task_ready
    local -a task_exec_args
    shift 4
    if [[ -z "$task_host" ]]; then
        source "${task_project_dir}/scripts/detect_host.sh"
        task_host="$(detect_atd_host)"
    fi
    task_container="atd_${task_host}_dev"
    task_exec_args=(exec -i)
    if [[ -t 0 && -t 1 ]]; then task_exec_args+=(-t); fi
    task_exec_args+=(-u admin --workdir /workspaces/isaac_ros-dev "$task_container"
        /usr/local/bin/atd-shell "$task_helper")
    if [[ "$task_dry_run" = true ]]; then
        printf '[dry-run] host: %s\ncommand: docker' "$task_host"
        printf ' %q' "${task_exec_args[@]}"
        if (($#)); then printf ' %q' "$@"; fi
        printf '\n'
        return 0
    fi

    command -v docker >/dev/null 2>&1 || { printf 'エラー: Docker CLI がありません。scripts/setup.sh を実行してください\n' >&2; return 1; }
    command -v python3 >/dev/null 2>&1 || { printf 'エラー: python3 が必要です。scripts/setup.sh の tools を実行してください\n' >&2; return 1; }
    source "${task_project_dir}/scripts/lib/runtime.sh"
    task_revision="$(atd_runtime_revision "$task_project_dir")" || return 1
    task_inspect="$(docker container inspect "$task_container" 2>/dev/null)" \
        || { printf 'エラー: %s を確認できません。先に scripts/dev.sh で起動してください\n' "$task_container" >&2; return 1; }
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
' "$task_revision" "$task_project_dir" || return 1
    task_ready="$(docker exec "$task_container" cat /etc/atd/runtime.ready 2>/dev/null)" \
        || { printf 'エラー: コンテナの初期設定完了を確認できません。起動ログを確認してください\n' >&2; return 1; }
    [[ "$task_ready" = "$task_revision" ]] \
        || { printf 'エラー: コンテナの初期設定が未完了、または現在の構成と異なります\n' >&2; return 1; }
    exec docker "${task_exec_args[@]}" "$@"
}

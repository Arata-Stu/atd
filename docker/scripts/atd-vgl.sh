#!/usr/bin/env bash
# Notebook の ONNX export と Jetson の native builder を別環境で実行する。
set -euo pipefail
task_root=/workspaces/atd
[[ -r "$task_root/tools/vgl/cli.py" ]] || {
    printf 'ATD VGL: project mount がありません。scripts/vgl.sh から実行してください。\n' >&2
    exit 1
}
for task_argument in "$@"; do
    case "$task_argument" in
        --dry-run|--help|-h) exec /usr/bin/python3 "$task_root/tools/vgl/cli.py" "$@" ;;
    esac
done
case "${1:-}" in
    export) exec /usr/local/bin/atd-python vgl-export run -- python "$task_root/tools/vgl/cli.py" "$@" ;;
    doctor)
        if [[ "$(uname -m)" = x86_64 ]]; then
            exec /usr/local/bin/atd-python vgl-export run -- python "$task_root/tools/vgl/cli.py" "$@"
        fi
        ;;
esac
exec /usr/bin/python3 "$task_root/tools/vgl/cli.py" "$@"

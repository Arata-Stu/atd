#!/usr/bin/env bash
# ROS workspace の build 処理。Python 環境の共通関数を使い、colcon はここで実行する。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: atd-build [colcon build の引数...]

例: atd-build
    atd-build --packages-up-to atd_web_bridge
    atd-build --packages-select atd_teleop --cmake-args -DCMAKE_BUILD_TYPE=Release

ROS 用 Python・CMake interpreter と workspace の src を使います。
通常のコンテナユーザーから実行してください。
EOF
}
fail() { printf 'ATD Build: %s\n' "$*" >&2; exit 1; }

if [[ "${1:-}" = --help || "${1:-}" = -h ]]; then usage; exit 0; fi
if [[ "${1:-}" = -- ]]; then shift; fi
task_script_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$task_script_dir/lib/python-environment.sh"

atd_python_context ros
task_build=("$task_venv/bin/python" /usr/bin/colcon build)
# checkout と workspace が同じ場所でも、編集元と src/atd/ を二重探索しない。
task_has_base_paths=false
for task_arg in "$@"; do
    case "$task_arg" in --base-paths|--base-paths=*) task_has_base_paths=true ;; esac
done
if ! $task_has_base_paths; then task_build+=(--base-paths "${task_workspace}/src"); fi
task_has_cmake=false
for task_arg in "$@"; do
    task_cmake_value="$task_arg"
    case "$task_arg" in --cmake-args=*) task_cmake_value="${task_arg#*=}" ;; esac
    case "$task_cmake_value" in
        -DPython3_EXECUTABLE=*|-DPython3_EXECUTABLE:*=*|-DPYTHON_EXECUTABLE=*|-DPYTHON_EXECUTABLE:*=*)
            fail 'Python interpreter は ros 環境に固定します。Python executable の CMake 上書きは指定しないでください' ;;
    esac
    case "$task_arg" in
        --cmake-args|--cmake-args=*)
            $task_has_cmake && fail '--cmake-args は一度だけ指定してください'
            task_has_cmake=true
            task_build+=(--cmake-args "-DPython3_EXECUTABLE=${task_venv}/bin/python")
            if [[ "$task_arg" != --cmake-args && -n "$task_cmake_value" ]]; then task_build+=("$task_cmake_value"); fi
            ;;
        *) task_build+=("$task_arg") ;;
    esac
done
if ! $task_has_cmake; then task_build+=(--cmake-args "-DPython3_EXECUTABLE=${task_venv}/bin/python"); fi
atd_python_describe
atd_python_sync
atd_python_activate
cd -- "$task_workspace"
exec "${task_build[@]}"

#!/usr/bin/env bash
# コンテナ内の用途別 uv 環境。ROS の apt 基盤と学習依存を分離する。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: atd-python [ros|train|vgl-export|web|raceline] [sync|check|shell|run|build] [-- COMMAND/ARGS]

引数なし: fzf で用途・操作を選択（未導入時は番号選択）
  sync    lock に従って環境を準備。依存の更新はしない
  check   ファイル・venv の設定を確認。作成・同期・通信はしない
  shell   同期後、選択した環境の Bash に入る
  run     同期後、-- に続くコマンドを実行
  build   ROS 環境で colcon build。続く引数は colcon へ渡す

例: atd-python train run -- python train.py
    atd-python ros build -- --packages-select atd_example
保存先: $ISAAC_ROS_WS/.atd-python/<architecture>/{ros,train,vgl-export,web,raceline}
通常のコンテナユーザーから実行する。ホストの Python 環境とは共有しない。
EOF
}
fail() { printf 'ATD Python: %s\n' "$*" >&2; exit 1; }

choose() {
    local task_header="$1" task_choices="$2" task_answer task_number task_index=0 task_key task_label
    [[ -t 0 && -t 1 ]] || { usage >&2; fail '対話端末で起動するか、用途と操作を指定してください'; }
    if command -v fzf >/dev/null 2>&1; then
        task_answer="$(printf '%s\n' "$task_choices" | fzf --delimiter=$'\t' --with-nth=2 \
            --height=60% --border --prompt='ATD Python > ' --header="$task_header")" || exit 1
        task_key="${task_answer%%$'\t'*}"
        while IFS=$'\t' read -r task_answer task_label; do
            if [[ "$task_key" = "$task_answer" ]]; then printf '%s\n' "$task_key"; return 0; fi
        done <<< "$task_choices"
        fail '選択した操作がありません'
    fi
    while IFS=$'\t' read -r task_key task_label; do
        task_index=$((task_index + 1))
        printf '%d) %s\n' "$task_index" "$task_label" >&2
    done <<< "$task_choices"
    read -r -p '番号: ' task_number || exit 1
    [[ "$task_number" =~ ^[0-9]+$ && ${#task_number} -le 2 ]] || exit 1
    task_number=$((10#$task_number))
    ((task_number >= 1 && task_number <= task_index)) || exit 1
    printf '%s\n' "$task_choices" | awk -F '\t' -v item="$task_number" 'NR==item {print $1}'
}

if [[ "${1:-}" = --help || "${1:-}" = -h ]]; then usage; exit 0; fi
task_profile="${1:-}"
if (($#)); then shift; else task_profile="$(choose 'Python の用途' $'ros\tROS 2 の追加 Python 依存\ntrain\tAI 学習・解析\nvgl-export\tNotebook の VGL ONNX export\nweb\tNotebook の Web API\nraceline\tNotebook の最小曲率ライン最適化')" || exit 1; fi
case "$task_profile" in ros|train|vgl-export|web|raceline) ;; *) fail '用途は ros、train、vgl-export、web、raceline を指定してください' ;; esac
task_action="${1:-}"
if (($#)); then
    shift
else
    task_choices=$'sync\t環境を準備・同期\ncheck\t設定を確認（変更なし）\nshell\t環境の Bash に入る'
    if [[ "$task_profile" = ros ]]; then task_choices+=$'\nbuild\tROS workspace を build'; fi
    task_action="$(choose "${task_profile}: 操作を選択" "$task_choices")" || exit 1
fi
case "$task_action" in sync|check|shell|run|build) ;; *) fail "不明な操作: $task_action" ;; esac
if [[ "${1:-}" = -- ]]; then shift; fi
case "$task_action" in
    run) (($#)) || fail 'run の後に実行するコマンドを指定してください' ;;
    build) [[ "$task_profile" = ros ]] || fail 'build は ros 環境で使用してください' ;;
    *) (($# == 0)) || fail "${task_action} に追加引数は指定できません" ;;
esac

task_workspace="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
[[ "$task_workspace" = /* && -d "$task_workspace" ]] || fail "ROS workspace がありません: $task_workspace"
task_workspace="$(CDPATH= cd -- "$task_workspace" && pwd -P)"
task_arch="$(uname -m)"
case "$task_arch" in x86_64|aarch64|arm64) ;; *) fail "未対応のアーキテクチャ: $task_arch" ;; esac
if [[ "$task_profile" = vgl-export && ( "$task_arch" != x86_64 || "$(uname -s)" != Linux ) ]]; then
    fail 'vgl-export は Notebook の Linux / x86_64 Docker 専用です'
fi
if [[ ( "$task_profile" = web || "$task_profile" = raceline ) && ( "$task_arch" != x86_64 || "$(uname -s)" != Linux ) ]]; then
    fail "$task_profile は Notebook の Linux / x86_64 Docker 専用です"
fi
task_root="${task_workspace}/.atd-python"
task_venv="${task_root}/${task_arch}/${task_profile}"
if [[ -n "${ATD_PYTHON_PROJECTS_DIR:-}" ]]; then
    task_projects="$ATD_PYTHON_PROJECTS_DIR"
elif [[ -d /workspaces/atd/docker/python ]]; then
    task_projects=/workspaces/atd/docker/python
else
    task_projects=/opt/atd/python
fi
[[ "$task_projects" = /* ]] || fail 'Python project の参照先は絶対パスで指定してください'
task_project="${task_projects}/${task_profile}"
for task_file in pyproject.toml uv.lock; do
    [[ -r "${task_project}/${task_file}" ]] || fail "環境定義がありません: ${task_project}/${task_file}"
done
task_python="${ATD_SYSTEM_PYTHON:-/usr/bin/python3}"
[[ "$task_python" = /* && -x "$task_python" ]] || fail "system Python を実行できません: $task_python"
command -v uv >/dev/null 2>&1 || fail 'uv がありません。Docker イメージを再構築してください'
task_system_site=false
if [[ "$task_profile" = ros ]]; then task_system_site=true; fi

check_venv() {
    "$task_python" -I - "$task_venv" "$task_system_site" <<'PY'
from pathlib import Path
import json
import os
import subprocess
import sys

venv = Path(sys.argv[1])
try:
    values = dict(line.split('=', 1) for line in (venv / 'pyvenv.cfg').read_text().splitlines() if '=' in line)
    values = {key.strip(): value.strip() for key, value in values.items()}
    if values.get('include-system-site-packages', '').lower() != sys.argv[2]:
        sys.exit('ATD Python: 既存 venv の system-site-packages 設定が用途と異なります。別環境として作り直してください: ' + str(venv))
    version = values.get('version_info', values.get('version', '')).split('.')[:2]
    if version != [str(value) for value in sys.version_info[:2]]:
        sys.exit('ATD Python: 既存 venv の Python ABI が system Python と異なります: ' + str(venv))
    executable = venv / 'bin/python'
    if not executable.is_file() or not os.access(executable, os.X_OK):
        sys.exit('ATD Python: 既存 venv の Python がありません: ' + str(venv))
    base = subprocess.check_output([str(executable), '-I', '-c',
        'import json,sys; print(json.dumps(sys._base_executable))'], text=True)
    if Path(json.loads(base)).resolve() != Path(sys._base_executable).resolve():
        sys.exit('ATD Python: 既存 venv の interpreter が system Python と異なります: ' + str(venv))
except (OSError, ValueError, subprocess.CalledProcessError) as error:
    sys.exit('ATD Python: venv の設定を読めません: ' + str(error))
PY
}

printf 'Python profile: %s\nproject: %s\nvenv: %s\nsystem-site-packages: %s\n' \
    "$task_profile" "$task_project" "$task_venv" "$task_system_site"
if [[ "$task_action" = check ]]; then
    [[ -d "$task_venv" ]] || fail 'venv は未作成です。sync を実行してください'
    check_venv
    printf '[OK] uv / project / lock ファイル / venv 設定（依存の実 import・GPU 演算は別途確認）\n'
    exit 0
fi
[[ "$(id -u)" != 0 ]] || fail 'root ではなく通常のコンテナユーザーで実行してください'
[[ -w "$task_workspace" ]] || fail "workspace に書き込めません: $task_workspace"
if [[ -e "$task_venv" || -L "$task_venv" ]]; then check_venv; fi

# 別 venv や親シェルの Python 設定を同期先と誤認しない。
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    PATH="$("$task_python" -I -c 'import os,sys; old=os.path.normpath(sys.argv[1]+"/bin"); print(":".join(path for path in sys.argv[2].split(":") if os.path.normpath(path) != old))' "$VIRTUAL_ENV" "$PATH")"
    export PATH
fi
unset VIRTUAL_ENV PYTHONHOME __PYVENV_LAUNCHER__ UV_MANAGED_PYTHON
export PYTHONNOUSERSITE=1
if [[ "$task_profile" != ros ]]; then
    unset PYTHONPATH PYTHONSTARTUP
    export BASH_ENV=/dev/null ENV=/dev/null
fi
export UV_PROJECT_ENVIRONMENT="$task_venv" UV_CACHE_DIR="${task_root}/cache"
mkdir -p -- "$task_root" "$(dirname -- "$task_venv")"
: > "${task_root}/COLCON_IGNORE"
if [[ ! -e "$task_venv" ]]; then
    task_create=(uv venv --python "$task_python" --no-managed-python --no-python-downloads)
    if [[ "$task_profile" = ros ]]; then task_create+=(--system-site-packages); fi
    "${task_create[@]}" "$task_venv"
    check_venv
fi
uv sync --locked --project "$task_project" --python "$task_python" --no-managed-python --no-python-downloads
check_venv
if [[ "$task_action" = sync ]]; then exit 0; fi

if [[ "$task_profile" = ros ]]; then
    # ROS/overlay が旧 DDS 値を export しても、選択前の通信設定を維持する。
    task_dds_keys=(RMW_IMPLEMENTATION FASTDDS_DEFAULT_PROFILES_FILE ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS RMW_FASTRTPS_USE_QOS_FROM_XML CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER FASTDDS_ENVIRONMENT_FILE ROS_LOCALHOST_ONLY ROS2_EASY_MODE SKIP_DEFAULT_XML)
    task_dds_values=()
    task_dds_presence=()
    for task_key in "${task_dds_keys[@]}"; do
        task_dds_values+=("${!task_key-}")
        if declare -p "$task_key" >/dev/null 2>&1; then task_dds_presence+=(1); else task_dds_presence+=(0); fi
    done
    task_setup="${ATD_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash}"
    [[ -r "$task_setup" ]] || fail "ROS setup がありません: $task_setup"
    set +u
    source "$task_setup"
    if [[ -r "${task_workspace}/install/setup.bash" ]]; then source "${task_workspace}/install/setup.bash"; fi
    set -u
    for task_index in "${!task_dds_keys[@]}"; do
        task_key="${task_dds_keys[$task_index]}"
        if [[ "${task_dds_presence[$task_index]}" = 1 ]]; then export "${task_key}=${task_dds_values[$task_index]}"; else unset "$task_key"; fi
    done
fi
export VIRTUAL_ENV="$task_venv" PATH="${task_venv}/bin:$PATH"
case "$task_action" in
    shell) exec /bin/bash --noprofile --norc -i ;;
    run) exec "$@" ;;
    build)
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
        cd -- "$task_workspace"
        exec "${task_build[@]}"
        ;;
esac

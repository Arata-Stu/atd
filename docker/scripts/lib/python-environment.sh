#!/usr/bin/env bash
# Python 環境の共通処理。source 時には関数定義だけを読み込む。
# 呼出元は fail() を定義し、context → check または sync → activate の順で使う。
# context は既存の task_workspace / task_venv 等を設定し、保存先を一か所で管理する。

atd_python_context() {
    task_profile="$1"
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
    elif [[ -n "${ATD_RUNTIME_DIR:-}" ]]; then
        task_projects="${ATD_RUNTIME_DIR}/python"
    elif [[ -d /workspaces/atd ]]; then
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
}

atd_python_describe() {
    printf 'Python profile: %s\nproject: %s\nvenv: %s\nsystem-site-packages: %s\n' \
        "$task_profile" "$task_project" "$task_venv" "$task_system_site"
}

atd_python_check() {
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

atd_python_sync() {
    [[ "$(id -u)" != 0 ]] || fail 'root ではなく通常のコンテナユーザーで実行してください'
    [[ -w "$task_workspace" ]] || fail "workspace に書き込めません: $task_workspace"
    if [[ -e "$task_venv" || -L "$task_venv" ]]; then atd_python_check; fi

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
        atd_python_check
    fi
    uv sync --locked --project "$task_project" --python "$task_python" --no-managed-python --no-python-downloads
    atd_python_check
}

atd_python_activate() {
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
}

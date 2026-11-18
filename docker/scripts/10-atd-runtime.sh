#!/usr/bin/env bash
# Isaac ROS workspace-entrypoint.sh から root で source される。
# 上流 entrypoint に errexit はないため、各段階の失敗を明示的に伝える。

_atd_prepare_runtime() {
    local task_runtime_dir task_config_dir task_ready_temp task_image_dir task_shell_path task_python_path task_build_path task_vgl_path task_index
    local -a task_dynamic_paths task_baked_paths
    if [[ "$(id -u)" != 0 ]]; then
        printf 'ATD runtime: コンテナの root 起動処理で実行してください。\n' >&2
        return 1
    fi
    if [[ -n "${ATD_RUNTIME_DIR:-}" ]]; then
        task_runtime_dir="$ATD_RUNTIME_DIR"
    elif [[ -d /workspaces/atd/docker ]]; then
        task_runtime_dir=/workspaces/atd/docker
    else
        task_runtime_dir=/opt/atd
    fi
    task_config_dir="${ATD_DDS_CONFIG_DIR:-/etc/atd}"
    task_image_dir="${ATD_IMAGE_DIR:-/opt/atd}"
    task_shell_path="${ATD_SHELL_PATH:-/usr/local/bin/atd-shell}"
    task_python_path="${ATD_PYTHON_PATH:-/usr/local/bin/atd-python}"
    task_build_path="${ATD_BUILD_PATH:-/usr/local/bin/atd-build}"
    task_vgl_path="${ATD_VGL_PATH:-/usr/local/bin/atd-vgl}"
    if [[ "$task_runtime_dir" != /* || "$task_config_dir" != /* \
        || "$task_image_dir" != /* || "$task_shell_path" != /* || "$task_python_path" != /* || "$task_build_path" != /* || "$task_vgl_path" != /* ]]; then
        printf 'ATD runtime: runtime / DDS 設定先は絶対パスを指定してください。\n' >&2
        return 1
    fi
    if ! mkdir -p -- "$task_config_dir"; then return 1; fi
    # 前回の成功記録を残して途中失敗を成功扱いにしない。
    if ! rm -f -- "${task_config_dir}/runtime.ready"; then return 1; fi

    # 起動時に更新できないイメージ内の入口を、最新 revision と誤認しない。
    task_dynamic_paths=(
        "${task_runtime_dir}/Dockerfile.atd"
        "${task_runtime_dir}/scripts/10-atd-runtime.sh"
        "${task_runtime_dir}/scripts/atd-shell.sh"
        "${task_runtime_dir}/scripts/atd.bashrc"
        "${task_runtime_dir}/scripts/atd-python.sh"
        "${task_runtime_dir}/scripts/atd-build.sh"
        "${task_runtime_dir}/scripts/lib/python-environment.sh"
        "${task_runtime_dir}/scripts/atd-vgl.sh"
    )
    task_baked_paths=(
        "${task_image_dir}/Dockerfile.atd"
        "${BASH_SOURCE[0]}"
        "$task_shell_path"
        "${task_config_dir}/bashrc"
        "$task_python_path"
        "$task_build_path"
        "${task_image_dir}/scripts/lib/python-environment.sh"
        "$task_vgl_path"
    )
    for task_index in "${!task_dynamic_paths[@]}"; do
        if ! cmp -s -- "${task_dynamic_paths[$task_index]}" "${task_baked_paths[$task_index]}"; then
            printf 'ATD runtime: イメージ内の起動設定が古いか不足しています: %s\n--build で再構築してください。\n' \
                "${task_dynamic_paths[$task_index]}" >&2
            return 1
        fi
    done

    if ! command -v uv >/dev/null 2>&1 || [[ "$(uv --version | awk '{print $2}')" != 0.13.8 ]]; then
        printf 'ATD runtime: 固定版 uv 0.13.8 がありません。--build で再構築してください。\n' >&2
        return 1
    fi

    if ! bash "${task_runtime_dir}/scripts/install-dependencies.sh"; then
        printf 'ATD runtime: 依存関係の準備に失敗しました。\n' >&2
        return 1
    fi
    if ! bash "${task_runtime_dir}/scripts/configure-serial-access.sh"; then
        printf 'ATD runtime: USB serial のアクセス設定に失敗しました。\n' >&2
        return 1
    fi
    if ! bash "${task_runtime_dir}/scripts/configure-dds.sh"; then
        printf 'ATD runtime: DDS 設定に失敗しました。\n' >&2
        return 1
    fi
    if [[ ! -r "${task_config_dir}/dds.env.sh" ]]; then
        printf 'ATD runtime: DDS 環境ファイルがありません。\n' >&2
        return 1
    fi
    # source して、後続の gosu / 初回コマンドへ環境を引き継ぐ。
    if ! source "${task_config_dir}/dds.env.sh"; then
        printf 'ATD runtime: DDS 環境を読み込めません。\n' >&2
        return 1
    fi
    # ROS の追加 Python 依存も起動時に同期する。ホストに共有する venv は
    # root 所有にせず、entrypoint が準備した通常ユーザーで作成する。
    if ! gosu "${USERNAME:-admin}" env "ATD_PYTHON_PROJECTS_DIR=${task_runtime_dir}/python" \
        "$task_python_path" ros sync; then
        printf 'ATD runtime: ROS 用 uv 環境の準備に失敗しました。\n' >&2
        return 1
    fi
    task_ready_temp="$(mktemp "${task_config_dir}/.runtime-ready.XXXXXX")" || return 1
    if ! printf '%s\n' "${ATD_RUNTIME_REV:-image}" > "$task_ready_temp" \
        || ! chmod 0644 "$task_ready_temp" \
        || ! mv -f -- "$task_ready_temp" "${task_config_dir}/runtime.ready"; then
        rm -f -- "$task_ready_temp"
        return 1
    fi
    printf 'ATD runtime: 準備完了 (%s)\n' "${ATD_RUNTIME_REV:-image}"
}

_atd_prepare_runtime || exit $?
unset -f _atd_prepare_runtime

# source 元の positional parameters を差し替え、ホストの bashrc を読まない。
# 明示的な別コマンドや引数付き Bash は変更しない。
if [[ $# == 1 && "$1" == /bin/bash ]]; then
    set -- "${ATD_SHELL_PATH:-/usr/local/bin/atd-shell}"
fi

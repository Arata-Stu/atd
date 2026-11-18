#!/usr/bin/env bash
# 初回起動と docker exec の両方で使う ATD の shell / command 入口。
set -euo pipefail

task_config_dir="${ATD_DDS_CONFIG_DIR:-/etc/atd}"
if [[ "$task_config_dir" != /* || ! -r "${task_config_dir}/dds.env.sh" ]]; then
    printf 'ATD shell: 起動時の DDS 設定がありません。コンテナを再作成してください。\n' >&2
    exit 1
fi
if [[ ! -r "${task_config_dir}/bashrc" ]]; then
    printf 'ATD shell: ATD の bashrc がありません。イメージを再構築してください。\n' >&2
    exit 1
fi
if (($#)); then
    # ROS setup は nounset を前提としていない。対話部分は bashrc 内で guard する。
    set +u
    source "${task_config_dir}/bashrc"
    set -u
    exec "$@"
fi
source "${task_config_dir}/dds.env.sh"
# CLI が選んだ ROS workspace から対話入室するときは checkout を開く。
task_project_dir="${ATD_PROJECT_DIR:-/workspaces/atd}"
if [[ . -ef "${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}" && -d "$task_project_dir" ]]; then
    cd -- "$task_project_dir"
fi
exec /bin/bash --noprofile --rcfile "${task_config_dir}/bashrc" -i

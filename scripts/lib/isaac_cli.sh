#!/usr/bin/env bash

atd_check_isaac_cli() {
    local task_root="${ATD_SETUP_ROOT:-/}" task_launcher
    task_launcher="${task_root%/}/usr/lib/isaac-ros-cli/run_dev.py"
    if [[ -r "$task_launcher" ]] && grep -Fq 'ISAAC_ROS_WS must be <kart project>/ros2_ws beside packages.repos' "$task_launcher"; then
        printf '%s\n' \
            'kart 専用の Isaac ROS CLI が ATD の workspace を拒否します。' \
            'apt-cache policy isaac-ros-cli で導入版を確認し、docs/setup/pc.md#kart-cli の手順で NVIDIA 公式版に切り替えてください。' >&2
        return 1
    fi
}

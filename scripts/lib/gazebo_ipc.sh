#!/usr/bin/env bash
# 連続配信で、Gazebo と Notebook の Isaac ROS が同じ Fast DDS SHM を参照する。
atd_gazebo_ipc_args() {
    local dry_run="$1" dev_id dev_mode
    # Isaac ROS CLI の既定値。Gazebo を先に起動する場合も同じ IPC を選ぶ。
    sim_ipc_args=(--ipc=host)
    if [[ "$dry_run" = true ]]; then return 0; fi
    dev_id="$(docker container ls -q --filter 'name=^/atd_pc_dev$')" || return 1
    if [[ -z "$dev_id" ]]; then return 0; fi
    dev_mode="$(docker container inspect --format '{{.HostConfig.NetworkMode}} {{.HostConfig.IpcMode}}' "$dev_id")" || return 1
    case "$dev_mode" in
        'host host') ;;
        'host shareable') sim_ipc_args=("--ipc=container:$dev_id") ;;
        'host '*)
            printf 'gazebo: atd_pc_dev の IPC は %s です。SHM を使うには Isaac ROS CLI の既定の host、または shareable で起動してください。UDP で試す場合は serve --dds udp を使用できます。\n' "${dev_mode#host }" >&2
            return 1 ;;
        *)
            printf 'gazebo: atd_pc_dev のネットワークは %s です。ループバック上のシミュレーションを購読するには --network host で起動してください。\n' "${dev_mode%% *}" >&2
            return 1 ;;
    esac
}

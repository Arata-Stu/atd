# ATD 専用の対話 Bash 設定。ホストの ~/.bashrc / ~/.profile は source しない。
if [[ $- == *i* ]]; then
    # Ubuntu の対話 Bash は --rcfile より先に /etc/bash.bashrc を読む。
    # ここで再度 source すると sudo 案内などのシステム初期化が重複する。
    if [[ -r /usr/share/bash-completion/bash_completion ]]; then
        source /usr/share/bash-completion/bash_completion
    fi
    shopt -s checkwinsize
    if [[ -z "${PS1:-}" ]]; then
        PS1='\u@\h:\w\$ '
    fi
fi

if [[ -r "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash" ]]; then
    source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
fi
if [[ -r "${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}/install/setup.bash" ]]; then
    source "${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}/install/setup.bash"
fi

# ROS / overlay の読込み後にも、このコンテナの通信設定を適用する。
if [[ ! -r "${ATD_DDS_CONFIG_DIR:-/etc/atd}/dds.env.sh" ]]; then
    printf 'ATD shell: DDS 環境ファイルがありません。\n' >&2
    exit 1
fi
source "${ATD_DDS_CONFIG_DIR:-/etc/atd}/dds.env.sh" || exit $?

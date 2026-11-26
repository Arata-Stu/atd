"""ROS 起動前に bag 再生用 DDS profile を選ぶ。"""
import os
from pathlib import Path


def select_replay_dds():
    """CLI と Web が要求した再生ポリシーを、最初の ROS participant より前に適用する。"""
    mode = os.environ.get('ATD_REPLAY_DDS')
    if mode is None:
        return  # ROS executable の直接起動では呼び出し元の DDS 設定を引き継ぐ。
    if mode == 'shm':
        path = Path(os.environ.get('ATD_PROJECT_DIR', '/workspaces/atd')) / 'docker/gazebo_monza/fastdds.xml'
        label = '共有メモリ・UDPv4 loopback'
    elif mode == 'udp':
        path = Path(os.environ.get('ATD_DDS_CONFIG_DIR', '/etc/atd')) / 'fastdds.xml'
        label = 'UDPv4 loopback'
    else:
        raise ValueError('ATD_REPLAY_DDS は udp または shm を指定してください')
    if not path.is_file():
        raise ValueError(f'再生用 Fast DDS 設定がありません: {path}')
    # LOCALHOST を指定すると rmw_fastrtps が XML の transport に別の SHM/UDP を追加する。
    os.environ.update(FASTDDS_DEFAULT_PROFILES_FILE=str(path), ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT')
    print(f'別プロセス間の DDS 設定: {label} ({path})', flush=True)

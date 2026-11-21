"""Notebook の VSLAM 再生に使う記録の選択と、ステレオ入力の事前検査。"""

from pathlib import Path
import shutil
import subprocess
import sys

from atd_mapping.bag_inputs import inspect_bag


STEREO_WORKFLOW = {
    'left_image': '/camera/camera/infra1/image_rect_raw',
    'right_image': '/camera/camera/infra2/image_rect_raw',
    'left_info': '/camera/camera/infra1/camera_info',
    'right_info': '/camera/camera/infra2/camera_info',
    'tf_static': '/tf_static',
}
STEREO_TOPICS = tuple(STEREO_WORKFLOW.values())
SLAM_ODOMETRY = '/visual_slam/vis/slam_odometry'
GROUND_TRUTH_PATH = '/atd/replay/ground_truth_path'
GROUND_TRUTH_ERROR = '/atd/replay/position_error'
GROUND_TRUTH_NODE = 'atd_replay_ground_truth'


def select_bag(value, root=Path('/data/record/rosbags')):
    """絶対パス・記録ルートからの相対パスを指定、または対話形式で選択する。"""
    if value is None:
        if not sys.stdin.isatty():
            raise ValueError('非対話端末では atd replay BAG で bag のパスを指定してください')
        choices = sorted(p.parent.relative_to(root).as_posix() for p in root.rglob('metadata.yaml')
                         if p.is_file())
        if not choices:
            raise ValueError(f'{root} に metadata.yaml を含む bag がありません')
        try:
            if shutil.which('fzf'):
                result = subprocess.run(
                    ['fzf', '--read0', '--print0', '--prompt=再生する bag > '],
                    input='\0'.join(choices), stdout=subprocess.PIPE, text=True)
                if result.returncode in (1, 130):
                    return None
                if result.returncode:
                    raise RuntimeError(f'fzf が終了コード {result.returncode} で失敗しました')
                value = result.stdout.removesuffix('\0')
                if value not in choices:
                    raise ValueError('一覧から bag を選んでください')
            else:
                for index, name in enumerate(choices, 1):
                    print(f'{index:2d}) {name}')
                number = input('再生する bag の番号（空欄で終了）: ').strip()
                if not number:
                    return None
                if not number.isdecimal() or not 1 <= int(number) <= len(choices):
                    raise ValueError('一覧にある bag の番号を指定してください')
                value = choices[int(number) - 1]
        except (EOFError, KeyboardInterrupt):
            return None

    bag = (root / Path(value).expanduser()).resolve()
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError(f'bag の metadata.yaml が見つかりません: {bag / "metadata.yaml"}')
    return bag


def validate_bag(path):
    """metadata と /tf_static の記録から、車体と左右カメラの接続を検査する。"""
    inspect_bag(path, STEREO_WORKFLOW, {
        'tracking_mode': 0,
        'base_frame': 'base_link',
        'camera_optical_frames': ['camera_infra1_optical_frame', 'camera_infra2_optical_frame'],
    })

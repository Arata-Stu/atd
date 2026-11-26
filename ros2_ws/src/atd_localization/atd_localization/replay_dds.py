"""再生時の RMW と Fast DDS 設定ファイルを記録する。"""
import hashlib
import os
from pathlib import Path


def replay_dds_profile():
    try:
        from rclpy.utilities import get_rmw_implementation_identifier
        rmw = get_rmw_implementation_identifier()
    except (ImportError, RuntimeError):
        rmw = os.environ.get('RMW_IMPLEMENTATION')
    path = os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE') or os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    digest = None
    if path:
        try:
            profile = Path(path)
            if profile.is_file():
                digest = hashlib.sha256(profile.read_bytes()).hexdigest()
        except OSError:
            pass  # 読めない設定はパスだけを残し、評価自体は実行する。
    return dict(rmw_implementation=rmw, path=path, sha256=digest)

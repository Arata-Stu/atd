"""車両配置に保存された実測ホイールベースを読む。"""
from atd_vehicle_layout.profile import load
from yaml import YAMLError


def load_measured_wheelbase(profile_path):
    try:
        field = load(profile_path)['dimensions']['wheelbase']
    except FileNotFoundError:
        return None
    except (OSError, UnicodeError, YAMLError, ValueError) as error:
        raise ValueError(f'車両配置を読み込めません: {error}') from error
    return field['value_m'] if field['source'] == 'manual' else None

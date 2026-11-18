"""Call the installed C++ SDK helper; no Python librealsense binding is used."""
import json
from pathlib import Path
import subprocess

from .profile import FORMATS, STREAMS, VIDEO, normalize


def executable():
    from ament_index_python.packages import get_package_prefix
    return str(Path(get_package_prefix('atd_sensors')) / 'lib/atd_sensors/atd_realsense_probe')


def call(arguments, program=None):
    result = subprocess.run([program or executable(), *arguments], capture_output=True,
                            text=True, timeout=20, check=False)
    if result.returncode:
        raise ValueError(result.stderr.strip() or 'D455 requested stream combination cannot resolve')
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise ValueError('SDK helper returned invalid JSON') from error


def devices(program=None):
    payload = call(['--list'], program)
    if payload.get('schema_version') != 1 or not isinstance(payload.get('devices'), list):
        raise ValueError('unsupported SDK capability response')
    return [device for device in payload['devices'] if device.get('is_d455') is True]


def select(available, serial=''):
    matches = [device for device in available if not serial or device.get('serial') == serial]
    if len(matches) != 1:
        if not matches:
            raise ValueError('指定したD455が見つかりません' if serial else 'D455が見つかりません。USBとコンテナの認識を確認してください')
        raise ValueError('D455が複数あります。camera_serial または設定TUIで1台を選択してください')
    return matches[0]


def modes(device, key):
    values = set()
    for value in device.get('profiles', []):
        if value.get('stream') != key or value.get('format') != FORMATS[key]:
            continue
        fields = ('width', 'height', 'fps') if key in VIDEO else ('fps',)
        numbers = tuple(value.get(field) for field in fields)
        if all(type(number) is int and number > 0 for number in numbers):
            values.add(numbers)
    return sorted(values)


def unsupported(settings, device):
    errors = []
    for key, value in normalize(settings)['streams'].items():
        if value['enabled']:
            requested = tuple(value[field] for field in (('width', 'height', 'fps') if key in VIDEO else ('fps',)))
            if requested not in modes(device, key):
                errors.append(f'{key}: ' + 'x'.join(map(str, requested)))
    return errors


def validate(settings, device=None, program=None):
    settings = normalize(settings)
    device = device or select(devices(program), settings['serial'])
    if settings['serial'] and device.get('serial') != settings['serial']:
        raise ValueError('D455 capability serial does not match selected settings')
    settings['serial'] = device['serial']
    rejected = unsupported(settings, device)
    if rejected:
        raise ValueError('実機が対応しない要求値: ' + ', '.join(rejected) + '。設定TUIで対応モードを選ぶか対象streamをOFFにしてください')
    arguments = ['--serial', settings['serial'], '--validate']
    for key in STREAMS:
        value = settings['streams'][key]
        if value['enabled']:
            numbers = (value['width'], value['height'], value['fps']) if key in VIDEO else (value['fps'],)
            arguments += ['--stream', key + ':' + ':'.join(map(str, numbers))]
    response = call(arguments, program)
    if response.get('can_resolve') is not True or response.get('serial') != settings['serial']:
        raise ValueError('requested D455 stream combination cannot resolve; no fallback was applied')
    return settings

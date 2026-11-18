"""D455 startup settings; requested defaults are never advertised as hardware capabilities."""
from copy import deepcopy
import json
import os
from pathlib import Path
import re
import tempfile

DEFAULT_PATH = Path('/data/results/runs/bringup/d455.yaml')
STREAMS = ('infra1', 'infra2', 'color', 'depth', 'accel', 'gyro')
VIDEO = STREAMS[:4]
FORMATS = {'infra1': 'Y8', 'infra2': 'Y8', 'color': 'RGB8', 'depth': 'Z16',
           'accel': 'MOTION_XYZ32F', 'gyro': 'MOTION_XYZ32F'}
DEFAULTS = {
    'version': 1, 'serial': '', 'unite_imu_method': 0,
    'streams': {
        'infra1': {'enabled': True, 'width': 640, 'height': 480, 'fps': 90},
        'infra2': {'enabled': True, 'width': 640, 'height': 480, 'fps': 90},
        'color': {'enabled': True, 'width': 640, 'height': 480, 'fps': 30},
        'depth': {'enabled': False, 'width': 640, 'height': 480, 'fps': 30},
        'accel': {'enabled': True, 'fps': 200},
        'gyro': {'enabled': True, 'fps': 400},
    },
}
OVERRIDE_ARGUMENTS = ('enable_infra1', 'enable_infra2', 'enable_color', 'enable_depth',
                      'enable_accel', 'enable_gyro', 'infra_profile', 'color_profile',
                      'depth_profile', 'accel_fps', 'gyro_fps', 'unite_imu_method')


def normalize(value):
    if not isinstance(value, dict) or set(value) - set(DEFAULTS):
        raise ValueError('D455 settings must contain version/serial/streams/unite_imu_method only')
    result = deepcopy(DEFAULTS)
    for key in ('version', 'serial', 'unite_imu_method'):
        if key in value:
            result[key] = value[key]
    if type(result['version']) is not int or result['version'] != 1:
        raise ValueError('unsupported D455 settings version')
    serial = result['serial']
    if not isinstance(serial, str) or (serial and not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}', serial)):
        raise ValueError('serial must be an empty string or the SDK device serial')
    if type(result['unite_imu_method']) is not int or result['unite_imu_method'] not in (0, 1, 2):
        raise ValueError('unite_imu_method must be 0, 1 or 2')
    streams = value.get('streams', {})
    if not isinstance(streams, dict) or set(streams) - set(STREAMS):
        raise ValueError('unknown D455 stream')
    for key, settings in streams.items():
        if not isinstance(settings, dict) or set(settings) - set(result['streams'][key]):
            raise ValueError(f'invalid settings for {key}')
        result['streams'][key].update(settings)
    for key, settings in result['streams'].items():
        if type(settings['enabled']) is not bool:
            raise ValueError(f'{key}.enabled must be boolean')
        for field, number in settings.items():
            if field == 'enabled':
                continue
            maximum = 2000 if field == 'fps' else 16384
            if type(number) is not int or not 1 <= number <= maximum:
                raise ValueError(f'{key}.{field} must be a positive integer <= {maximum}')
    left, right = (result['streams'][key] for key in ('infra1', 'infra2'))
    if left['enabled'] and right['enabled'] and mode(left) != mode(right):
        raise ValueError('driver 4.56.3 uses one shared infra profile for infra1/infra2')
    if result['unite_imu_method'] and not all(result['streams'][key]['enabled'] for key in ('accel', 'gyro')):
        raise ValueError('united IMU requires both accel and gyro; use unite_imu_method=0 for separate streams')
    return result


def mode(settings):
    return tuple(settings[key] for key in ('width', 'height', 'fps'))


def absolute_path(value):
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise ValueError('camera config path must be absolute')
    return path


def load(path=DEFAULT_PATH):
    path = absolute_path(path)
    if not path.exists() and path == DEFAULT_PATH:
        return normalize({})
    text = path.read_text(encoding='utf-8')
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        try:
            import yaml
        except ImportError as error:
            raise ValueError('human-written YAML needs python3-yaml (provided in the ATD Docker image)') from error
        value = yaml.safe_load(text)
    return normalize(value)


def save(path, settings):
    path = absolute_path(path)
    settings = normalize(settings)
    path.parent.mkdir(parents=True, exist_ok=True)
    # JSON is a YAML subset; the same settings remain readable without SDK/PyYAML.
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix=f'.{path.name}.', delete=False) as output:
            temporary = Path(output.name)
            json.dump(settings, output, ensure_ascii=False, indent=2)
            output.write('\n')
            output.flush()
            os.fsync(output.fileno())
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def boolean(value):
    if value not in ('true', 'false'):
        raise ValueError('enabled launch arguments must be true or false')
    return value == 'true'


def with_overrides(settings, values):
    result = deepcopy(settings)
    serial = values.get('serial', '')
    if serial:
        result['serial'] = serial
    for key in STREAMS:
        argument = values.get(f'enable_{key}', '')
        if argument:
            result['streams'][key]['enabled'] = boolean(argument)
    for argument, keys in (('infra_profile', ('infra1', 'infra2')), ('color_profile', ('color',)),
                           ('depth_profile', ('depth',))):
        value = values.get(argument, '')
        if value:
            if not re.fullmatch(r'[0-9]+[x,][0-9]+[x,][0-9]+', value):
                raise ValueError(f'{argument} must be widthxheightxfps')
            width, height, fps = map(int, re.split('[x,]', value))
            for key in keys:
                result['streams'][key].update(width=width, height=height, fps=fps)
    for argument, key in (('accel_fps', 'accel'), ('gyro_fps', 'gyro')):
        value = values.get(argument, '')
        if value:
            if not value.isdecimal():
                raise ValueError(f'{argument} must be an integer')
            result['streams'][key]['fps'] = int(value)
    if values.get('unite_imu_method', ''):
        value = values['unite_imu_method']
        if value not in ('0', '1', '2'):
            raise ValueError('unite_imu_method must be 0, 1 or 2')
        result['unite_imu_method'] = int(value)
    return normalize(result)


def driver_arguments(settings):
    settings = normalize(settings)
    streams = settings['streams']
    # The driver removes a leading underscore; it prevents numeric serial coercion in launch.
    result = {'camera_namespace': 'camera', 'camera_name': 'camera',
              'serial_no': '_' + settings['serial'], 'device_type': 'D455',
              'publish_tf': 'true', 'tf_publish_rate': '0.0',
              'enable_infra': 'false', 'enable_motion': 'false',
              'pointcloud.enable': 'false', 'align_depth.enable': 'false',
              'enable_rgbd': 'false', 'enable_sync': 'false',
              'depth_module.infra1_format': 'Y8', 'depth_module.infra2_format': 'Y8',
              'rgb_camera.color_format': 'RGB8', 'depth_module.depth_format': 'Z16',
              'unite_imu_method': str(settings['unite_imu_method'])}
    for key in STREAMS:
        result[f'enable_{key}'] = str(streams[key]['enabled']).lower()
    infrared = streams['infra1'] if streams['infra1']['enabled'] else streams['infra2']
    for argument, stream in (('depth_module.infra_profile', infrared),
                             ('rgb_camera.color_profile', streams['color']),
                             ('depth_module.depth_profile', streams['depth'])):
        result[argument] = ','.join(map(str, mode(stream)))
    result['accel_fps'] = str(streams['accel']['fps'])
    result['gyro_fps'] = str(streams['gyro']['fps'])
    return result

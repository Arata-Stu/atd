"""Jetson の既存コンテナ内で実機照会→設定→保存→bringupを行う。"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from . import probe
from .profile import DEFAULT_PATH, OVERRIDE_ARGUMENTS, STREAMS, VIDEO, load, mode, save, with_overrides


class Cancelled(Exception):
    pass


def choose(title, choices):
    if not choices:
        raise ValueError('実機が報告した対応モードがありません')
    mapping = dict(choices)
    if len(mapping) != len(choices):
        raise ValueError('duplicate menu choice')
    if shutil.which('fzf'):
        result = subprocess.run(['fzf', '--delimiter=\t', '--with-nth=2', '--height=75%',
                                 '--border', '--prompt', title + ' > '],
                                input=''.join(key + '\t' + label + '\n' for key, label in choices),
                                stdout=subprocess.PIPE, text=True, check=False)
        if result.returncode:
            raise Cancelled()
        key = result.stdout.rstrip('\n').split('\t', 1)[0]
        if key not in mapping:
            raise ValueError('選択内容が候補にありません')
        return key
    print(title, file=sys.stderr)
    for index, (_, label) in enumerate(choices, 1):
        print(f'  {index}) {label}', file=sys.stderr)
    print('番号（空欄で中止）: ', end='', file=sys.stderr, flush=True)
    answer = sys.stdin.readline().strip()
    if not answer:
        raise Cancelled()
    if not answer.isdecimal() or len(answer) > 6 or not 1 <= int(answer) <= len(choices):
        raise ValueError('候補の番号を指定してください')
    return choices[int(answer) - 1][0]


def launch_values(arguments):
    return dict(argument.split(':=', 1) for argument in arguments if ':=' in argument)


def mode_label(value):
    return f'{value[0]}x{value[1]} @ {value[2]} fps' if len(value) == 3 else f'{value[0]} Hz'


def select_device(available):
    if not available:
        raise ValueError('D455が見つかりません。対応モードは未確認です。USBとコンテナの認識を確認してください')
    if len(available) == 1:
        return available[0]
    key = choose('D455を選択', [(value['serial'], value['name'] + ' / ' + value['serial'] +
                                ' / USB ' + value.get('usb_type', '?')) for value in available])
    return probe.select(available, key)


def infrared_modes(settings, device):
    enabled = [key for key in ('infra1', 'infra2') if settings['streams'][key]['enabled']]
    keys = enabled or ['infra1', 'infra2']
    candidates = [set(probe.modes(device, key)) for key in keys]
    return sorted(set.intersection(*candidates)) if candidates else []


def edit_stream(settings, device, key):
    if key.startswith('infra'):
        value = choose(key, [('on', 'ON（左右共通Infraモードを使用）'), ('off', 'OFF')])
        settings['streams'][key]['enabled'] = value == 'on'
        # The driver has one shared infrared mode, including when a previously-off side is enabled.
        if value == 'on':
            other = settings['streams']['infra2' if key == 'infra1' else 'infra1']
            settings['streams'][key].update({field: other[field] for field in ('width', 'height', 'fps')})
        return
    supported = probe.modes(device, key)
    options = [('off', 'OFF')] + [(str(index), mode_label(value)) for index, value in enumerate(supported)]
    selected = choose(key + '（実機対応モード）', options)
    settings['streams'][key]['enabled'] = selected != 'off'
    if selected != 'off':
        fields = ('width', 'height', 'fps') if key in VIDEO else ('fps',)
        settings['streams'][key].update(zip(fields, supported[int(selected)]))
    if not all(settings['streams'][key]['enabled'] for key in ('accel', 'gyro')):
        settings['unite_imu_method'] = 0


def launch_arguments(arguments, config_path):
    # TUI edits replace incoming camera overrides; input/vehicle arguments remain literal and ordered.
    owned = {'camera_config', 'camera_serial', *OVERRIDE_ARGUMENTS}
    return [argument for argument in arguments if argument.split(':=', 1)[0] not in owned] + [
        'camera_config:=' + str(config_path)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bringup-profile', choices=('d455', 'manual'), default='d455')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('launch_arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    forwarded = list(args.launch_arguments)
    if forwarded[:1] == ['--']:
        forwarded.pop(0)
    values = launch_values(forwarded)
    path = Path(values.get('camera_config', str(DEFAULT_PATH)))
    settings = with_overrides(load(path), {**values, 'serial': values.get('camera_serial', '')})
    if args.dry_run:
        print('D455要求設定（実機capability未照会、対応確認済みではありません）')
        print(json.dumps(settings, ensure_ascii=False, indent=2))
        return 0
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise ValueError('設定TUIは対話端末で実行してください。非TTYではbringupのlaunch引数/保存設定を使います')
    available = probe.devices()
    device = probe.select(available, settings['serial']) if settings['serial'] else select_device(available)
    settings['serial'] = device['serial']
    while True:
        print(f'\nD455: {device["name"]} / {device["serial"]} / USB {device.get("usb_type", "?")}', file=sys.stderr)
        print(f'保存先: {path}', file=sys.stderr)
        rejected = probe.unsupported(settings, device)
        if rejected:
            print('未対応の要求値: ' + ', '.join(rejected) + '（自動変更しません）', file=sys.stderr)
        choices = [('device', '接続するD455を選び直す')]
        for key in STREAMS:
            value = settings['streams'][key]
            numbers = mode(value) if key in VIDEO else (value['fps'],)
            label = key + ': ' + ('ON / ' + mode_label(numbers) if value['enabled'] else 'OFF')
            choices.append((key, label))
        choices += [('infra_mode', '左右共通 Infra 解像度・FPS'),
                    ('imu_method', f'IMU出力: {settings["unite_imu_method"]} (0=別々 / 1=copy / 2=補間)'),
                    ('start', '保存して起動'), ('save', '保存のみで終了'), ('quit', '保存せず終了')]
        try:
            selected = choose('D455設定', choices)
            if selected == 'quit':
                return 0
            if selected == 'device':
                device = select_device(available)
                settings['serial'] = device['serial']
            elif selected == 'infra_mode':
                supported = infrared_modes(settings, device)
                value = choose('Infra実機対応モード', [(str(index), mode_label(item))
                                                    for index, item in enumerate(supported)])
                for key in ('infra1', 'infra2'):
                    settings['streams'][key].update(zip(('width', 'height', 'fps'), supported[int(value)]))
            elif selected == 'imu_method':
                options = [('0', '別々の accel / gyro sample')]
                if all(settings['streams'][key]['enabled'] for key in ('accel', 'gyro')):
                    options += [('1', 'copyで統合'), ('2', '線形補間で統合')]
                settings['unite_imu_method'] = int(choose('IMU出力', options))
            elif selected in STREAMS:
                edit_stream(settings, device, selected)
            elif selected in ('start', 'save'):
                settings = probe.validate(settings, device)
                save(path, settings)
                print(f'保存しました: {path}', file=sys.stderr)
                if selected == 'save':
                    return 0
                command = ['ros2', 'launch', 'atd_bringup', args.bringup_profile + '.launch.py',
                           *launch_arguments(forwarded, path)]
                os.execvp(command[0], command)
        except Cancelled:
            return 0
        except (ValueError, OSError, subprocess.TimeoutExpired) as error:
            print(f'D455設定: {error}', file=sys.stderr)


def entrypoint():
    try:
        return main()
    except Cancelled:
        return 0
    except (ValueError, OSError, subprocess.TimeoutExpired) as error:
        print(f'D455設定: {error}', file=sys.stderr)
        return 1

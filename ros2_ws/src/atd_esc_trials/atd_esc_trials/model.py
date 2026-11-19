"""指令列から推定する ESC の状態。実車の動きは停止確認で別途扱う。"""
from dataclasses import dataclass
import math


@dataclass
class EscState:
    state: str = 'neutral'
    throttle: float = 0.0

    def apply(self, step):
        action = step['action']
        if action == 'forward':
            self.state = 'forward'
            self.throttle = step['level']
        elif action == 'brake' and self.state in ('forward', 'braking'):
            self.state = 'braking'
            self.throttle = -step['level']
        elif action == 'wait_stopped' and self.state == 'braking':
            self.state = 'wait_stopped'  # 停止を確認するまで同じ負指令を維持
        elif action == 'neutral':
            self.state = 'neutral_after_stop' if self.state in ('stopped_braking', 'neutral_after_stop') else 'neutral'
            self.throttle = 0.0
        elif action == 'reverse' and self.state in ('neutral_after_stop', 'reverse'):
            self.state = 'reverse'
            self.throttle = -step['level']
        else:
            raise ValueError(f'{self.state} から {action} へは移行できません。'
                             '後退前に forward → brake → wait_stopped → neutral を指定してください')

    def confirm_stopped(self):
        if self.state != 'wait_stopped':
            raise ValueError('停止確認の手順ではありません')
        self.state = 'stopped_braking'


def _number(value):
    return type(value) is int or type(value) is float and math.isfinite(value)


def validate_steps(steps):
    if not isinstance(steps, list) or not 1 <= len(steps) <= 128:
        raise ValueError('手順は 1～128 個で指定してください')
    normalized = []
    machine = EscState()
    for index, item in enumerate(steps, start=1):
        try:
            if not isinstance(item, dict) or item.keys() - {'action', 'level', 'duration_s'}:
                raise ValueError('手順の項目が不正です')
            action = item.get('action')
            duration = item.get('duration_s')
            if action not in ('forward', 'neutral', 'brake', 'wait_stopped', 'reverse'):
                raise ValueError('操作の種類が不正です')
            if not _number(duration) or not 0.02 <= duration <= 300:
                raise ValueError('時間は 0.02～300 秒で指定してください')
            step = {'action': action, 'duration_s': float(duration)}
            level = item.get('level', 0)
            if action in ('forward', 'brake', 'reverse'):
                if not _number(level) or not 0 < level <= 1:
                    raise ValueError('指令の大きさは 0 より大きく 1 以下で指定してください')
                if round(level * 1000) < 2:
                    raise ValueError('指令が Pico の PWM 分解能 1 µs 未満です')
                step['level'] = float(level)
            elif not _number(level) or level != 0:
                raise ValueError('この手順は指令の大きさを指定できません')
            machine.apply(step)
            if action == 'wait_stopped':
                # 事前の経路検査のみ。実行側では新しい停止確認が必要。
                machine.confirm_stopped()
        except ValueError as exc:
            raise ValueError(f'手順 {index}: {exc}') from exc
        normalized.append(step)
    return normalized

"""低速自動走行の JPB1 出力。前進中の停止要求は一続きの制動の後に解除する。"""
import math
import time
from time import time_ns

from atd_esc_trials.board import ManualBoard


class AutoBoard(ManualBoard):
    def __init__(self, port='', serial='', *, brake_level, brake_hold_s):
        super().__init__(port, serial)
        if (type(brake_level) not in (float, int) or not math.isfinite(brake_level)
                or not 0 < brake_level <= 1 or round(brake_level * 1000) < 2
                or type(brake_hold_s) not in (float, int) or not .1 <= brake_hold_s <= 2.):
            raise ValueError('実測したブレーキ量と保持時間 (0.1〜2 s) が必要です')
        self.brake_level = brake_level
        self.brake_hold_s = brake_hold_s
        self._first_forward_report = None
        self._brake_until = None
        self._last_command = None

    def _write(self, throttle, enabled, reverse=False, steering=0.):
        sequence = self._sequence
        try:
            super()._write(throttle, enabled, reverse, steering)
        except (OSError, RuntimeError):
            self._last_command = None
            raise
        # JPB1 の整数値と Rev6 host_pwm の整数除算（負の舵角も 0 方向へ切り捨て）。
        steer, longitudinal = round(steering * 1000), round(throttle * 1000)
        if not enabled:
            kind = 'disable'
        elif longitudinal < 0:
            kind = 'brake'
        else:
            kind = 'arm' if self._phase == 'arming' else 'drive'
        self._last_command = dict(stamp_ns=time_ns(), sequence=sequence, kind=kind,
                                  steering=steer / 1000, throttle=longitudinal / 1000,
                                  servo_us=1500 + int(steer / 2) if enabled else None,
                                  esc_us=1500 - int(longitudinal / 2) if enabled else None)

    def latest_command(self, wall_ns, steady_ns):
        cmd = self._last_command
        if (self.fd is not None and cmd and 0 <= wall_ns - cmd['stamp_ns'] < 150_000_000
                and 0 <= steady_ns - round(self._sent * 1e9) < 150_000_000):
            return cmd
        return None

    @property
    def braking(self):
        return self._brake_until is not None

    def disarm(self):
        self._brake_until = None
        self._first_forward_report = None
        super().disarm()

    def drive(self, steering, throttle):
        if self.braking or type(throttle) not in (int, float) or not 0 <= throttle <= 1:
            self.disarm()
            raise ValueError('自動走行では制動中の再加速と後退指令を受理できません')
        if not super().drive(steering, throttle):
            return False
        if round(throttle * 1000) < 2:
            self._first_forward_report = None
        elif self._first_forward_report is None:
            # この指令より後の S フレームで基板の前進出力を確認する。
            self._first_forward_report = self._sample['sequence']
        return True

    def _send_brake(self):
        try:
            self._write(-self.brake_level, True, reverse=False)
        except (OSError, RuntimeError):
            self.close()
            raise

    def stop_forward(self):
        """呼出側で正の車速を確認済み。前進PWMの実報告もある場合のみ一回制動する。"""
        if self.braking:
            return
        sample = super().poll()
        if (self._phase != 'active' or not sample or not sample['host_selected']
                or self._first_forward_report is None or sample['sequence'] == self._first_forward_report
                or sample['esc_us'] >= 1500):
            self.disarm()
            return
        self._first_forward_report = None
        self._brake_until = time.monotonic() + self.brake_hold_s
        self._send_brake()

    def poll(self):
        sample = super().poll()
        if self.braking:
            # CH3 は Pico が判定する。HOST が切れた後や中立を挟んだ後に制動を再発行しない。
            if (not sample or not sample['host_selected'] or sample['active_path'] != 2
                    or sample['esc_us'] == 1500 or time.monotonic() >= self._brake_until):
                self.disarm()
            else:
                self._send_brake()
        return sample

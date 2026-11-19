"""Pico Rev6 の CDC0 を排他使用するローカル ESC 試験用 JPB1 通信。"""
import binascii
import fcntl
import math
import os
from pathlib import Path
import re
import select
import stat
import termios
import time
import tty


_STATUS_FIELDS = ('sequence', 'rx_steer_us', 'rx_throttle_us', 'rx_mode_us', 'servo_us',
                  'esc_us', 'vbec_mv', 'selector', 'active_path', 'fault_bits')
# S は 50ms 周期。指令 timeout (200ms) より先にホスト側でもセッションを失効させる。
_FRESH_S = .15


def _serial(tty_device):
    actual = tty_device.resolve(strict=True)
    interface = None
    for node in (actual, *actual.parents):
        if interface is None and (node / 'bInterfaceNumber').is_file():
            interface = (node / 'bInterfaceNumber').read_text().strip()
        if (node / 'idVendor').is_file():
            attrs = {name: (node / name).read_text().strip() for name in ('idVendor', 'idProduct', 'product', 'serial')}
            if (interface == '00' and attrs['idVendor'].lower() == 'cafe' and attrs['idProduct'] == '4002'
                    and attrs['product'] == 'JPBB Rev6 Pico' and re.fullmatch('[0-9a-fA-F]{16}', attrs['serial'])):
                return attrs['serial']
            break
    raise ValueError('選択したポートは JPBB Rev6 Pico の CDC0 (if00) ではありません')


def discover_ports(sysfs=Path('/sys/class/tty'), dev=Path('/dev')):
    candidates = []
    for tty_dir in sorted(sysfs.glob('ttyACM*')):
        try:
            _serial(tty_dir / 'device')
            if (dev / tty_dir.name).is_char_device():
                candidates.append(str(dev / tty_dir.name))
        except (ValueError, OSError):
            continue
    return candidates


def _identify(fd, sysfs=Path('/sys/dev/char')):
    device = os.fstat(fd)
    if not stat.S_ISCHR(device.st_mode):
        raise ValueError('Pico Rev6 の CDC0 tty を指定してください')
    return _serial(sysfs / f'{os.major(device.st_rdev)}:{os.minor(device.st_rdev)}' / 'device')


def _packet(payload):
    return payload + b',' + f'{binascii.crc_hqx(payload, 0xffff):04X}\r\n'.encode('ascii')


def _decode(line):
    payload, _, crc = line.rpartition(b',')
    if (not re.fullmatch(rb'[0-9A-Fa-f]{4}', crc)
            or binascii.crc_hqx(payload, 0xffff) != int(crc, 16)):
        raise RuntimeError('Pico 状態フレームの CRC が一致しません')
    parts = payload.split(b',')
    if (parts[:2] != [b'JPB1', b'S'] or len(parts) != 12
            or any(not re.fullmatch(rb'[0-9]{1,10}', p) for p in parts[2:])):
        raise RuntimeError('Pico から不正な JPB1 状態フレームを受け取りました')
    fields = dict(zip(_STATUS_FIELDS, map(int, parts[2:]), strict=True))
    if (fields['sequence'] > 0xffffffff or fields['fault_bits'] > 0xffffffff or fields['vbec_mv'] > 65535
            or fields['selector'] > 1 or fields['active_path'] > 3
            or any(not 1000 <= fields[k] <= 2000 for k in ('esc_us', 'servo_us'))):
        raise RuntimeError('Pico の状態値が範囲外です')
    return fields


class Board:
    """Jetson 上で一試験だけを実行する。異常・disable後の再armには新しいセッションが必要。"""

    def __init__(self, port):
        self.port = port
        self.fd = None
        self.serial = None
        self._sequence = 0
        self._buffer = bytearray()
        self._sample = None
        self._received = 0.
        self._sent = 0.
        self._armed = False
        self._ended = False
        self._verified = False
        self._short_write = False

    def __enter__(self):
        if self._ended or self.fd is not None:
            raise RuntimeError('この試験のポートは使用済みです')
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
        try:
            self.serial = _identify(self.fd)
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            tty.setraw(self.fd, termios.TCSANOW)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
            self._verified = True
            self._write(0, False)
            self._wait_path(0)
            return self
        except BaseException as error:
            self.__exit__(type(error), error, None)
            raise

    def __exit__(self, *exc):
        try:
            if self._verified and self.fd is not None:
                if exc[0] is None and not self._ended:
                    self.command(0, False)
                elif exc[0] is not None:
                    self._abort()
        finally:
            self._ended = True
            if self.fd is not None:
                os.close(self.fd)
                self.fd = None

    def _write(self, throttle, enabled, reverse=False):
        milli = round(throttle * 1000)
        negative = max(0, -milli)
        forward, reverse_milli, brake = max(0, milli), negative if reverse else 0, 0 if reverse else negative
        flags = 3 if enabled else 1
        payload = f'JPB1,C,{self._sequence},0,{forward},{reverse_milli},{brake},{flags}'
        frame = _packet(payload.encode('ascii'))
        # 途中で途切れた前の指令を firmware の行パーサで破棄してから disable を送る。
        if self._short_write:
            frame = b'\x00\n' + frame
        if os.write(self.fd, frame) != len(frame):
            self._short_write = True
            raise RuntimeError('Pico に制御フレーム全体を書き込めませんでした')
        self._short_write = False
        self._sequence = (self._sequence + 1) & 0xffffffff
        self._sent = time.monotonic()

    def _receive(self, *, wait=0., expected_path=None):
        if not select.select([self.fd], [], [], wait)[0]:
            return
        chunk = os.read(self.fd, 4096)
        if not chunk:
            raise RuntimeError('Pico との通信が切断されました')
        self._buffer.extend(chunk)
        while b'\n' in self._buffer:
            line, _, rest = self._buffer.partition(b'\n')
            self._buffer = bytearray(rest)
            sample = _decode(bytes(line).rstrip(b'\r'))
            if self._sample and sample['sequence'] != ((self._sample['sequence'] + 1) & 0xffffffff):
                raise RuntimeError('Pico の状態シーケンスが途切れました')
            if not (sample['selector'] == 1 and sample['fault_bits'] == 0
                    and 1700 <= sample['rx_mode_us'] <= 2000
                    and all(1000 <= sample[k] <= 2000 for k in ('rx_steer_us', 'rx_throttle_us'))):
                raise RuntimeError('プロポが HOST を許可していないか、Pico が異常を報告しました')
            if expected_path is not None and sample['active_path'] != expected_path:
                raise RuntimeError('Pico の操作権が変化しました')
            if expected_path == 2 and sample['servo_us'] != 1500:
                raise RuntimeError('試験中にステアリングが中立から変化しました')
            self._sample = sample
            self._received = time.monotonic()
        if len(self._buffer) > 192:
            raise RuntimeError('Pico 状態フレームが長すぎます')

    def _wait_path(self, target):
        # S の seq は基板独自の連番。実際に選ばれた path と PWM を確認する。
        deadline = time.monotonic() + _FRESH_S
        while (remaining := deadline - time.monotonic()) > 0:
            self._receive(wait=remaining)
            if self._sample and self._sample['active_path'] == target:
                if self._sample['servo_us'] != 1500 or self._sample['esc_us'] != 1500:
                    raise RuntimeError('Pico の中立出力を確認できません')
                return
        raise RuntimeError('Pico の操作権と中立出力を時間内に確認できません')

    def _fresh(self):
        # プロセス停止中に溜まった状態で操作権を復活させないよう、読む前に経過時間を検査。
        if time.monotonic() - self._received >= _FRESH_S or (self._armed and time.monotonic() - self._sent >= _FRESH_S):
            raise RuntimeError('Pico の状態または試験指令が古くなりました')
        self._receive(expected_path=2 if self._armed else 0)

    def _abort(self):
        self._ended = True
        try:
            self._write(0, False)
        except (OSError, RuntimeError):
            pass

    def feedback(self):
        if self.fd is None or self._ended:
            raise RuntimeError('試験用ポートが有効ではありません')
        try:
            self._fresh()
            return dict(self._sample)
        except (OSError, RuntimeError):
            self._abort()
            raise

    def command(self, throttle, enabled, *, reverse=False):
        if self.fd is None or (self._ended and enabled):
            raise RuntimeError('試験セッションが失効しています')
        try:
            if (isinstance(throttle, bool) or not math.isfinite(throttle) or abs(throttle) > 1 or not isinstance(enabled, bool)
                    or not isinstance(reverse, bool) or (not enabled and throttle != 0) or (reverse and throttle > 0)):
                raise ValueError('ESC 試験の指令値が不正です')
            if enabled:
                self._fresh()
                if not self._armed and throttle != 0:
                    raise RuntimeError('最初の指令は中立で arm してください')
            self._write(throttle, enabled, reverse)
            if enabled and not self._armed:
                self._wait_path(2)
            elif not enabled and not self._ended:
                self._wait_path(0)
            self._armed = enabled
            self._ended = not enabled
        except (ValueError, TypeError, OSError, RuntimeError):
            self._abort()
            raise

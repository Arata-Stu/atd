"""速度観測に対応する操作経路・S.BUS CH2・DualSense の正規化要求を照合する。"""
from numbers import Integral


PROPO_MAX_AGE_NS = 50_000_000
# atd_teleop の既定 input_timeout_s。再配信時刻で元の操作の有効期限を延ばさない。
JOY_MAX_AGE_NS = 250_000_000


def _get(obj, key):
    return obj.get(key) if isinstance(obj, dict) else getattr(obj, key, None)


def pico_reading(msg):
    """(Pico 経路, 手動 CH2 raw または None, CH2 欠測理由)。"""
    if _get(_get(msg, 'header'), 'frame_id') != 'pico' or not _get(msg, 'instance_id') or not _get(msg, 'serial'):
        return 'unknown', None, 'Pico の座標系または接続識別子が不正です'
    path = _get(msg, 'active_path')
    if type(path) is not int or path not in (0, 1, 2, 3):
        return 'unknown', None, 'Pico の操作経路が不明です'
    if path == 0:
        return 'standby', None, 'Pico は待機中です'
    if path == 3:
        return 'fault', None, 'Pico は異常経路を報告しています'
    if _get(msg, 'power_ready') is not True or _get(msg, 'fault_bits') != 0:
        return 'unknown', None, 'Pico の電源または異常状態を確認してください'
    if path == 2:
        return 'host', None, 'Pico が HOST 経路にあります'
    flags, age = _get(msg, 'sbus_flags'), _get(msg, 'sbus_age_ms')
    if (_get(msg, 'rc_valid') is not True or _get(msg, 'sbus_fresh') is not True
            or type(flags) is not int or not 0 <= flags <= 15 or flags & 12
            or type(age) is not int or not 0 <= age <= 50):
        return 'unknown', None, 'プロポの S.BUS 受信状態が無効です'
    channels = _get(msg, 'channels')
    raw = channels[1] if channels is not None and len(channels) == 16 else None
    # rosidl_generator_py は固定長の uint16[] に NumPy の整数を使う。
    if isinstance(raw, bool) or not isinstance(raw, Integral) or not 0 <= raw <= 2047:
        return 'unknown', None, 'Pico の CH2 raw が不正です'
    return 'propo', int(raw), ''


def manual_reading(msg):
    _, raw, reason = pico_reading(msg)
    return raw, reason


def joy_reading(msg):
    """(正規化スロットルまたは None, enable 要求または None, 欠測理由)。"""
    if _get(msg, 'source') != 'joy':
        return None, None, 'DriveCommand の入力元が Joy 以外です'
    if _get(msg, 'valid') is not True:
        return None, None, 'DualSense の運転要求が無効です'
    throttle, enable = _get(msg, 'throttle'), _get(msg, 'enable_requested')
    if type(throttle) not in (float, int) or not -1 <= throttle <= 1 or type(enable) is not bool:
        return None, None, 'DualSense の正規化スロットルまたは enable 要求が不正です'
    return float(throttle), enable, ''


class JoyStream:
    """同じ入力を繰り返す teleop の配信を、元入力の更新と失効の時系列にする。"""

    def __init__(self):
        self.watermark = 0
        self.last = None
        self.invalid_at = 0

    def event(self, msg, received_ns):
        """有効開始時刻・元入力時刻・正規化スロットル・enable 要求・理由を返す。"""
        stamp = _get(_get(msg, 'header'), 'stamp')
        sec, nsec = _get(stamp, 'sec'), _get(stamp, 'nanosec')
        original = (sec * 1_000_000_000 + nsec if type(sec) is int and sec >= 0
                    and type(nsec) is int and 0 <= nsec < 1_000_000_000 else 0)
        if type(received_ns) is not int or received_ns <= 0:
            return None
        reading = joy_reading(msg)
        throttle, _, _ = reading
        if throttle is not None:
            if original <= 0 or original > received_ns:
                reading = (None, None, 'DualSense の元入力時刻が不正です')
            elif original <= self.watermark:
                if original == self.watermark and self.last == reading:
                    return None
                if self.last and self.last[0] is None:
                    return None
                reading = (None, None, 'DualSense の元入力時刻が逆行または競合しています')
            elif original <= self.invalid_at:
                return None

        if 0 < original <= received_ns:
            self.watermark = max(self.watermark, original)
        if reading[0] is None:
            self.invalid_at = max(self.invalid_at, received_ns)
            if self.last == reading:
                return None
            # 切断・設定変更・期限切れは古い Joy stamp のまま、または時刻ゼロで通知される。
            # 直前まで正しかった過去の操作を消さず、通知された時刻から欠測とする。
            effective = received_ns
        else:
            effective = original
        self.last = reading
        return effective, original, *reading


def match_manual(stamp_ns, pico=None, joy=None, *, pico_missing='', joy_missing='', joy_drives=True):
    """直近の各入力イベントを速度の元時刻で照合。範囲外や未観測は None。"""
    path = 'unknown'
    raw = raw_age = throttle = joy_age = enable = None
    propo_reason = pico_missing or 'この速度観測に対応する Pico 入力がありません'
    joy_reason = joy_missing or 'この速度観測に対応する DualSense 入力がありません'
    if pico is not None:
        source, selected, value, reason = pico
        if 0 <= stamp_ns - source <= PROPO_MAX_AGE_NS:
            path, raw, propo_reason = selected, value, reason
            if raw is not None:
                raw_age = (stamp_ns - source) / 1e9
        else:
            propo_reason = '速度観測の直前 50 ms に Pico の入力がありません'
    if joy is not None:
        source, value, requested, reason = joy
        if value is None:
            joy_reason = reason
        elif 0 <= stamp_ns - source <= JOY_MAX_AGE_NS:
            throttle, enable, joy_reason = value, requested, ''
            joy_age = (stamp_ns - source) / 1e9
        else:
            joy_reason = '速度観測の直前 250 ms に新しい DualSense 入力がありません'
    return dict(propo_raw=raw, propo_age_s=raw_age, propo_reason=propo_reason,
                joy_throttle=throttle, joy_age_s=joy_age, joy_enable_requested=enable, joy_reason=joy_reason,
                manual_source='host_joy' if joy_drives and path == 'host' and throttle is not None else path)

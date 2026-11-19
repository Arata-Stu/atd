"""Jetson の有効な Joy 要求を、プロポが選んだ HOST セッションで送る。"""
import math


JOY_TIMEOUT_NS = 250_000_000


def drive_feedback(board, gate, session_id, reason):
    """最後の基板報告と、この Jetson の Joy 送信状態を配信用にまとめる。"""
    fields = dict(stamp_ns=0, has_report=False, serial=(board.serial or '') if board else '',
                  session_id=session_id, joy_active=False, reason=reason)
    report = board.latest_report() if board else None
    if report is None:
        if board is not None:
            fields['reason'] = 'Pico CDC0 の状態フレームを待機中'
        return fields
    stamp_ns, sample = report
    if not 0 <= sample['rx_mode_us'] <= 65535:
        fields['reason'] = 'Pico が報告した CH3 値が範囲外です'
        return fields
    fields.update(stamp_ns=stamp_ns, has_report=True, sequence_number=sample['sequence'],
                  joy_active=(gate.active and not gate.interrupted and sample['host_selected']
                              and sample['active_path'] == 2 and sample['fault_bits'] == 0))
    for key in ('rx_mode_us', 'selector', 'active_path', 'fault_bits', 'servo_us', 'esc_us'):
        fields[key] = sample[key]
    return fields


class DriveGate:
    def __init__(self):
        self.host_since = None
        self.stamp = 0
        self.received = 0
        self.axes = None
        self.requested = False
        self.active = False
        self.interrupted = False

    def invalidate(self):
        self.axes = None
        self.interrupted = True

    def receive(self, msg, wall_ns, steady_ns):
        stamp = msg.header.stamp
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        axes = (msg.steering, msg.throttle)
        if (not msg.valid or msg.source != 'joy' or msg.header.frame_id != 'atd_gamepad'
                or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or source_ns <= 0
                or not 0 <= wall_ns - source_ns < JOY_TIMEOUT_NS
                or not all(math.isfinite(v) and -1 <= v <= 1 for v in axes)):
            self.invalidate()
            return
        if source_ns <= self.stamp:
            if source_ns != self.stamp or self.axes != axes:
                self.invalidate()
            return
        if self.axes is not None and (source_ns - self.stamp >= JOY_TIMEOUT_NS
                                      or steady_ns - self.received >= JOY_TIMEOUT_NS):
            self.invalidate()
        self.axes = axes
        self.stamp = source_ns
        self.received = steady_ns

    def stop(self, board):
        if self.requested:
            board.disarm()
        self.requested = self.active = False
        self.host_since = None

    def step(self, board, status, wall_ns, steady_ns):
        if self.interrupted:
            self.stop(board)
            self.interrupted = False
        if not status or not status['host_selected']:
            self.stop(board)
            return 'プロポ CH3 の HOST 選択を待機中'
        if self.host_since is None:
            self.host_since = wall_ns
        if (self.axes is None or self.stamp <= self.host_since
                or not 0 <= wall_ns - self.stamp < JOY_TIMEOUT_NS
                or not 0 <= steady_ns - self.received < JOY_TIMEOUT_NS):
            if self.requested:
                self.stop(board)
            return '新しい有効な Joy 入力を待機中'
        if not self.requested:
            if self.axes != (0., 0.):
                return '操舵・スロットルを中立に戻してください'
            if status['active_path'] != 0:
                return 'Pico の HOST 待機状態を確認中'
            if not board.arm():
                self.stop(board)
                return 'プロポ CH3 の HOST 選択を待機中'
            self.requested = True
            return 'Pico の HOST・中立出力を確認中'
        if not self.active:
            if status['active_path'] != 2:
                return 'Pico の HOST・中立出力を確認中'
            if status['servo_us'] != 1500 or status['esc_us'] != 1500:
                self.stop(board)
                return 'Pico の HOST 初期出力が中立と一致しません'
            self.active = True
        elif status['active_path'] != 2:
            self.stop(board)
            return 'Pico の HOST 経路が解除されました'
        if not board.drive(*self.axes):
            self.stop(board)
            return 'プロポ CH3 の HOST 選択を待機中'
        return 'DualSense 手動運転中（CH3 HOST）'

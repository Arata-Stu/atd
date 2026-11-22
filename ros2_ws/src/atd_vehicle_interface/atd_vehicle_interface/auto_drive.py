"""同じ車体観測から得た低速前進指令を、Pico の CH3 が許可した間だけ送る。"""
from dataclasses import dataclass
import math
import re

from atd_control.core import CONTROLLER, STANDSTILL_MPS
from .auto_status import DriveSpeeds, MAX_PENDING_FRAMES

NS = 1_000_000_000
MAX_AGE_NS = 250_000_000
TOPICS = ('localization', 'steering', 'speed')


@dataclass
class Observation:
    msg: object
    expires: int
    signature: tuple


class AutoDriveGate:
    def __init__(self, map_id, line, max_speed_mps, calibration):
        if not map_id or line not in ('raceline', 'centerline', 'custom'):
            raise ValueError('地図 ID と保存済み走行ラインを選択してください')
        if not math.isfinite(max_speed_mps) or not 0 < max_speed_mps <= 1.:
            raise ValueError('低速前進の上限は 0 より大きく 1.0 m/s 以下で指定してください')
        self.map_id, self.line, self.max_speed = map_id, line, max_speed_mps
        self.calibration = calibration
        self.speeds = DriveSpeeds(map_id, line, MAX_AGE_NS)
        self.last_stop = None
        self.samples = {kind: {} for kind in TOPICS}
        self.identity = None
        self.velocity = None
        self.rc_seen = False
        self.host_since = None
        self.requested = self.active = False
        self.fault = ''

    def invalidate(self, reason):
        for rows in self.samples.values():
            rows.clear()
        if self.host_since is not None:
            self.fault = self.fault or reason

    def receive(self, kind, msg, wall, steady):
        self.speeds.receive(kind, msg, wall, steady)
        stamp = msg.header.stamp
        ns = stamp.sec * NS + stamp.nanosec
        is_loc = kind == 'localization'
        ttl = MAX_AGE_NS
        if not is_loc:
            if not msg.valid or not math.isfinite(msg.validity_s) or msg.validity_s <= 0:
                self.invalidate(msg.reason or '制御指令が無効です')
                return
            ttl = min(ttl, int(min(1., msg.validity_s) * NS))
        if (msg.header.frame_id != 'map' or msg.map_id != self.map_id or not msg.session_id
                or not all(re.fullmatch('[0-9a-f]{64}', v) for v in (msg.map_sha256, msg.hdmap_sha256))
                or not is_loc and msg.line != self.line):
            self.invalidate('走行指令の地図・ライン・座標系または追跡セッションが不一致です')
            return
        if ns <= 0 or not 0 <= stamp.nanosec < NS or not 0 <= wall - ns < ttl:
            self.invalidate('元観測の時刻が有効期間外です')
            return
        identity = (msg.map_sha256, msg.hdmap_sha256, msg.session_id, msg.epoch)
        if self.identity != identity:
            if self.identity is not None:
                # 追跡喪失でも localization は epoch を進める。この場合は下の診断が停止理由になる。
                tracking_loss = (is_loc and msg.state != 'tracking' and identity[:3] == self.identity[:3]
                                 and msg.epoch > self.identity[3])
                if not tracking_loss:
                    self.invalidate('地図または追跡セッションが変わりました')
            self.identity = identity
        if is_loc:
            if msg.state != 'tracking' or not math.isfinite(msg.speed_mps):
                self.invalidate(msg.reason or '自己位置の追跡を失いました')
                return
            signature = (msg.speed_mps,)
        elif kind == 'steering':
            if msg.controller != CONTROLLER or not msg.has_steering or not math.isfinite(msg.steering_rad):
                self.invalidate('実測ホイールベースによる Pure Pursuit 舵角を確認できません')
                return
            signature = (msg.steering_rad, ttl)
        else:
            if not msg.has_speeds or not 0 < msg.target_speed_mps <= self.max_speed:
                self.invalidate('目標速度が低速前進の許可範囲外です')
                return
            if (not -STANDSTILL_MPS <= msg.estimated_speed_mps <= self.max_speed
                    or not all(0 <= raw <= 2047 for raw in (msg.feedforward_raw, msg.calculated_raw))):
                self.invalidate('速度制御が報告した推定速度または CH2 指令が範囲外です')
                return
            signature = (msg.estimated_speed_mps, msg.target_speed_mps, msg.feedforward_raw, msg.calculated_raw,
                         msg.calibration_run_id, msg.proposal_sha256, ttl)

        rows = self.samples[kind]
        if rows and ns <= max(rows):
            if ns != max(rows) or rows[ns].signature != signature:
                self.invalidate('元観測が逆行、または同時刻の指令内容が変化しました')
            return  # 同じ観測からの再配信で期限を延長しない。
        if is_loc:
            self.velocity = (ns, steady, msg.speed_mps)
            if not -STANDSTILL_MPS <= msg.speed_mps <= self.max_speed:
                self.invalidate('推定速度が低速前進の許可範囲外です')
                return
        if len(rows) >= MAX_PENDING_FRAMES:
            rows.pop(min(rows))
        rows[ns] = Observation(msg, steady + ttl - (wall - ns), signature)
        return all(ns in stream for stream in self.samples.values())

    def trip(self, board, reason, wall, steady):
        if self.requested or self.rc_seen:
            self.last_stop = dict(stamp_ns=wall, reason=reason)
        if self.requested:
            measured = self.velocity
            # 元観測が通常の期限を迎えた直後も、直近の正速度に基づく一回の制動を許す。
            if (self.active and measured and measured[2] > STANDSTILL_MPS
                    and 0 <= wall - measured[0] < MAX_AGE_NS + 100_000_000
                    and 0 <= steady - measured[1] < MAX_AGE_NS + 100_000_000):
                board.stop_forward()
            else:
                board.disarm()
        self.requested = self.active = self.rc_seen = False
        self.fault = reason
        return reason + '。停止を要求しました。CH3 をプロポ側へ戻してください'

    def step(self, board, status, wall, steady):
        if not status or not status['host_selected']:
            relinquished_host = self.host_since is not None
            if self.requested:
                board.disarm()
            self.requested = self.active = False
            self.host_since = None
            if (status and status['selector'] == 0 and status['active_path'] == 1
                    and status['fault_bits'] == 0 and 1000 <= status['rx_mode_us'] <= 1300):
                self.fault = ''
                self.rc_seen = True
                return 'プロポ手動・CH3 の HOST 選択を待機中'
            if relinquished_host or not status or status['fault_bits'] not in (0, 8):
                self.rc_seen = False
            return 'Pico の操作経路を確認中'

        if self.fault:
            return self.trip(board, self.fault, wall, steady)
        if not self.rc_seen:
            return 'CH3 を一度プロポ側にしてから HOST に切り替えてください'
        if self.host_since is None:
            self.host_since = (wall, steady)
        common = set(self.samples['localization']).intersection(self.samples['steering'], self.samples['speed'])
        stamp = max(common, default=0)
        if stamp <= self.host_since[0]:
            if steady - self.host_since[1] < MAX_AGE_NS:
                return '切替後の自己位置・操舵・スロットルを待機中'
            return self.trip(board, '切替後の新しい制御指令がそろいません', wall, steady)
        loc, lateral, speed = (self.samples[kind][stamp] for kind in TOPICS)
        if not 0 <= wall - stamp < MAX_AGE_NS or min(loc.expires, lateral.expires, speed.expires) <= steady:
            return self.trip(board, '自己位置・操舵・スロットルの元観測が失効しました', wall, steady)
        if loc.msg.speed_mps != speed.msg.estimated_speed_mps:
            return self.trip(board, '自己位置と速度制御の推定速度が不一致です', wall, steady)
        try:
            steering, throttle = self.calibration.commands(lateral.msg.steering_rad, speed.msg,
                                                          max_speed_mps=self.max_speed)
            if not -1 <= steering <= 1 or not 0 <= throttle <= 1:
                raise ValueError('変換後の指令が低速前進の許可範囲外です')
        except ValueError as exc:
            return self.trip(board, str(exc), wall, steady)

        if not self.requested:
            if loc.msg.speed_mps > STANDSTILL_MPS:
                return self.trip(board, '開始時の停車を確認できません', wall, steady)
            if status['active_path'] != 0 or not board.arm():
                return self.trip(board, 'Pico の HOST 中立待機を確認できません', wall, steady)
            self.requested = True
            return 'Pico の HOST 中立出力を確認中'
        if not self.active:
            if status['active_path'] == 0:
                return 'Pico の HOST 中立出力を確認中'
            if status['active_path'] != 2 or (status['servo_us'], status['esc_us']) != (1500, 1500):
                return self.trip(board, 'Pico の初回中立出力が不正です', wall, steady)
            self.active = True
        if status['active_path'] != 2 or not board.drive(steering, throttle):
            board.disarm()
            self.requested = self.active = False
            return self.trip(board, 'CH3 の HOST 許可が解除されました', wall, steady)
        return '低速前進の自動走行中'

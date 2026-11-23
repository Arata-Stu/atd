"""保存地図上の観測と計画を照合し、Gazebo の仮想アクチュエータだけを駆動する。"""
import json
import math
from dataclasses import asdict
from pathlib import Path

from atd_control.bicycle import FULL_SPEED_MPS, STEERING_LIMIT, WHEELBASE_M, resolve_full_throttle_speed
from atd_control.simulation import FORWARD_SPAN_RAW, NEUTRAL_RAW, simulation_feedforward
from atd_control.core import LookaheadConfig, STANDSTILL_MPS, forward_control_speed, pure_pursuit
from atd_control.speed import FeedforwardTable, PIDConfig, SpeedController
from atd_planning.core import DEFAULT_HORIZON_M, LINE_KEYS, yaw_from_quaternion


# The Gazebo plant consumes the image timestamp and independently expires it after 0.30 sim s.
COMMAND_TTL_NS = 250_000_000
# Covers valid 90 Hz observations even if the other stream or /clock arrives later.
MAX_PENDING_FRAMES = 32
TUNABLE_PARAMETERS = ('line', 'max_speed_mps', *asdict(LookaheadConfig()),
                      *('speed_' + key for key in asdict(PIDConfig())))


def default_min_lookahead(line):
    # The Odaiba timing line spawn is ~0.49 m from the saved sample raceline.
    return .65 if line == 'raceline' else LookaheadConfig().min_lookahead_m


def simulation_table(path='', full_throttle_speed_mps=FULL_SPEED_MPS):
    if not path:
        return simulation_feedforward(full_throttle_speed_mps)
    table = FeedforwardTable.from_document(json.loads(Path(path).read_text(encoding='utf-8')))
    if table.direction != 1 or min(table.raw) < NEUTRAL_RAW or max(table.raw) > NEUTRAL_RAW + FORWARD_SPAN_RAW:
        raise ValueError('選んだ FF 表は Gazebo の前進 CH2 範囲 992〜1792 に対応していません')
    return table


def validate_speed_cap(max_speed, full_throttle_speed_mps, table):
    resolve_full_throttle_speed(max_speed, full_throttle_speed_mps)
    if not table.speeds[0] <= max_speed <= table.speeds[-1]:
        raise ValueError(f'速度上限 {max_speed:g} m/s が FF 表の範囲 {table.speeds[0]:g}〜'
                         f'{table.speeds[-1]:g} m/s の外です')


def timestamp(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def tracking_id(msg):
    return (msg.map_id, msg.map_sha256, msg.hdmap_sha256, msg.session_id, msg.epoch)


class GazeboController:
    """1つの計画と同一観測の自己位置から1回だけ PID を進める。全時刻は /clock。"""
    def __init__(self, route, line, max_speed, table, *, full_throttle_speed_mps=FULL_SPEED_MPS,
                 lookahead=None, pid=PIDConfig(), available_lines=None):
        validate_speed_cap(max_speed, full_throttle_speed_mps, table)
        self.route, self.line, self.max_speed = route, line, max_speed
        self.available_lines = tuple(available_lines) if available_lines is not None else (line,)
        if line not in self.available_lines or any(name not in LINE_KEYS for name in self.available_lines):
            raise ValueError('起動ラインと選択可能な保存ラインを確認してください')
        self.switch_stamp = None
        self.switch_pending = False
        self.lookahead = lookahead or LookaheadConfig(min_lookahead_m=default_min_lookahead(line))
        self.launch_speed_cap = max_speed
        self.full_throttle_speed = full_throttle_speed_mps
        self.horizon = max(DEFAULT_HORIZON_M, self.lookahead.max_lookahead_m)
        self.pid = SpeedController(table, pid)
        self.locations, self.plans = {}, {}
        self.observation = None
        self.pursuit_sample = None
        self.session = None
        self.state, self.reason = 'waiting', '保存地図での自己位置推定と走行指令を待っています'
        self.stamp = self.expires = self.clock_ns = 0

    def tuning(self, updates):
        """候補をまとめて検査し、走行中の状態は採用されるまで変更しない。"""
        if updates.keys() - set(TUNABLE_PARAMETERS):
            raise ValueError('Gazebo の走行中に変更できない設定です')
        line = updates.get('line', self.line)
        if line not in self.available_lines:
            raise ValueError(f'地図と速度 FF 表に適合するラインを選んでください（{", ".join(self.available_lines)}）')
        speed = updates.get('max_speed_mps', self.max_speed)
        validate_speed_cap(speed, self.full_throttle_speed, self.pid.table)
        if speed > self.launch_speed_cap:
            raise ValueError(f'起動時の速度上限 {self.launch_speed_cap:g} m/s を超えています')
        lookahead = LookaheadConfig(**{k: updates.get(k, v) for k, v in asdict(self.lookahead).items()})
        if lookahead.max_lookahead_m > self.horizon:
            raise ValueError(f'最大先読みは計画の前方軌道長 {self.horizon:g} m 以内です')
        pid = PIDConfig(**{k: updates.get('speed_' + k, v) for k, v in asdict(self.pid.config).items()})
        return line, speed, lookahead, pid

    def apply_tuning(self, updates, now=None):
        line, speed, lookahead, pid = self.tuning(updates)
        if (line, speed, lookahead, pid) == (self.line, self.max_speed, self.lookahead, self.pid.config):
            return False
        if line != self.line:
            self.switch_stamp = self.clock_ns if now is None else now
            self.switch_pending = True
            self.locations.clear()
            self.plans.clear()
            self.wait_for_observation(f'{line} に切替中です。新しい自己位置と計画を待っています')
        if line != self.line or speed != self.max_speed or pid != self.pid.config:
            self.pid = SpeedController(self.pid.table, pid)
        self.line, self.max_speed, self.lookahead = line, speed, lookahead
        return True

    def inhibit(self, reason):
        if self.state != 'stopped':
            self.reason = reason
            if self.stamp:
                self.state = 'stopped'
        self.observation = None
        self.pursuit_sample = None
        self.locations.clear()
        self.plans.clear()
        self.pid.reset()

    def wait_for_observation(self, reason):
        if self.state in ('driving', 'waiting_observation'):
            self.state, self.reason = 'waiting_observation', reason
            self.observation = None
            self.pursuit_sample = None
            self.pid.reset()

    def tick(self, now):
        if now < self.clock_ns:
            self.inhibit('シミュレーション時計が逆行しました')
        self.clock_ns = now
        if self.state == 'driving' and now >= self.expires:
            self.wait_for_observation('前の指令が失効しました。同じ観測時刻の新しい自己位置と計画を待っています')

    def check_sources(self, pose_publishers, plan_publishers):
        if pose_publishers == plan_publishers == 1:
            return True
        self.inhibit(f'配信元を確認できません（自己位置 {pose_publishers}、計画 {plan_publishers}、各1つ必要）')
        return False

    def observe(self, kind, msg, now):
        self.tick(now)
        if self.state == 'stopped':
            return None
        is_pose = kind == 'localization'
        identity = tracking_id(msg)
        stamp = timestamp(msg)
        if (not is_pose and self.switch_stamp is not None and msg.header.frame_id == 'map'
                and identity[:3] == self.route and msg.line in self.available_lines
                and (0 < stamp <= self.switch_stamp or self.switch_pending and msg.line != self.line)):
            # In-flight plans from the previous line can arrive until the planner receives the selection.
            return None
        if (msg.header.frame_id != 'map' or identity[:3] != self.route or not msg.session_id
                or not is_pose and msg.line != self.line):
            self.inhibit('自己位置・計画の地図、走行ラインまたは座標系が不一致です')
            return None
        if is_pose and msg.state == 'stale' and (self.session is None or self.session == identity):
            self.wait_for_observation(msg.reason or '新しい自己位置を待っています')
            self.locations.clear()
            self.plans.clear()
            return None
        if is_pose and msg.state != 'tracking' or not is_pose and not msg.valid:
            self.inhibit(msg.reason or ('自己位置を追跡できません' if is_pose else '計画が無効です'))
            return None
        if self.session is not None and self.session != identity:
            self.inhibit('自己位置の追跡セッションが変化しました')
            if self.state == 'stopped':
                return None
        self.session = identity
        if stamp <= 0 or stamp > now + 50_000_000:
            self.inhibit('元観測のシミュレーション時刻が有効期間外です')
            return None
        if self.switch_stamp is not None and stamp <= self.switch_stamp:
            return None
        if stamp <= self.stamp or now - stamp >= COMMAND_TTL_NS:
            return None
        pending = self.locations if is_pose else self.plans
        pending[stamp] = msg
        if len(pending) > MAX_PENDING_FRAMES:
            pending.pop(min(pending))
        # The /clock sample may arrive after the messages over DDS.
        return self.ready(now)

    def ready(self, now):
        self.tick(now)
        stamps = self.locations.keys() & self.plans.keys()
        stamp = max((t for t in stamps if self.stamp < t <= now), default=0)
        if not stamp or self.state == 'stopped':
            return None
        pose, plan = self.locations[stamp], self.plans[stamp]
        try:
            if not math.isfinite(plan.validity_s) or not 0 < plan.validity_s:
                raise ValueError('計画の有効期限が不正です')
            expiry = stamp + int(min(COMMAND_TTL_NS, plan.validity_s * 1e9))
            if now >= expiry:
                del self.locations[stamp], self.plans[stamp]
                return None
            if tracking_id(pose) != tracking_id(plan):
                raise ValueError('同じ追跡区間の新しい計画がありません')
            measured = pose.speed_mps
            if measured != plan.estimated_speed_mps or not math.isfinite(measured):
                raise ValueError('自己位置と計画の速度が一致しません')
            if self.state == 'waiting' and abs(measured) > STANDSTILL_MPS:
                raise ValueError('自動発進前の停止を確認できません')
            if not math.isfinite(plan.target_speed_mps) or plan.target_speed_mps <= 0:
                raise ValueError('計画の目標速度が前進範囲外です')
            target = min(self.max_speed, plan.target_speed_mps)
            q, p = pose.pose.orientation, pose.pose.position
            if not math.isfinite(plan.lateral_error_m) or not math.isfinite(p.z):
                raise ValueError('車体姿勢または横ずれが有限値ではありません')
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            steering = pure_pursuit([(point.x, point.y) for point in plan.trajectory],
                                    x=p.x, y=p.y, yaw=yaw, speed_mps=measured, wheelbase_m=WHEELBASE_M,
                                    config=self.lookahead, low_speed_drive=True)
            if not steering['valid']:
                if self.state == 'waiting':
                    raise ValueError(f'{steering["reason"]}（発進時の最小先読み {self.lookahead.min_lookahead_m:g} m、'
                                     f'走行ラインからの横ずれ {plan.lateral_error_m:.2f} m）')
                raise ValueError(steering['reason'])
            speed = self.pid.update(stamp, target, forward_control_speed(measured, True), self.session)
            if not speed['valid']:
                raise ValueError(speed['reason'])
            angle = max(-STEERING_LIMIT, min(STEERING_LIMIT, steering['steering_rad']))
            throttle = (speed['calculated_raw'] - NEUTRAL_RAW) / FORWARD_SPAN_RAW
            if not math.isfinite(angle) or not 0 <= throttle <= 1.:
                raise ValueError('模擬アクチュエータ指令が範囲外です')
        except ValueError as exc:
            self.inhibit(str(exc))
            return None
        self.stamp, self.expires = stamp, expiry
        self.switch_pending = False
        self.locations = {t: value for t, value in self.locations.items() if t > stamp}
        self.plans = {t: value for t, value in self.plans.items() if t > stamp}
        self.observation = dict(image_stamp_ns=stamp, x_m=p.x, y_m=p.y, yaw_rad=yaw,
                                estimated_speed_mps=measured, profile_target_mps=plan.target_speed_mps,
                                target_speed_mps=target, lateral_error_m=plan.lateral_error_m,
                                feedforward_ch2_raw=speed['feedforward_raw'], calculated_ch2_raw=speed['calculated_raw'])
        self.pursuit_sample = plan, pose.pose, steering
        self.state, self.reason = 'driving', ''
        return (stamp, angle, throttle)

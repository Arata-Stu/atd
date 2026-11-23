"""Notebook 用の前進 2D 閉ループ比較。ここでの寸法・応答・FF 表はすべて模擬値。"""
from functools import partial
import heapq
import math

from atd_planning.core import Planner, ReferenceLine

from .bicycle import (ACCEL_MPS2, DECEL_MPS2, FULL_SPEED_MPS, SPEED_TAU_S, STEERING_LIMIT,
                      STEERING_TAU_S, WHEELBASE_M, Bicycle as _Vehicle)
from .core import LookaheadConfig, pure_pursuit
from .speed import FeedforwardTable, SpeedController


# default, minimum, maximum
_PARAMETERS = {
    'fixed_lookahead_m': (.65, .15, 2.),
    'min_lookahead_m': (.35, .15, 2.),
    'max_lookahead_m': (1.5, .15, 2.),
    'lookahead_time_s': (.4, 0., 2.),
    'steering_delay_ms': (0., 0., 1000.),
    'throttle_delay_ms': (0., 0., 1000.),
    'steering_tau_s': (STEERING_TAU_S, .02, 2.),
    'speed_tau_s': (SPEED_TAU_S, .02, 2.),
}
_DT_NS = 20_000_000
_LAPS = 2
_TIME_LIMIT_S = 90

# Simulated CH2 raw scale and equilibrium speed at full forward throttle.
NEUTRAL_RAW = 992
FORWARD_SPAN_RAW = 800


def simulation_feedforward(full_throttle_speed_mps=FULL_SPEED_MPS):
    """模擬車両の定常速度とスロットルの比例関係から、前進用 FF 表を作る。"""
    if not math.isfinite(full_throttle_speed_mps) or full_throttle_speed_mps <= 0:
        raise ValueError('模擬車両の全開時速度は正の有限値 [m/s] を指定してください')
    low_speed = min(.2, full_throttle_speed_mps / 4)
    low_raw = NEUTRAL_RAW + FORWARD_SPAN_RAW * (low_speed / full_throttle_speed_mps)
    return FeedforwardTable('simulation', '', (low_speed, full_throttle_speed_mps / 2, full_throttle_speed_mps),
                            (low_raw, NEUTRAL_RAW + FORWARD_SPAN_RAW / 2, NEUTRAL_RAW + FORWARD_SPAN_RAW), 1)


def validate_settings(overrides):
    if not isinstance(overrides, dict):
        raise ValueError('シミュレーション設定にはオブジェクトを指定してください')
    values = {name: spec[0] for name, spec in _PARAMETERS.items()}
    for name, value in overrides.items():
        if name not in _PARAMETERS:
            raise ValueError(f'不明なシミュレーション設定です: {name}')
        _, low, high = _PARAMETERS[name]
        if type(value) not in (int, float) or not low <= value <= high:
            raise ValueError(f'{name} は {low:g}〜{high:g} の有限の数値を指定してください')
        values[name] = float(value)
    if values['min_lookahead_m'] > values['max_lookahead_m']:
        raise ValueError('最小先読み距離は最大先読み距離以下にしてください')
    return values


def _sample_course():
    """半径の異なる 2 円の共通外接線と円弧をつなぎ、直線の中央から開始する。"""
    wide_radius, tight_radius, separation = 2.15, .85, 6.8
    tangent_angle = math.acos((wide_radius - tight_radius) / separation)
    spacing = .10
    points, speeds = [], []

    def circle(cx, radius, angle):
        return cx + radius * math.cos(angle), radius * math.sin(angle)

    def straight(start, end):
        count = math.ceil(math.dist(start, end) / spacing)
        for i in range(count):
            fraction = i / count
            points.append(tuple(a + (b - a) * fraction for a, b in zip(start, end)))
            speeds.append(1.95)

    def bend(cx, radius, start, end):
        count = math.ceil((end - start) * radius / spacing)
        for i in range(count):
            points.append(circle(cx, radius, start + (end - start) * i / count))
            speeds.append(math.sqrt(1.15 * radius))

    wide_bottom = circle(0., wide_radius, -tangent_angle)
    tight_bottom = circle(separation, tight_radius, -tangent_angle)
    start = tuple((a + b) / 2 for a, b in zip(wide_bottom, tight_bottom))
    straight(start, tight_bottom)
    bend(separation, tight_radius, -tangent_angle, tangent_angle)
    straight(circle(separation, tight_radius, tangent_angle), circle(0., wide_radius, tangent_angle))
    bend(0., wide_radius, tangent_angle, 2 * math.pi - tangent_angle)
    straight(wide_bottom, start)

    # Cyclic v²(s) envelopes for accelerating out of and slowing before the bends.
    count = len(points)
    lengths = [math.dist(p, points[(i + 1) % count]) for i, p in enumerate(points)]
    for k in range(2 * count):
        i, j = k % count, (k + 1) % count
        speeds[j] = min(speeds[j], math.sqrt(speeds[i] ** 2 + 2 * .9 * lengths[i]))
    for k in reversed(range(2 * count)):
        i, j = k % count, (k + 1) % count
        speeds[i] = min(speeds[i], math.sqrt(speeds[j] ** 2 + 2 * .95 * lengths[i]))
    return points, ReferenceLine(points, speeds, closed=True)


def _drive_interval(vehicle, pending, now, steering, throttle, settings, contact=None):
    # Split a physics interval at actual command arrival times, including delays shorter than dt.
    heapq.heappush(pending, (now + round(settings['steering_delay_ms'] * 1e6), 'steering', steering))
    heapq.heappush(pending, (now + round(settings['throttle_delay_ms'] * 1e6), 'throttle', throttle))
    cursor, end = now, now + _DT_NS
    def move_until(arrival):
        earlier = (vehicle.x, vehicle.y, vehicle.yaw)
        vehicle.advance((arrival - cursor) * 1e-9, settings)
        return contact is not None and contact(earlier, (vehicle.x, vehicle.y, vehicle.yaw))

    while pending and pending[0][0] <= end:
        arrival, axis, value = heapq.heappop(pending)
        if arrival > cursor and move_until(arrival):
            return arrival
        if axis == 'steering':
            vehicle.steering_setpoint = value
        else:
            vehicle.throttle_setpoint = value
        cursor = arrival
    if cursor < end and move_until(end):
        return end
    return None


def _run(reference, steer, mode, settings, *, laps=_LAPS, time_limit_s=_TIME_LIMIT_S,
         start_offset_m=.06, start_yaw_rad=0., contact=None, sample_stride=5, check=lambda: None):
    planner = Planner(reference, horizon_m=2.5)
    # This synthetic feedforward table lives in memory and is never read from or written to calibration.
    speed_controller = SpeedController(simulation_feedforward())
    edge = reference.segments[0]
    vehicle = _Vehicle(edge.x - start_offset_m * edge.uy, edge.y + start_offset_m * edge.ux,
                       math.atan2(edge.uy, edge.ux) + start_yaw_rad)
    pending, samples = [], []
    progress, previous_station = 0., None
    peak_error, squared_error_ns, saturated_ns = 0., 0., 0
    lap_times, last_crossing = [], 0.
    outcome, reason, collision = 'timeout', '', None
    previous_sample = None
    max_steps = math.ceil(time_limit_s * 1_000_000_000 / _DT_NS)
    for step in range(max_steps + 1):
        now = step * _DT_NS
        if step % 50 == 0:
            check()
        if step == 0 and contact is not None and contact((vehicle.x, vehicle.y, vehicle.yaw),
                                                        (vehicle.x, vehicle.y, vehicle.yaw)):
            outcome, reason = 'collision', '車体外形がコース境界に接触しました'
            collision = dict(t_s=now * 1e-9, x_m=vehicle.x, y_m=vehicle.y, yaw_rad=vehicle.yaw)
            break
        stamp = 1_000_000_000 + now  # ROS-free controllers require a positive monotonic observation stamp.
        plan = planner.update(stamp_ns=stamp, x=vehicle.x, y=vehicle.y, yaw=vehicle.yaw,
                              speed_mps=vehicle.speed, session_id='simulation', epoch=0)
        if not plan['valid']:
            outcome, reason = 'tracking_lost', plan['reason']
            break
        station = plan['progress_m']
        if previous_station is not None:
            delta = reference.delta(station, previous_station)
            next_finish = reference.length * (len(lap_times) + 1)
            if progress < next_finish <= progress + delta:
                fraction = (next_finish - progress) / delta
                crossing = (step - 1 + fraction) * _DT_NS * 1e-9
                lap_times.append(crossing - last_crossing)
                last_crossing = crossing
            progress += delta
        previous_station = station
        steering = steer(plan['trajectory'], x=vehicle.x, y=vehicle.y, yaw=vehicle.yaw,
                         speed_mps=vehicle.speed, wheelbase_m=WHEELBASE_M)
        speed = speed_controller.update(stamp, plan['target_speed_mps'], vehicle.speed)
        if steering['valid'] is not True or not speed['valid']:
            outcome = 'controller_error'
            reason = steering.get('reason') or speed['reason'] or '操舵アルゴリズムが無効な指令を返しました'
            break
        command = steering['steering_rad']
        if isinstance(command, bool) or not math.isfinite(command):
            outcome, reason = 'controller_error', '操舵アルゴリズムの指令角が不正です'
            break
        throttle = (speed['calculated_raw'] - NEUTRAL_RAW) / FORWARD_SPAN_RAW
        limited = max(-STEERING_LIMIT, min(STEERING_LIMIT, command))
        error = plan['lateral_error_m']
        peak_error = max(peak_error, abs(error))
        if sample_stride:
            previous_sample = dict(t_s=now * 1e-9, x_m=vehicle.x, y_m=vehicle.y, yaw_rad=vehicle.yaw,
                                   lateral_error_m=error, target_speed_mps=plan['target_speed_mps'],
                                   speed_mps=vehicle.speed, steering_command_deg=math.degrees(command),
                                   steering_deg=math.degrees(vehicle.steering), throttle=throttle,
                                   lookahead_m=steering['requested_lookahead_m'], progress_m=progress,
                                   target_x_m=steering['target'][0], target_y_m=steering['target'][1])
            if step % sample_stride == 0:
                samples.append(previous_sample)
        if len(lap_times) >= laps:
            outcome = 'finished'
            break
        if step >= max_steps:
            reason = '規定時間内に周回を完了しませんでした'
            break
        hit = _drive_interval(vehicle, pending, now, limited, throttle, settings, contact)
        duration_ns = _DT_NS if hit is None else hit - now
        squared_error_ns += error ** 2 * duration_ns
        if command != limited:
            saturated_ns += duration_ns
        if hit is not None:
            now = hit
            outcome, reason = 'collision', '車体外形がコース境界に接触しました'
            collision = dict(t_s=now * 1e-9, x_m=vehicle.x, y_m=vehicle.y, yaw_rad=vehicle.yaw)
            break

    if previous_sample is not None and (not samples or samples[-1] is not previous_sample):
        samples.append(previous_sample)
    metrics = dict(max_abs_lateral_error_m=peak_error,
                   rms_lateral_error_m=math.sqrt(squared_error_ns / now) if now else peak_error,
                   steering_saturation_s=saturated_ns * 1e-9, elapsed_s=now * 1e-9,
                   laps_completed=len(lap_times))
    return dict(mode=mode, completed=outcome == 'finished', outcome=outcome, reason=reason, metrics=metrics,
                samples=samples, collision=collision, lap_times_s=lap_times, progress_m=progress)


def default_controllers(settings=None):
    """固定・速度連動の先読み設定を持ち、試行ごとに新しい操舵関数を作るファクトリ。"""
    values = validate_settings({} if settings is None else settings)
    fixed = LookaheadConfig(values['fixed_lookahead_m'], 0., values['fixed_lookahead_m'])
    adaptive = LookaheadConfig(values['min_lookahead_m'], values['lookahead_time_s'], values['max_lookahead_m'])
    return {'fixed': lambda: partial(pure_pursuit, config=fixed),
            'adaptive': lambda: partial(pure_pursuit, config=adaptive)}


def _model(laps):
    return dict(wheelbase_m=WHEELBASE_M, steering_limit_deg=math.degrees(STEERING_LIMIT),
                dt_s=_DT_NS * 1e-9, laps=laps, accel_limit_mps2=ACCEL_MPS2,
                decel_limit_mps2=DECEL_MPS2, full_throttle_speed_mps=FULL_SPEED_MPS)


def compare(config):
    """一つの模擬ライン、初期姿勢、車体設定で固定・速度連動先読みの 2 条件を計算する。"""
    settings = validate_settings(config)
    points, reference = _sample_course()
    return dict(settings=settings,
                course=dict(name='ヘアピンと高速コーナー', points=[dict(x_m=x, y_m=y) for x, y in points],
                            length_m=reference.length),
                model=_model(_LAPS),
                runs=[_run(reference, build(), mode, settings) for mode, build in default_controllers(settings).items()])

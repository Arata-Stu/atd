"""RC simulations share this forward-only rear-axle model; parameters are simulated."""

from dataclasses import dataclass
import math


WHEELBASE_M = .257
STEERING_LIMIT = math.radians(28.)
ACCEL_MPS2 = 2.4
DECEL_MPS2 = 1.8
FULL_SPEED_MPS = 3.2
STEERING_TAU_S = .12
SPEED_TAU_S = .5


def resolve_full_throttle_speed(max_speed_mps, full_throttle_speed_mps=None):
    """Keep the original simulated motor for low-speed runs unless explicitly overridden."""
    if not math.isfinite(max_speed_mps) or max_speed_mps < .2:
        raise ValueError('Gazebo の走行速度上限には 0.2 m/s 以上の有限値を指定してください')
    if full_throttle_speed_mps is None:
        return max(FULL_SPEED_MPS, max_speed_mps)
    if not math.isfinite(full_throttle_speed_mps) or full_throttle_speed_mps < max_speed_mps:
        raise ValueError(f'模擬車両の全開時速度には走行速度上限 {max_speed_mps:g} m/s 以上の有限値を指定してください')
    return full_throttle_speed_mps


@dataclass
class Bicycle:
    x: float
    y: float
    yaw: float
    speed: float = 0.
    steering: float = 0.
    steering_setpoint: float = 0.
    throttle_setpoint: float = 0.
    full_throttle_speed_mps: float = FULL_SPEED_MPS

    def __post_init__(self):
        if not math.isfinite(self.full_throttle_speed_mps) or self.full_throttle_speed_mps <= 0:
            raise ValueError('模擬車両の全開時速度は正の有限値 [m/s] を指定してください')

    def advance(self, dt, settings):
        # Rear-axle kinematic bicycle, first-order actuators and bounded acceleration.
        next_steering = self.steering + (self.steering_setpoint - self.steering) * (
            -math.expm1(-dt / settings['steering_tau_s']))
        equilibrium = self.throttle_setpoint * self.full_throttle_speed_mps
        dv = (equilibrium - self.speed) * -math.expm1(-dt / settings['speed_tau_s'])
        next_speed = self.speed + max(-DECEL_MPS2 * dt, min(ACCEL_MPS2 * dt, dv))
        distance = (self.speed + next_speed) * .5 * dt
        dyaw = distance * math.tan((self.steering + next_steering) * .5) / WHEELBASE_M
        self.x += distance * math.cos(self.yaw + .5 * dyaw)
        self.y += distance * math.sin(self.yaw + .5 * dyaw)
        self.yaw = math.remainder(self.yaw + dyaw, 2 * math.pi)
        self.speed, self.steering = next_speed, next_steering

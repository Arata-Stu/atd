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


@dataclass
class Bicycle:
    x: float
    y: float
    yaw: float
    speed: float = 0.
    steering: float = 0.
    steering_setpoint: float = 0.
    throttle_setpoint: float = 0.

    def advance(self, dt, settings):
        # Rear-axle kinematic bicycle, first-order actuators and bounded acceleration.
        next_steering = self.steering + (self.steering_setpoint - self.steering) * (
            -math.expm1(-dt / settings['steering_tau_s']))
        equilibrium = self.throttle_setpoint * FULL_SPEED_MPS
        dv = (equilibrium - self.speed) * -math.expm1(-dt / settings['speed_tau_s'])
        next_speed = self.speed + max(-DECEL_MPS2 * dt, min(ACCEL_MPS2 * dt, dv))
        distance = (self.speed + next_speed) * .5 * dt
        dyaw = distance * math.tan((self.steering + next_steering) * .5) / WHEELBASE_M
        self.x += distance * math.cos(self.yaw + .5 * dyaw)
        self.y += distance * math.sin(self.yaw + .5 * dyaw)
        self.yaw = math.remainder(self.yaw + dyaw, 2 * math.pi)
        self.speed, self.steering = next_speed, next_steering

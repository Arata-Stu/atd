"""Analytic rig motion in simulation time, independently of rendering speed."""
from math import cos, pi, sin
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rig_imu import GRAVITY_M_S2, NANO, RigImu


class RigImuTest(unittest.TestCase):
    def test_rates_gravity_and_optical_pitch(self):
        for rate in (200, 400):
            with self.subTest(rate=rate):
                imu = RigImu(rate, .18, 30)
                rows = {"imu": [], "gyro": [], "accel": []}
                # A 900 Hz physics clock with nonaligned sample times.
                for tick in range(920):
                    for kind, t, gyro, accel in imu.observe(round(NANO*tick/900), 12, -8, .7):
                        rows[kind].append(t)
                        if kind != "accel":
                            self.assertEqual(gyro, (0., 0., 0.))
                        else:
                            self.assertIsNone(gyro)
                        if kind != "gyro":
                            self.assertAlmostEqual(accel[0], 0)
                            self.assertAlmostEqual(accel[1], -cos(pi/6)*GRAVITY_M_S2)
                            self.assertAlmostEqual(accel[2], -sin(pi/6)*GRAVITY_M_S2)
                        else:
                            self.assertIsNone(accel)
                for kind, stamps in rows.items():
                    hz = 200 if kind == "accel" else rate
                    self.assertEqual(len([t for t in stamps if 10_000_000 < t <= NANO + 10_000_000]), hz)
                    self.assertEqual({b-a for a, b in zip(stamps, stamps[1:])}, {NANO // hz})

    def test_lever_arm_and_wrap_in_constant_yaw_rotation(self):
        imu = RigImu(400, .18, 0)
        for tick in range(301):
            stamp = round(NANO*tick/900)
            # Turn left on a unit circle at 2 rad/s through the ±pi angle wrap.
            a = 3.1 + 2 * stamp / NANO
            yaw = (a + pi) % (2*pi) - pi
            for kind, t, gyro, accel in imu.observe(stamp, sin(a), -cos(a), yaw):
                if kind == "gyro":
                    self.assertAlmostEqual(gyro[1], -2, places=7)
                elif kind == "accel":
                    self.assertAlmostEqual(accel[0], -4, delta=.2)
                    self.assertAlmostEqual(accel[1], -GRAVITY_M_S2, places=8)
                    self.assertAlmostEqual(accel[2], -.18 * 4, delta=.015)

    def test_combined_accel_interpolates_raw_readings_at_gyro_times(self):
        imu = RigImu(400, 0, 0)
        accel, combined, gyro = {}, {}, {}
        for tick in range(51):
            stamp = tick * 1_000_000
            seconds = stamp / NANO
            for kind, t, w, a in imu.observe(stamp, seconds**3, 0, 0):
                if kind == "imu":
                    combined[t] = a
                    self.assertEqual(gyro[t], w)
                elif kind == "accel":
                    accel[t] = a
                else:
                    gyro[t] = w
        self.assertGreater(len(combined), 10)
        for t, a in combined.items():
            if t in accel:
                self.assertEqual(a, accel[t])
            else:
                self.assertEqual(a, tuple((b+c)/2 for b, c in zip(accel[t-2_500_000], accel[t+2_500_000])))
            self.assertAlmostEqual(a[2], 6 * t / NANO, delta=.015)


if __name__ == "__main__":
    unittest.main()

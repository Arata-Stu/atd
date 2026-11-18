import math
from pathlib import Path
import tempfile
import unittest

from atd_vehicle_calibration.inputs import multiply, rotate
from atd_vehicle_calibration.motion import estimate_motion, load_tum


IDENTITY = [0., 0., 0., 1.]
MOUNT = [dict(parent='base_link', child='camera_link', translation=[.3, .08, .1], rotation=IDENTITY)]


def yaw(value):
    return [0., 0., math.sin(value/2), math.cos(value/2)]


def trajectory(position=lambda t: [0., 0., 0.], orientation=lambda t: IDENTITY, *, mount=None, count=101, step=.02):
    result = []
    for i in range(count):
        t = i*step
        q = orientation(t)
        p = position(t)
        if mount:
            p = [a+b for a, b in zip(p, rotate(mount[0]['translation'], q))]
            q = multiply(q, mount[0]['rotation'])
        result.append(dict(stamp=1000.+t, position=p, orientation=q))
    return result


class MotionTest(unittest.TestCase):
    def estimate(self, poses, **kwargs):
        return estimate_motion(poses, pose_frame='base_link', static_transforms=[], **kwargs)

    def test_forward_reverse_and_acceleration(self):
        for sign in (-1, 1):
            result = self.estimate(trajectory(lambda t: [sign*(2*t+.5*t*t), 0, 0]))
            self.assertGreater(len(result['samples']), 80)
            for row in result['samples']:
                t = row['stamp']-1000
                self.assertAlmostEqual(row['speed_mps'], sign*(2+t), places=8)
                self.assertAlmostEqual(row['acceleration_body'][0], sign, places=7)
                self.assertAlmostEqual(row['yaw_rate_rps'], 0, places=8)
                self.assertLess(row['window_start'], row['stamp'])
                self.assertGreater(row['window_end'], row['stamp'])

    def test_rotating_offset_camera_does_not_create_rear_axle_speed(self):
        poses = trajectory(orientation=lambda t: yaw(t), mount=MOUNT)
        result = estimate_motion(poses, pose_frame='camera_link', static_transforms=MOUNT)
        for row in result['samples']:
            self.assertAlmostEqual(row['speed_mps'], 0, places=8)
            self.assertLess(max(abs(x) for x in row['acceleration_body']), 1e-8)
            self.assertAlmostEqual(row['yaw_rate_rps'], 1, places=4)
        # The uncorrected camera trajectory actually moves; this exercises the lever arm.
        wrong = self.estimate(poses)
        self.assertGreater(max(abs(r['velocity_body'][1]) for r in wrong['samples']), .25)

    def test_full_mount_rotation_and_translating_turn(self):
        mount = [dict(MOUNT[0], rotation=yaw(.4))]
        poses = trajectory(position=lambda t: [2*math.sin(t), 2*(1-math.cos(t)), 0],
                           orientation=lambda t: yaw(t), mount=mount)
        result = estimate_motion(poses, pose_frame='camera_link', static_transforms=mount)
        for row in result['samples']:
            self.assertAlmostEqual(row['velocity_body'][0], 2, places=4)
            self.assertAlmostEqual(row['velocity_body'][1], 0, places=3)
            self.assertAlmostEqual(row['acceleration_body'][1], 2, places=2)

    def test_stationary_and_quaternion_sign_flips(self):
        poses = trajectory(lambda t: [1, 2, 3])
        for index in range(1, len(poses), 2):
            poses[index]['orientation'] = [0., 0., 0., -1.]
        result = self.estimate(poses)
        self.assertTrue(result['samples'])
        self.assertLess(max(abs(v) for row in result['samples'] for v in row['velocity_body']), 1e-10)
        self.assertLess(max(abs(v) for row in result['samples'] for v in row['angular_velocity_body']), 1e-10)

    def test_body_omega_z_is_not_euler_yaw_derivative(self):
        # Fixed world roll applied before a changing body rotation: q=roll*yaw.
        roll = [math.sin(.3), 0, 0, math.cos(.3)]
        poses = trajectory(orientation=lambda t: multiply(roll, yaw(t)))
        result = self.estimate(poses)
        for row in result['samples']:
            self.assertAlmostEqual(row['angular_velocity_body'][0], 0, places=5)
            self.assertAlmostEqual(row['angular_velocity_body'][1], 0, places=5)
            self.assertAlmostEqual(row['yaw_rate_rps'], 1, places=4)

    def test_gap_windows_never_bridge_missing_data(self):
        poses = trajectory(lambda t: [t, 0, 0])
        poses = poses[:35]+poses[65:]
        result = self.estimate(poses)
        self.assertEqual(len({r['segment'] for r in result['samples']}), 2)
        for row in result['samples']:
            self.assertFalse(row['window_start'] < 1000.7 < row['window_end'])
            self.assertAlmostEqual(row['speed_mps'], 1, places=8)

    def test_duplicate_invalid_and_jump_do_not_leak_into_neighbors(self):
        poses = trajectory(lambda t: [t, 0, 0])
        poses.insert(25, dict(poses[24]))
        poses[50]['position'] = [float('nan'), 0, 0]
        poses[75]['position'] = [100, 0, 0]
        result = self.estimate(poses)
        reasons = [r['reason'] for r in result['rejected']]
        self.assertIn('duplicate pose stamp', reasons)
        self.assertIn('non-finite or malformed vector', reasons)
        self.assertIn('pose translation/rotation jump', reasons)
        for row in result['samples']:
            self.assertAlmostEqual(row['speed_mps'], 1, places=7)

    def test_clock_rollback_never_produces_overlapping_output(self):
        poses = trajectory(lambda t: [t, 0, 0], count=40)
        poses += trajectory(lambda t: [t, 0, 0], count=60)
        result = self.estimate(poses)
        stamps = [r['stamp'] for r in result['samples']]
        self.assertEqual(len(stamps), len(set(stamps)))
        self.assertEqual(stamps, sorted(stamps))

    def test_unmeasured_mount_bad_frame_and_unsupported_map(self):
        with self.assertRaisesRegex(ValueError, 'missing directed'):
            estimate_motion(trajectory(), pose_frame='camera_link', static_transforms=[])
        with self.assertRaisesRegex(ValueError, 'odom'):
            self.estimate(trajectory(), world_frame='map')
        poses = trajectory()
        poses[40]['pose_frame'] = 'different_camera'
        result = self.estimate(poses)
        self.assertTrue(any('frame disagrees' in r['reason'] for r in result['rejected']))
        poses = trajectory()
        poses[40]['orientation'] = [0, 0, 0, 0]
        result = self.estimate(poses)
        self.assertTrue(any('unit norm' in r['reason'] for r in result['rejected']))

    def test_nonuniform_local_times_and_short_segments(self):
        poses = []
        for i in range(100):
            t = i*.02 + (.002 if i % 2 else 0)
            poses.append(dict(stamp=1700000000.+t, position=[t*t, 0, 0], orientation=IDENTITY))
        result = self.estimate(poses)
        self.assertGreater(len(result['samples']), 80)
        for row in result['samples']:
            self.assertAlmostEqual(row['acceleration_body'][0], 2, delta=.001)
        short = self.estimate(trajectory(count=4))
        self.assertEqual(short['samples'], [])
        self.assertEqual(len(short['rejected']), 4)

    def test_tum_uses_explicit_odom_pose_convention(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'odom_poses.tum'
            path.write_text('# source ROS seconds\n1000.1 1 2 3 0 0 0 1\n1000.2 2 2 3 0 0 0 1 # ok\n')
            poses = load_tum(path, pose_frame='camera_link')
            self.assertEqual(poses[0]['position'], [1., 2., 3.])
            self.assertEqual(poses[0]['pose_frame'], 'camera_link')
            self.assertEqual(poses[0]['world_frame'], 'odom')
            with self.assertRaises(ValueError):
                load_tum(path, pose_frame='camera_link', world_frame='map')
            path.write_text('1000.1 1 2 3 0 0 1\n')
            with self.assertRaisesRegex(ValueError, 'line 1'):
                load_tum(path, pose_frame='camera_link')


if __name__ == '__main__':
    unittest.main()

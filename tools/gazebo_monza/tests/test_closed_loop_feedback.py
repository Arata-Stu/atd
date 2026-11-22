"""Drive the saved Odaiba lines with camera-paced ideal VO and map corrections at 1 Hz."""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[3]
for package in ('tools/gazebo_monza', 'ros2_ws/src/atd_bringup', 'ros2_ws/src/atd_control',
                'ros2_ws/src/atd_localization', 'ros2_ws/src/atd_mapping', 'ros2_ws/src/atd_planning'):
    sys.path.insert(0, str(ROOT / package))

from atd_bringup.gazebo_control import GazeboController, simulation_table
from atd_localization.state import RigidPose, TrackingState
from atd_planning.core import Planner, ReferenceLine
from circuits import load_lap
from closed_loop_eval import CourseEvaluation
from closed_loop_motion import CommandPlant, JOINTS


class OdaibaFeedbackTest(unittest.TestCase):
    def test_drives_two_laps_from_timing_line_with_default_controllers(self):
        document = json.loads((ROOT / 'samples/odaiba/hdmap/map.json').read_text())
        boundaries, course, _ = load_lap('odaiba')
        identity = ('odaiba-test', '1' * 64, '2' * 64)
        for line, camera_hz in (('centerline', 10), ('raceline', 10), ('raceline', 30), ('centerline', 90)):
            with self.subTest(line=line, camera_hz=camera_hz):
                planner = Planner(ReferenceLine.from_document(document, line))
                control = GazeboController(identity, line, 1.5, simulation_table())
                vehicle = CommandPlant(course[0][1:], .30)
                score = CourseEvaluation(boundaries, course, 2)
                tracking = TrackingState(allow_sparse_tf=True)
                correction = RigidPose((2., -1., 0.), (0., 0., 0., 1.))
                for frame in range(180 * camera_hz):
                    now = 1_000_000_000 + round(frame * 1e9 / camera_hz)
                    pose = vehicle.vehicle
                    time_s = now * 1e-9
                    tracking.clock(now)
                    if frame == 0:
                        tracking.hint(now)
                    for side in ('left', 'right'):
                        tracking.camera(side, now, time_s, matches=True)
                    if frame % camera_hz == 0:
                        tracking.transform(now, time_s, correction)
                    tracking.diagnostic(now, time_s, {'localized_in_exist_map': 'Yes', 'vo_status': 'OK'})
                    q = (0., 0., math.sin(pose.yaw / 2), math.cos(pose.yaw / 2))
                    vo = RigidPose((pose.x - 2., pose.y + 1., 0.), q)
                    tracking.odometry(now, time_s, vo, pose.speed, 0.)
                    state, reason, observation = tracking.status(time_s)
                    self.assertEqual(state, 'tracking', reason)
                    self.assertEqual(observation.stamp, now)
                    x, y, z = observation.pose.xyz
                    qx, qy, qz, qw = observation.pose.xyzw
                    plan = planner.update(stamp_ns=observation.stamp, x=x, y=y, yaw=2 * math.atan2(qz, qw),
                                          speed_mps=observation.speed, session_id='qa', epoch=tracking.epoch)
                    plan['trajectory'] = [SimpleNamespace(x=x, y=y) for x, y in plan['trajectory']]
                    seconds, nanos = divmod(now, 1_000_000_000)
                    header = SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=nanos), frame_id='map')
                    common = dict(header=header, map_id=identity[0], map_sha256=identity[1],
                                  hdmap_sha256=identity[2], session_id='qa', epoch=tracking.epoch)
                    orientation = SimpleNamespace(x=qx, y=qy, z=qz, w=qw)
                    position = SimpleNamespace(x=x, y=y, z=z)
                    loc = SimpleNamespace(**common, state=state, reason=reason, speed_mps=observation.speed,
                                          pose=SimpleNamespace(position=position, orientation=orientation))
                    control.observe('localization', loc, now)
                    command = control.observe('planning', SimpleNamespace(**common, line=line,
                                              validity_s=.5, **plan), now)
                    self.assertIsNotNone(command, control.reason)
                    stamp, steer, throttle = command
                    self.assertEqual(stamp, now)
                    vehicle.receive(now, stamp, JOINTS, [steer, throttle, 1.])
                    self.assertEqual(vehicle.command_stamp_ns, stamp)
                    outcome = score.observe(now * 1e-9, pose.x, pose.y, pose.yaw)
                    if outcome:
                        break
                    for step in range(1, 90 // camera_hz + 1):
                        vehicle.advance(now + round(step * 1e9 / 90), 1. / 90)
                else:
                    self.fail('2周完走前に180シミュレーション秒が経過しました')
                self.assertEqual(outcome['status'], 'completed', outcome)
                self.assertEqual(outcome['laps_completed'], 2)

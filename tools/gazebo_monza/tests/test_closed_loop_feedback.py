"""Drive the real Odaiba lines using ideal localization as a camera-stack test double."""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[3]
for package in ('tools/gazebo_monza', 'ros2_ws/src/atd_bringup', 'ros2_ws/src/atd_control',
                'ros2_ws/src/atd_planning'):
    sys.path.insert(0, str(ROOT / package))

from atd_bringup.gazebo_control import GazeboController, simulation_table
from atd_planning.core import Planner, ReferenceLine
from circuits import load_lap
from closed_loop_eval import CourseEvaluation
from closed_loop_motion import CommandPlant, JOINTS


class OdaibaFeedbackTest(unittest.TestCase):
    def test_drives_two_laps_from_timing_line_with_default_controllers(self):
        document = json.loads((ROOT / 'samples/odaiba/hdmap/map.json').read_text())
        boundaries, course, _ = load_lap('odaiba')
        identity = ('odaiba-test', '1' * 64, '2' * 64)
        for line in ('centerline', 'raceline'):
            with self.subTest(line=line):
                planner = Planner(ReferenceLine.from_document(document, line))
                control = GazeboController(identity, line, 1.5, simulation_table())
                vehicle = CommandPlant(course[0][1:], .30)
                score = CourseEvaluation(boundaries, course, 2)
                for frame in range(1800):
                    now = 1_000_000_000 + frame * 100_000_000
                    pose = vehicle.vehicle
                    plan = planner.update(stamp_ns=now, x=pose.x, y=pose.y, yaw=pose.yaw,
                                          speed_mps=pose.speed, session_id='qa', epoch=0)
                    plan['trajectory'] = [SimpleNamespace(x=x, y=y) for x, y in plan['trajectory']]
                    seconds, nanos = divmod(now, 1_000_000_000)
                    header = SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=nanos), frame_id='map')
                    common = dict(header=header, map_id=identity[0], map_sha256=identity[1],
                                  hdmap_sha256=identity[2], session_id='qa', epoch=0)
                    orientation = SimpleNamespace(x=0., y=0., z=math.sin(pose.yaw / 2), w=math.cos(pose.yaw / 2))
                    position = SimpleNamespace(x=pose.x, y=pose.y, z=0.)
                    loc = SimpleNamespace(**common, state='tracking', reason='', speed_mps=pose.speed,
                                          pose=SimpleNamespace(position=position, orientation=orientation))
                    control.observe('localization', loc, now)
                    command = control.observe('planning', SimpleNamespace(**common, line=line,
                                              validity_s=.5, **plan), now)
                    self.assertIsNotNone(command, control.reason)
                    stamp, steer, throttle = command
                    vehicle.receive(now, stamp, JOINTS, [steer, throttle, 1.])
                    outcome = score.observe(now * 1e-9, pose.x, pose.y, pose.yaw)
                    if outcome:
                        break
                    for step in range(1, 10):
                        vehicle.advance(now + round(step * 1e9 / 90), 1. / 90)
                else:
                    self.fail('2周完走前に180シミュレーション秒が経過しました')
                self.assertEqual(outcome['status'], 'completed', outcome)
                self.assertEqual(outcome['laps_completed'], 2)

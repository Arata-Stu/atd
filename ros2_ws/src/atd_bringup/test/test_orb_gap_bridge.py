"""画像判定と IMU の配送順、再捕捉補正、カメラ配信停止中の出力を検証する。"""
import csv
import json
import math
import sys
from types import SimpleNamespace as NS

import pytest

from atd_mapping.frames import stamp_ns
from atd_bringup.orb_gap_bridge import CONTINUOUS_ODOMETRY, CONTINUOUS_STATUS, OrbGapBridge


def header(t, frame='map'):
    sec, ns = divmod(t, 10**9)
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=ns))


def pose(x, y, yaw):
    return NS(position=NS(x=x, y=y, z=0.45),
              orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))


def frame(ms, state='tracking', *, epoch=1, x=0., y=0., yaw=0., session='one', map_id='map1'):
    return NS(header=header(ms*10**6), pose=pose(x, y, yaw), epoch=epoch, state=state,
              session_id=session, map_id=map_id, map_sha256='hash', speed_mps=1., yaw_rate_rps=0.)


def imu(ms, rate=0., reference='base_link'):
    return NS(header=header(ms*10**6, reference), angular_velocity=NS(x=0., y=0., z=rate),
              angular_velocity_covariance=[0.]*9)


class FakeNode:
    def __init__(self):
        self.outputs = {}
        self.subs = {}
        self.context = NS(ok=lambda: True)

    def create_publisher(self, _, topic, qos):
        self.outputs[topic] = []
        return NS(publish=self.outputs[topic].append)

    def create_subscription(self, _, topic, cb, qos):
        self.subs[topic] = cb
        return topic

    def destroy_subscription(self, topic):
        del self.subs[topic]

    def destroy_publisher(self, pub):
        pass

    def get_clock(self):
        def jump_callback(_, post_callback):
            self.rewind = lambda: post_callback(None)
            return NS(unregister=lambda: None)
        return NS(create_jump_callback=jump_callback)


@pytest.fixture
def bridge(monkeypatch, tmp_path):
    def odom():
        return NS(header=header(0), pose=NS(pose=pose(0., 0., 0.)),
                  twist=NS(twist=NS(linear=NS(x=0.), angular=NS(z=0.))))

    class Status(NS):
        OK, WARN = 0, 1

    for module, types in {
        'atd_interfaces.msg': dict(LocalizationStatus=object),
        'diagnostic_msgs.msg': dict(DiagnosticArray=lambda: NS(header=header(0)), DiagnosticStatus=Status, KeyValue=NS),
        'nav_msgs.msg': dict(Odometry=odom), 'rclpy.clock': dict(JumpThreshold=NS),
        'rclpy.duration': dict(Duration=NS), 'rclpy.qos': dict(DurabilityPolicy=NS(TRANSIENT_LOCAL=1),
                                                              ReliabilityPolicy=NS(BEST_EFFORT=1), QoSProfile=NS),
        'sensor_msgs.msg': dict(Imu=object), 'tf2_msgs.msg': dict(TFMessage=object),
    }.items():
        monkeypatch.setitem(sys.modules, module, NS(**types))
    node = FakeNode()
    job = OrbGapBridge(node, tmp_path, 'map1', 'hash')
    yield job, node, tmp_path
    job.close()


def read_rows(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def test_image_time_recovery_measures_correction_and_display_step(bridge):
    job, node, directory = bridge
    # First the image, then the matching IMU arrives on its independent subscription.
    job._frame(frame(100))
    job._frame(frame(200, x=.1))
    for ms in range(100, 205, 5):
        job._imu(imu(ms))
    job._frame(frame(250, 'lost', epoch=2))
    assert len(node.outputs[CONTINUOUS_ODOMETRY]) == 2
    for ms in range(205, 305, 5):
        job._imu(imu(ms))
    job._frame(frame(300, epoch=2, x=.2, y=.04, yaw=math.radians(3.)))
    result = job.close()
    assert result == job.close() == json.loads((directory / 'gyro_bridge_summary.json').read_text())
    assert result['predicted'] == 1 and result['observed'] == 3
    assert result['counts']['frame_decisions'] == 4
    assert result['measured_jumps'] == result['reacquisitions'] == 1
    assert result['position_jump_m']['median'] == pytest.approx(.04)
    assert result['abs_yaw_jump_deg']['max'] == pytest.approx(3.)
    [event] = read_rows(directory / 'reacquisitions.csv')
    assert event['previous_output_source'] == 'predicted'
    assert float(event['display_step_m']) == pytest.approx(math.hypot(.05, .04))
    assert [r['source'] for r in read_rows(directory / 'gyro_trajectory.csv')] == [
        'observed', 'observed', 'predicted', 'observed']
    status = node.outputs[CONTINUOUS_STATUS][-2].status[0]
    assert status.message == 'predicted' and float(status.values[0].value) == pytest.approx(.05)
    assert not node.subs


def test_stalled_camera_predicts_bounded_and_late_images_never_rewind_output(bridge):
    job, node, directory = bridge
    job._frame(frame(100))
    job._frame(frame(200, x=.1))
    for ms in range(100, 2700, 5):
        job._imu(imu(ms))
    outputs = node.outputs[CONTINUOUS_ODOMETRY]
    stamps = [stamp_ns(m.header.stamp) for m in outputs]
    assert 2150*10**6 < stamps[-1] <= 2200*10**6
    job._frame(frame(1000, 'lost', epoch=2))
    job._frame(frame(1100, epoch=2, x=.95, y=.1))
    assert [stamp_ns(m.header.stamp) for m in outputs] == stamps == sorted(set(stamps))
    # The prediction is compared at 1.1 s even though a later extrapolation was already displayed.
    summary = job.close()
    assert summary['counts']['late_observed'] == 1 and summary['unavailable']['horizon_exceeded'] > 0
    [event] = read_rows(directory / 'reacquisitions.csv')
    assert float(event['position_jump_m']) == pytest.approx(math.hypot(.05, .1))
    assert event['display_step_m'] == ''


def test_no_history_or_gyro_and_session_rollback(bridge):
    job, node, directory = bridge
    job._frame(frame(100))
    job._frame(frame(120, 'lost'))
    job._frame(frame(170, epoch=2, x=1.))
    job._frame(frame(200, epoch=2, x=2.))
    job._frame(frame(240, 'lost'))
    job._frame(frame(300, epoch=3, x=3.))
    job._frame(frame(330, epoch=3, x=3.1))
    node.rewind()
    job._frame(frame(350, epoch=3, x=5.))  # retired session
    job._frame(frame(50, 'lost', session='two'))
    job._frame(frame(250, session='two', x=0., map_id='other'))
    job._frame(frame(100, session='two', x=0.))
    job._frame(frame(120, session='two', x=.02))
    summary = job.close()
    assert summary['predicted'] == summary['measured_jumps'] == 0
    assert summary['reacquisitions'] == 2
    assert [r['result'] for r in read_rows(directory / 'reacquisitions.csv')] == [
        'insufficient_history', 'insufficient_gyro_coverage']
    assert node.outputs[CONTINUOUS_ODOMETRY][-1].pose.pose.position.x == .02


def test_uses_sensor_mount_and_rejects_conflicts(bridge):
    job, _, _ = bridge
    msg = imu(100, reference='optical')
    job._imu(msg)
    assert job.counts['imu_without_tf'] == 1 and not job.counts['gyro_samples']
    q = math.sqrt(.5)
    tf = NS(header=header(0, 'base_link'), child_frame_id='optical',
            transform=NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=q, z=0., w=q)))
    job._tf(NS(transforms=[tf]))
    job._frame(frame(100))
    job._frame(frame(150, x=.05))
    for ms in range(100, 205, 5):
        msg = imu(ms, reference='optical')
        msg.angular_velocity.x = -2.
        job._imu(msg)
    job._frame(frame(200, 'lost'))
    assert job.last_output[3] == pytest.approx(.1)
    with pytest.raises(ValueError, match='座標系が変わり'):
        job._imu(imu(210))

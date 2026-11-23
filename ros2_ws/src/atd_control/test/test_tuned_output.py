import json
from types import SimpleNamespace as NS

import pytest

from atd_control.drive_tuning import DEFAULT_DRIVE_TUNING
from atd_control.tuned_output import TunedOutput


def test_revision_requires_verified_writer_and_fresh_matching_plan():
    endpoint = NS(node_namespace='/', node_name='atd_auto_drive', endpoint_gid=b'writer')
    endpoints, applied, published, warnings = [], [], [], []
    output = TunedOutput.__new__(TunedOutput)
    output.node = NS(get_publishers_info_by_topic=lambda topic: endpoints, get_logger=lambda: NS(warning=warnings.append))
    output.apply, output.message_type, output.publisher = applied.append, NS, NS(publish=published.append)
    output.stamp_ns, output.route, output.pending = 0, None, None

    settings = dict(DEFAULT_DRIVE_TUNING.values(), stamp_ns=100, map_id='odaiba', map_sha256='a'*64,
                    hdmap_sha256='b'*64)
    def send(gid=b'writer', **changes):
        output.receive(NS(data=json.dumps(settings | changes)), {'publisher_gid': {'data': gid}})

    def status(ns, line='raceline', valid=True):
        return NS(header=NS(stamp=NS(sec=0, nanosec=ns)), map_id='odaiba', map_sha256='a'*64,
                  hdmap_sha256='b'*64, line=line, valid=valid)

    send()
    output.publish(status(101))
    assert not applied and not published
    endpoints.append(endpoint)
    output.resolve()
    assert len(applied) == 1
    send(stamp_ns=99)
    send()  # 同一 revision の再配信で PID を初期化しない。
    send(b'bag-player', stamp_ns=110)
    endpoints.append(NS(node_namespace='/', node_name='rosbag2_player', endpoint_gid=b'bag-player'))
    send(stamp_ns=110)
    assert len(applied) == 1
    endpoints.pop()
    send(max_speed_mps=1.5, stamp_ns=120)
    assert warnings and output.stamp_ns == 100
    for msg in (status(99), status(100), status(101, 'centerline')):
        output.publish(msg)
    wrong_map = status(101)
    wrong_map.hdmap_sha256 = 'c'*64
    output.publish(wrong_map)
    assert not published
    # 同時刻での追跡喪失も有効な版からの即時停止として届く。
    output.publish(status(101))
    output.publish(status(101, valid=False))
    assert [(m.tuning_stamp_ns, m.status.valid) for m in published] == [(100, True), (100, False)]
    send(line='centerline', stamp_ns=202)
    output.publish(status(204))
    output.publish(status(202, 'centerline'))
    output.publish(status(203, 'centerline'))
    assert [m.tuning_stamp_ns for m in published] == [100, 100, 202]
    assert applied[-1].line == 'centerline'


def test_speed_and_steering_apply_one_revision_and_forget_previous_pid():
    pytest.importorskip('rclpy')
    from atd_control.monitor import ControlMonitor
    from atd_control.speed import FeedforwardTable, SpeedController
    from atd_control.speed_monitor import SpeedMonitor

    old = DEFAULT_DRIVE_TUNING
    changed = old.from_dict(dict(old.values(), speed_kp=2., max_speed_mps=.4, min_lookahead_m=.22))
    table = FeedforwardTable('run', 'f'*64, (.2, 1.), (1100., 1300.), 1)
    pid = SpeedController(table, old.pid, max_target_speed_mps=.5)
    for ns in (100_000_000, 200_000_000):
        pid.update(ns, .3, .23)
    assert pid.last['i_raw'] > 0
    speed = NS(tables=NS(table=table), plan=object(), controller=pid)
    steering = NS(source=object())
    SpeedMonitor.apply_tuning(speed, changed)
    ControlMonitor.apply_tuning(steering, changed)
    assert steering.config == changed.lookahead and steering.source is None
    assert speed.plan is None and speed.controller.stamp_ns is None and speed.config == changed.pid
    fresh = speed.controller.update(300_000_000, .5, .25)
    assert fresh['target_speed_mps'] == .4 and fresh['i_raw'] == fresh['d_raw'] == 0.

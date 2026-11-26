import csv
import json
import sys
from types import SimpleNamespace


from atd_localization.replay_frame_trace import Event, FrameTrace


def test_processed_gap_counts_exact_bag_pairs_and_overlapping_localizations(tmp_path):
    ns = 1_000_000_000
    left = [s * ns for s in (1, 2, 3, 4)]
    right = [stamp + 1_000_000 for stamp in left]
    trace = FrameTrace()
    for i, (l_stamp, r_stamp) in enumerate(zip(left, right)):
        trace.events.append(Event('left_published', 20 * ns + l_stamp, l_stamp))
        if i != 2:
            trace.events.append(Event('right_published', 20 * ns + r_stamp, r_stamp))
    trace.events.extend([
        Event('localize_start', 21 * ns), Event('localize_failure', 23 * ns),
        Event('processed', 22 * ns, right[0], callback_s=.9, track_s=.002),
        Event('processed', 24 * ns, right[3], callback_s=.2, track_s=.003),
        Event('hint', 22 * ns, left[1]), Event('odometry', 24 * ns, right[3]),
        Event('localize_start', 24 * ns, detail='final outcome was not observed')])
    trace.events.sort(key=lambda e: e.received_ns)
    trace._log(SimpleNamespace(name='visual_slam', msg='Delta between current and previous frame '
                               '[3000.000000 ms] is above threshold [34.000000 ms]'))
    result = trace.write(tmp_path, {'left': left, 'right': right})
    assert json.loads((tmp_path / 'frame_trace.json').read_text()) == result
    with (tmp_path / 'frame_trace.csv').open() as stream:
        assert list(csv.DictReader(stream))[-1]['warning_ms'] == '3000.0'
    assert (result['recorded_pairs'], result['publication_pairs_observed'], result['processed_frames']) == (4, 3, 2)
    assert (result['odometry_frames'], result['largest_warning_ms']) == (1, 3000.)
    gap = result['largest_processed_gap']
    assert (gap['duration_s'], gap['recorded_pairs_inside'], gap['publication_pairs_observed_inside']) == (3., 2, 1)
    assert gap['largest_notice_arrival_gap_s'] == 2.
    assert gap['largest_recorded_pair_gap_s'] == 1.
    assert gap['before_callback_s'] == .9 and gap['after_track_s'] == .003
    assert gap['hint_stamps_inside'] == [left[1]]
    assert gap['overlapping_complete_localizations'] == [0]
    assert result['localization_windows'][1]['end_received_ns'] is None


def test_missing_notices_and_sparse_status_do_not_imply_bag_loss():
    trace = FrameTrace()
    stamps = {'left': [1, 2, 3], 'right': [1, 2, 3]}
    trace.events.append(Event('processed', 1, 1))
    trace._log(SimpleNamespace(name='different_source', msg='Trying to localize in map'))
    result = trace.summary(stamps)
    assert result['processed_frames'] == 1 and result['largest_processed_gap'] is None
    assert result['largest_warning_ms'] is None and result['localization_windows'] == []
    assert result['publication_pairs_observed'] == 0 and result['left_publication_notices'] == 0
    trace.events.append(Event('processed', 4, 3))
    gap = trace.summary(stamps)['largest_processed_gap']
    assert gap['recorded_pairs_inside'] == 1 and gap['publication_pairs_observed_inside'] == 0
    assert gap['largest_notice_arrival_gap_s'] is None and gap['overlapping_complete_localizations'] is None


def test_subscription_and_publisher_clock(monkeypatch):
    for module, definitions in {
        'std_msgs.msg': dict(Header=type('Header', (), {})), 'nav_msgs.msg': dict(Odometry=object),
        'geometry_msgs.msg': dict(PoseWithCovarianceStamped=object), 'rcl_interfaces.msg': dict(Log=object),
        'isaac_ros_visual_slam_interfaces.msg': dict(VisualSlamStatus=object),
        'rclpy.qos': dict(QoSProfile=SimpleNamespace, ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=2)),
    }.items():
        monkeypatch.setitem(sys.modules, module, SimpleNamespace(**definitions))
    subs = {}
    def subscribe(msg_type, topic, callback, qos, *, raw=False):
        subs[topic] = (callback, raw)
    node = SimpleNamespace(create_subscription=subscribe)
    trace = FrameTrace()
    trace.subscribe(node, '/bag_input')
    assert set(subs) == {'/bag_input/left_published', '/bag_input/right_published', '/visual_slam/status',
                         '/visual_slam/tracking/odometry', '/localization/pose_hint', '/rosout'}
    monkeypatch.setitem(sys.modules, 'tf2_msgs.msg', SimpleNamespace(TFMessage=object))
    trace.subscribe(node, '/bag_input', vgl_corrections=True)
    stamp = SimpleNamespace(sec=5, nanosec=0)
    subs['/tf'][0](SimpleNamespace(transforms=[
        SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id='odom'), child_frame_id='base_link'),
        SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id='map'), child_frame_id='odom'),
    ]))
    assert [(event.stream, event.stamp_ns) for event in trace.events] == [('map_to_odom', 5_000_000_000)]
    stamps = [1_000_000_000, 2_000_000_000, 3_000_000_000]
    for i, stamp in enumerate(stamps):
        for topic in ('/bag_input/left_published', '/bag_input/right_published'):
            callback, raw = subs[topic]
            assert not raw
            callback(SimpleNamespace(stamp=SimpleNamespace(sec=i + 1, nanosec=0)),
                     dict(source_timestamp=10_000_000_000 + i * 5_000_000, publication_sequence_number=i + 1))
        if i != 1:
            msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=i + 1, nanosec=0)),
                                  vo_state=1, node_callback_execution_time=.004, track_execution_time=.001)
            subs['/visual_slam/status'][0](msg)
    summary = trace.summary({'left': stamps, 'right': stamps})
    assert summary['largest_processed_gap']['largest_notice_send_gap_s'] == .005
    assert summary['max_track_s'] == .001
    assert trace.observed_tail(dict(left_images=3, right_images=3, last_stamp_ns=stamps[-1]))
    assert not trace.observed_tail(dict(left_images=3, right_images=4, last_stamp_ns=stamps[-1]))
    assert not trace.observed_tail(dict(left_images=3, right_images=3, last_stamp_ns=stamps[-1] + 50_000_000))

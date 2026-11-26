"""ROS/GPU を使わない別 bag 検証の入出力・時刻・失敗状態の回帰テスト。"""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from atd_localization import assets, replay
from atd_localization.replay_inputs import camera_signature, independent_bag, stereo_window
from atd_localization.replay_report import TrackingReport


def bag(root, name, start):
    folder = root / name
    folder.mkdir(parents=True)
    metadata = dict(rosbag2_bagfile_information=dict(storage_identifier='mcap', starting_time=dict(nanoseconds_since_epoch=start),
                    duration=dict(nanoseconds=10**9), topics_with_message_count=[]))
    data = yaml.safe_dump(metadata).encode()
    (folder / 'metadata.yaml').write_bytes(data)
    return hashlib.sha256(data).hexdigest()


def test_separate_recording_detects_duplicate_and_rewritten_metadata(tmp_path):
    source = 'day1/run'
    digest = bag(tmp_path, source, 10)
    bag(tmp_path, 'copy/renamed', 10)
    other = bag(tmp_path, 'day2/run', 20)
    for relative in (source, 'copy/renamed'):
        with pytest.raises(ValueError, match='別走行'):
            independent_bag(tmp_path, relative, source, digest)
    with (tmp_path / 'copy/renamed/metadata.yaml').open('a') as stream:
        stream.write('# changed\n')
    with pytest.raises(ValueError, match='同一'):
        independent_bag(tmp_path, 'copy/renamed', source, digest)
    assert independent_bag(tmp_path, 'day2/run', source, digest) == (tmp_path / 'day2/run', other)
    with pytest.raises(ValueError, match='相対パス'):
        independent_bag(tmp_path, '../escape', source, digest)


def test_observed_stereo_clock_and_calibration():
    m = 1_000_000
    result = stereo_window([10*m, 20*m, 30*m, 40*m], [2*m, 21*m, 33*m, 52*m])
    assert result == dict(stereo_pairs=2, first_stamp_ns=21*m, last_stamp_ns=33*m, left_images=4,
                          right_images=4, largest_pair_gap_s=.012)
    with pytest.raises(ValueError, match='ステレオ観測'):
        stereo_window([m, 2*m], [20*m, 21*m])
    info = SimpleNamespace(k=[1.]*9, p=[1.]*12, r=[1.]*9, d=[], distortion_model='plumb_bob')
    assert camera_signature(info)
    info.k[0] = 0.
    with pytest.raises(ValueError, match='未校正'):
        camera_signature(info)


def test_coverage_only_counts_distinct_fresh_samples_in_same_episode():
    n = 10**9
    evidence = dict(first_stamp_ns=n, last_stamp_ns=5*n)
    stats = TrackingReport(evidence, max_gap_s=.6, tail_tolerance_s=.1)
    assert stats.summary()['outcome'] == 'no_tracking'
    value = (0., 0., 0., 0., 0., 0., 1., .8, .1)
    def pose(stamp, epoch=0):
        return stats.observe('run', epoch, 'tracking', '', int(stamp*n), value)
    assert pose(1.5)
    assert not pose(1.5)
    pose(2.)
    stats.observe('run', 1, 'lost', 'lost', int(2.25*n))
    assert stats.summary()['outcome'] == 'partial_tracking'
    pose(2.5, 1)
    pose(4.5, 1)
    pose(5., 1)
    result = stats.summary()
    assert result['outcome'] == 'tracked_to_end' and result['pose_samples'] == 5
    assert result['tracking_time_fraction'] == .25 and result['first_tracking_delay_s'] == .5
    assert len(result['tracking_intervals']) == 3
    with pytest.raises(ValueError, match='セッション'):
        stats.observe('another', 1, 'lost', '', 5*n)
    with pytest.raises(ValueError, match='逆行'):
        pose(4., 1)
    with pytest.raises(ValueError, match='区間外'):
        pose(6., 1)


@pytest.mark.parametrize('status, code', [('tracking', 0), ('lost', 2), ('error', 1)])
def test_cli_runs_common_launch_and_persists_outcome(tmp_path, monkeypatch, status, code):
    records = tmp_path / 'record'
    source_hash = bag(records, 'a/run', 1)
    bag(records, 'b/run', 2)
    results = tmp_path / 'results'
    hdmap = results / 'maps/course/hdmap/map.json'
    hdmap.parent.mkdir(parents=True)
    hdmap.write_bytes(b'{"id":"course","frame":"map","lines":{}}')
    meta = dict(map_id='course', map_sha256='a'*64, hdmap_sha256=hashlib.sha256(hdmap.read_bytes()).hexdigest(), source_bag='a/run',
                source_metadata_sha256=source_hash, input_shape=[1,3,480,640])
    monkeypatch.setattr(assets, 'map_metadata', lambda *_: meta)
    window = dict(first_stamp_ns=10**9, last_stamp_ns=15*10**8)
    monkeypatch.setattr(replay, 'inspect_inputs', lambda *args: (['/left', '/right', '/left_info', '/right_info', '/tf_static'], .5, window))
    def fake_ros(args, resolved, replay_bag, topics, duration, report, output):
        assert resolved == meta and replay_bag == records / 'b/run'
        launch = replay.launch_command(args.results_root, args.map_id, args.engine_dir)
        assert launch[1:4] == ['launch','atd_localization','localization.launch.py'] and 'use_sim_time:=true' in launch
        play = replay.play_command(replay_bag, topics, '/tmp/qos', args.rate)
        assert '--clock' in play and play[play.index('--topics')+1:] == topics
        assert not {'/tf','/localization/status','/vehicle/command','/clock'} & set(topics)
        if status == 'error':
            raise ValueError('launch failed')
        report.observe('run', 0, status, '', 10**9, [0., .2] if status == 'tracking' else None)
        if status == 'tracking':
            report.observe('run', 0, status, '', 15*10**8, [1., .2])
    monkeypatch.setattr(replay, 'run_ros', fake_ros)
    run_id = 'a' * 32
    args = ['--map', 'course', '--bag', 'b/run', '--run-id', run_id,
            '--results-root', str(results), '--record-root', str(records)]
    assert replay.main(args) == code
    [path] = (results / 'runs/localization').glob('*/report.json')
    assert path.parent.name == run_id
    result = json.loads(path.read_text())
    assert (path.parent / 'hdmap.json').read_bytes() == hdmap.read_bytes()
    assert result['outcome'] == {'tracking':'tracked_to_end','lost':'no_tracking','error':'failed'}[status]
    assert result['bag'] == 'b/run' and result['source_metadata_sha256'] == source_hash
    assert replay.main(args) == 1
    assert json.loads(path.read_text()) == result
    assert replay.main(['--map','course','--bag','a/run','--results-root',str(results),'--record-root',str(records)]) == 1


def test_inspect_bag_reads_real_topic_contract_and_refuses_frame_mismatch(tmp_path, monkeypatch):
    import sys
    from atd_localization.replay_inputs import inspect_inputs
    workflow = dict(left_image='/l', right_image='/r', left_info='/li', right_info='/ri',
                    base_frame='base_link', left_frame='left_optical', right_frame='right_optical', tf_static='/tf_static')
    description = dict(workflow=workflow, input_shape=[1, 3, 480, 640])
    class Image:
        pass
    class CameraInfo:
        pass
    class TFMessage:
        pass
    def h(frame, tick):
        return SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=10, nanosec=tick))
    def camera(frame, tick):
        return SimpleNamespace(header=h(frame,tick), height=480, width=640, k=[1.]*9,
                               p=[1.]*12, r=[1.]*9, d=[], distortion_model='plumb_bob')
    raw = [('/tf_static', SimpleNamespace(transforms=[
        SimpleNamespace(header=h('base_link',0), child_frame_id=workflow[k])
        for k in ('left_frame', 'right_frame')]))]
    for side in ('left','right'):
        frame = workflow[side+'_frame']
        for tick in (0, 20_000_000):
            for kind in ('image', 'info'):
                raw.append((workflow[f'{side}_{kind}'],camera(frame, tick)))
    types = dict.fromkeys(['/l','/r'],'sensor_msgs/msg/Image')
    types.update(dict.fromkeys(['/li','/ri'],'sensor_msgs/msg/CameraInfo'))
    types['/tf_static'] = 'tf2_msgs/msg/TFMessage'
    counts = [dict(topic_metadata=dict(name=topic,type=kind), message_count=sum(t==topic for t,_ in raw)) for topic,kind in types.items()]
    (tmp_path / 'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=dict(
        storage_identifier='mcap', duration=dict(nanoseconds=1000000000), topics_with_message_count=counts))))
    class Reader:
        def open(self, storage, converter):
            assert storage.uri == str(tmp_path)
        def set_filter(self, filtering):
            self.records = [(topic,msg,0) for topic,msg in raw if topic in filtering.topics]
            self.index = 0
        def has_next(self):
            return self.index < len(self.records)
        def read_next(self):
            self.index += 1
            return self.records[self.index-1]
    monkeypatch.setitem(sys.modules,'rosbag2_py',SimpleNamespace(StorageOptions=SimpleNamespace, SequentialReader=Reader,
        ConverterOptions=lambda *_:None, StorageFilter=SimpleNamespace))
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',SimpleNamespace(CameraInfo=CameraInfo, Image=Image))
    monkeypatch.setitem(sys.modules,'tf2_msgs.msg',SimpleNamespace(TFMessage=TFMessage))
    monkeypatch.setitem(sys.modules,'rclpy.serialization',SimpleNamespace(deserialize_message=lambda data,cls:data))
    topics, duration, evidence = inspect_inputs(tmp_path, description)
    assert set(topics) == set(types) and duration == 1 and evidence['stereo_pairs'] == 2
    raw[-1][1].header.frame_id = 'unexpected'
    with pytest.raises(ValueError, match='入力フレーム'):
        inspect_inputs(tmp_path, description)
    raw[-1][1].header.frame_id = 'right_optical'
    raw[0][1].transforms.pop()
    with pytest.raises(ValueError, match='Missing static TF'):
        inspect_inputs(tmp_path, description)


def test_orb_startup_publishes_one_shot_calibration_before_ready_and_play(monkeypatch):
    import sys
    from unittest.mock import Mock
    from atd_localization import orb_replay

    bootstrap = orb_replay.OrbBootstrap.__new__(orb_replay.OrbBootstrap)
    bootstrap.node = Mock()
    bootstrap.ready = False
    outputs = [Mock(), Mock(), Mock()]
    bootstrap.seeds = list(zip(outputs, ['tf', 'left_info', 'right_info']))
    # Startup discovery initially sees only a subset of the camera subscribers.
    for publisher in outputs:
        publisher.get_subscription_count.return_value = 1
    outputs[-1].get_subscription_count.side_effect = [0, 1, 1]
    clock = [0.]
    def spin(node, timeout_sec):
        clock[0] += .1
        if all(p.publish.called for p in outputs):
            bootstrap.ready = True
    monkeypatch.setitem(sys.modules, 'rclpy', SimpleNamespace(spin_once=spin))
    monkeypatch.setattr(orb_replay, 'monotonic', lambda: clock[0])
    check = Mock()
    bootstrap.wait(check, 1.)
    assert check.call_count == 2
    for publisher, message in bootstrap.seeds:
        publisher.publish.assert_called_once_with(message)


def test_orb_validation_uses_only_dynamic_images_in_the_paused_player():
    command = replay.play_command('/bag', ['/infra1', '/infra2'], '/qos', .5, paused_player='orb_player')
    assert '--start-paused' in command and '__node:=orb_player' in command
    assert command[command.index('--topics')+1:] == ['/infra1', '/infra2']
    window = stereo_window([10, 20, 30], [11, 21, 31], 1, left_stamp=True)
    assert (window['first_stamp_ns'], window['last_stamp_ns']) == (10, 30)


def test_fixed_map_paused_player_waits_for_slam_vgl_and_monitor_on_remapped_topics(monkeypatch):
    import sys
    from unittest.mock import Mock

    workflow = dict(left_image='/l', right_image='/r', left_info='/li', right_info='/ri', tf_static='/tf_static')
    topic_types = dict.fromkeys(('/l', '/r'), 'sensor_msgs/msg/Image')
    topic_types.update(dict.fromkeys(('/li', '/ri'), 'sensor_msgs/msg/CameraInfo'))
    topic_types.update({'/tf_static': 'tf2_msgs/msg/TFMessage', '/clock': 'rosgraph_msgs/msg/Clock'})
    def endpoint(name, topic, qos=True):
        return SimpleNamespace(node_name=name, node_namespace='/', topic_type=topic_types[topic], qos_profile=qos)
    pubs = {t: [endpoint('player', t)] for t in topic_types}
    pubs['/tf_static'].append(endpoint('visual_slam', '/tf_static'))
    subs = {t: [endpoint(n, t) for n in ('visual_slam', 'visual_global_localization', 'atd_localization_monitor')]
            for t in topic_types}
    subs['/tf_static'] = [endpoint('transform_listener_impl_12', '/tf_static')]
    node = Mock()
    node.get_service_names_and_types.return_value = []
    node.get_publishers_info_by_topic.side_effect = pubs.__getitem__
    node.get_subscriptions_info_by_topic.side_effect = subs.__getitem__
    monkeypatch.setitem(sys.modules, 'rclpy.qos', SimpleNamespace(QoSCompatibility=SimpleNamespace(ERROR=False),
                        qos_check_compatible=lambda pub, sub: (pub == sub, '')))
    assert not replay.validation_connections(node, '/player', workflow, orb=False)
    subs['/li'][-1].qos_profile = False
    subs['/r'] = [endpoint('visual_slam', '/r')]
    issues = replay.validation_connections(node, '/player', workflow, orb=False)
    assert any('/li' in i and 'atd_localization_monitor' in i and 'QoS' in i for i in issues)
    assert any('/r' in i and 'visual_global_localization' in i for i in issues)
    for topic in ('/clock', '/l', '/r'):
        subs[topic] = [endpoint('orbslam3_node', topic)]
    assert not replay.validation_connections(node, '/player', workflow, orb=True)

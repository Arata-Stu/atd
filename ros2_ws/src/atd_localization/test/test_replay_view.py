"""固定地図再生の表示は有効な姿勢を使い、喪失区間をつながない。"""
import json
from pathlib import Path
from unittest.mock import Mock

import pytest
import yaml

pytest.importorskip('rclpy')
from atd_interfaces.msg import LocalizationStatus
from atd_localization.replay_view import ReplayView
from visualization_msgs.msg import Marker, MarkerArray


def test_saved_map_and_observed_segments_are_rendered_in_the_preset(tmp_path):
    path = tmp_path / 'hdmap.json'
    path.write_text(json.dumps(dict(closed=True, left=[[0, 0], [0, 4], [3, 4]], right=[],
                                    lines={'custom': dict(closed=False, points=[[0, 1], [0, 2]])})))
    node = Mock()
    hdmap, trajectory = Mock(), Mock()
    node.create_publisher.side_effect = [hdmap, trajectory]
    view = ReplayView(node, path)
    config = Path(__file__).resolve().parents[1] / 'config/fixed_map_replay.rviz'
    manager = yaml.safe_load(config.read_text())['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'map'
    topics = {item['Topic']['Value']: item for item in manager['Displays'] if 'Topic' in item}
    for call in node.create_publisher.call_args_list:
        kind, name, qos = call.args
        assert kind is MarkerArray
        display = topics.pop(name)
        assert display['Enabled'] and display['Class'] == 'rviz_default_plugins/MarkerArray'
        assert display['Topic']['Durability Policy'] == 'Transient Local' and qos.durability.name == 'TRANSIENT_LOCAL'
        assert display['Topic']['Reliability Policy'] == 'Reliable' and qos.reliability.name == 'RELIABLE'
    assert not topics
    boundary, reference = hdmap.publish.call_args.args[0].markers
    assert boundary.ns == 'left' and boundary.points[0] == boundary.points[-1]
    assert reference.ns == 'custom' and len(reference.points) == 2
    hdmap.get_subscription_count.return_value = 1
    trajectory.get_subscription_count.return_value = 0
    assert not view.ready()
    trajectory.get_subscription_count.return_value = 1
    assert view.ready()

    def track(x, stamp, interval):
        msg = LocalizationStatus()
        msg.header.frame_id, msg.header.stamp.sec = 'map', stamp
        msg.pose.position.x, msg.pose.orientation.w = float(x), 1.
        view.observe(msg, interval)

    first, reacquired = {}, {}
    track(0, 1, first)
    track(1, 2, first)
    view.observe(LocalizationStatus(state='lost'), None)
    view.flush()
    trail, arrow = trajectory.publish.call_args.args[0].markers
    assert [p.x for p in trail.points] == [0, 1] and arrow.action == Marker.DELETE
    track(7, 4, reacquired)
    track(8, 5, reacquired)
    third = {}  # A time gap can start a new interval without an intermediate lost status.
    track(12, 7, third)
    track(13, 8, third)
    view.flush(wait_for_ack=True)
    trail, arrow = trajectory.publish.call_args.args[0].markers
    assert trail.type == Marker.LINE_LIST and [p.x for p in trail.points] == [0, 1, 7, 8, 12, 13]
    assert arrow.type == Marker.ARROW and arrow.action == Marker.ADD
    assert arrow.header.frame_id == 'map' and arrow.header.stamp.sec == 8 and arrow.pose.position.x == 13
    assert trajectory.wait_for_all_acked.call_args.args[0].nanoseconds == 1_000_000_000

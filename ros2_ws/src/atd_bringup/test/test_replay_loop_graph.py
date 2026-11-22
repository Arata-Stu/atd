from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from atd_bringup.replay_gt_geometry import PlanarPose, Trace
from atd_bringup.replay_loop_graph import GRAPH_LIMIT, ReplayLoopGraph, inspect_graph
from atd_bringup.replay_result import CourseProgress


def header(ns):
    sec, nanosec = divmod(ns, 1_000_000_000)
    return NS(frame_id='map', stamp=NS(sec=sec, nanosec=nanosec))


class LoopGraphTests(unittest.TestCase):
    def setUp(self):
        trace = Trace(tuple(i * 500_000_000 for i in range(5)),
                      tuple(PlanarPose(x, 0., 0.) for x in (0., 2., 0., 2., 0.)))
        self.progress = CourseProgress(trace, dict(laps=2, lap_duration_s=1., lap_length_m=4., start_ns=0))
        self.xyz = [NS(x=i * .1, y=0., z=0.) for i in range(7)]
        self.nodes = NS(poses=[NS(position=xyz) for xyz in self.xyz], header=header(1_900_000_000))
        self.poses = [NS(pose=pose, header=header(100_000_000 + i * 300_000_000))
                      for i, pose in enumerate(self.nodes.poses)]
        self.chain = [(i, i+1) for i in range(6)]

    def check_graph(self, links):
        edges = NS(points=[self.xyz[i] for link in links for i in link], header=self.nodes.header)
        return inspect_graph(self.progress, self.nodes, edges, self.poses)

    def test_graph_chain_and_immediate_lap_transition_are_not_revisit_evidence(self):
        self.assertEqual(self.check_graph(self.chain)['status'], 'not_observed')
        self.assertEqual(self.check_graph(self.chain + [(0, 2)])['status'], 'not_observed')

    def test_only_a_verified_cross_lap_cycle_counts(self):
        report = self.check_graph(self.chain + [(0, 4)])
        self.assertEqual(report['status'], 'cross_lap_cycle')
        self.assertEqual(report['cross_lap_cycle_edges'], 1)
        a = report['example']
        self.assertEqual((a['from_lap'], a['to_lap']), (1, 2))
        self.assertAlmostEqual(a['from_station_m'], .4)
        self.assertAlmostEqual(a['to_station_m'], 1.2)
        self.assertAlmostEqual(a['gt_separation_m'], .8)
        self.assertEqual(self.check_graph(self.chain[1:] + [(0, 4)])['status'], 'not_observed')

    def test_coordinate_collision_stale_graph_and_truncated_views(self):
        links = self.chain + [(0, 4)]
        self.poses.append(NS(pose=self.nodes.poses[0], header=header(1_850_000_000)))
        self.assertEqual(self.check_graph(links)['status'], 'not_observed')
        self.poses.pop()
        self.nodes.poses.append(self.nodes.poses[0])
        self.assertEqual(self.check_graph(links)['status'], 'not_observed')
        self.nodes.poses.pop()
        for pose in self.poses:
            pose.pose = NS(position=NS(x=42., y=0., z=0.))
        self.assertEqual(self.check_graph(links)['status'], 'unavailable')
        self.nodes.poses.extend([self.nodes.poses[0]] * GRAPH_LIMIT)
        self.assertIn('上限', self.check_graph(links)['reason'])

    def test_only_same_stamp_node_and_edge_snapshots_are_paired(self):
        graph = ReplayLoopGraph.__new__(ReplayLoopGraph)
        graph.latest = graph.nodes = graph.edges = None
        edges = NS(header=header(1), ns='edges', id=0, type=5, action=0, LINE_LIST=5, ADD=0)
        graph.on_edges(edges)
        graph.on_nodes(NS(header=header(2)))
        self.assertIsNone(graph.latest)
        pair = NS(header=header(2), ns='edges', id=0, type=5, action=0, LINE_LIST=5, ADD=0)
        graph.on_edges(pair)
        self.assertIs(graph.latest[1], pair)
        graph.on_nodes(NS(header=header(1)))
        graph.on_edges(edges)
        self.assertIs(graph.latest[1], pair)

    def test_old_snapshot_cannot_match_future_revisited_point(self):
        self.nodes.header = header(800_000_000)
        self.assertIn('1周目', self.check_graph(self.chain + [(0, 4)])['reason'])
        self.nodes.header = header(1_100_000_000)
        # s=1.3s の点のXYZが旧グラフにも出現したという入力。旧スナップショットの辺に将来の時刻を与えない。
        self.assertEqual(self.check_graph(self.chain + [(0, 4)])['status'], 'not_observed')
        self.poses = [self.poses[i] for i in (0, 1, 2, 4)]
        self.assertIn('異なる2周', self.check_graph(self.chain + [(0, 4)])['reason'])

    def test_final_inspection_timeout_preserves_unknown_result(self):
        graph = ReplayLoopGraph.__new__(ReplayLoopGraph)
        graph.latest = None
        graph.client = Mock()
        future = graph.client.call_async.return_value
        future.done.return_value = False
        srv = NS(GetAllPoses=NS(Request=NS))
        with patch.dict('sys.modules', {'isaac_ros_visual_slam_interfaces.srv': srv}), patch(
                'atd_bringup.replay_loop_graph.time.monotonic', side_effect=[0., 1., 1., 5.]):
            result = graph.finish(Mock())
        self.assertEqual(result['status'], 'unavailable')
        self.assertIn('応答', result['reason'])
        future.cancel.assert_called_once_with()

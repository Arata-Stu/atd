"""cuVSLAMの確定した姿勢グラフから、異なる周回を直接結ぶ閉路を照合する。"""
from bisect import bisect_right
from collections import Counter
import math
import time

from atd_mapping.frames import stamp_ns


# Isaac ROS 5 の pose_graph_helper が ReadPoseGraph に設定する可視化要素数。
GRAPH_LIMIT = 2048
POSE_LIMIT = 100_000


def _xyz(point):
    return (point.x, point.y, point.z)


def inspect_graph(progress, nodes, edges, poses):
    """両端の時刻が一意な長時間辺と、短時間辺だけで結べる別経路の両方を要求する。"""
    limits = progress.boundaries
    snapshot_ns = stamp_ns(edges.header.stamp)
    report = dict(status='unavailable', graph_stamp_ns=str(snapshot_ns),
                  nodes=len(nodes.poses), edges=len(edges.points) // 2, poses=len(poses))
    if len(limits) < 3:
        return dict(report, reason='周回の時刻境界を照合できません')
    if snapshot_ns < limits[1]:
        return dict(report, reason='採取できた姿勢グラフが1周目で終わっています')
    if (len(nodes.poses) >= GRAPH_LIMIT or len(edges.points) >= 2 * GRAPH_LIMIT or len(poses) >= POSE_LIMIT):
        return dict(report, reason='取得件数が上限に達しました')
    if len(edges.points) % 2 or not nodes.poses or not poses:
        return dict(report, reason='姿勢またはグラフの頂点・辺が不足しています')
    # ROS 5 の pose_graph_edges はIDを省略する。両トピックとget_all_posesは同じmap座標変換を使う。
    # 異なる時刻や頂点が同じXYZにある場合、最近傍による補完は誤って閉路を作るので除外する。
    node_positions = Counter(_xyz(node.position) for node in nodes.poses)
    matched = {}
    for pose in poses:
        # GetAllPoses の ROS 5 実装は stamp のみ設定し、map 座標を返す。
        if pose.header.frame_id not in ('', 'map'):
            return dict(report, reason='最適化後の姿勢が map 座標と一致しません')
        xyz = _xyz(pose.pose.position)
        if node_positions[xyz] == 1:
            matched[xyz] = None if xyz in matched else stamp_ns(pose.header.stamp)
    start = max(limits[0], progress.trace.stamps[0])
    finish = min(limits[-1], progress.trace.stamps[-1], snapshot_ns)
    stamps = {xyz: stamp for xyz, stamp in matched.items() if stamp is not None and start <= stamp <= finish}
    # 一つの画像時刻に複数の座標が対応した場合も除く。
    frequency = Counter(stamps.values())
    stamps = {xyz: stamp for xyz, stamp in stamps.items() if frequency[stamp] == 1}
    laps = {min(bisect_right(limits, stamp), len(limits) - 1) for stamp in stamps.values()}
    if len(laps) < 2:
        return dict(report, matched_nodes=len(stamps), reason='異なる2周のグラフ頂点を一意に照合できません')
    parents = {stamp: stamp for stamp in stamps.values()}

    def root(stamp):
        while parents[stamp] != stamp:
            parents[stamp] = parents[parents[stamp]]
            stamp = parents[stamp]
        return stamp

    half_lap_ns = (limits[1] - limits[0]) // 2
    candidates = set()
    matched_edges = 0
    for start, end in zip(edges.points[::2], edges.points[1::2]):
        s, t = stamps.get(_xyz(start)), stamps.get(_xyz(end))
        if s is None or t is None or s == t:
            continue
        matched_edges += 1
        s, t = sorted((s, t))
        if t - s < half_lap_ns:
            parents[root(t)] = root(s)
        elif min(bisect_right(limits, s), len(limits) - 1) != min(bisect_right(limits, t), len(limits) - 1):
            candidates.add((s, t))

    report.update(matched_nodes=len(stamps), matched_edges=matched_edges, cross_lap_cycle_edges=0)
    if not matched_edges:
        return dict(report, reason='最適化後の姿勢時刻とグラフ辺の座標を一意に照合できません')
    for s, t in sorted(candidates):
        if root(s) != root(t):
            continue
        report['cross_lap_cycle_edges'] += 1
        a, b = [progress.point(u, progress.trace.at(u)) for u in (s, t)]
        delta = abs(math.remainder(a['station_m'] - b['station_m'], progress.lap_length))
        if 'example' not in report or delta < report['example']['station_difference_m']:
            report['example'] = dict(from_stamp_ns=str(s), to_stamp_ns=str(t), from_lap=a['lap'], to_lap=b['lap'],
                                     from_station_m=a['station_m'], to_station_m=b['station_m'],
                                     station_difference_m=delta,
                                     gt_separation_m=math.hypot(a['gt_x_m']-b['gt_x_m'], a['gt_y_m']-b['gt_y_m']))
    report['status'] = 'cross_lap_cycle' if report['cross_lap_cycle_edges'] else 'not_observed'
    return report


class ReplayLoopGraph:
    """最後に揃った同一時刻の可視化と再生終了時の最適化後姿勢を使う。"""

    def __init__(self, node, progress, qos):
        from geometry_msgs.msg import PoseArray
        from isaac_ros_visual_slam_interfaces.srv import GetAllPoses
        from visualization_msgs.msg import Marker

        self.progress = progress
        self.latest = None
        self.nodes = None
        self.edges = None
        self.client = node.create_client(GetAllPoses, '/visual_slam/get_all_poses')
        node.create_subscription(PoseArray, '/visual_slam/vis/pose_graph_nodes', self.on_nodes, qos)
        node.create_subscription(Marker, '/visual_slam/vis/pose_graph_edges', self.on_edges, qos)

    def on_nodes(self, msg):
        if msg.header.frame_id == 'map':
            self.nodes = msg
            self._pair()

    def on_edges(self, msg):
        if (msg.header.frame_id == 'map' and msg.ns == 'edges' and msg.id == 0
                and msg.type == msg.LINE_LIST and msg.action == msg.ADD):
            self.edges = msg
            self._pair()

    def _pair(self):
        if (self.nodes is not None and self.edges is not None and self.nodes.header.stamp == self.edges.header.stamp
                and (self.latest is None or stamp_ns(self.nodes.header.stamp) > stamp_ns(self.latest[0].header.stamp))):
            self.latest = self.nodes, self.edges

    def finish(self, spin):
        from isaac_ros_visual_slam_interfaces.srv import GetAllPoses

        # ROS と SDK の可視化が独立したスレッドで動くため、末尾のグラフ配信を短時間待つ。
        settle = time.monotonic() + .3
        while time.monotonic() < settle:
            spin(timeout_sec=.05)
        if not self.client.service_is_ready():
            return dict(status='unavailable', reason='get_all_poses サービスを利用できません')
        request = GetAllPoses.Request()
        request.max_count = POSE_LIMIT
        future = self.client.call_async(request)
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline and (not future.done() or self.latest is None):
            spin(timeout_sec=.05)
        if not future.done():
            future.cancel()
            return dict(status='unavailable', reason='get_all_poses の応答がありません')
        response = future.result()
        if self.latest is None or not response.success:
            return dict(status='unavailable', reason='同一時刻の姿勢グラフまたは最適化後の姿勢を取得できません')
        return inspect_graph(self.progress, *self.latest, response.poses)

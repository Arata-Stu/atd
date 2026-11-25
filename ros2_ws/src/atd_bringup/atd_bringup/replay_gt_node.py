"""再生した Gazebo の走行軌跡を cuVSLAM の初期姿勢に合わせて RViz に配信する。"""

import argparse
from bisect import bisect_right
from contextlib import contextmanager
import math
import sys
from typing import NamedTuple, Callable

from atd_mapping.frames import stamp_ns

from .replay_gt_geometry import Alignment, Trace
from .replay_result import ReplayRewound


class ReplayObserver(NamedTuple):
    spin_once: Callable
    inspect_loop_graph: Callable


class GroundTruthPath:
    def __init__(self, trace, make_pose, result=None, *, fixed_alignment=None):
        self.fixed_alignment = fixed_alignment
        self.result = result
        self.trace = trace
        self.make_pose = make_pose
        self.clock_ns = None
        self._reset()

    def _reset(self):
        self.alignment = None
        self.anchor_ns = None
        self.last_tip_ns = None
        self.next_index = 0
        self.history = []
        self.latest_estimate_ns = None
        self.pending_error = None

    def set_clock(self, stamp_ns):
        rewind = self.clock_ns is not None and stamp_ns < self.clock_ns
        if rewind:
            if self.result is not None and (self.result.last_stamp is not None
                                            or self.result.last_cloud_stamp is not None):
                raise ReplayRewound('再生時計が戻ったため、直前までの比較結果を保存しました')
            self._reset()
        self.clock_ns = stamp_ns
        return rewind

    def observe_odometry(self, odom):
        if odom.header.frame_id != 'map' or odom.child_frame_id != 'base_link':
            return False
        stamp = stamp_ns(odom.header.stamp)
        # seek 前の推定値が遅着しても、未来の初期姿勢に固定しない。
        anchor = self.anchor_ns is None or (
            self.result is None and stamp < self.anchor_ns and self.last_tip_ns is None
            and (self.clock_ns is None or self.clock_ns < self.anchor_ns))
        if not anchor and stamp <= self.latest_estimate_ns:
            return False
        sample = self.trace.at(stamp)
        if sample is None:
            return False
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        position = (p.x, p.y, p.z)
        if not all(map(math.isfinite, position)):
            raise ValueError('推定位置に非有限値があります')
        orientation = (q.x, q.y, q.z, q.w)
        if anchor:
            self.alignment = self.fixed_alignment or Alignment.from_anchor(sample, position, orientation)
            self.anchor_ns = stamp
            self.history = [self.make_pose(stamp, *self.alignment.apply(sample))]
            self.next_index = bisect_right(self.trace.stamps, stamp)
        if self.result is not None:
            self.result.record(stamp, sample, self.alignment, position, orientation)
        self.pending_error = (stamp, position, sample)
        self.latest_estimate_ns = stamp
        return anchor

    def take_error(self):
        if self.pending_error is None or self.clock_ns is None or self.clock_ns < self.anchor_ns:
            return None
        stamp, position, sample = self.pending_error
        self.pending_error = None
        truth, _ = self.alignment.apply(sample)
        return stamp, position, truth

    def update(self):
        if self.anchor_ns is None or self.clock_ns is None or self.clock_ns < self.anchor_ns:
            return None
        tip = min(self.clock_ns, self.trace.stamps[-1])
        if tip == self.last_tip_ns:
            return None
        self.last_tip_ns = tip
        end = bisect_right(self.trace.stamps, tip, lo=self.next_index)
        for index in range(self.next_index, end):
            pose = self.alignment.apply(self.trace.poses[index])
            self.history.append(self.make_pose(self.trace.stamps[index], *pose))
        self.next_index = end
        if tip == self.anchor_ns or tip == self.trace.stamps[end - 1]:
            return tip, self.history
        # 補間した先端だけは次の更新で置き換え、実測点を履歴に保持する。
        current = self.alignment.apply(self.trace.at(tip))
        return tip, [*self.history, self.make_pose(tip, *current)]


@contextmanager
def ground_truth_viewer(trace, result=None, *, visualize=True, orb=None):
    import rclpy
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.clock import Clock as WallClock, ClockType
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2

    from .replay_inputs import (
        GROUND_TRUTH_ERROR, GROUND_TRUTH_NODE, GROUND_TRUTH_PATH, LOOP_CLOSURE_CLOUD, SLAM_ODOMETRY,
    )

    def set_header(header, stamp):
        header.frame_id = 'map'
        header.stamp.sec, header.stamp.nanosec = divmod(stamp, 1_000_000_000)

    def make_pose(stamp, position, orientation):
        msg = PoseStamped()
        set_header(msg.header, stamp)
        p, q = msg.pose.position, msg.pose.orientation
        p.x, p.y, p.z = position
        q.x, q.y, q.z, q.w = orientation
        return msg

    path = GroundTruthPath(trace, make_pose, result, fixed_alignment=orb.fixed_alignment if orb else None)
    executor = SingleThreadedExecutor()
    node = rclpy.create_node(GROUND_TRUTH_NODE)
    try:
        if visualize:
            from visualization_msgs.msg import Marker
            output_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
            publisher = node.create_publisher(Path, GROUND_TRUTH_PATH, output_qos)
            errors = node.create_publisher(Marker, GROUND_TRUTH_ERROR, output_qos)

        def publish(stamp, poses):
            msg = Path()
            set_header(msg.header, stamp)
            msg.poses = poses
            publisher.publish(msg)

        def publish_error(stamp, *positions):
            marker = Marker()
            set_header(marker.header, stamp)
            marker.ns = 'position_error'
            marker.id = 0
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD if positions else Marker.DELETE
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.08
            marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in positions]
            errors.publish(marker)

        def on_odometry(msg):
            try:
                if path.observe_odometry(msg):
                    node.get_logger().info(f'Gazebo 正解軌跡と観測姿勢の比較を開始しました '
                                           f'(sim time {path.anchor_ns / 1e9:.9f} s)')
            except ValueError as exc:
                node.get_logger().warning(str(exc), throttle_duration_sec=5.0)

        warned = False

        def on_clock(msg):
            nonlocal warned
            stamp = stamp_ns(msg.clock)
            if path.set_clock(stamp):
                warned = False
                if visualize:
                    publish(stamp, [])
                    publish_error(stamp)
            if not warned and path.anchor_ns is None and stamp >= trace.stamps[-1]:
                node.get_logger().warning('Gazebo 軌跡の末尾に到達しましたが、走行区間内の '
                                          'map → base_link 姿勢がまだ届いていません')
                warned = True

        def on_timer():
            if visualize:
                update = path.update()
                if update is not None:
                    publish(*update)
                error = path.take_error()
                if error is not None:
                    publish_error(*error)
            if result is not None:
                result.checkpoint()

        # RViz 用の長い Path の送信中も小さい姿勢メッセージをキューに残す。
        node.create_subscription(Odometry, SLAM_ODOMETRY, on_odometry,
                                 QoSProfile(depth=256, reliability=ReliabilityPolicy.BEST_EFFORT))
        loop_graph = None
        if result is not None and not orb:
            def on_closure_cloud(msg):
                if msg.header.frame_id != 'map':
                    return
                first = result.record_closure_cloud(stamp_ns(msg.header.stamp), msg.width * msg.height)
                if first is not None:
                    station = (f"{first['lap']} 周目 {first['station_m']:.1f} m" if first['lap'] is not None
                               else f"累積 {first['distance_m']:.1f} m")
                    node.get_logger().info(f"ループ閉じ込み用点群を観測: {station}、{first['point_count']} 点")

            node.create_subscription(PointCloud2, LOOP_CLOSURE_CLOUD, on_closure_cloud,
                                     QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT))
            if len(result.progress.boundaries) >= 3:
                from .replay_loop_graph import ReplayLoopGraph
                loop_graph = ReplayLoopGraph(node, result.progress,
                                            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        def inspect_loop_graph():
            if loop_graph is not None:
                try:
                    result.summary['loop_closure_graph'] = loop_graph.finish(executor.spin_once)
                # optional な診断でサービスや middleware が失敗しても、収集済みの比較結果を確定する。
                except Exception as exc:
                    result.summary['loop_closure_graph'] = dict(status='unavailable', reason=str(exc))

        node.create_subscription(Clock, '/clock', on_clock, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        # 一時停止や EOF 後にも、最後に受信した時計までの描画を反映する。
        node.create_timer(0.2, on_timer, clock=WallClock(clock_type=ClockType.STEADY_TIME))
        executor.add_node(node)
        yield ReplayObserver(executor.spin_once, inspect_loop_graph)
    finally:
        executor.shutdown()
        node.destroy_node()


def _run(trace):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=[])
    try:
        with ground_truth_viewer(trace) as observer:
            while rclpy.ok():
                observer.spin_once(timeout_sec=0.2)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', help='この bag と同じ Gazebo 試行の rig_motion.jsonl')
    args = parser.parse_args(argv)
    try:
        _run(Trace.from_file(args.trace))
    except (OSError, ValueError) as exc:
        print(f'Gazebo 正解軌跡: {exc}', file=sys.stderr)
        return 1
    return 0

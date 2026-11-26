"""固定地図リプレイの画像入力、cuVSLAM 処理完了、再定位の時系列を保存する。"""
import csv
from dataclasses import astuple, dataclass, fields
from itertools import pairwise
import json
import math
import re
import time

from atd_mapping.frames import stamp_ns
from .replay_inputs import stereo_pairs


@dataclass
class Event:
    stream: str
    received_ns: int  # 同じ観測プロセスの monotonic clock。ROS の画像時刻とは別の時計。
    stamp_ns: int | None = None  # 入出力メッセージの画像時刻。ROS ログには画像時刻がない。
    detail: str = ''
    callback_s: float | None = None
    track_s: float | None = None
    warning_ms: float | None = None
    source_ns: int | None = None  # DDS writer の時刻。未対応の RMW では空欄。
    publication_sequence: int | None = None


def _duration(value):
    return value if math.isfinite(value) and value >= 0 else None


def print_frame_summary(summary):
    print(f"bag {summary['recorded_pairs']} 対、型付き入力の発行通知 {summary['publication_pairs_observed']} 対、"
          f"cuVSLAM の処理完了を観測 {summary['processed_frames']} 時刻")
    gap = summary['largest_processed_gap']
    if gap is None:
        print('処理完了ステータスが2時刻未満のため区間を確定できません。', flush=True)
        return

    def ms(value):
        return '未計測' if value is None else f'{value * 1000:.1f} ms'

    print(f"処理画像の最大間隔 {gap['duration_s']:.4f} 秒、その間に観測した左右の発行通知 "
          f"{gap['publication_pairs_observed_inside']} / {gap['recorded_pairs_inside']} 対")
    print(f"同区間での発行通知の最大間隔　通知の DDS 送信 {ms(gap['largest_notice_send_gap_s'])}、"
          f"診断ノードへの到着 {ms(gap['largest_notice_arrival_gap_s'])}")
    print(f"区間前後の cuVSLAM コールバック {ms(gap['before_callback_s'])} / {ms(gap['after_callback_s'])}、"
          f"うち VO 計算 {ms(gap['before_track_s'])} / {ms(gap['after_track_s'])}", flush=True)


def _localization_windows(events):
    # 両端を観測した試行のみを完了区間とする。開始/結果の取りこぼしも記録に残す。
    windows = []
    for event in events:
        if event.stream == 'localize_start':
            windows.append(dict(start_received_ns=event.received_ns, end_received_ns=None, result=None))
        elif event.stream in ('localize_success', 'localize_failure'):
            if not windows or windows[-1]['end_received_ns'] is not None:
                windows.append(dict(start_received_ns=None, end_received_ns=None, result=None))
            windows[-1].update(end_received_ns=event.received_ns, result=event.stream)
    return windows


class FrameTrace:
    def __init__(self):
        self.events = []
        self.notices = {'left': 0, 'right': 0}
        self.last_processed_stamp = None

    def observed_tail(self, stereo):
        return (all(self.notices[side] == stereo[side + '_images'] for side in self.notices)
                and self.last_processed_stamp is not None
                and self.last_processed_stamp >= stereo['last_stamp_ns'] - 5_000_000)

    def subscribe(self, node, sensor_source, *, vgl_corrections=False):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        from nav_msgs.msg import Odometry
        from rcl_interfaces.msg import Log
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Header

        # 画像を別プロセスで購読すると転送経路自体が変わる。各 publish 直後の小さな通知だけを観測する。
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)

        def header_callback(stream):
            def collect(message):
                self.events.append(Event(stream, time.monotonic_ns(), stamp_ns(message.header.stamp)))
            return collect

        def publication_callback(side):
            def collect(header, info):
                self.notices[side] += 1
                source = info.get('source_timestamp') or 0
                self.events.append(Event(side + '_published', time.monotonic_ns(), stamp_ns(header.stamp),
                                         source_ns=source if source > 0 else None,
                                         publication_sequence=info.get('publication_sequence_number')))
            return collect

        for side in ('left', 'right'):
            node.create_subscription(Header, sensor_source + '/' + side + '_published', publication_callback(side), 256)
        node.create_subscription(Odometry, '/visual_slam/tracking/odometry', header_callback('odometry'), qos)
        node.create_subscription(PoseWithCovarianceStamped, '/localization/pose_hint', header_callback('hint'), qos)
        node.create_subscription(VisualSlamStatus, '/visual_slam/status', self._status, qos)
        node.create_subscription(Log, '/rosout', self._log, qos)
        if vgl_corrections:
            from tf2_msgs.msg import TFMessage
            node.create_subscription(TFMessage, '/tf', self._map_correction, qos)

    def _map_correction(self, message):
        for tf in message.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                self.events.append(Event('map_to_odom', time.monotonic_ns(), stamp_ns(tf.header.stamp)))

    def _status(self, message):
        stamp = stamp_ns(message.header.stamp)
        self.last_processed_stamp = stamp
        self.events.append(Event('processed', time.monotonic_ns(), stamp,
                                 str(message.vo_state), _duration(message.node_callback_execution_time),
                                 _duration(message.track_execution_time)))

    def _log(self, message):
        if message.name.split('.')[-1] != 'visual_slam':
            return
        text = message.msg
        if text.startswith('Trying to localize in map'):
            stream = 'localize_start'
        elif text.startswith('Successfully localized at'):
            stream = 'localize_success'
        elif text.startswith('Failed to localize in map'):
            stream = 'localize_failure'
        elif match := re.match(r'Delta between current and previous frame \[([\d.]+) ms\]', text):
            self.events.append(Event('frame_warning', time.monotonic_ns(), detail=text,
                                     warning_ms=float(match.group(1))))
            return
        else:
            return
        self.events.append(Event(stream, time.monotonic_ns(), detail=text))

    def summary(self, recorded_stamps):
        left, right = recorded_stamps['left'], recorded_stamps['right']
        notices = {'left_published': {}, 'right_published': {}}
        processed = {}
        for event in self.events:
            if event.stream in notices:
                notices[event.stream].setdefault(event.stamp_ns, event)
            elif event.stream == 'processed':
                processed.setdefault(event.stamp_ns, event)
        # bag の preflight と同じ左右の組に対して、元の画像時刻を持つ発行通知を突き合わせる。
        pairs = {}
        sent_pairs = {}
        for l_stamp, r_stamp in stereo_pairs(left, right):
            received = None
            stamp = max(l_stamp, r_stamp)
            if l_stamp in notices['left_published'] and r_stamp in notices['right_published']:
                l_event, r_event = notices['left_published'][l_stamp], notices['right_published'][r_stamp]
                received = max(l_event.received_ns, r_event.received_ns)
                if l_event.source_ns is not None and r_event.source_ns is not None:
                    sent_pairs[stamp] = max(l_event.source_ns, r_event.source_ns)
            pairs[stamp] = received
        # 受信順と画像時刻順は別々に評価する。複数 topic 間の DDS 配送順は保証されない。
        frames = sorted(processed.values(), key=lambda e: e.stamp_ns)
        windows = _localization_windows(self.events)
        gap = None
        if len(frames) >= 2:
            before, after = max(pairwise(frames), key=lambda ab: ab[1].stamp_ns - ab[0].stamp_ns)
            start, end = before.stamp_ns, after.stamp_ns
            interior = [received for stamp, received in pairs.items() if start < stamp < end]
            stamps = [stamp for stamp in pairs if start <= stamp <= end]
            arrivals = sorted(pairs[stamp] for stamp in stamps if pairs[stamp] is not None)
            sent = [sent_pairs[stamp] for stamp in stamps if stamp in sent_pairs]
            # 送信時刻は DDS が提供し、区間内の全ペアで取得でき、時計が逆行しない場合に使う。
            send_gap = None
            if len(sent) >= 2 and len(sent) == len(stamps) and all(a <= b for a, b in pairwise(sent)):
                send_gap = max((b - a) / 1e9 for a, b in pairwise(sent))
            # 同じ画像区間に対応する発行通知の到着時間と照合する。
            overlapping = [
                index for index, window in enumerate(windows)
                if len(arrivals) >= 2 and window['start_received_ns'] is not None
                and window['end_received_ns'] is not None and window['start_received_ns'] <= arrivals[-1]
                and window['end_received_ns'] >= arrivals[0]]
            gap = dict(start_stamp_ns=start, end_stamp_ns=end, duration_s=(end - start) / 1e9,
                       start_received_ns=before.received_ns, end_received_ns=after.received_ns,
                       before_callback_s=before.callback_s, after_callback_s=after.callback_s,
                       before_track_s=before.track_s, after_track_s=after.track_s,
                       recorded_pairs_inside=len(interior),
                       publication_pairs_observed_inside=sum(t is not None for t in interior),
                       largest_recorded_pair_gap_s=max(((b - a) / 1e9 for a, b in pairwise(stamps)), default=None),
                       largest_notice_send_gap_s=send_gap,
                       largest_notice_arrival_gap_s=max(((b - a) / 1e9 for a, b in pairwise(arrivals)), default=None),
                       first_notice_received_ns=arrivals[0] if arrivals else None,
                       last_notice_received_ns=arrivals[-1] if arrivals else None,
                       hint_stamps_inside=[e.stamp_ns for e in self.events
                                           if e.stream == 'hint' and start < e.stamp_ns < end],
                       overlapping_complete_localizations=overlapping if len(arrivals) >= 2 else None)
        return dict(schema='atd.cuvslam.frame_trace.v2', timeline='frame_trace.csv',
                    recorded_pairs=len(pairs),
                    publication_pairs_observed=sum(t is not None for t in pairs.values()),
                    recorded_left_images=len(left), recorded_right_images=len(right),
                    left_publication_notices=len(notices['left_published']), right_publication_notices=len(notices['right_published']),
                    processed_frames=len(frames),
                    odometry_frames=len({e.stamp_ns for e in self.events if e.stream == 'odometry'}),
                    first_processed_stamp_ns=frames[0].stamp_ns if frames else None,
                    last_processed_stamp_ns=frames[-1].stamp_ns if frames else None,
                    max_callback_s=max((e.callback_s for e in frames if e.callback_s is not None), default=None),
                    max_track_s=max((e.track_s for e in frames if e.track_s is not None), default=None),
                    largest_warning_ms=max((e.warning_ms for e in self.events if e.warning_ms is not None), default=None),
                    localization_windows=windows, largest_processed_gap=gap)

    def write(self, output, recorded_stamps):
        with (output / 'frame_trace.csv').open('x', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(f.name for f in fields(Event))
            writer.writerows(astuple(e) for e in self.events)
        summary = self.summary(recorded_stamps)
        (output / 'frame_trace.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
        return summary

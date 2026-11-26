"""固定地図リプレイの画像入力、cuVSLAM 処理完了、再定位の時系列を保存する。"""
import csv
from dataclasses import astuple, dataclass, fields
from itertools import pairwise
import json
import math
import re
import struct
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
    print(f"bag {summary['recorded_pairs']} 対、独立購読で受信 {summary['received_pairs']} 対、"
          f"cuVSLAM の処理完了を観測 {summary['processed_frames']} 時刻")
    gap = summary['largest_processed_gap']
    if gap is None:
        print('処理完了ステータスが2時刻未満のため区間を確定できません。', flush=True)
        return

    def ms(value):
        return '未計測' if value is None else f'{value * 1000:.1f} ms'

    print(f"処理画像の最大間隔 {gap['duration_s']:.4f} 秒、その間に受信した入力 "
          f"{gap['received_pairs_inside']} / {gap['recorded_pairs_inside']} 対")
    print(f"同区間での最大間隔　DDS 送信 {ms(gap['largest_input_publish_gap_s'])}、"
          f"独立購読での到着 {ms(gap['largest_input_receipt_gap_s'])}")
    print(f"区間前後の cuVSLAM コールバック {ms(gap['before_callback_s'])} / {ms(gap['after_callback_s'])}、"
          f"うち VO 計算 {ms(gap['before_track_s'])} / {ms(gap['after_track_s'])}", flush=True)


def _image_stamp(raw):
    # sensor_msgs/Image は Header で始まる。plain CDR の encapsulation に続く
    # int32 sec, uint32 nanosec だけを読む。画像を Python の配列に展開しない。
    representation = raw[:2]
    if len(raw) >= 12 and representation in (b'\x00\x00', b'\x00\x01'):
        sec, ns = struct.unpack_from('>iI' if representation == b'\x00\x00' else '<iI', raw, 4)
        return sec * 1_000_000_000 + ns
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    return stamp_ns(deserialize_message(raw, Image).header.stamp)


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

    def subscribe(self, node, workflow):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        from nav_msgs.msg import Odometry
        from rcl_interfaces.msg import Log
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image

        # best effort で独立に観測し、画像 payload は保持しない。観測自体の欠落は bag と照合する。
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)

        def header_callback(stream):
            def collect(message):
                self.events.append(Event(stream, time.monotonic_ns(), stamp_ns(message.header.stamp)))
            return collect

        def image_callback(side):
            def collect(raw, info):
                source = info.get('source_timestamp') or 0
                self.events.append(Event(side, time.monotonic_ns(), _image_stamp(raw),
                                         source_ns=source if source > 0 else None,
                                         publication_sequence=info.get('publication_sequence_number')))
            return collect

        for side in ('left', 'right'):
            node.create_subscription(Image, workflow[side + '_image'], image_callback(side), qos, raw=True)
        node.create_subscription(Odometry, '/visual_slam/tracking/odometry', header_callback('odometry'), qos)
        node.create_subscription(PoseWithCovarianceStamped, '/localization/pose_hint', header_callback('hint'), qos)
        node.create_subscription(VisualSlamStatus, '/visual_slam/status', self._status, qos)
        node.create_subscription(Log, '/rosout', self._log, qos)

    def _status(self, message):
        self.events.append(Event('processed', time.monotonic_ns(), stamp_ns(message.header.stamp),
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
        receipts = {'left': {}, 'right': {}}
        processed = {}
        for event in self.events:
            if event.stream in receipts:
                receipts[event.stream].setdefault(event.stamp_ns, event)
            elif event.stream == 'processed':
                processed.setdefault(event.stamp_ns, event)
        # 対応画像を探す基準は bag の preflight と同じ 5ms。受信側で別画像に組み替えない。
        pairs = {}
        sent_pairs = {}
        for l_stamp, r_stamp in stereo_pairs(left, right):
            received = None
            stamp = max(l_stamp, r_stamp)
            if l_stamp in receipts['left'] and r_stamp in receipts['right']:
                l_event, r_event = receipts['left'][l_stamp], receipts['right'][r_stamp]
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
            # 画像の同じ区間が観測側へ流れていた時間と照合する。cuVSLAM 内部の待機時間ではない。
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
                       received_pairs_inside=sum(t is not None for t in interior),
                       largest_recorded_pair_gap_s=max(((b - a) / 1e9 for a, b in pairwise(stamps)), default=None),
                       largest_input_publish_gap_s=send_gap,
                       largest_input_receipt_gap_s=max(((b - a) / 1e9 for a, b in pairwise(arrivals)), default=None),
                       first_input_received_ns=arrivals[0] if arrivals else None,
                       last_input_received_ns=arrivals[-1] if arrivals else None,
                       hint_stamps_inside=[e.stamp_ns for e in self.events
                                           if e.stream == 'hint' and start < e.stamp_ns < end],
                       overlapping_complete_localizations=overlapping if len(arrivals) >= 2 else None)
        return dict(schema='atd.cuvslam.frame_trace.v1', timeline='frame_trace.csv',
                    recorded_pairs=len(pairs),
                    received_pairs=sum(t is not None for t in pairs.values()),
                    recorded_left_images=len(left), recorded_right_images=len(right),
                    received_left_images=len(receipts['left']), received_right_images=len(receipts['right']),
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

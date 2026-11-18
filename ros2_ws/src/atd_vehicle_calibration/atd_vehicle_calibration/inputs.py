"""Bag adapter and strict recorded-frame validation. ROS imports stay at the edge."""
from __future__ import annotations

from bisect import bisect_right
import math
from numbers import Integral, Real
from pathlib import Path
import re


PICO_TOPIC = '/vehicle/pico/telemetry'
IMU_TOPICS = ('/camera/camera/accel/sample', '/camera/camera/gyro/sample')
CAMERA_FRAMES = ('camera_infra1_optical_frame', 'camera_infra2_optical_frame')
CAMERA_INFO_TOPICS = ('/camera/camera/infra1/camera_info', '/camera/camera/infra2/camera_info')
PICO_TYPE = 'atd_interfaces/msg/PicoTelemetry'
IMU_TYPE = 'sensor_msgs/msg/Imu'
TF_TYPE = 'tf2_msgs/msg/TFMessage'


def field(value, key):
    return value[key] if isinstance(value, dict) else getattr(value, key)


def finite_vector(value, length=3):
    result = list(value)
    if len(result) != length or any(isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v) for v in result):
        raise ValueError('non-finite or malformed vector')
    return [float(v) for v in result]


def quaternion(value):
    result = finite_vector(value, 4)
    norm = math.sqrt(sum(v*v for v in result))
    if abs(norm - 1) > .01:
        raise ValueError('quaternion must have unit norm')
    return [v / norm for v in result]


def multiply(a, b):
    x, y, z, w = a
    i, j, k, r = b
    return [w*i+x*r+y*k-z*j, w*j-x*k+y*r+z*i,
            w*k+x*j-y*i+z*r, w*r-x*i-y*j-z*k]


def conjugate(q):
    return [-q[0], -q[1], -q[2], q[3]]


def rotate(vector, q):
    return multiply(multiply(q, [*vector, 0]), conjugate(q))[:3]


def _frame(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9/]*', value) or '//' in value or value.endswith('/'):
        raise ValueError('explicit canonical frame ID required')
    return value


def _xyz(value):
    return finite_vector([field(value, key) for key in ('x', 'y', 'z')])


def _quat(value):
    return quaternion([field(value, key) for key in ('x', 'y', 'z', 'w')])


def stamp_seconds(stamp):
    sec, nsec = field(stamp, 'sec'), field(stamp, 'nanosec')
    if (isinstance(sec, bool) or isinstance(nsec, bool) or not isinstance(sec, Integral)
            or not isinstance(nsec, Integral) or sec < 0 or not 0 <= nsec < 1_000_000_000):
        raise ValueError('invalid ROS stamp')
    result = int(sec) + int(nsec) * 1e-9
    if result <= 0:
        raise ValueError('zero ROS stamp')
    return result


class StaticTransformGraph:
    """One immutable, directed TF tree; transform(A,B) gives T_A_B."""
    def __init__(self, transforms=()):
        self.parents = {}
        for record in transforms:
            self.add(record)

    def add(self, record):
        parent, child = _frame(record['parent']), _frame(record['child'])
        if parent == child or parent in ('map', 'odom') or child in ('map', 'odom'):
            raise ValueError('static TF must not contain self/map/odom edges')
        t, q = finite_vector(record['translation']), quaternion(record['rotation'])
        previous = self.parents.get(child)
        if previous:
            same_q = min(sum((a-b)**2 for a, b in zip(previous[2], q)),
                         sum((a+b)**2 for a, b in zip(previous[2], q))) < 1e-16
            if previous[0] != parent or max(abs(a-b) for a, b in zip(previous[1], t)) > 1e-8 or not same_q:
                raise ValueError('conflicting recorded static TF for ' + child)
            return
        cursor = parent
        while cursor in self.parents:
            cursor = self.parents[cursor][0]
            if cursor == child:
                raise ValueError('cycle in recorded static TF')
        self.parents[child] = (parent, t, q)

    def transform(self, parent, child):
        parent, child = _frame(parent), _frame(child)
        t, q = [0., 0., 0.], [0., 0., 0., 1.]
        cursor = child
        while cursor != parent:
            if cursor not in self.parents:
                raise ValueError(f'missing directed static TF {parent} -> {child}')
            ancestor, local_t, local_q = self.parents[cursor]
            t = [a+b for a, b in zip(local_t, rotate(t, local_q))]
            q = multiply(local_q, q)
            cursor = ancestor
        return {'parent': parent, 'child': child, 'translation': t, 'rotation': quaternion(q)}

    def records(self):
        return [{'parent': p, 'child': child, 'translation': t, 'rotation': q}
                for child, (p, t, q) in sorted(self.parents.items())]


def _integer(value, maximum):
    if isinstance(value, bool) or not isinstance(value, Integral) or not 0 <= value <= maximum:
        raise ValueError('integer field outside wire range')
    return int(value)


def _boolean(value):
    if type(value) is not bool:
        raise ValueError('expected boolean field')
    return value


def _pico(message, bag_stamp):
    header = field(message, 'header')
    stamp = stamp_seconds(field(header, 'stamp'))
    if field(header, 'frame_id') != 'pico':
        raise ValueError('unexpected Pico telemetry frame')
    instance, serial = field(message, 'instance_id'), field(message, 'serial')
    if not isinstance(instance, str) or not instance or not isinstance(serial, str) or not serial:
        raise ValueError('missing Pico session identity')
    channels = list(field(message, 'channels'))
    if len(channels) != 16:
        raise ValueError('Pico telemetry requires 16 raw channels')
    channels = [_integer(v, 2047) for v in channels]
    result = dict(stamp=stamp, bag_stamp=bag_stamp, instance_id=instance, serial=serial,
                  sequence_number=_integer(field(message, 'sequence_number'), 0xffffffff),
                  board_us=_integer(field(message, 'board_us'), 0xffffffffffffffff), channels=channels,
                  steering_raw=channels[0], throttle_raw=channels[1],
                  active_path=_integer(field(message, 'active_path'), 3),
                  sbus_flags=_integer(field(message, 'sbus_flags'), 15),
                  sbus_age_ms=_integer(field(message, 'sbus_age_ms'), 0xffffffff),
                  rc_valid=_boolean(field(message, 'rc_valid')),
                  power_ready=_boolean(field(message, 'power_ready')),
                  sbus_fresh=_boolean(field(message, 'sbus_fresh')),
                  fault_bits=_integer(field(message, 'fault_bits'), 0xffffffff))
    result['manual_valid'] = (result['rc_valid'] and result['active_path'] == 1
                              and result['power_ready'] and result['sbus_fresh']
                              and not result['sbus_flags'] & 12 and result['sbus_age_ms'] <= 50
                              and result['fault_bits'] == 0)
    result['reason'] = 'manual' if result['manual_valid'] else 'not_fresh_valid_pico_manual'
    return result


def _rotate_covariance(covariance, rotation):
    values = finite_vector(covariance, 9)
    if values[0] < 0:
        raise ValueError('IMU measurement unavailable')
    columns = [rotate([float(i == j) for i in range(3)], rotation) for j in range(3)]
    matrix = [[columns[j][i] for j in range(3)] for i in range(3)]
    return [sum(matrix[i][k] * values[k*3+l] * matrix[j][l] for k in range(3) for l in range(3))
            for i in range(3) for j in range(3)]


def decode_records(records, *, pico_topic=PICO_TOPIC, imu_topics=IMU_TOPICS,
                   expected_base_frame='base_link', expected_camera_frame='camera_link',
                   camera_frames=CAMERA_FRAMES, camera_info_topics=CAMERA_INFO_TOPICS, max_gap_s=.25,
                   imu_records=None, include_imu_covariance=False):
    """Decode {topic,type,message,bag_stamp_ns}; source header time is never replaced."""
    if expected_base_frame != 'base_link' or max_gap_s <= 0 or not math.isfinite(max_gap_s):
        raise ValueError('base_link and a finite positive continuity gap are required')
    graph = StaticTransformGraph()
    if len(camera_frames) != len(camera_info_topics) or len(set(camera_frames)) != len(camera_frames):
        raise ValueError('explicit distinct stereo frames and CameraInfo topics are required')
    sensor_records, rejected, observed_camera_frames = [], [], {}
    coverage = {topic: dict(received=0, accepted=0, rejected=0, first_stamp=None, last_stamp=None, frames=[])
                for topic in imu_topics}
    for index, record in enumerate(records):
        topic, kind, message = record['topic'], record['type'], record['message']
        if topic == '/tf_static':
            if kind != TF_TYPE:
                raise ValueError('wrong /tf_static message type')
            for tf in field(message, 'transforms'):
                transform = field(tf, 'transform')
                graph.add(dict(parent=field(field(tf, 'header'), 'frame_id'), child=field(tf, 'child_frame_id'),
                               translation=_xyz(field(transform, 'translation')), rotation=_quat(field(transform, 'rotation'))))
        elif topic == pico_topic or topic in imu_topics:
            expected_type = PICO_TYPE if topic == pico_topic else IMU_TYPE
            if kind != expected_type:
                raise ValueError('wrong sensor type for ' + topic)
            sensor_records.append((index, record))
            if topic in coverage:
                coverage[topic]['received'] += 1
        elif topic in camera_info_topics:
            if kind != 'sensor_msgs/msg/CameraInfo':
                raise ValueError('wrong CameraInfo type')
            frame = _frame(field(field(message, 'header'), 'frame_id'))
            expected = camera_frames[camera_info_topics.index(topic)]
            if frame != expected:
                raise ValueError('recorded CameraInfo frame differs from explicit stereo frame: ' + topic)
            stamp_seconds(field(field(message, 'header'), 'stamp'))
            observed_camera_frames[topic] = frame
    # A body-frame TUM does not excuse missing measured body/camera connectivity.
    graph.transform(expected_base_frame, expected_camera_frame)
    for frame in camera_frames:
        graph.transform(expected_base_frame, frame)
    if set(observed_camera_frames) != set(camera_info_topics):
        raise ValueError('missing recorded stereo CameraInfo/frame evidence')
    pico, imu = [], []
    segment, previous, seen_stamps = 0, None, set()
    for index, record in sensor_records:
        if record['topic'] != pico_topic:
            continue
        try:
            bag_stamp = _integer(record['bag_stamp_ns'], 0x7fffffffffffffff) * 1e-9
            sample = _pico(record['message'], bag_stamp)
            baseline = previous is None
            if sample['stamp'] in seen_stamps:
                raise ValueError('duplicate Pico ROS stamp')
            seen_stamps.add(sample['stamp'])
            if previous:
                dt = sample['stamp'] - previous['stamp']
                if dt <= 0:
                    raise ValueError('non-monotonic Pico ROS stamp')
                session_changed = any(sample[key] != previous[key] for key in ('instance_id', 'serial'))
                if session_changed or dt > max_gap_s or not previous['manual_valid'] or not sample['manual_valid']:
                    segment += 1
                baseline = session_changed or dt > max_gap_s
                if not session_changed:
                    advance = (sample['sequence_number'] - previous['sequence_number']) & 0xffffffff
                    if not 0 < advance < 0x80000000:
                        raise ValueError('non-advancing Pico sequence')
                    if sample['board_us'] <= previous['board_us']:
                        raise ValueError('Pico board clock reset without a new instance_id')
                    # Compare clock *increments*, never equate board uptime with ROS epoch.
                    board_dt = (sample['board_us'] - previous['board_us']) * 1e-6
                    if abs(board_dt-dt) > max_gap_s:
                        raise ValueError('Pico/ROS clock increment disagreement')
                    if advance != 1:
                        segment += 1
                        baseline = True
            if baseline and sample['manual_valid']:
                sample['manual_valid'] = False
                sample['reason'] = 'continuity_baseline'
            sample['segment'] = segment
            pico.append(sample)
            previous = sample
        except (ValueError, KeyError, AttributeError, TypeError) as error:
            rejected.append(dict(source=record['topic'], index=index, reason=str(error)))
            segment += 1
            previous = None
    if not pico:
        raise ValueError('no validly decoded Pico telemetry')
    stamps = [sample['stamp'] for sample in pico]
    # A ROS clock rollback prevents unambiguous cross-stream association.
    if any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError('Pico ROS clock rollback; split the bag before analysis')
    imu_previous, frame_cache = {}, {}
    if imu_records is None:
        imu_records = (record for _, record in sensor_records if record['topic'] in imu_topics)
        count_imu = False
    else:
        count_imu = True
        # The real bag adapter supplies a second filtered pass. Release full
        # Pico ROS messages before visiting the substantially faster IMU streams.
        sensor_records.clear()
    for index, record in enumerate(imu_records):
        topic = record['topic']
        try:
            if topic not in imu_topics or record['type'] != IMU_TYPE:
                raise ValueError('wrong IMU topic/type in second pass')
            if count_imu:
                coverage[topic]['received'] += 1
            message = record['message']
            stamp = stamp_seconds(field(field(message, 'header'), 'stamp'))
            frame = _frame(field(field(message, 'header'), 'frame_id'))
            if frame not in frame_cache:
                frame_cache[frame] = graph.transform(expected_base_frame, frame)
            transform = frame_cache[frame]
            if stamp <= imu_previous.get(topic, -math.inf):
                raise ValueError('duplicate/non-monotonic IMU ROS stamp')
            imu_previous[topic] = stamp
            bag_stamp = _integer(record['bag_stamp_ns'], 0x7fffffffffffffff) * 1e-9
            # Only explicitly named raw streams (or the explicit combined /imu topic).
            kinds = ('accel', 'gyro') if topic.endswith('/imu') else ('accel',) if '/accel/' in topic else ('gyro',) if '/gyro/' in topic else ()
            if not kinds:
                raise ValueError('cannot infer IMU measurement kind from explicit topic')
            position = bisect_right(stamps, stamp)-1
            session = pico[position]['segment'] if position >= 0 else None
            for kind in kinds:
                key = 'linear_acceleration' if kind == 'accel' else 'angular_velocity'
                vector = _xyz(field(message, key))
                covariance = finite_vector(field(message, key+'_covariance'), 9)
                if covariance[0] < 0:
                    raise ValueError('IMU measurement unavailable')
                row = dict(stamp=stamp, bag_stamp=bag_stamp, topic=topic, frame=frame, kind=kind,
                           vector_body=rotate(vector, transform['rotation']),
                           segment=session, translation_body_m=transform['translation'])
                if include_imu_covariance:
                    row['covariance_body'] = _rotate_covariance(covariance, transform['rotation'])
                imu.append(row)
            state = coverage[topic]
            state['accepted'] += 1
            if state['first_stamp'] is None:
                state['first_stamp'] = stamp
            state['last_stamp'] = stamp
            if frame not in state['frames']:
                state['frames'].append(frame)
        except (ValueError, KeyError, AttributeError, TypeError) as error:
            rejected.append(dict(source=topic, index=index, reason=str(error)))
            if topic in coverage:
                coverage[topic]['rejected'] += 1
    return dict(pico=pico, imu=imu, static_transforms=graph.records(), rejected=rejected,
                provenance=dict(clock='ROS source header seconds; Pico board_us is an independent uptime',
                                base_frame=expected_base_frame, camera_frame=expected_camera_frame,
                                camera_frames=list(camera_frames), max_gap_s=max_gap_s,
                                observed_camera_frames=observed_camera_frames,
                                imu_coverage=coverage,
                                imu_covariance_retained=include_imu_covariance,
                                imu_acceleration='raw specific force; gravity and sensor lever arm not corrected',
                                pico_input='raw CH1 steering / CH2 throttle; no actual PWM or angle measurement'))


def read_bag(bag_path, **options):
    """Read rosbag2 using installed ROS types; importing this module needs no ROS."""
    import yaml
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    path = Path(bag_path).expanduser().resolve(strict=True)
    document = yaml.safe_load((path / 'metadata.yaml').read_text())
    metadata = document.get('rosbag2_bagfile_information') if isinstance(document, dict) else None
    if not isinstance(metadata, dict) or not isinstance(metadata.get('storage_identifier'), str):
        raise ValueError('invalid rosbag2 metadata')
    topics = metadata.get('topics_with_message_count')
    if not isinstance(topics, list):
        raise ValueError('invalid topic metadata')
    names = {}
    for entry in topics:
        info = entry.get('topic_metadata') if isinstance(entry, dict) else None
        if not isinstance(info, dict) or not isinstance(info.get('name'), str) or not isinstance(info.get('type'), str):
            raise ValueError('invalid topic metadata entry')
        if info['name'] in names:
            raise ValueError('duplicate topic metadata')
        names[info['name']] = info['type']
    pico_topic = options.get('pico_topic', PICO_TOPIC)
    imu_topics = options.get('imu_topics', IMU_TOPICS)
    info_topics = options.get('camera_info_topics', CAMERA_INFO_TOPICS)
    required = {pico_topic: PICO_TYPE, '/tf_static': TF_TYPE, **{topic: IMU_TYPE for topic in imu_topics},
                **{topic: 'sensor_msgs/msg/CameraInfo' for topic in info_topics}}
    for topic, kind in required.items():
        if names.get(topic) != kind:
            raise ValueError(f'missing/wrong required topic {topic}: {kind}')
        count = next(entry.get('message_count') for entry in topics if entry['topic_metadata']['name'] == topic)
        if isinstance(count, bool) or not isinstance(count, Integral) or count < 1:
            raise ValueError('empty/invalid required topic count: ' + topic)
    classes = {name: get_message(kind) for name, kind in required.items()}
    def records(selected):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=metadata['storage_identifier']),
                    rosbag2_py.ConverterOptions('', ''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(selected)))
        while reader.has_next():
            topic, raw, stamp = reader.read_next()
            yield dict(topic=topic, type=required[topic], message=deserialize_message(raw, classes[topic]), bag_stamp_ns=stamp)
    first_pass = [topic for topic in required if topic not in imu_topics]
    result = decode_records(records(first_pass), imu_records=records(imu_topics), **options)
    if (set(sample['kind'] for sample in result['imu']) != {'accel', 'gyro'}
            or any(state['accepted'] == 0 for state in result['provenance']['imu_coverage'].values())):
        raise ValueError('no usable raw accelerometer/gyroscope samples after frame validation')
    result.update(metadata=metadata, bag_path=str(path))
    return result

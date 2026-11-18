import copy
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

from atd_vehicle_calibration.inputs import (
    CAMERA_FRAMES, CAMERA_INFO_TOPICS, IMU_TOPICS, IMU_TYPE, PICO_TOPIC, PICO_TYPE,
    TF_TYPE, StaticTransformGraph, decode_records, finite_vector, read_bag,
)


def stamp(value):
    nanoseconds = round(value * 1e9)
    return dict(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def record(topic, kind, message, time=100.):
    return dict(topic=topic, type=kind, message=message, bag_stamp_ns=round(time*1e9))


def transform(parent, child, translation=(0, 0, 0), rotation=(0, 0, 0, 1)):
    return dict(header=dict(frame_id=parent, stamp=stamp(100)), child_frame_id=child,
                transform=dict(translation=dict(zip(('x', 'y', 'z'), translation)),
                               rotation=dict(zip(('x', 'y', 'z', 'w'), rotation))))


def pico(time=100., sequence=1, board=1000000, instance='pico:1', **changes):
    value = dict(header=dict(stamp=stamp(time), frame_id='pico'), instance_id=instance,
                 serial='serial1', sequence_number=sequence, board_us=board,
                 channels=[992]*16, active_path=1, sbus_flags=0, sbus_age_ms=2,
                 rc_valid=True, power_ready=True, sbus_fresh=True, fault_bits=0)
    value.update(changes)
    return record(PICO_TOPIC, PICO_TYPE, value, time)


def imu(topic=IMU_TOPICS[0], time=100.02, frame='camera_accel_optical_frame', vector=(1, 2, 3)):
    measurement = dict(zip(('x', 'y', 'z'), vector))
    return record(topic, IMU_TYPE,
                  dict(header=dict(stamp=stamp(time), frame_id=frame),
                       linear_acceleration=measurement, angular_velocity=measurement,
                       linear_acceleration_covariance=[1., 0, 0, 0, 2., 0, 0, 0, 3.],
                       angular_velocity_covariance=[1., 0, 0, 0, 2., 0, 0, 0, 3.]), time)


def fixture():
    tf = [transform('base_link', 'camera_link', (.2, 0, .1)),
          *[transform('camera_link', frame) for frame in CAMERA_FRAMES],
          transform('camera_link', 'camera_accel_optical_frame', rotation=(0, 0, 2**-.5, 2**-.5)),
          transform('camera_link', 'camera_gyro_optical_frame')]
    return [record('/tf_static', TF_TYPE, dict(transforms=tf)),
            *[record(topic, 'sensor_msgs/msg/CameraInfo', dict(header=dict(stamp=stamp(100), frame_id=frame)))
              for topic, frame in zip(CAMERA_INFO_TOPICS, CAMERA_FRAMES)],
            pico(), pico(100.02, 2, 1020000), imu(),
            imu(IMU_TOPICS[1], frame='camera_gyro_optical_frame')]


class InputsTest(unittest.TestCase):
    def test_manual_baseline_raw_and_imu_body_rotation(self):
        result = decode_records(fixture(), include_imu_covariance=True)
        self.assertEqual(result['rejected'], [])
        self.assertFalse(result['pico'][0]['manual_valid'])
        self.assertEqual(result['pico'][0]['reason'], 'continuity_baseline')
        self.assertTrue(result['pico'][1]['manual_valid'])
        self.assertEqual(result['pico'][1]['steering_raw'], 992)
        self.assertEqual(result['pico'][1]['throttle_raw'], 992)
        self.assertNotIn('connected', result['pico'][1])
        accel = result['imu'][0]
        for actual, expected in zip(accel['vector_body'], (-2, 1, 3)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(accel['covariance_body'], (2, 0, 0, 0, 1, 0, 0, 0, 3)):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(accel['translation_body_m'], [.2, 0, .1])
        self.assertEqual(accel['segment'], result['pico'][1]['segment'])
        self.assertEqual(result['provenance']['imu_coverage'][IMU_TOPICS[0]]['accepted'], 1)

    def test_reconnect_resets_baseline_even_if_wire_rc_valid(self):
        records = fixture()+[pico(100.04, 1, 100, instance='pico:2'), pico(100.06, 2, 20100, instance='pico:2')]
        result = decode_records(records, include_imu_covariance=True)
        self.assertEqual([r['manual_valid'] for r in result['pico']], [False, True, False, True])
        self.assertNotEqual(result['pico'][1]['segment'], result['pico'][2]['segment'])

    def test_unhealthy_manual_input_never_becomes_valid(self):
        for changes in (dict(rc_valid=False), dict(active_path=2), dict(power_ready=False),
                        dict(sbus_fresh=False), dict(sbus_flags=4), dict(sbus_flags=8),
                        dict(sbus_age_ms=51), dict(fault_bits=1)):
            with self.subTest(changes=changes):
                records = fixture()+[pico(100.04, 3, 1040000, **changes)]
                self.assertFalse(decode_records(records)['pico'][-1]['manual_valid'])

    def test_sequence_clock_and_invalid_stamp_exclusion(self):
        for sample in (pico(100.04, 2, 1040000), pico(100.04, 3, 500),
                       pico(100.04, 3, 3000000), pico(100.02, 3, 1040000)):
            with self.subTest(sample=sample):
                result = decode_records(fixture()+[sample])
                self.assertEqual(len(result['pico']), 2)
                self.assertEqual(len(result['rejected']), 1)
        bad = pico(100.04, 3, 1040000)
        bad['message']['header']['stamp']['nanosec'] = 1_000_000_000
        self.assertEqual(len(decode_records(fixture()+[bad])['rejected']), 1)

    def test_gap_and_lost_frame_create_new_baseline(self):
        for sample in (pico(101., 3, 2000000), pico(100.04, 4, 1040000)):
            result = decode_records(fixture()+[sample])
            self.assertFalse(result['pico'][-1]['manual_valid'])
            self.assertNotEqual(result['pico'][1]['segment'], result['pico'][-1]['segment'])

    def test_sequence_wrap_is_valid_and_ros_clock_rollback_is_not(self):
        records = fixture()
        records[3] = pico(sequence=0xffffffff)
        records[4] = pico(100.02, 0, 1020000)
        self.assertTrue(decode_records(records)['pico'][-1]['manual_valid'])
        records += [pico(99., 1, 500, instance='pico:2'), pico(99.02, 2, 20500, instance='pico:2')]
        with self.assertRaisesRegex(ValueError, 'rollback'):
            decode_records(records)

    def test_imu_bad_value_duplicates_and_coverage(self):
        bad = imu(time=100.04, vector=(float('nan'), 0, 0))
        result = decode_records(fixture()+[imu(), bad])
        self.assertEqual(len(result['imu']), 2)
        coverage = result['provenance']['imu_coverage'][IMU_TOPICS[0]]
        self.assertEqual((coverage['received'], coverage['accepted'], coverage['rejected']), (3, 1, 2))
        absent = decode_records([r for r in fixture() if r['topic'] != IMU_TOPICS[1]])
        self.assertEqual(absent['provenance']['imu_coverage'][IMU_TOPICS[1]]['accepted'], 0)

    def test_static_conflict_cycle_wrong_direction_and_missing_camera(self):
        for transforms in ([dict(parent='base_link', child='camera_link', translation=[0]*3, rotation=[0, 0, 0, 1]),
                            dict(parent='other', child='camera_link', translation=[0]*3, rotation=[0, 0, 0, 1])],
                           [dict(parent='a', child='b', translation=[0]*3, rotation=[0, 0, 0, 1]),
                            dict(parent='b', child='a', translation=[0]*3, rotation=[0, 0, 0, 1])]):
            with self.assertRaises(ValueError):
                StaticTransformGraph(transforms)
        records = fixture()
        records[0]['message']['transforms'][0] = transform('camera_link', 'base_link')
        with self.assertRaisesRegex(ValueError, 'directed'):
            decode_records(records)
        with self.assertRaisesRegex(ValueError, 'CameraInfo'):
            decode_records([r for r in fixture() if r['topic'] not in CAMERA_INFO_TOPICS])
        records = fixture()
        records[1]['message']['header']['frame_id'] = 'wrong_optical_frame'
        with self.assertRaisesRegex(ValueError, 'CameraInfo frame'):
            decode_records(records)

    def test_static_duplicate_sign_and_nonunit_rejection(self):
        edge = dict(parent='a', child='b', translation=[1, 0, 0], rotation=[0, 0, 0, 1])
        graph = StaticTransformGraph([edge, dict(edge, rotation=[0, 0, 0, -1])])
        self.assertEqual(graph.transform('a', 'b')['translation'], [1., 0., 0.])
        for invalid in (dict(edge, rotation=[0, 0, 0, 0]), dict(edge, parent='map')):
            with self.assertRaises(ValueError):
                StaticTransformGraph([invalid])
        with self.assertRaises(ValueError):
            finite_vector([True, 0, 0])

    def test_numpy_generated_fixed_arrays(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest('NumPy unavailable; ROS generated fixed-array representation test')
        records = fixture()
        for entry in records:
            if entry['topic'] == PICO_TOPIC:
                entry['message']['channels'] = np.array([992]*16, dtype=np.uint16)
            if entry['topic'] in IMU_TOPICS:
                for key in ('linear_acceleration_covariance', 'angular_velocity_covariance'):
                    entry['message'][key] = np.array(entry['message'][key], dtype=np.float64)
        result = decode_records(records, include_imu_covariance=True)
        self.assertEqual(result['rejected'], [])
        self.assertIs(type(result['pico'][1]['steering_raw']), int)
        self.assertTrue(all(type(v) is float for v in result['imu'][0]['covariance_body']))

    def test_ros_adapter_uses_source_stamps_and_actual_types(self):
        records = fixture()
        all_topics = {r['topic']: r['type'] for r in records}
        metadata = dict(storage_identifier='mcap', topics_with_message_count=[
            dict(topic_metadata=dict(name=name, type=kind), message_count=sum(r['topic'] == name for r in records))
            for name, kind in all_topics.items()])
        readers = []
        class Reader:
            def __init__(self):
                self.index = 0
                readers.append(self)
            def open(self, storage, converter):
                self.storage = storage
            def set_filter(self, selection):
                self.selection = selection
                self.selected = [r for r in records if r['topic'] in selection['topics']]
            def has_next(self):
                return self.index < len(self.selected)
            def read_next(self):
                item = self.selected[self.index]
                self.index += 1
                return item['topic'], item['message'], item['bag_stamp_ns']+5000000
        rosbag = types.SimpleNamespace(SequentialReader=Reader, StorageOptions=lambda **kw: kw,
                                       ConverterOptions=lambda *args: args, StorageFilter=lambda **kw: kw)
        serialization = types.SimpleNamespace(deserialize_message=lambda raw, cls: raw)
        utilities = types.SimpleNamespace(get_message=lambda kind: kind)
        with tempfile.TemporaryDirectory() as folder, patch.dict('sys.modules', {
            'rosbag2_py': rosbag, 'rclpy.serialization': serialization, 'rosidl_runtime_py.utilities': utilities,
        }):
            (Path(folder)/'metadata.yaml').write_text(json.dumps(dict(rosbag2_bagfile_information=metadata)))
            result = read_bag(folder)
            self.assertEqual(result['pico'][1]['stamp'], 100.02)
            self.assertAlmostEqual(result['pico'][1]['bag_stamp'], 100.025)
            self.assertEqual(readers[0].storage['storage_id'], 'mcap')
            self.assertEqual(len(readers), 2)
            self.assertFalse(set(readers[0].selection['topics']) & set(IMU_TOPICS))
            self.assertEqual(set(readers[1].selection['topics']), set(IMU_TOPICS))
            records[:] = [r for r in records if r['topic'] != IMU_TOPICS[1]]
            with self.assertRaisesRegex(ValueError, 'no usable raw'):
                read_bag(folder)
            metadata['topics_with_message_count'][0]['message_count'] = 0
            (Path(folder)/'metadata.yaml').write_text(json.dumps(dict(rosbag2_bagfile_information=metadata)))
            with self.assertRaisesRegex(ValueError, 'empty/invalid'):
                read_bag(folder)


if __name__ == '__main__':
    unittest.main()

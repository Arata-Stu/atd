import csv
from hashlib import sha256
from importlib.util import find_spec
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from atd_vehicle_calibration.cli import check_bag, main, source_info


class CalibrationRunTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bag = self.root / 'record' / 'synthetic'
        self.bag.mkdir(parents=True)
        (self.bag / 'metadata.yaml').write_text('synthetic recording\n')
        self.poses = self.root / 'map' / 'nvidia' / 'poses' / 'odom_poses.tum'
        self.poses.parent.mkdir(parents=True)
        # A forward run with constant acceleration, timestamped in ROS seconds.
        self.times = [2000 + i / 50 for i in range(76)]
        self.poses.write_text(''.join(
            f'{s:.3f} {(s-2000) + 0.25*(s-2000)**2:.8f} 0 0 0 0 0 1\n' for s in self.times))
        self.manifest = self.root / 'map' / 'mapping.json'
        digest = sha256((self.bag / 'metadata.yaml').read_bytes()).hexdigest()
        self.manifest.write_text(json.dumps({'source': {'metadata_sha256': digest}}))
        self.output = self.root / 'run'
        self.args = ['--bag', str(self.bag), '--poses', str(self.poses), '--pose-frame', 'base_link',
                     '--output', str(self.output), '--wheelbase-m', '0.25']

    def decoded_bag(self):
        return dict(pico=[dict(stamp=s, instance_id='pico', serial='fixture', segment=0, rc_valid=True,
                               manual_valid=True, active_path=1, steering_raw=1000, throttle_raw=1200)
                          for s in self.times],
                    imu=[dict(stamp=s, segment=0, kind=kind, vector_body=vector) for s in self.times
                         for kind, vector in [('gyro', [0, 0, 0]), ('accel', [.5, 0, 9.81])]],
                    static_transforms=[], rejected=[], provenance={'source': 'synthetic fixture'})

    def test_trajectory_to_report_and_independent_run_guard(self):
        with patch('atd_vehicle_calibration.inputs.read_bag', return_value=self.decoded_bag()):
            self.assertEqual(main(self.args), 0)
        summary_path = self.output / 'summary.json'
        saved = summary_path.read_bytes()
        summary = json.loads(saved)
        self.assertEqual(summary['source']['association'], 'mapping_metadata_sha256')
        self.assertGreater(summary['eligible_samples'], 60)
        with (self.output / 'timeseries.csv').open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        middle = rows[len(rows)//2]
        self.assertAlmostEqual(float(middle['vo_ax_mps2']), .5, places=5)
        self.assertAlmostEqual(float(middle['vo_vx_mps']), 1 + .5*(float(middle['stamp'])-2000), places=5)
        self.assertAlmostEqual(float(middle['equivalent_steering_rad']), 0)
        self.assertEqual(middle['imu_ax_at_sensor_mps2'], '')
        self.assertTrue((self.output / 'response.png').read_bytes().startswith(b'\x89PNG'))
        self.assertEqual(main(self.args), 1)
        self.assertEqual(summary_path.read_bytes(), saved)
        self.assertEqual(main(self.args + ['--output', str(self.root / 'validation'),
                                          '--reference-summary', str(summary_path)]), 1)
        self.assertFalse((self.root / 'validation').exists())

    def test_refuse_incorrect_bag_association_or_global_slam(self):
        self.manifest.write_text(json.dumps({'source': {'metadata_sha256': 'another bag'}}))
        with self.assertRaisesRegex(ValueError, '一致'):
            source_info(self.bag, self.poses, 'base_link')
        with self.assertRaisesRegex(ValueError, 'odom_poses'):
            source_info(self.bag, self.poses.with_name('slam_poses.tum'), 'base_link')

    def test_check_bag_before_vo(self):
        from atd_vehicle_calibration.inputs import CAMERA_INFO_TOPICS, IMU_TOPICS, PICO_TOPIC, PICO_TYPE
        types = {PICO_TOPIC: PICO_TYPE, '/tf_static': 'tf2_msgs/msg/TFMessage',
                 **{name: 'sensor_msgs/msg/Imu' for name in IMU_TOPICS},
                 **{name: 'sensor_msgs/msg/CameraInfo' for name in CAMERA_INFO_TOPICS}}
        topics = [dict(topic_metadata=dict(name=name, type=kind), message_count=1) for name, kind in types.items()]
        files = ['split.mcap']
        def publish():
            metadata = dict(storage_identifier='mcap', relative_file_paths=files, topics_with_message_count=topics)
            (self.bag / 'metadata.yaml').write_text(json.dumps(dict(rosbag2_bagfile_information=metadata)))
        publish()
        (self.bag / files[0]).write_bytes(b'fixture')
        args = ['--bag', str(self.bag)]
        self.assertEqual(check_bag(args), 0)
        self.assertEqual(check_bag(args + ['--require-stereo']), 3)
        for name in ('infra1', 'infra2'):
            topics.append(dict(topic_metadata=dict(name=f'/camera/camera/{name}/image_rect_raw',
                               type='sensor_msgs/msg/Image'), message_count=1))
        publish()
        self.assertEqual(check_bag(args + ['--require-stereo']), 0)
        topics[-1]['message_count'] = 0
        publish()
        self.assertEqual(check_bag(args + ['--require-stereo']), 3)
        for unsafe in ('../meta.mcap', '/absolute.mcap', 'missing.mcap'):
            files[0] = unsafe
            publish()
            self.assertEqual(check_bag(args), 3)

    @unittest.skipUnless(find_spec('rosbag2_py'), 'ROS 2 MCAP runtime required')
    def test_ros_serialized_pico_and_imu_to_report(self):
        import rosbag2_py
        from atd_interfaces.msg import PicoTelemetry
        from geometry_msgs.msg import TransformStamped
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import CameraInfo, Imu
        from tf2_msgs.msg import TFMessage
        from atd_vehicle_calibration.inputs import CAMERA_FRAMES, CAMERA_INFO_TOPICS, IMU_TOPICS, PICO_TOPIC

        bag = self.root / 'serialized'
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        topics = {'/tf_static': 'tf2_msgs/msg/TFMessage', PICO_TOPIC: 'atd_interfaces/msg/PicoTelemetry',
                  **dict.fromkeys(IMU_TOPICS, 'sensor_msgs/msg/Imu'),
                  **dict.fromkeys(CAMERA_INFO_TOPICS, 'sensor_msgs/msg/CameraInfo')}
        for topic, kind in topics.items():
            writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=kind, serialization_format='cdr'))
        frames = ['camera_link', *CAMERA_FRAMES, 'camera_accel_optical_frame', 'camera_gyro_optical_frame']
        transforms = []
        for child in frames:
            tf = TransformStamped(child_frame_id=child)
            tf.header.frame_id = 'base_link' if child == 'camera_link' else 'camera_link'
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        start_ns = 2000 * 10**9
        writer.write('/tf_static', serialize_message(TFMessage(transforms=transforms)), start_ns)
        for topic, frame in zip(CAMERA_INFO_TOPICS, CAMERA_FRAMES):
            info = CameraInfo()
            info.header.frame_id, info.header.stamp.sec = frame, 2000
            writer.write(topic, serialize_message(info), start_ns)
        for i in range(len(self.times)):
            stamp = start_ns + i * 20_000_000
            pico = PicoTelemetry(instance_id='synthetic', serial='fixture', sequence_number=i, board_us=i*20_000,
                                 channels=[1000, 1200] + [992]*14, rc_valid=True, active_path=1,
                                 power_ready=True, sbus_fresh=True)
            pico.header.stamp.sec, pico.header.stamp.nanosec = divmod(stamp, 10**9)
            writer.write(PICO_TOPIC, serialize_message(pico), stamp)
            for topic, frame in zip(IMU_TOPICS, frames[-2:]):
                imu = Imu()
                imu.header.frame_id, imu.header.stamp = frame, pico.header.stamp
                imu.linear_acceleration.x, imu.linear_acceleration.z = 0.5, 9.81
                writer.write(topic, serialize_message(imu), stamp)
        del writer  # finalize bag metadata before opening the reader
        self.manifest.unlink()  # external fixture poses with a confirmed frame
        self.assertEqual(main(self.args + ['--bag', str(bag)]), 0)
        summary = json.loads((self.output / 'summary.json').read_text())
        self.assertGreater(summary['eligible_samples'], 60)
        self.assertGreater(summary['analysis']['coverage']['imu_gyro_samples'], 60)


if __name__ == '__main__':
    unittest.main()

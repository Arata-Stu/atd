from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from atd_bringup.replay_inputs import IMU_TOPIC, ImuProfile, select_bag, validate_bag


class ReplaySelectionTests(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.names = ['Gazebo/Monza lap', 'Track/Monza lap']
        for name in self.names:
            folder = self.root / name
            folder.mkdir(parents=True)
            (folder / 'metadata.yaml').touch()

    def test_direct_paths_work_without_a_terminal(self):
        with patch.object(sys.stdin, 'isatty', return_value=False):
            for name in self.names:
                self.assertEqual(select_bag(name, self.root), self.root / name)
                self.assertEqual(select_bag(self.root / name, self.root), self.root / name)
            with self.assertRaisesRegex(ValueError, 'atd replay BAG'):
                select_bag(None, self.root)
            with self.assertRaisesRegex(ValueError, 'metadata.yaml'):
                select_bag('missing', self.root)

    def test_fzf_returns_a_parent_qualified_path_or_cancels(self):
        with patch.object(sys.stdin, 'isatty', return_value=True), \
                patch('shutil.which', return_value='fzf'), patch('subprocess.run') as run:
            run.return_value = SimpleNamespace(returncode=0, stdout=self.names[1] + '\0')
            self.assertEqual(select_bag(None, self.root), self.root / self.names[1])
            self.assertEqual(run.call_args.kwargs['input'].split('\0'), self.names)
            run.return_value = SimpleNamespace(returncode=130, stdout='')
            self.assertIsNone(select_bag(None, self.root))

    def test_numbered_menu_shows_parents_and_validates_selection(self):
        with patch.object(sys.stdin, 'isatty', return_value=True), patch('shutil.which', return_value=None), \
                patch('builtins.input', side_effect=['2', '', '0']), redirect_stdout(StringIO()) as output:
            self.assertEqual(select_bag(None, self.root), self.root / self.names[1])
            for name in self.names:
                self.assertIn(name, output.getvalue())
            self.assertIsNone(select_bag(None, self.root))
            with self.assertRaisesRegex(ValueError, '番号'):
                select_bag(None, self.root)


class ReplayInputValidationTests(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.bag = Path(temporary.name)
        self.topics = {'/tf_static': 'tf2_msgs/msg/TFMessage'}
        for side in (1, 2):
            self.topics[f'/camera/camera/infra{side}/image_rect_raw'] = 'sensor_msgs/msg/Image'
            self.topics[f'/camera/camera/infra{side}/camera_info'] = 'sensor_msgs/msg/CameraInfo'
        self.reader = Mock()
        mocks = {
            'rosbag2_py': SimpleNamespace(SequentialReader=lambda: self.reader,
                                          StorageOptions=SimpleNamespace, StorageFilter=SimpleNamespace,
                                          ConverterOptions=lambda *_: None),
            'rclpy.serialization': SimpleNamespace(deserialize_message=lambda raw, _: raw),
            'tf2_msgs.msg': SimpleNamespace(TFMessage=object),
            'sensor_msgs.msg': SimpleNamespace(Imu=object),
        }
        modules = patch.dict(sys.modules, mocks)
        modules.start()
        self.addCleanup(modules.stop)

    def write_metadata(self, empty=None):
        rows = [dict(topic_metadata=dict(name=name, type=kind), message_count=0 if name == empty else 4)
                for name, kind in self.topics.items()]
        (self.bag / 'metadata.yaml').write_text(json.dumps({
            'rosbag2_bagfile_information': dict(storage_identifier='mcap', topics_with_message_count=rows),
        }))

    def set_static_frames(self, infrared_frames):
        edges = [('base_link', 'camera_link'), *(('camera_link', frame) for frame in infrared_frames)]
        transforms = [SimpleNamespace(header=SimpleNamespace(frame_id=parent), child_frame_id=child)
                      for parent, child in edges]
        self.reader.has_next.side_effect = [True, False]
        self.reader.read_next.return_value = ('/tf_static', SimpleNamespace(transforms=transforms), 0)

    def test_requires_correct_nonempty_stereo_and_static_tf_topics(self):
        for name in self.topics:
            with self.subTest(name=name):
                self.write_metadata(empty=name)
                with self.assertRaisesRegex(ValueError, name):
                    validate_bag(self.bag)
        self.topics['/camera/camera/infra2/image_rect_raw'] = 'sensor_msgs/msg/CompressedImage'
        self.write_metadata()
        with self.assertRaisesRegex(ValueError, 'infra2/image_rect_raw'):
            validate_bag(self.bag)
        self.reader.open.assert_not_called()

    def test_reads_only_tf_and_checks_both_camera_extrinsics(self):
        self.write_metadata()
        frames = ('camera_infra1_optical_frame', 'camera_infra2_optical_frame')
        self.set_static_frames(frames)
        validate_bag(self.bag)
        self.assertEqual(self.reader.set_filter.call_args.args[0].topics, ['/tf_static'])
        self.set_static_frames(frames[:1])
        with self.assertRaisesRegex(ValueError, 'base_link to camera_infra2_optical_frame'):
            validate_bag(self.bag)

    def test_vio_reads_sensor_time_and_requires_imu_extrinsics(self):
        self.write_metadata()
        with self.assertRaisesRegex(ValueError, '統合 IMU'):
            validate_bag(self.bag, vio=True)
        self.topics[IMU_TOPIC] = 'sensor_msgs/msg/Imu'
        self.write_metadata()
        imu_frame = 'camera_gyro_optical_frame'
        for include_imu_tf in (True, False):
            frames = ['camera_infra1_optical_frame', 'camera_infra2_optical_frame']
            self.set_static_frames(frames + ([imu_frame] if include_imu_tf else []))
            tf = self.reader.read_next.return_value
            self.reader.has_next.side_effect = [True] * 4 + [False, True, False]
            samples = [(IMU_TOPIC, SimpleNamespace(header=SimpleNamespace(frame_id=imu_frame,
                        stamp=SimpleNamespace(sec=1, nanosec=i * 2_500_000))), 0) for i in range(4)]
            self.reader.read_next.side_effect = [*samples, tf]
            if include_imu_tf:
                self.assertEqual(validate_bag(self.bag, vio=True), ImuProfile(imu_frame, 400.0))
            else:
                with self.assertRaisesRegex(ValueError, 'Missing static TF.*' + imu_frame):
                    validate_bag(self.bag, vio=True)

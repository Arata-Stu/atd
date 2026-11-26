"""Gazebo 内のセンサー転送で画素、校正、IMU の時刻・値が保持されることを検査する。"""
from array import array
from contextlib import nullcontext
import os
from pathlib import Path
import socket
import sys
import tempfile
from threading import Lock
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sensor_ingress import SensorIngress
import world

try:
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import CameraInfo, Image, Imu
except ImportError:
    serialize_message = None


class SensorIngressTests(unittest.TestCase):
    def test_only_composed_world_installs_the_native_sender(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled), tempfile.TemporaryDirectory() as tmp:
                out = Path(tmp)
                world.build(out, imu_hz=400, sensor_ingress=enabled)
                root = ET.parse(out / 'world.sdf')
                ingress = root.find("world/plugin[module_name='sensor_ingress']")
                self.assertEqual(ingress is not None, enabled)
                if enabled:
                    self.assertEqual(ingress.findtext('ready_file'), str(out.resolve() / 'sensor_ingress.ready'))
                    self.assertEqual(ingress.findtext('imu_hz'), '400')
                    self.assertEqual(ingress.findtext('course'), 'odaiba')

    @unittest.skipUnless(sys.platform == 'linux', 'Linux abstract SOCK_SEQPACKET and SO_PEERCRED')
    def test_paused_world_announces_enabled_streams_without_waiting_for_a_sample(self):
        sender = SensorIngress()
        sender.lock, sender.peer, sender.has_imu = Lock(), None, True
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as server:
            server.bind('\0atd-gazebo-test-' + str(os.getpid()))
            server.listen(1)
            with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
                client.connect(server.getsockname())
                sender.admit(server.accept()[0])
                self.assertEqual(client.recv(64), bytes([255, 1]))
                with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as duplicate:
                    duplicate.connect(server.getsockname())
                    sender.admit(server.accept()[0])
                    self.assertEqual(duplicate.recv(64), b'')
            sender.has_imu = False
            with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as restarted:
                restarted.connect(server.getsockname())
                sender.admit(server.accept()[0])
                self.assertEqual(restarted.recv(64), bytes([255, 0]))
            sender.peer.close()

    @unittest.skipUnless(serialize_message, 'ROS 2 Python bindings')
    def test_fd_pixels_calibration_projection_and_inertial_covariance(self):
        sender = SensorIngress()
        sender.types, sender.serialize, sender.mono8 = (Image, CameraInfo, Imu), serialize_message, 1
        sender.lock, sender.failed = Lock(), False
        header = NS(data=[NS(key='frame_id', value=['camera_infra2_optical_frame'])], stamp=NS(sec=4, nsec=17))
        pixels = bytes(range(8))
        image = NS(header=header, data=pixels, step=4, width=3, height=2, pixel_format_type=1)
        # Linux uses the production memfd. Darwin can exercise the same SCM_RIGHTS
        # ownership transfer and CDR payload with an immediately unlinked file.
        def anonymous_file(*_):
            fd, name = tempfile.mkstemp()
            os.unlink(name)
            return fd
        left, right = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        sender.peer = left
        with left, right, (nullcontext() if hasattr(os, 'memfd_create') else
                          patch.multiple(os, memfd_create=anonymous_file, MFD_CLOEXEC=1, create=True)):
            sender.sample(1, image)
            data, anc, _, _ = right.recvmsg(4096, socket.CMSG_SPACE(4))
            self.assertEqual(data[0], 1)
            info = deserialize_message(data[1:], Image)
            self.assertEqual((info.width, info.height, info.step, info.encoding, info.data.tolist()),
                             (3, 2, 4, 'mono8', []))
            self.assertEqual((info.header.stamp.sec, info.header.stamp.nanosec, info.header.frame_id),
                             (4, 17, 'camera_infra2_optical_frame'))
            self.assertEqual(anc[0][:2], (socket.SOL_SOCKET, socket.SCM_RIGHTS))
            fd, = array('i', anc[0][2])
            try:
                self.assertEqual(os.pread(fd, 8, 0), pixels)
            finally:
                os.close(fd)
            k = [300., 0., 320., 0., 300., 240., 0., 0., 1.]
            # For rectified stereo the right projection can carry -fx * baseline.
            p = [300., 0., 320., -28.5, 0., 300., 240., 0., 0., 0., 1., 0.]
            camera = NS(header=header, width=640, height=480, HasField=lambda k: False,
                        intrinsics=NS(k=k), rectification_matrix=k, projection=NS(p=p))
            sender.sample(3, camera)
            data, anc, _, _ = right.recvmsg(4096, socket.CMSG_SPACE(4))
            self.assertFalse(anc)
            info = deserialize_message(data[1:], CameraInfo)
            self.assertEqual(data[0], 3)
            self.assertEqual((list(info.k), list(info.p), list(info.r)), (k, p, k))
            self.assertEqual(info.header, deserialize_message(sender.message(1, image)[0], Image).header)
            unknown = [-1.] + [0.] * 8
            imu = NS(header=header, orientation=NS(x=0., y=0., z=0., w=1.),
                     angular_velocity=NS(x=1., y=2., z=3.), linear_acceleration=NS(x=0., y=-9.8, z=0.),
                     orientation_covariance=NS(data=unknown), angular_velocity_covariance=NS(data=[]),
                     linear_acceleration_covariance=NS(data=[]))
            sender.sample(4, imu)
            data, anc, _, _ = right.recvmsg(4096, socket.CMSG_SPACE(4))
            converted = deserialize_message(data[1:], Imu)
            self.assertEqual(data[0], 4)
            self.assertEqual((converted.linear_acceleration.y, converted.angular_velocity.z), (-9.8, 3.))
            self.assertEqual(list(converted.orientation_covariance), unknown)
            self.assertEqual(converted.header, info.header)
            self.assertFalse(anc)

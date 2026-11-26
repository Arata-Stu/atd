"""Gazebo プロセス内から Isaac のセンサー component に画像の共有メモリ FD を渡す。"""
from array import array
from functools import partial
import os
from pathlib import Path
import socket
import struct
from threading import Lock, Thread


# One receiver per Gazebo domain. Linux abstract sockets follow the shared network
# namespace of the two containers and disappear when the process exits.
def endpoint():
    return '\0atd-gazebo-sensors-v1-' + str(int(os.environ.get('ROS_DOMAIN_ID', '0')))


def check_error(output):
    failure = output / 'scene/sensor_ingress.failed'
    if failure.is_file():
        raise RuntimeError('Gazebo センサーの共有メモリ転送に失敗しました: ' + failure.read_text())


class SensorIngress:
    def configure(self, entity, sdf, ecm, events):
        from gz.msgs.camera_info_pb2 import CameraInfo
        from gz.msgs.image_pb2 import Image, L_INT8
        from gz.msgs.imu_pb2 import IMU
        from gz.transport import Node
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import CameraInfo as RosInfo, Image as RosImage, Imu as RosImu

        self.types = RosImage, RosInfo, RosImu
        self.serialize = serialize_message
        self.mono8 = L_INT8
        self.lock = Lock()
        self.peer = None
        self.failed = False
        self.dropped = 0
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.server.bind(endpoint())
        self.server.listen(1)
        self.node = Node()
        circuit = sdf.get_string('course')
        streams = [(Image, 'left/image'), (Image, 'right/image'),
                   (CameraInfo, 'left/camera_info'), (CameraInfo, 'right/camera_info')]
        self.has_imu = bool(int(sdf.get_string('imu_hz')))
        if self.has_imu:
            streams.extend((IMU, kind) for kind in ('imu', 'gyro', 'accel'))
        for channel, (kind, suffix) in enumerate(streams):
            topic = f'/{circuit}/{suffix}'
            if not self.node.subscribe(kind, topic, partial(self.sample, channel)):
                raise RuntimeError(f'Gazebo センサーを購読できません: {topic}')
        self.failure = Path(sdf.get_string('ready_file')).with_suffix('.failed')
        Thread(target=self.accept, daemon=True, name='atd-gazebo-admission').start()
        Path(sdf.get_string('ready_file')).touch(exist_ok=False)

    def message(self, channel, msg):
        image_type, info_type, imu_type = self.types
        pixels = None
        if channel < 2:
            pixels = msg.data
            step = msg.step or msg.width
            if (msg.pixel_format_type != self.mono8 or step < msg.width or
                    not msg.height or len(pixels) != step * msg.height):
                raise ValueError('mono8 画像の幅・高さ・step・データ長が一致しません')
            ros = image_type(width=msg.width, height=msg.height, step=step, encoding='mono8')
        elif channel < 4:
            ros = info_type(width=msg.width, height=msg.height)
            if msg.HasField('distortion'):
                enum = msg.distortion.DESCRIPTOR.fields_by_name['model'].enum_type
                model = enum.values_by_number[msg.distortion.model].name.lower()
                if model in ('plumb_bob', 'rational_polynomial', 'equidistant'):
                    ros.distortion_model = model
                ros.d = array('d', msg.distortion.k)
            for key, values in (('k', msg.intrinsics.k), ('r', msg.rectification_matrix), ('p', msg.projection.p)):
                if values:
                    setattr(ros, key, list(values))
        else:
            ros = imu_type()
            for key in ('orientation', 'angular_velocity', 'linear_acceleration'):
                source, target = getattr(msg, key), getattr(ros, key)
                for axis in ('x', 'y', 'z', 'w') if key == 'orientation' else ('x', 'y', 'z'):
                    setattr(target, axis, getattr(source, axis))
                cov = getattr(msg, key + '_covariance').data
                if cov:
                    setattr(ros, key + '_covariance', list(cov))
        ros.header.stamp.sec, ros.header.stamp.nanosec = msg.header.stamp.sec, msg.header.stamp.nsec
        for entry in msg.header.data:
            if entry.key == 'frame_id' and entry.value:
                ros.header.frame_id = entry.value[0].replace('::', '/')
        return self.serialize(ros), pixels

    def accept(self):
        # Advertise the actual streams before the paused world renders a frame.
        while True:
            peer, _ = self.server.accept()
            self.admit(peer)

    def admit(self, peer):
        _, uid, _ = struct.unpack('3i', peer.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
        with self.lock:
            if self.peer is not None:
                try:
                    connected = self.peer.recv(1, socket.MSG_PEEK) != b''
                except BlockingIOError:
                    connected = True
                except ConnectionResetError:
                    connected = False
                if not connected:
                    self.peer.close()
                    self.peer = None
            if uid != os.getuid() or self.peer is not None:
                peer.close()
                return
            peer.setblocking(False)
            try:
                peer.send(bytes([255, self.has_imu]))
            except OSError:
                peer.close()
                return
            self.peer = peer

    def sample(self, channel, msg):
        with self.lock:
            if self.peer is None or self.failed:
                return
            fd = None
            try:
                header, pixels = self.message(channel, msg)
                rights = []
                if pixels is not None:
                    fd = os.memfd_create('atd-gazebo-image', os.MFD_CLOEXEC)
                    if os.write(fd, pixels) != len(pixels):
                        raise IOError('共有メモリに画像全体を書き込めませんでした')
                    rights = [(socket.SOL_SOCKET, socket.SCM_RIGHTS, array('i', [fd]))]
                self.peer.sendmsg([bytes([channel]), header], rights)
            except BlockingIOError:
                self.dropped += 1
                if self.dropped == 1 or self.dropped % 100 == 0:
                    print(f'Gazebo センサー受信の遅れで {self.dropped} 件を破棄しました', flush=True)
            except (BrokenPipeError, ConnectionResetError):
                self.peer.close()
                self.peer = None
            except Exception as exc:
                self.failed = True
                self.failure.write_text(str(exc))
                raise
            finally:
                if fd is not None:
                    os.close(fd)


def get_system():
    return SensorIngress()

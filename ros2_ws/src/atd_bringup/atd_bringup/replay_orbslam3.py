"""ORB-SLAM3 の再生ジョブ、保存地図の確定、計測時刻付きの再定位結果。"""
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time
from uuid import uuid4

import yaml
from atd_mapping.frames import stamp_ns

from .replay_gt_geometry import Alignment, PlanarPose
from .replay_inputs import SLAM_ODOMETRY
from .replay_orb_inputs import inputs_from_bag, noise_settings


def orb_executable():
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError

    try:
        prefix = Path(get_package_prefix('atd_orbslam3'))
    except PackageNotFoundError:
        issue = '現在の ROS 2 環境に atd_orbslam3 が見つかりません'
    else:
        executable = prefix / 'lib/atd_orbslam3/orbslam3_node'
        if executable.is_file() and os.access(executable, os.X_OK):
            return str(executable)
        issue = f'現在の ROS 2 環境で {executable} を実行できません'
    raise RuntimeError(
        f'{issue}。ORB 対応コンテナ内で次を実行してください。\n'
        'atd-build --packages-up-to atd_orbslam3 --cmake-clean-cache --cmake-args -DATD_REQUIRE_ORBSLAM3=ON\n'
        'atd-shell ros2 pkg executables atd_orbslam3')


def saved_alignment(atlas_path, circuit):
    path = Path(atlas_path).with_name('gazebo_alignment.json')
    if not path.is_file() or not circuit:
        return None
    data = json.loads(path.read_bytes())
    if data.get('circuit') != circuit:
        return None
    world, origin, quaternion = (data.get(key) for key in ('world_origin_xy_yaw', 'map_origin_xyz', 'map_from_world_xyzw'))
    for vector, length in ((world, 3), (origin, 3), (quaternion, 4)):
        if not isinstance(vector, list) or len(vector) != length or not all(
                type(v) in (float, int) and math.isfinite(v) for v in vector):
            raise ValueError(f'地図作成時の Gazebo 座標変換が不正です: {path}')
    if not math.isclose(math.hypot(*quaternion), 1., rel_tol=1e-5):
        raise ValueError(f'地図作成時の Gazebo 回転が正規化されていません: {path}')
    return Alignment(PlanarPose(*world), tuple(origin), tuple(quaternion))


class OrbReplay:
    tracker = '/orbslam3_node'
    seed_source = '/atd_bag_replay'

    def __init__(self, stack, root, node, bag, options, imu, course, executable):
        import rclpy
        from atd_interfaces.msg import LocalizationStatus
        from atd_localization.orb_assets import map_workspace, orb_map_metadata
        from atd_localization.replay_inputs import independent_bag
        from atd_localization.orb_replay import OrbBootstrap, recorded_calibration
        from nav_msgs.msg import Odometry
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import Header
        from std_srvs.srv import Trigger

        self.node, self.root, self.bag = node, root, bag
        self.options, self.course = options, course
        self.stage = self.fixed_alignment = self.save_client = None
        self.last_processed = self.first_pose = self.first_tracking = None
        self.poses = self.processed = 0
        folder = Path(os.environ.get('ATD_REPLAY_RECORD_ROOT', '/data/record/rosbags')).expanduser().resolve()
        self.bag_id = str(bag.relative_to(folder)) if bag.is_relative_to(folder) else bag.name
        self.bag_hash = hashlib.sha256((bag / 'metadata.yaml').read_bytes()).hexdigest()
        self.inputs = inputs_from_bag(bag, options.start_offset, vio=bool(imu))
        self.params = dict(mode='mapping', use_sim_time=True, stereo_inertial=bool(imu),
                           camera_fps=self.inputs.camera_fps)
        self.atlas = None
        self.metadata = None
        if options.map:
            self.metadata = meta = orb_map_metadata(root, options.map)
            if meta['tracking'] != 'vo':
                raise ValueError('固定地図への再定位にはステレオ単独で作成した地図を選んでください')
            if bag.is_relative_to(folder):
                independent_bag(folder, self.bag_id, meta['source_bag'], meta['source_metadata_sha256'])
            elif self.bag_hash == meta['source_metadata_sha256']:
                raise ValueError('地図作成に使用した bag と同じです。別走行を指定してください')
            self.params.update(mode='localization', atlas_path=str(meta['atlas_path']),
                               **{key: meta[key] for key in ('map_id', 'map_sha256', 'hdmap_sha256')})
            self.fixed_alignment = saved_alignment(meta['atlas_path'], course.get('circuit'))
        elif options.save_map:
            self.stage = stack.enter_context(map_workspace(root, options.save_map))
            self.atlas = self.stage / 'orbslam3/atlas.osa'
            self.params.update(atlas_path=str(self.atlas))
            self.save_client = node.create_client(Trigger, '/orbslam3/save_map')
        if imu:
            self.params.update(imu_frame=imu.frame, imu_frequency=imu.frequency,
                               **noise_settings(options.imu_calibration))

        name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid4().hex[:12]
        self.directory = root / 'runs/orbslam3' / name
        self.directory.mkdir(parents=True)
        params_path = self.directory / 'parameters.yaml'
        params_path.write_text(yaml.safe_dump({self.tracker: {'ros__parameters': self.params}}))
        self.command = [executable, '--ros-args', '--params-file', str(params_path)]
        self.summary = dict(backend='orbslam3', bag=str(bag), metadata_sha256=self.bag_hash,
                            tracking=options.tracking, rate=options.rate, map_id=options.map or options.save_map,
                            map_sha256=self.metadata['map_sha256'] if self.metadata else None,
                            mode=self.params['mode'], start_offset_s=options.start_offset,
                            input_pairs=self.inputs.pairs, first_input_stamp_ns=self.inputs.first_left_ns,
                            last_input_stamp_ns=self.inputs.last_left_ns, camera_fps=self.inputs.camera_fps,
                            status='running')
        self._summary()
        stack.push(self._exit)
        trajectory = csv.writer(stack.enter_context((self.directory / 'trajectory.csv').open('x', newline='')))
        trajectory.writerow(('image_stamp_ns', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))
        events = csv.writer(stack.enter_context((self.directory / 'status.csv').open('x', newline='')))
        events.writerow(('stamp_ns', 'state', 'epoch', 'reason'))
        calibration, self.image_size = recorded_calibration(bag)
        bootstrap = OrbBootstrap(node, calibration)
        self.seed_topics = bootstrap.topics
        self.seed = bootstrap.wait

        def processed(msg):
            stamp = stamp_ns(msg.stamp)
            if self.last_processed is None or stamp > self.last_processed:
                self.last_processed = stamp
                self.processed += 1

        def odometry(msg):
            stamp = stamp_ns(msg.header.stamp)
            if msg.header.frame_id != 'map' or msg.child_frame_id != 'base_link':
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            trajectory.writerow((stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            self.poses += 1
            if self.first_pose is None:
                self.first_pose = stamp

        def status(msg):
            if not self.metadata or msg.map_id != options.map or msg.map_sha256 != self.metadata['map_sha256']:
                return
            stamp = stamp_ns(msg.header.stamp)
            if msg.state == 'tracking' and self.first_tracking is None:
                self.first_tracking = stamp
            events.writerow((stamp, msg.state, msg.epoch, msg.reason))

        node.create_subscription(Header, '/orbslam3/processed', processed,
                                 QoSProfile(depth=256, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Odometry, SLAM_ODOMETRY, odometry, QoSProfile(depth=256))
        if self.metadata:
            node.create_subscription(LocalizationStatus, '/localization/status', status, QoSProfile(depth=256))
        self.spin = lambda timeout_sec: rclpy.spin_once(node, timeout_sec=timeout_sec)
        print(f'ORB-SLAM3 の入力 {self.inputs.camera_fps:g} Hz、対象 {self.inputs.pairs} 対。'
              f'状態と軌跡の保存先: {self.directory}', flush=True)

    def _summary(self):
        temp = self.directory / 'summary.json.tmp'
        temp.write_text(json.dumps(self.summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        temp.replace(self.directory / 'summary.json')

    def _exit(self, error_type, error, traceback):
        if self.summary['status'] != 'complete' and error is not None:
            self.summary.update(status='interrupted' if isinstance(error, KeyboardInterrupt) else 'failed',
                                reason=str(error) or type(error).__name__)
        self.close()

    def close(self):
        if self.summary['status'] == 'running':
            self.summary['status'] = 'incomplete'
        self.summary.update(processed_frames_observed=self.processed, last_processed_stamp_ns=self.last_processed,
                            trajectory_samples=self.poses, first_pose_stamp_ns=self.first_pose,
                            first_tracking_stamp_ns=self.first_tracking)
        if self.first_tracking is not None:
            self.summary['time_to_tracking_s'] = (self.first_tracking - self.inputs.first_left_ns) / 1e9
        self._summary()

    def finish(self, spin, check_running, result):
        from atd_localization.orb_assets import publish_map
        from std_srvs.srv import Trigger

        # Wait for actual tracking completion. A successful bag process alone cannot certify a saved atlas.
        deadline = time.monotonic() + 30
        while self.last_processed != self.inputs.last_left_ns and time.monotonic() < deadline:
            check_running()
            spin(.02)
        if self.last_processed != self.inputs.last_left_ns:
            raise RuntimeError(f'ORB-SLAM3 の末尾画像処理を確認できません: '
                               f'{self.last_processed} / {self.inputs.last_left_ns} ns。再生倍率を下げて再試行してください')
        deadline = time.monotonic() + .75
        while time.monotonic() < deadline:
            spin(.01)
        if not self.poses or (self.metadata and self.first_tracking is None):
            raise RuntimeError('対象地図への再定位を確認できません' if self.metadata else '有効なステレオ姿勢がありません')
        if result and not result.summary['sample_count']:
            raise RuntimeError('GT と画像時刻を照合できた ORB-SLAM3 の姿勢が 0 件です')
        if result and self.metadata:
            self.summary['initial_error'] = result.summary['initial_error']
        if self.save_client is not None:
            if not self.save_client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('/orbslam3/save_map が利用できません')
            future = self.save_client.call_async(Trigger.Request())
            deadline = time.monotonic() + 180
            while not future.done() and time.monotonic() < deadline:
                check_running()
                spin(.05)
            if not future.done() or not future.result().success:
                raise RuntimeError('ORB-SLAM3 の地図を保存できません: ' +
                                   (future.result().message if future.done() else 'timeout'))
            if result and result.summary.get('alignment') and self.course.get('circuit'):
                data = dict(circuit=self.course['circuit'], **result.summary['alignment'])
                self.atlas.with_name('gazebo_alignment.json').write_text(json.dumps(data, allow_nan=False) + '\n')
            publish_map(self.stage, self.root, self.options.save_map, self.bag_id, self.bag_hash,
                        self.inputs.camera_fps, image_size=self.image_size)
            print(f'ORB-SLAM3 地図を保存しました: {self.options.save_map}', flush=True)
        self.summary['status'] = 'complete'
        self.close()

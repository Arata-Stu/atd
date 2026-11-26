"""ORB-SLAM3 の再生ジョブ、保存地図の確定、計測時刻付きの再定位結果。"""
from contextlib import ExitStack
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
from atd_localization.replay_dds import replay_dds_profile
from atd_mapping.frames import stamp_ns

from .replay_gt_geometry import Alignment, PlanarPose
from .replay_inputs import IMU_TOPIC, ORB_MAP_MATCH, ORB_VIO_ODOMETRY, SLAM_ODOMETRY, STEREO_WORKFLOW, recorded_imu
from .replay_orb_inputs import inputs_from_bag, resolve_imu_noise


def orb_executable():
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError

    try:
        prefix = Path(get_package_prefix('atd_orbslam3'))
    except PackageNotFoundError:
        issue = '現在の ROS 2 環境に atd_orbslam3 が見つかりません'
    else:
        executable = prefix / 'lib/atd_orbslam3/orbslam3_replay_node'
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


def gyro_calibration(bag, topic, calibration):
    from .orb_gap_imu import _add_transform, _body_axis
    profile = recorded_imu(bag, topic)
    parents = {}
    for tf in calibration[STEREO_WORKFLOW['tf_static']].transforms:
        _add_transform(parents, tf)
    _, path = _body_axis(profile.frame, parents)
    return profile, path


class OrbReplay:
    tracker = '/orbslam3_node'
    seed_source = '/atd_bag_replay'
    sensor_source = '/orbslam3_bag_input'

    def __init__(self, stack, root, node, bag, options, imu, course, executable, *, gyro_topic=None, gazebo=False):
        import rclpy
        from atd_interfaces.msg import LocalizationStatus
        from atd_localization.orb_assets import map_workspace, orb_map_metadata
        from atd_localization.replay_inputs import independent_bag
        from atd_localization.orb_replay import OrbBootstrap, recorded_calibration
        from nav_msgs.msg import Odometry
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import Bool, Header
        from std_srvs.srv import Trigger

        self.node, self.root, self.bag = node, root, bag
        self.options, self.course = options, course
        self.player_name = f'atd_replay_player_{os.getpid()}'
        self.playback_finished = False
        self.gyro_topic = gyro_topic
        self.bridge = None
        self.stage = self.fixed_alignment = self.save_client = None
        self.last_processed = self.last_imu_skipped = self.first_pose = self.first_tracking = None
        self.poses = self.processed = self.imu_skipped = 0
        self.first_local_vio = self.last_local_vio = self.first_map_match = self.last_map_match = None
        self.local_vio_poses = self.map_matches = 0
        folder = Path(os.environ.get('ATD_REPLAY_RECORD_ROOT', '/data/record/rosbags')).expanduser().resolve()
        self.bag_id = str(bag.relative_to(folder)) if bag.is_relative_to(folder) else bag.name
        self.bag_hash = hashlib.sha256((bag / 'metadata.yaml').read_bytes()).hexdigest()
        imu_noise = {}
        if imu:
            source, values = resolve_imu_noise(options.imu_calibration, gazebo=gazebo)
            imu_noise = dict(source=source, values=values)
            print(f'ORB-SLAM3 Stereo-Inertial の IMU ノイズ設定: {source}', flush=True)
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
            self.params.update(imu_frame=imu.frame, imu_frequency=imu.frequency, **imu_noise['values'])

        name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid4().hex[:12]
        self.directory = root / 'runs/orbslam3' / name
        self.directory.mkdir(parents=True)
        self.summary = dict(backend='orbslam3', bag=str(bag), metadata_sha256=self.bag_hash,
                            tracking=options.tracking, rate=options.rate, map_id=options.map or options.save_map,
                            map_sha256=self.metadata['map_sha256'] if self.metadata else None,
                            mode=self.params['mode'], start_offset_s=options.start_offset,
                            input_pairs=self.inputs.pairs, first_input_stamp_ns=self.inputs.first_left_ns,
                            last_input_stamp_ns=self.inputs.last_left_ns, camera_fps=self.inputs.camera_fps,
                            sensor_input_transport='ros2_intra_process', dds_profile=replay_dds_profile(),
                            status='running')
        if self.metadata:
            self.summary['source_metadata_sha256'] = self.metadata['source_metadata_sha256']
            self.summary['localization_method'] = 'fixed_stereo_atlas_vio' if imu else 'fixed_stereo_atlas'
        if imu:
            self.summary.update(imu_noise=imu_noise, imu_frame=imu.frame, imu_frequency_hz=imu.frequency)
        self._summary()
        stack.push(self._exit)
        self.recording = stack.enter_context(ExitStack())
        trajectory = csv.writer(self.recording.enter_context((self.directory / 'trajectory.csv').open('x', newline='')))
        trajectory.writerow(('image_stamp_ns', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))
        events = csv.writer(self.recording.enter_context((self.directory / 'status.csv').open('x', newline='')))
        events.writerow(('stamp_ns', 'state', 'epoch', 'reason'))
        calibration, self.image_size = recorded_calibration(bag)
        if gyro_topic:
            from .orb_gap_bridge import OrbGapBridge
            try:
                profile, tf_path = gyro_calibration(bag, gyro_topic, calibration)
            except ValueError as exc:
                if options.gyro_bridge == 'on':
                    raise
                self.gyro_topic = None
                self.summary['gyro_bridge'] = dict(topic=gyro_topic, disabled_reason=str(exc))
                print(f'ジャイロ補完を使用できないため観測姿勢で再生します。{exc}', flush=True)
            else:
                self.bridge = OrbGapBridge(node, self.directory, options.map, self.metadata['map_sha256'],
                                           imu_topic=gyro_topic)
                self.recording.callback(self.bridge.close)
                self.summary['gyro_bridge'] = dict(topic=gyro_topic, imu_frame=profile.frame, tf_path=tf_path,
                                                    report='gyro_bridge_summary.json')
                print(f'途絶補完ジャイロ: {gyro_topic} ({profile.frequency:g} Hz, {profile.frame})', flush=True)
        bootstrap = OrbBootstrap(node, calibration)
        self.seed_topics = bootstrap.topics
        self.seed = bootstrap.wait
        sensor_params = dict(bag_path=str(bag), start_offset_s=float(options.start_offset),
                             rate=float(options.rate), player_name=self.player_name,
                             imu_topic=IMU_TOPIC if imu else (self.gyro_topic or ''))
        params_path = self.directory / 'parameters.yaml'
        params_path.write_text(yaml.safe_dump({self.tracker: {'ros__parameters': self.params},
                                               self.sensor_source: {'ros__parameters': sensor_params}}))
        self.command = [executable, '--ros-args', '--params-file', str(params_path)]

        def playback_finished(msg):
            if msg.data:
                self.playback_finished = True

        def processed(msg):
            stamp = stamp_ns(msg.stamp)
            if self.last_processed is None or stamp > self.last_processed:
                self.last_processed = stamp
                self.processed += 1

        def imu_skipped(msg):
            stamp = stamp_ns(msg.stamp)
            if self.inputs.first_left_ns <= stamp <= self.inputs.last_left_ns and (
                    self.last_imu_skipped is None or stamp > self.last_imu_skipped):
                self.last_imu_skipped = stamp
                self.imu_skipped += 1

        def odometry(msg):
            stamp = stamp_ns(msg.header.stamp)
            if msg.header.frame_id != 'map' or msg.child_frame_id != 'base_link':
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            trajectory.writerow((stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            self.poses += 1
            if self.first_pose is None:
                self.first_pose = stamp
                if imu and not self.metadata:
                    seconds = (stamp - self.inputs.first_left_ns) / 1e9
                    self.summary['time_to_first_vio_pose_s'] = seconds
                    print(f'ORB-SLAM3 慣性初期化後の最初の有効姿勢: 先頭の対象画像から {seconds:.3f} 秒', flush=True)

        def local_vio(msg):
            stamp = stamp_ns(msg.header.stamp)
            if (msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link' or
                    not self.inputs.first_left_ns <= stamp <= self.inputs.last_left_ns or
                    (self.last_local_vio is not None and stamp <= self.last_local_vio)):
                return
            self.local_vio_poses += 1
            self.last_local_vio = stamp
            if self.first_local_vio is None:
                self.first_local_vio = stamp
                self.summary['time_to_first_vio_pose_s'] = (stamp - self.inputs.first_left_ns) / 1e9

        def map_match(msg):
            stamp = stamp_ns(msg.stamp)
            if (self.inputs.first_left_ns <= stamp <= self.inputs.last_left_ns and
                    (self.last_map_match is None or stamp > self.last_map_match)):
                self.map_matches += 1
                self.last_map_match = stamp
                if self.first_map_match is None:
                    self.first_map_match = stamp

        def status(msg):
            if not self.metadata or msg.map_id != options.map or msg.map_sha256 != self.metadata['map_sha256']:
                return
            stamp = stamp_ns(msg.header.stamp)
            if msg.state == 'tracking' and self.first_tracking is None:
                self.first_tracking = stamp
            events.writerow((stamp, msg.state, msg.epoch, msg.reason))

        def subscribe(kind, topic, callback, qos):
            sub = node.create_subscription(kind, topic, callback, qos)
            self.recording.callback(node.destroy_subscription, sub)

        subscribe(Bool, self.sensor_source + '/finished', playback_finished,
                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        subscribe(Header, '/orbslam3/processed', processed,
                  QoSProfile(depth=256, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if imu:
            subscribe(Header, '/orbslam3/imu_skipped', imu_skipped,
                      QoSProfile(depth=256, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        subscribe(Odometry, SLAM_ODOMETRY, odometry, QoSProfile(depth=256))
        if imu and self.metadata:
            subscribe(Odometry, ORB_VIO_ODOMETRY, local_vio, QoSProfile(depth=256))
            subscribe(Header, ORB_MAP_MATCH, map_match, QoSProfile(depth=256))
        if self.metadata:
            subscribe(LocalizationStatus, '/localization/status', status, QoSProfile(depth=256))
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
        if self.options.tracking == 'vio':
            self.summary.update(imu_skipped_frames_observed=self.imu_skipped,
                                last_imu_skipped_stamp_ns=self.last_imu_skipped)
            if self.metadata:
                self.summary.update(local_vio_samples=self.local_vio_poses, first_local_vio_stamp_ns=self.first_local_vio,
                                    map_matches_observed=self.map_matches, first_map_match_stamp_ns=self.first_map_match)
        if self.first_tracking is not None:
            self.summary['time_to_tracking_s'] = (self.first_tracking - self.inputs.first_left_ns) / 1e9
        self._summary()

    def finish(self, spin, check_running, result):
        from atd_localization.orb_assets import publish_map
        from std_srvs.srv import Trigger

        # Wait for actual tracking completion. A successful bag process alone cannot certify a saved atlas.
        deadline = time.monotonic() + 30
        def bridge_pending():
            return self.bridge is not None and self.bridge.last_frame_ns != self.inputs.last_left_ns

        def tail_pending():
            return self.inputs.last_left_ns not in (self.last_processed, self.last_imu_skipped)

        while (tail_pending() or bridge_pending()) and time.monotonic() < deadline:
            check_running()
            spin(.02)
        if tail_pending():
            hint = ('末尾画像まで画像対と統合 IMU が届いたか確認してください'
                    if self.options.tracking == 'vio' else '再生倍率を下げて再試行してください')
            raise RuntimeError(f'ORB-SLAM3 の末尾画像処理を確認できません: '
                               f'{self.last_processed} / {self.inputs.last_left_ns} ns。{hint}')
        if bridge_pending():
            raise RuntimeError(f'ジャイロ補完で末尾画像を確認できません: {self.bridge.last_frame_ns} / '
                               f'{self.inputs.last_left_ns} ns')
        fixed_vio = self.options.tracking == 'vio' and self.metadata is not None

        def feedback_progress():
            counts = (self.poses, self.processed, self.imu_skipped, self.first_tracking)
            return counts + (self.local_vio_poses, self.map_matches) if fixed_vio else counts

        deadline = time.monotonic() + .75
        while time.monotonic() < deadline:
            previous = feedback_progress()
            check_running()
            spin(.01)
            if feedback_progress() != previous:
                deadline = time.monotonic() + .75
        check_running()
        if self.metadata:
            if bool(self.poses) != (self.first_tracking is not None):
                raise RuntimeError('保存地図に対する tracking 状態と姿勢出力が一致しません')
            if fixed_vio and (bool(self.poses) != bool(self.map_matches) or (self.poses and not self.local_vio_poses)):
                raise RuntimeError('地図基準の VIO 姿勢と、固定地図の照合・局所 VIO の観測が一致しません')
            if self.poses:
                self.summary['outcome'] = 'tracking_observed'
            else:
                reason = 'no_local_vio_pose' if fixed_vio and not self.local_vio_poses else 'no_map_match'
                self.summary.update(outcome='no_tracking', no_tracking_reason=reason)
                print(f'末尾まで再生しました。固定地図基準の姿勢は 0 件です ({reason})。', flush=True)
        elif not self.poses:
            if self.options.tracking == 'vio':
                raise RuntimeError('慣性初期化後の有効な ORB-SLAM3 姿勢を観測できませんでした。'
                                   'IMU の時刻・固定 TF と、移動開始・旋回時の初期化ログを確認してください')
            raise RuntimeError('有効なステレオ姿勢がありません')
        if result and self.poses and not result.summary['sample_count']:
            raise RuntimeError('GT と画像時刻を照合できた ORB-SLAM3 の姿勢が 0 件です')
        if result and self.metadata:
            result.summary['outcome'] = self.summary['outcome']
            if self.poses:
                self.summary['initial_error'] = result.summary['initial_error']
        # RViz can stay open after EOF. Finalize all outputs while the graph is still alive.
        self.recording.close()
        if self.options.tracking == 'vio':
            poses = (f'局所 VIO 姿勢 {self.local_vio_poses} 件、固定地図の照合 {self.map_matches} 件、'
                     f'地図基準の VIO 姿勢 {self.poses} 件' if fixed_vio else f'慣性初期化後の有効姿勢 {self.poses} 件')
            print(f'ORB-SLAM3 Stereo-Inertial: 対象 {self.inputs.pairs} 対、処理完了通知 {self.processed} 件、'
                  f'IMU 不足で見送った画像対の通知 {self.imu_skipped} 件、{poses}', flush=True)
        if self.bridge:
            report = self.bridge.summary
            if not report['counts'].get('gyro_samples'):
                message = '車体座標に変換できたジャイロ観測が0件です。補完を評価できません'
                if self.options.gyro_bridge == 'on':
                    raise RuntimeError(message)
                print(message, flush=True)
            print(f"補完姿勢 {report['predicted']} 件、再取得 {report['reacquisitions']} 回、"
                  f"同時刻の段差を計測できたのは {report['measured_jumps']} 回", flush=True)
            if report['measured_jumps']:
                xy, yaw = report['position_jump_m'], report['abs_yaw_jump_deg']
                print(f"再取得時の位置段差 中央値 {xy['median']:.3f} m / 最大 {xy['max']:.3f} m、"
                      f"方位段差 中央値 {yaw['median']:.2f}° / 最大 {yaw['max']:.2f}°", flush=True)
            print(f"再取得の区間別結果: {self.directory / 'reacquisitions.csv'}", flush=True)
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

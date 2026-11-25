"""地図作成 bag との分離、入力フレーム・校正・観測したステレオ時刻の検査。"""
from array import array
import hashlib
import math
from pathlib import Path

from atd_mapping.bag_inputs import inspect_bag, load_metadata
from atd_mapping.frames import stamp_ns


def independent_bag(root, bag_id, source_id, source_hash):
    root = Path(root).resolve()
    relative = Path(bag_id)
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:
        raise ValueError('bag は記録ルートからの相対パスで指定してください')
    bag = (root / relative).resolve(strict=True)
    meta_path = (bag / 'metadata.yaml').resolve(strict=True)
    if not bag.is_relative_to(root) or not meta_path.is_relative_to(bag):
        raise ValueError('記録ルートの外を参照する bag は使用できません')
    digest = hashlib.sha256(meta_path.read_bytes()).hexdigest()
    source = (root / source_id).resolve()
    if bag == source or digest == source_hash:
        raise ValueError('地図作成に使用した bag と同じです。別走行の bag を選んでください')
    meta = load_metadata(bag)
    if source.is_relative_to(root) and (source / 'metadata.yaml').is_file():
        source_bytes = (source / 'metadata.yaml').read_bytes()
        if hashlib.sha256(source_bytes).hexdigest() == source_hash:
            src = load_metadata(source)
            # A renamed split file or re-serialized metadata can describe the same recording.
            def signature(info):
                return (info.get('starting_time'), info.get('duration'),
                        sorted((t['topic_metadata']['name'], t['message_count'])
                               for t in info['topics_with_message_count']))
            if meta.get('starting_time') and meta.get('duration') and signature(meta) == signature(src):
                raise ValueError('記録開始時刻・長さ・トピック件数が地図作成 bag と同一です')
    return bag, digest


def stereo_window(left, right, tolerance_ns=5_000_000, *, left_stamp=False):
    """Tracker の許容時間差と出力時刻で、対応するステレオ入力区間を求める。"""
    i = j = pairs = 0
    first = previous = None
    max_gap = 0
    while i < len(left) and j < len(right):
        delta = left[i] - right[j]
        if abs(delta) <= tolerance_ns:
            stamp = left[i] if left_stamp else max(left[i], right[j])
            if previous is None:
                first = stamp
            else:
                max_gap = max(max_gap, stamp - previous)
            previous = stamp
            pairs += 1
            i += 1
            j += 1
        elif delta < 0:
            i += 1
        else:
            j += 1
    if pairs < 2 or first == previous:
        raise ValueError(f'左右の画像が {tolerance_ns/1e6:g} ms 以内で対応するステレオ観測が不足しています')
    return dict(stereo_pairs=pairs, left_images=len(left), right_images=len(right),
                first_stamp_ns=first, last_stamp_ns=previous, largest_pair_gap_s=max_gap / 1e9)


def camera_signature(message):
    # K[0] == 0 is the ROS CameraInfo convention for an uncalibrated camera.
    numbers = (*message.k, *message.p, *message.r, *message.d)
    if not all(math.isfinite(value) for value in numbers) or min(message.k[0], message.k[4], message.p[0], message.p[5]) <= 0:
        raise ValueError('未校正または非有限の CameraInfo です')
    return (message.distortion_model, *numbers)


def inspect_inputs(bag, meta):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image
    workflow = meta['workflow']
    frames = [workflow[k + '_frame'] for k in ('left', 'right')]
    topics, metadata = inspect_bag(bag, workflow, dict(tracking_mode=0, base_frame=workflow['base_frame'],
                                                       camera_optical_frames=frames))
    settings = {}
    for side, frame in zip(('left', 'right'), frames):
        for kind, cls in (('image', Image), ('info', CameraInfo)):
            settings[workflow[f'{side}_{kind}']] = (side, frame, cls)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(settings)))
    stamps = {side: array('q') for side in ('left', 'right')}
    signatures = {}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        side, frame, cls = settings[topic]
        message = deserialize_message(raw, cls)
        allowed_frames = {frame}
        if meta.get('backend') == 'orbslam3' and side == 'right' and cls is CameraInfo:
            # D455 can express the right projection matrix in the left optical frame.
            allowed_frames.add(frames[0])
        if message.header.frame_id not in allowed_frames or [message.height, message.width] != list(meta['input_shape'][-2:]):
            raise ValueError(f'地図の入力フレームまたはモデル解像度と異なります: {topic}')
        stamp = stamp_ns(message.header.stamp)
        if stamp <= 0:
            raise ValueError(f'ゼロまたは負の観測時刻です: {topic}')
        if cls is Image:
            times = stamps[side]
            if times and times[-1] >= stamp:
                raise ValueError(f'同じ画像ストリーム内で時刻が逆行・重複しました: {topic}')
            times.append(stamp)
        else:
            signature = camera_signature(message)
            if side in signatures and signature != signatures[side]:
                raise ValueError(f'走行の途中で CameraInfo の校正値が変わっています: {topic}')
            signatures[side] = signature
    if set(signatures) != {'left', 'right'}:
        raise ValueError('左右 CameraInfo が揃っていません')
    duration = metadata.get('duration', {}).get('nanoseconds')
    if type(duration) is not int or duration <= 0:
        raise ValueError('bag の再生時間を確認できません')
    orb = meta.get('backend') == 'orbslam3'
    stereo = stereo_window(stamps['left'], stamps['right'], 1_000_000 if orb else 5_000_000, left_stamp=orb)
    return topics, duration / 1e9, stereo

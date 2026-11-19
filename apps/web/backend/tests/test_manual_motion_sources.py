import csv
import hashlib

import pytest

from atd_api import manual_motion_sources as motion
from atd_api.store import Store, atomic_json, read_json

NS = 10**9
START = 1_810_000_000 * NS
BAG = '2026-11-19/lap'


@pytest.fixture
def record(tmp_path):
    store = Store(tmp_path/'rosbags', tmp_path/'out')
    bag = store.record_root / BAG
    bag.mkdir(parents=True)
    (bag/'metadata.yaml').write_text('bag metadata')
    return store


def calibrated(store, run='cal', relative=BAG):
    folder = store.result_root / 'runs/vehicle_calibration' / run
    folder.mkdir(parents=True)
    (folder/'response.png').write_bytes(b'png')
    path = folder/'timeseries.csv'
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow('stamp,motion_valid,valid,motion_segment,vo_vx_mps,speed_mps'.split(','))
        # 有効なVOはプロポ入力が無効でも使う。絶対速度の列は後退も正。
        for t, seg, ok, vx in [(0,0,True,1),(.05,0,True,.1),(.1,0,False,None),
                               (.15,1,True,-.4),(.2,1,True,-.5)]:
            writer.writerow([START/NS+t, ok, False, seg, vx, abs(vx) if ok else None])
    atomic_json(folder/'summary.json', dict(schema_version=1,
        source=dict(bag=str(store.record_root/relative), metadata_sha256=motion._bag_digest(store, BAG),
                    poses='nvidia/poses/odom_poses.tum', poses_sha256='a'*64, pose_frame='camera_link',
                    body_frame='base_link', association='mapping_metadata_sha256'),
        motion=dict(world_frame='odom',base_frame='base_link', input_pose_frame='camera_link', max_gap_s=.2,
                    body_to_pose=dict(parent='base_link',child='camera_link',translation=[.14,0,.2],rotation=[0,0,0,1])),
        analysis={}, eligible_samples=0, timeseries_samples=5))
    return path


def localized(store):
    folder = store.result_root / 'runs/localization/l'
    folder.mkdir(parents=True)
    times = [START, START+50_000_000, START+400_000_000]
    with (folder/'trajectory.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow('stamp_ns,epoch,x_m,y_m,speed_mps'.split(','))
        for t, epoch, vx in zip(times, (0,0,1), (.5,.12,-.3)):
            writer.writerow([t, epoch, 0.,0.,vx])
    atomic_json(folder/'report.json', dict(schema='atd.localization.validation.v1', map_id='course', bag=BAG,
        source_bag='survey/lap', frame='map', body_frame='base_link', outcome='partial_tracking',
        map_sha256='a'*64, hdmap_sha256='a'*64, source_metadata_sha256='b'*64,
        metadata_sha256=motion._bag_digest(store,BAG), trajectory='trajectory.csv', pose_samples=3,
        gap_tolerance_s=.2, stereo=dict(first_stamp_ns=START,last_stamp_ns=START+NS), tracking_intervals=[
            dict(first_stamp_ns=times[0], last_stamp_ns=times[1], samples=2, epoch=0),
            dict(first_stamp_ns=times[2], last_stamp_ns=times[2], samples=1, epoch=1)]))


def odom(store, key='map'):
    directory = store.maps_root/key
    path = directory / motion.offline.ODOM_POSES
    path.parent.mkdir(parents=True)
    # 車体 yaw=180° でワールド+xに走る。車体前後速度は負になる。
    path.write_text(''.join(f'{START//NS + i/20:.9f} {i/20} 0 0 0 0 1 0\n' for i in range(28)))
    src = dict(kind='nvidia_offline', inputs=dict(base_frame='base_link'), bag=BAG,
               odom_poses=dict(path=motion.offline.ODOM_POSES.as_posix(),frame='base_link',
                               sha256=hashlib.sha256(path.read_bytes()).hexdigest()),
               metadata_sha256=motion._bag_digest(store,BAG))
    atomic_json(directory/'mapping.json', dict(source=src))
    return path


def test_calibration_signed_speed_clock_gaps_and_identity(record):
    data = calibrated(record)
    calibrated(record, 'other', 'other/lap')
    localized(record)
    assert [x['id'] for x in motion.sources(record, BAG)] == ['calibration:cal','localization:l']
    src = 'calibration:cal'
    rows = motion.window(record,BAG,src,START,0,NS)
    assert [row['speed_mps'] for row in rows] == [1,.1,-.4,-.5]
    assert rows[1]['end_s'] == pytest.approx(.05, abs=1e-6)
    assert rows[2]['start_s'] == pytest.approx(.15, abs=1e-6)
    assert rows[2]['segment'] > rows[1]['segment']
    assert motion.window(record,BAG,src,START,70_000_000,140_000_000) == []
    with pytest.raises(ValueError):
        motion.window(record,BAG,'calibration:other',START,0,NS)
    data.write_text(data.read_text().replace(',-0.4,', ',-0.7,'))
    assert motion.window(record,BAG,src,START,165_000_000,170_000_000)[0]['speed_mps'] == -.7
    (record.record_root/BAG/'metadata.yaml').write_text('another bag')
    assert motion.sources(record,BAG) == []
    with pytest.raises(ValueError):
        motion.window(record,BAG,src,START,0,NS)


def test_localization_uses_tracking_only_and_never_stretches_last_fix(record):
    localized(record)
    rows = motion.window(record,BAG,'localization:l',START,20_000_000,NS)
    assert rows == [dict(start_s=.02,end_s=.05,speed_mps=.5,segment=0),
                    dict(start_s=.05,end_s=.050000001,speed_mps=.12,segment=0),
                    dict(start_s=.4,end_s=.400000001,speed_mps=-.3,segment=1)]
    assert list(motion.spans(record,BAG,'localization:l',START,NS)) == [
        (0,50_000_000,.5,0), (50_000_000,50_000_001,.12,0), (400_000_000,400_000_001,-.3,1)]


def test_selected_calibration_with_invalid_frame_or_csv_reports_errors(record):
    data = calibrated(record)
    summary = data.parent/'summary.json'
    document = read_json(summary)
    rows = data.read_text()
    data.write_text(rows + 'not,an,expected,sample\n')
    with pytest.raises(ValueError):
        motion.prepare(record,BAG,'calibration:cal',START,NS)
    data.write_text(rows)
    document['motion']['body_to_pose']['rotation'] = [0,0,0,0]
    atomic_json(summary, document)
    assert not motion.sources(record,BAG)
    with pytest.raises(ValueError):
        motion.window(record,BAG,'calibration:cal',START,0,NS)


def test_odom_source_uses_verified_body_trajectory_once_and_rejects_edits(record, monkeypatch):
    path = odom(record)
    assert [item['id'] for item in motion.sources(record,BAG)] == ['odom:map']
    rows = motion.window(record,BAG,'odom:map',START,300_000_000,700_000_000)
    motion.prepare(record, BAG, 'odom:map', START, NS)
    with pytest.raises(ValueError, match='収録時間'):
        motion.prepare(record, BAG, 'odom:map', START+5*NS, NS)
    assert rows and all(row['speed_mps'] == pytest.approx(-1, abs=.001) for row in rows)
    assert rows[0]['start_s'] == .3 and rows[-1]['end_s'] == .7
    def unexpected(*args, **kwargs):
        pytest.fail('同じ速度を各表示区間で再計算している')
    monkeypatch.setattr(motion, '_samples', unexpected)
    assert motion.window(record,BAG,'odom:map',START,380_000_000,410_000_000)
    path.write_text(path.read_text()+'# modified\n')
    assert not motion.sources(record,BAG)
    with pytest.raises(ValueError):
        motion.window(record,BAG,'odom:map',START,0,NS)

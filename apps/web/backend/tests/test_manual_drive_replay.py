"""記録元時刻付きのメッセージを抽出し、Notebook が返す表示区間まで検証する。"""
from contextlib import closing
from collections import Counter
from pathlib import Path
import sqlite3
from types import SimpleNamespace

from fastapi.testclient import TestClient
import pytest

from atd_api import manual_drive_replay as replay
from atd_api.main import create_app
from atd_api.worker import execute

URL = '/api/manual-drive/replay'
BAG = 'day-one/course/lap'
NS = 10**9
START = 1_810_000_000 * NS


@pytest.fixture
def history(tmp_path, monkeypatch):
    packages = Path(__file__).resolve().parents[4] / 'ros2_ws/src'
    for name in ('atd_web_bridge', 'atd_vehicle_calibration'):
        monkeypatch.syspath_prepend(str(packages / name))
    from atd_web_bridge import bag_drive_timeline as timeline
    app = create_app(tmp_path / 'record', tmp_path / 'results')
    store = app.state.store
    directory = store.record_root / BAG
    directory.mkdir(parents=True)
    (directory / 'metadata.yaml').write_text('synthetic fixture')
    (directory / 'lap_0.mcap').touch()
    with TestClient(app) as api:
        yield api, store, timeline


def header(frame, seconds):
    sec, nano = divmod(START + round(seconds * NS), NS)
    return SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=sec, nanosec=nano))


def extract(store, timeline, topics, messages, duration=2, stereo=False):
    src = replay.recording(store, BAG)

    def run(argv):
        assert argv[:-1] == ['ros2', 'run', 'atd_web_bridge', 'extract_drive_timeline',
                             '--bag', str(src.directory), '--output']
        with closing(sqlite3.connect(argv[-1])) as db:
            timeline.build_index(db, START, round(duration * NS), topics, messages, stereo_ready=stereo)

    execute(store, SimpleNamespace(run=run, check=lambda: None), 'manual_drive_replay',
            dict(bag=BAG, resource=src.resource))


def test_common_clock_handoff_gaps_seek_and_changed_bag(history):
    api, store, timeline = history
    joy = SimpleNamespace(header=header('atd_gamepad', .2), source='joy', valid=True,
                          steering=0., throttle=-.4, enable_requested=False)
    rc = SimpleNamespace(header=header('pico', .1), serial='PICO', session_id='usb1', has_report=True,
                         sequence_number=1, selector=0, active_path=1, rx_mode_us=1100, fault_bits=0,
                         joy_active=False, servo_us=1480, esc_us=1500, reason='')
    switched = SimpleNamespace(**{**vars(rc), 'header':header('pico', .2), 'sequence_number':2,
                                  'rx_mode_us':1850, 'servo_us':1550})
    driving = SimpleNamespace(**{**vars(switched), 'header':header('pico', .23), 'sequence_number':3,
                                 'active_path':2, 'selector':1, 'joy_active':True, 'servo_us':1500, 'esc_us':1300})
    released = SimpleNamespace(**{**vars(driving), 'joy_active':False})
    data = [(timeline.CDC0, rc, .101), (timeline.JOY, joy, .203), (timeline.CDC0, switched, .205),
            (timeline.CDC0, driving, .232), (timeline.CDC0, released, .28)]
    messages = [(k, v, START + round(t * NS)) for k, v, t in data]
    extract(store, timeline, {timeline.JOY:(1,''), timeline.CDC0:(4,'')}, messages)
    assert api.post(URL, json={'bag':BAG}).json()['ready'] is True
    assert api.get(URL, params={'bag':BAG}).json()['warnings'] == []

    def window(a, b):
        reply = api.get(URL + '/window', params=dict(bag=BAG, start_s=a, end_s=b))
        assert reply.status_code == 200, reply.text
        return reply.json()

    response = window(.21, .6)
    assert response['joy'] == [dict(start_s=.21, end_s=.45, steering=0., throttle=-.4)]
    assert [(row['start_s'], row['end_s'], row['ch3'], row['control_source'], row['esc_us'])
            for row in response['board']] == [(.21,.23,'jetson','propo',1500), (.23,.28,'jetson','dualsense',1300),
                                               (.28,.38,'jetson','host',1300)]
    assert response['board'][0]['pwm_enabled'] is True
    assert window(.45, 2)['joy'] == [] and window(.45, 2)['board'] == []
    assert window(0, .2)['board'][0]['servo_us'] == 1480
    assert window(.21, .6) == response
    for a, b in [(1, .9), (2, 3), (0, 31), (-1, 0), (.1, .1)]:
        assert api.get(URL + '/window', params=dict(bag=BAG, start_s=a, end_s=b)).status_code in (400, 422)

    (store.record_root / BAG / 'lap_0.mcap').write_bytes(b'changed')
    assert api.get(URL, params={'bag':BAG}).json()['ready'] is False
    assert api.get(URL + '/window', params=dict(bag=BAG, start_s=0, end_s=1)).status_code == 400


def test_older_bag_independent_channels_and_last_instant(history):
    api, store, timeline = history
    telemetry = SimpleNamespace(header=header('pico', .5), serial='PICO', instance_id='s', sequence_number=1,
                                channels=[992]*16, active_path=2, fault_bits=0, rc_valid=True, sbus_fresh=True, sbus_flags=0)
    extract(store, timeline, {timeline.CDC1:(1,''), timeline.JOY:(2,'記録型が異なります')},
            [(timeline.CDC1, telemetry, START + NS//2)], duration=.5)
    meta = api.get(URL, params=dict(bag=BAG)).json()
    assert meta['duration_s'] == .5 and len(meta['warnings']) == 2
    assert any(timeline.CDC0 in warning for warning in meta['warnings'])
    assert any('記録型' in warning for warning in meta['warnings'])
    window = api.get(URL + '/window', params=dict(bag=BAG, start_s=0, end_s=.5)).json()
    row, = window['board']
    assert not window['joy'] and row['start_s'] == .5 < row['end_s']
    assert (row['rx_mode_us'], row['ch3_raw'], row['servo_us'], row['esc_us']) == (None, 992, None, None)
    assert row['control_source'] == 'host' and row['pwm_enabled'] is None


def test_confined_paths_and_concurrent_requests(history, monkeypatch):
    api, store, _ = history
    jobs = api.app.state.jobs
    started = []

    def start(action, payload, resources, title):
        assert action == 'manual_drive_replay' and payload['bag'] == BAG and 'record:' + BAG in resources
        job = dict(id='test-job', status='running', resources=resources)
        started.append(job)
        return job

    monkeypatch.setattr(jobs, 'start', start)
    monkeypatch.setattr(jobs, 'list', lambda: started)
    monkeypatch.setattr(jobs, 'get', lambda key: started[0])
    for _ in range(2):
        result = api.post(URL, json=dict(bag=BAG)).json()
        assert result['job_id'] == 'test-job' and not result['ready']
    assert len(started) == 1
    for bag in ('../lap', '/tmp/bag', 'day-two/course/lap'):
        assert api.get(URL, params=dict(bag=bag)).status_code == 400
    (store.record_root / BAG / 'link').symlink_to(store.result_root)
    assert api.post(URL, json=dict(bag=BAG)).status_code == 400


def motion_recording(store, timeline):
    loc, accel = '/localization/status', '/camera/camera/accel/sample'
    pos = SimpleNamespace(x=0., y=0., z=0.)
    orientation = SimpleNamespace(x=0., y=0., z=0., w=1.)
    pose = SimpleNamespace(position=pos, orientation=orientation)
    readings = []
    for t, v in [(.1,.8), (.2,.4), (.3,.12), (.4,.02), (.5,-.01), (.6,-.22), (.7,-.34), (.8,-.4)]:
        status = SimpleNamespace(header=header('map', t), state='tracking', pose=pose, speed_mps=v, yaw_rate_rps=0.,
                                 session_id='s', epoch=1, map_id='m', map_sha256='1'*64, hdmap_sha256='2'*64, reason='')
        readings.append((loc, status, t))
    for t in (.59, .60, .61):
        msg = SimpleNamespace(header=header('base_link', t), linear_acceleration_covariance=[0.]*9,
                              linear_acceleration=SimpleNamespace(x=-1.4, y=0., z=9.8))
        readings.append((accel, msg, t))
    rc = SimpleNamespace(header=header('pico', .2), serial='p', instance_id='usb', sequence_number=1,
                         channels=[992, 1100, 172]+[992]*13, rc_valid=True, sbus_fresh=True, sbus_age_ms=0,
                         sbus_flags=0, power_ready=True, active_path=1, fault_bits=0)
    joy = SimpleNamespace(header=header('atd_gamepad', .57), source='joy', valid=True,
                          steering=0., throttle=-.5, enable_requested=True)
    board = SimpleNamespace(header=header('pico', .58), serial='p', session_id='usb', has_report=True,
                            sequence_number=1, rx_mode_us=1850, selector=1, active_path=2, fault_bits=0,
                            servo_us=1500, esc_us=1400, joy_active=True, reason='')
    readings.extend([(timeline.CDC1, rc, .2), (timeline.JOY, joy, .57), (timeline.CDC0, board, .58)])
    messages = [(k,msg,START+round(t*NS)) for k,msg,t in sorted(readings, key=lambda r:r[2])]
    topics = {k:(n,'') for k,n in Counter(k for k,_,_ in readings).items()}
    extract(store, timeline, topics, messages)


def test_recorded_body_motion_matches_input_and_board_time(history):
    api, store, timeline = history
    motion_recording(store, timeline)
    status = api.get(URL, params=dict(bag=BAG)).json()
    assert status['default_speed_source'] == 'recorded' and not status['imu_reason']
    assert not status['can_generate_vo']
    params = dict(bag=BAG, start_s=.605, end_s=.607, speed_source='recorded')
    w = api.get(URL+'/window', params=params).json()
    assert (w['speed'][0]['speed_mps'], w['imu'][0]['accel_mps2'], w['joy'][0]['throttle'],
            w['board'][0]['esc_us'], w['board'][0]['control_source']) == (-.22,-1.4,-.5,1400,'dualsense')
    assert not w['propo']
    before = api.get(URL+'/window', params=dict(params, start_s=.21, end_s=.22)).json()
    assert before['propo'][0]['throttle_raw'] == 1100 and before['board'][0]['control_source'] == 'propo'
    missing = api.get(URL+'/window', params=dict(params, start_s=1., end_s=2.)).json()
    assert not missing['speed'] and not missing['imu']
    other = api.get(URL+'/window', params=dict(params, speed_source='localization:missing')).json()
    assert other['speed_issue'] and not other['speed'] and other['imu'] == w['imu']
    changes = api.get(URL+'/transitions', params=dict(bag=BAG, speed_source='recorded')).json()
    assert [(e['time_s'],e['from'],e['to'],e['near_zero_start_s']) for e in changes['transitions']] == [
        (.4,'forward','near_zero',None), (.6,'near_zero','reverse',.4)]


def test_speed_changes_need_sustained_observations_in_one_segment():
    # 短い速度ノイズ、新しい追跡区間、空白を挟んだ符号変化は切替として数えない。
    rows = [(0,.2,1.,1),(.2,.29,-.08,1),(.29,.55,1.,1),(.55,.75,-.2,2),(1.,1.2,.5,2)]
    assert not replay._transitions(((round(a*NS),round(b*NS),v,s) for a,b,v,s in rows), .05)


def test_generate_vo_preserves_source_and_finishes_with_signed_speed(history, monkeypatch):
    api, store, timeline = history
    origin = SimpleNamespace(x=0., y=0., z=0.)
    rotation = SimpleNamespace(x=0., y=0., z=0., w=1.)
    transforms = [SimpleNamespace(header=header('base_link', 0), child_frame_id=f'camera_infra{i}_optical_frame',
                                  transform=SimpleNamespace(translation=origin, rotation=rotation)) for i in (1,2)]
    extract(store, timeline, {'/tf_static':(1,'')},
            [('/tf_static',SimpleNamespace(transforms=transforms),START)], stereo=True)
    jobs = api.app.state.jobs
    started = []

    def enqueue(action, payload, resources, title):
        assert action == 'manual_drive_vo' and 'offline-mapping' in resources and 'record:'+BAG in resources
        job = dict(id='gpu', status='running', resources=resources, payload=payload, result=None)
        started.append(job)
        return job

    monkeypatch.setattr(jobs, 'start', enqueue)
    monkeypatch.setattr(jobs, 'list', lambda: started)
    monkeypatch.setattr(jobs, 'get', lambda key: started[0])
    for _ in range(2):
        assert api.post(URL+'/vo', json=dict(bag=BAG)).json()['vo_job_id'] == 'gpu'
    assert len(started) == 1

    def run(argv, env):
        assert argv[:3] == ['ros2','run','isaac_mapping_ros'] and '--sensor_data_bag='+str(store.record_root/BAG) in argv
        root = Path(next(v.split('=',1)[1] for v in argv if v.startswith('--base_output_folder=')))
        pose = root/'poses/odom_poses.tum'
        pose.parent.mkdir()
        # 後退する base_link の連続 VO 出力。グラフ化には実際の検証・微分処理を通す。
        pose.write_text(''.join(f'{START//NS+i/20:.9f} {-i/20} 0 0 0 0 0 1\n' for i in range(24)))
        (root/'cuvslam_map').mkdir()
        (root/'cuvslam_map/m.mdb').write_bytes(b'map')

    monkeypatch.setattr(replay.offline.shutil, 'which', lambda _: '/test/ros2')
    task = SimpleNamespace(run=run, check=lambda: None, log=lambda _: None, key='gpu')
    job = started[0]
    result = execute(store, task, 'manual_drive_vo', job['payload'])
    job.update(result=result, status='succeeded')
    status = api.get(URL, params=dict(bag=BAG)).json()
    assert status['default_speed_source'] == result['speed_source'] and not status['can_generate_vo']
    w = api.get(URL+'/window', params=dict(bag=BAG, start_s=.29, end_s=.31, speed_source=result['speed_source'])).json()
    assert w['speed'] and all(r['speed_mps'] == pytest.approx(-1., abs=1e-5) for r in w['speed'])
    assert api.post(URL+'/vo', json=dict(bag=BAG)).json()['vo_job_id'] == 'gpu' and len(started)==1

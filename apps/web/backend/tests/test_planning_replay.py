"""実 worker / 同じ planner core / SQLite の時刻照会を合成 bag 観測で検証する。"""
import csv
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

from fastapi.testclient import TestClient
import pytest

from atd_api import planning_replay as replay
from atd_api.main import create_app
from atd_api.store import atomic_json

NS = 10**9
START = 1_810_000_000 * NS
FIELDS = 'stamp_ns epoch x_m y_m z_m qx qy qz qw speed_mps yaw_rate_rps'.split()
URL = '/api/localization/runs/drive/planning'


def generated(points, speed):
    station = 0.
    rows = []
    for i, (x, y) in enumerate(points):
        if i:
            station += math.dist(points[i-1], (x, y))
        rows.append([station, x, y, 0., 0., speed, 0.])
    return dict(points=points, profile=rows)


@pytest.fixture
def history(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[4] / 'ros2_ws/src/atd_localization'))
    from atd_localization.replay_report import TrackingReport
    app = create_app(tmp_path/'record', tmp_path/'results')
    store = app.state.store
    doc = store.create_map('course')
    doc.update(lines=dict(raceline=generated([[0, 0], [4, 0], [4, 3], [0, 3]], 1.5),
                          centerline=generated([[0, .25], [4, .25], [4, 3], [0, 3]], 1.),
                          custom=generated([[0, -.25], [4, -.25], [4, 3], [0, 3]], 2.)))
    folder = store.result_root / 'runs/localization/drive'
    folder.mkdir(parents=True)
    m = store.folder('course')
    atomic_json(m/'hdmap/map.json', doc)
    (folder/'hdmap.json').write_bytes((m/'hdmap/map.json').read_bytes())
    atomic_json(m/'vgl.json', dict(status='complete', cuvslam_sha256={'a': 'a'*64}))
    ref = store.live_reference('course')
    report = TrackingReport(dict(first_stamp_ns=START, last_stamp_ns=START+4*NS), max_gap_s=.5)
    # Same-epoch resumption after a loss must reacquire far from the previous station.
    events = [('tracking', .2, .5), ('tracking', .25, .55), ('lost', .26, None),
              ('tracking', .4, 3.), ('tracking', .45, 3.05), ('stale', .5, None)]
    with (folder/'trajectory.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        for state, t, x in events:
            stamp = START+round(t*NS)
            sample = (x, .125, 0., 0., 0., 0., 1., -.2, 0.) if x is not None else None
            if report.observe('session', 2, state, state, stamp, sample):
                writer.writerow([stamp, 2, *sample])
    atomic_json(folder/'report.json', dict(schema='atd.localization.validation.v1', map_id='course', bag='b/lap',
        source_bag='a/lap', map_sha256=ref['map_sha256'], hdmap_sha256=ref['hdmap_sha256'],
        metadata_sha256='a'*64, source_metadata_sha256='b'*64, frame='map', body_frame='base_link',
        trajectory='trajectory.csv', **report.summary()))
    with TestClient(app) as api:
        yield api, store, folder, doc


def build(api, line):
    response = api.post(URL, json=dict(line=line))
    assert response.status_code == 200, response.text
    first = response.json()
    if first['job_id']:
        api.app.state.jobs.children[first['job_id']].wait(timeout=10)
        assert api.get('/api/jobs/'+first['job_id']).json()['status'] == 'succeeded', api.get(
            '/api/jobs/'+first['job_id']+'/log').json()
    return api.get(URL, params=dict(line=line)).json()


@pytest.mark.parametrize(('line','speed','offset'), [('raceline',1.5,.125), ('centerline',1.,-.125), ('custom',2.,.375)])
def test_saved_capture_actual_worker_and_seek_matches_live(history, line, speed, offset):
    api, _, _, document = history
    assert api.get(URL, params=dict(line=line)).json()['ready'] is False
    status = build(api, line)
    assert status['ready'] and status['valid_samples'] == status['pose_samples'] == 4
    assert api.post(URL, json=dict(line=line)).json()['ready']
    def sample(t):
        response = api.get(URL+'/frame', params=dict(line=line, time_s=t))
        assert response.status_code == 200, response.text
        return response.json()
    core = replay._core()
    live = core.Planner(core.ReferenceLine.from_document(document, line))
    for t, x in [(.2,.5), (.25,.55), (.4,3.), (.45,3.05)]:
        if t == .4:
            live.reset()
        direct = live.update(stamp_ns=START+round(t*NS), x=x, y=.125, yaw=0., speed_mps=-.2, session_id='session', epoch=2)
        data = sample(t)
        assert data['vehicle'] == {'x': x, 'y': .125, 'yaw': 0}
        for key, value in direct.items():
            assert data[key] == value
        assert data['target_speed_mps'] == pytest.approx(speed) and data['lateral_error_m'] == pytest.approx(offset)
        assert not data['control']['valid'] and '後退' in data['control']['reason']
    assert sample(.249)['sample_s'] == .2
    for t in (0, .251, .30, .46, 4):
        data = sample(t)
        assert not data['valid'] and data['vehicle'] is None and not data['trajectory']
        assert data['target_speed_mps'] is None and data['estimated_speed_mps'] is None
        assert data['control']['reason'] == data['reason'] and data['control']['target'] is None
    assert api.get(URL+'/frame', params=dict(line=line, time_s=4.01)).status_code == 400


def test_replay_control_uses_observed_speed_and_same_geometry_as_live(history):
    api, store, folder, _ = history
    observations = folder/'trajectory.csv'
    observations.write_text(observations.read_text().replace(',-0.2,', ',0.8,'))
    info = build(api, 'raceline')
    params = dict(line='raceline', time_s=.2)
    index = replay.source(store, 'drive', 'raceline').index
    timestamp = index.stat().st_mtime_ns
    original = api.get(URL+'/frame', params=params).json()
    core = replay._core('atd_control')
    config = core.LookaheadConfig()
    actual = original['control']
    assert actual['valid'] and actual['config'] == info['control_config']
    assert actual['requested_lookahead_m'] == pytest.approx(config.distance(.8))
    assert actual['steering_rad'] is None and actual['curvature_per_m'] < 0

    tuning = dict(min_lookahead_m=.4, lookahead_time_s=3., max_lookahead_m=3.)
    response = api.get(URL+'/frame', params=dict(params, **tuning, wheelbase_m=.257))
    assert response.status_code == 200, response.text
    tuned = response.json()
    predicted = core.pure_pursuit(tuned['trajectory'], **original['vehicle'], speed_mps=.8, wheelbase_m=.257,
                                  config=core.LookaheadConfig(**tuning))
    assert tuned['control'] == dict(predicted, config=dict(tuning, wheelbase_m=.257))
    assert tuned['control']['actual_lookahead_m'] == pytest.approx(2.8)
    assert tuned['control']['preview_limited'] is False
    assert tuned['control']['steering_rad'] < 0
    assert tuned['trajectory'][-1][0] > original['trajectory'][-1][0]
    assert index.stat().st_mtime_ns == timestamp

    # A new bag seek uses its own settings and clears the previous target on loss.
    lost = api.get(URL+'/frame', params=dict(params, time_s=.3)).json()
    assert not lost['control']['valid'] and lost['control']['target'] is None
    assert api.get(URL+'/frame', params=params).json() == original
    for invalid in [dict(wheelbase_m=-1), dict(lookahead_time_s='inf'), dict(max_lookahead_m=.2)]:
        assert api.get(URL+'/frame', params=dict(params, **invalid)).status_code in (400, 422)


def test_source_changes_require_reanalysis_and_hash_verified_map(history):
    api, store, folder, _ = history
    build(api, 'raceline')
    trajectory = folder/'trajectory.csv'
    trajectory.write_text(trajectory.read_text().replace(',-0.2,', ',0.6,'))
    assert not api.get(URL, params=dict(line='raceline')).json()['ready']
    assert api.get(URL+'/frame', params=dict(line='raceline', time_s=.2)).status_code == 400
    (folder/'hdmap.json').unlink()
    assert build(api, 'raceline')['ready']
    # Older runs may consult current map, but only while BOTH hashes still match.
    store.save_map('course', dict(store.load_map('course'), title='changed'))
    assert api.get(URL, params=dict(line='raceline')).status_code == 400
    (folder/'hdmap.json').write_text('{}')
    assert 'SHA256' in api.get(URL, params=dict(line='custom')).json()['detail']


def test_cancel_or_corrupt_csv_never_exposes_partial_index(history):
    _, store, folder, _ = history
    src = replay.source(store, 'drive', 'raceline')
    payload = dict(run_id='drive', line=src.line, resource=src.resource)
    from atd_api.jobs import Cancelled
    def stop():
        raise Cancelled('cancelled')
    with pytest.raises(Cancelled):
        replay.execute(store, SimpleNamespace(log=lambda _: None, check=stop), payload)
    assert not src.index.exists()
    with (folder/'trajectory.csv').open('a') as out:
        out.write('junk\n')
    src = replay.source(store,'drive','raceline')
    with pytest.raises(ValueError, match='試料数'):
        replay.execute(store, SimpleNamespace(log=lambda _: None, check=lambda:None), dict(payload,resource=src.resource))
    assert not src.index.exists()


def test_open_customline_stops_at_its_end_and_other_lines_are_unavailable(history):
    api, _, folder, doc = history
    doc.update(closed=False, lines={'custom': generated([[0, -.25], [3.1, -.25]], .5)})
    atomic_json(folder/'hdmap.json', doc)
    report = json.loads((folder/'report.json').read_bytes())
    report['hdmap_sha256'] = hashlib.sha256((folder/'hdmap.json').read_bytes()).hexdigest()
    atomic_json(folder/'report.json', report)
    assert api.get(URL, params=dict(line='raceline')).status_code == 400
    assert api.post(URL, json=dict(line='centerline')).status_code == 400
    assert build(api, 'custom')['valid_samples'] == 4
    frame = api.get(URL+'/frame', params=dict(line='custom', time_s=.45)).json()
    assert frame['trajectory'][0] == [3.05, -.25]
    assert frame['trajectory'][-1] == [3.1, -.25]


def speed_settings(store):
    from atd_api.speed_tables import TABLES, _digest
    table = dict(schema='atd.throttle_feedforward.v1', usable=True, run_id='bench', bag='calib',
                 input_unit='m/s', output_unit='sbus_raw_ch2', interpolation='linear', extrapolation='none',
                 proposal_sha256='d'*64, direction='increasing', speed_min_mps=1., speed_max_mps=2.,
                 points=[dict(speed_mps=1., throttle_raw=1100), dict(speed_mps=2., throttle_raw=1400)])
    atomic_json(store.result_root / TABLES / ('d'*64 + '.json'), dict(table=table, table_sha256=_digest(table)))
    return dict(line='raceline', table_sha256='d'*64)


def test_speed_real_worker_existing_report_without_original_recording(history):
    api, store, folder, _ = history
    base = URL.replace('planning', 'speed')
    choice = speed_settings(store)
    assert not api.get(base, params=choice).json()['planning_ready']
    assert api.post(base, json=choice).status_code == 400
    trajectory = folder/'trajectory.csv'
    trajectory.write_text(trajectory.read_text().replace(',-0.2,', ',1.0,'))
    build(api, 'raceline')
    job = api.post(base, json=choice).json()
    api.app.state.jobs.children[job['job_id']].wait(timeout=10)
    ready = api.get(base, params=choice).json()
    assert ready['ready'] and '欠測' in ready['propo_reason'] and '欠測' in ready['joy_reason']
    def at(t):
        return api.get(base+'/frame', params=dict(choice, time_s=t)).json()
    late = at(.45)
    assert late['sample']['valid'] and late['sample']['calculated_raw'] == pytest.approx(1270.2)
    assert late['sample']['propo_raw'] is None and 'bag' in late['sample']['propo_reason']
    assert late['sample']['joy_throttle'] is None and late['sample']['manual_source'] == 'unknown'
    assert late['history'][2]['segment'] != late['history'][1]['segment']
    assert late['history'][2]['i_raw'] == 0.   # Tracking loss clears I even in the same localization epoch.
    assert not at(.3)['sample']['valid'] and at(.3)['sample']['calculated_raw'] is None
    at(.2)
    assert at(.45) == late  # Seek order cannot alter I/D.
    assert not api.get(base, params=dict(choice, ki=0)).json()['ready']
    assert api.post(base, json=dict(choice, kp=-1)).status_code == 422


def test_old_installed_extractor_does_not_poison_manual_input_cache(history, tmp_path):
    from contextlib import closing
    import sqlite3
    from atd_api import speed_replay as speed
    _, store, _, _ = history
    src = SimpleNamespace(input_index=tmp_path/'shared.sqlite', bag=tmp_path/'bag')

    def old_extractor(args):
        with closing(sqlite3.connect(args[-1])) as db:
            db.execute('CREATE TABLE propo (stamp,raw,reason)')

    with pytest.raises(ValueError, match='atd_control'):
        speed._extract(store, SimpleNamespace(run=old_extractor, check=lambda: None), src, tmp_path)
    assert not src.input_index.exists()


@pytest.mark.parametrize('has_pico,has_joy', [(True, True), (True, False), (False, True)])
def test_speed_recorded_manual_inputs_gaps_cache_and_source_changes(history, has_pico, has_joy):
    import sqlite3
    from atd_api import speed_replay as speed
    api, store, folder, _ = history
    base = URL.replace('planning', 'speed')
    choice = speed_settings(store)
    bag = store.record_root/'b/lap'
    bag.mkdir(parents=True)
    meta = bag/'metadata.yaml'
    meta.write_bytes(b'fixture for extraction')
    original = bag/'lap.mcap'
    original.write_bytes(b'fixture')
    report = json.loads((folder/'report.json').read_text())
    atomic_json(folder/'report.json', dict(report, metadata_sha256=hashlib.sha256(meta.read_bytes()).hexdigest()))
    trajectory = folder/'trajectory.csv'
    trajectory.write_text(trajectory.read_text().replace(',-0.2,', ',1.0,'))
    build(api, 'raceline')
    calls = []
    def extract(args):
        calls.append(args)
        assert args[:3] == ['ros2','run','atd_control'] and args[3:7] == ['extract_propo','--bag',str(bag),'--output']
        with sqlite3.connect(args[-1]) as db:
            db.execute('CREATE TABLE topics (topic,messages)')
            db.execute('CREATE TABLE propo (stamp,path,raw,reason)')
            db.execute('CREATE TABLE joy (effective,stamp,throttle,enabled,reason)')
            if has_pico:
                db.execute("INSERT INTO topics VALUES ('/vehicle/pico/telemetry',4)")
                db.executemany('INSERT INTO propo VALUES (?,?,?,?)', [
                    (START+190_000_000,'propo',1177,''), (START+210_000_000,'propo',1200,''),
                    (START+249_000_000,'host',None,'Pico が HOST 経路にあります'),
                    (START+390_000_000,'propo',1260,'')])
            if has_joy:
                db.execute("INSERT INTO topics VALUES ('/teleop/command',2)")
                db.executemany('INSERT INTO joy VALUES (?,?,?,?,?)', [
                    (START+240_000_000,START+240_000_000,-.375,1,''),
                    (START+420_000_000,START+240_000_000,None,None,'DualSense の運転要求が無効です')])
    def calculate(params):
        src = speed.source(store, 'drive', speed.Comparison(**params))
        speed.execute(store, SimpleNamespace(log=lambda _:None, check=lambda:None, run=extract),
                      dict(run_id='drive', selection=params, resource=src.resource))
    calculate(choice)
    def at(t):
        return api.get(base+'/frame', params=dict(choice,time_s=t)).json()['sample']
    status = api.get(base, params=choice).json()
    assert bool(status['propo_reason']) == (not has_pico)
    assert bool(status['joy_reason']) == (not has_joy)
    assert at(.2)['propo_raw'] == (1177 if has_pico else None)
    assert at(.2)['propo_age_s'] == (.01 if has_pico else None)
    assert at(.2)['joy_throttle'] is None
    host = at(.25)
    assert host['propo_raw'] is None and host['joy_throttle'] == (-.375 if has_joy else None)
    assert host['joy_age_s'] == (.01 if has_joy else None)
    assert host['joy_enable_requested'] is (True if has_joy else None)
    expected_host = ('host_joy' if has_joy else 'host') if has_pico else 'unknown'
    assert host['manual_source'] == expected_host
    assert at(.4)['manual_source'] == ('propo' if has_pico else 'unknown')
    assert at(.4)['propo_raw'] == (1260 if has_pico else None)
    assert at(.4)['joy_throttle'] == (-.375 if has_joy else None)
    assert at(.45)['propo_raw'] is None and at(.45)['propo_reason']
    assert at(.45)['joy_throttle'] is None and at(.45)['joy_reason'] and at(.45)['manual_source'] == 'unknown'
    assert at(.3)['joy_throttle'] is None and at(.3)['manual_source'] == 'unknown'
    calculate(dict(choice, kp=12.))
    assert len(calls) == 1  # Retuning reuses the bag input index.
    original.write_bytes(b'replaced')
    assert not api.get(base, params=choice).json()['ready']
    # A fresh pose outside the allowable projection distance yields no PlanningStatus
    # speeds on Jetson. The replay must clear the matched pair at the same observation.
    trajectory.write_text(trajectory.read_text().replace(',0.5,0.125,', ',0.5,20.0,', 1))
    build(api, 'raceline')
    calculate(choice)
    rejected = at(.2)
    assert not rejected['valid'] and rejected['estimated_speed_mps'] is None
    assert rejected['calculated_raw'] is None and rejected['propo_raw'] is None and rejected['joy_throttle'] is None


def test_remove_only_selected_analysis_or_entire_validation(history):
    from atd_api import speed_replay
    api, store, folder, _ = history
    choice = speed_settings(store)
    speed_url = URL.replace('planning', 'speed')
    for line in ('raceline', 'centerline'):
        assert build(api, line)['ready']
    job = api.post(speed_url, json=choice).json()
    api.app.state.jobs.children[job['job_id']].wait(timeout=10)
    speed = speed_replay.source(store, 'drive', speed_replay.Comparison(**choice))
    assert speed.index.is_relative_to(folder) and speed.index.is_file()
    wrong_id = '0'*64
    assert api.delete(speed_url, params=dict(choice, index_id=wrong_id)).status_code == 400 and speed.index.exists()
    assert api.delete(speed_url, params=dict(choice, index_id=speed.index.stem)).is_success
    assert not speed.index.exists() and api.get(URL, params={'line': 'raceline'}).json()['ready']
    plan = replay.source(store, 'drive', 'raceline')
    assert api.delete(URL, params=dict(line='raceline', index_id=wrong_id)).status_code == 400 and plan.index.exists()
    assert api.delete(URL, params=dict(line='raceline', index_id=plan.index.stem)).is_success
    assert not api.get(URL, params={'line':'raceline'}).json()['ready']

    # An old worker put the selected result in the shared index directory.
    src = replay.source(store, 'drive', 'centerline')
    legacy = store.state_root / 'planning' / src.index.name
    legacy.parent.mkdir(exist_ok=True)
    src.index.rename(legacy)
    assert api.get(URL, params={'line':'centerline'}).json()['ready']
    assert api.delete('/api/analysis-results/localization/drive').is_success
    assert not folder.exists() and not legacy.exists() and store.folder('course').is_dir()
    assert api.get('/api/localization/runs').json()['items'] == []
    assert api.get('/api/speed/tables').json()['items'][0]['proposal_sha256'] == choice['table_sha256']

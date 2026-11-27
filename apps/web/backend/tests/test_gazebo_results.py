import hashlib
import json
import math

from fastapi.testclient import TestClient
import pytest

from atd_api.main import create_app
from atd_processing.gazebo_review import DisplaySeries, GroundTruth, LEGACY_FOOTPRINT, PoseComparison

URL = '/api/simulation/gazebo-runs'
RUN = '/odaiba/contact'
LEFT = [[0, 1], [3, 1], [3, 2], [0, 2]]
RIGHT = [[-2, -1], [5, -1], [5, 4], [-2, 4]]


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def log(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(row) + '\n' for row in data))


def pose(ns, x, y, yaw):
    return dict(image_stamp_ns=ns, received_sim_ns=ns + 20_000_000, state='tracking', session_id='one', epoch=1,
                x_m=x, y_m=y, yaw_rad=yaw, orientation_xyzw=[0, 0, math.sin(yaw/2), math.cos(yaw/2)])


@pytest.fixture
def archive(tmp_path):
    app = create_app(tmp_path/'record', tmp_path/'results')
    base = app.state.store.result_root
    folder = base/'runs/gazebo/odaiba/contact'
    save(folder/'closed_loop.result.json', dict(status='off_course', finish_sim_s=1.15, laps_completed=0, laps_requested=2))
    save(folder/'closed_loop.host.json', dict(map_id='map1', line='raceline', started=50))
    save(folder/'scene/scene.json', dict(course_evaluation=dict(frame='gazebo_world', left=LEFT, right=RIGHT,
                                                                 footprint=LEGACY_FOOTPRINT)))
    doc = dict(id='map1', frame='map', closed=True,
               lines=dict(raceline=dict(points=[[10, 20], [10, 21], [11, 20]], speeds=[.5, 2, 1.2])))
    save(folder/'hdmap.json', doc)
    digest = hashlib.sha256((folder/'hdmap.json').read_bytes()).hexdigest()
    save(folder/'closed_loop.reference.json', dict(map_id='map1', frame='map', line='raceline', hdmap_sha256=digest))
    loc = [pose(1_025_000_000, 10, 20, math.pi/2), pose(1_075_000_000, 9.88, 20.05, math.pi/2+.2)]
    log(folder/'closed_loop.localization.jsonl', [loc[0], dict(state='stale'), loc[1]])
    log(folder/'closed_loop.vslam.jsonl', [dict(image_stamp_ns=1_075_000_000, vo_state=1,
                                                 track_execution_time_s=.004, node_callback_execution_time_s=.007)])
    samples = []
    for i in range(4):
        ns = 1_000_000_000 + i*50_000_000
        state = dict(state='waiting')
        if i >= 1:
            latest = loc[min(i-1, 1)]
            state = dict(latest, state='driving', line='raceline', map_id='map1',
                         stamp_ns=latest['image_stamp_ns'] + 12_000_000, lateral_error_m=.5)
        image = state.get('image_stamp_ns')
        samples.append(dict(sim_time_s=ns/1e9, x_m=i*.05, y_m=-.95 if i == 3 else 0, yaw_rad=0,
                            speed_mps=1.1, steering_rad=.1, requested_steering_rad=.2,
                            command_stamp_ns=image, command_applied_ns=image+18_000_000 if image else None,
                            actuator_state=state['state'], control_state=state))
    log(folder/'scene/rig_motion.jsonl', samples)
    with TestClient(app) as client:
        yield client, folder, base


def test_contact_review_uses_image_stamp_and_archived_line(archive):
    api, folder, base = archive
    save(base/'maps/map1/hdmap/map.json', {})
    assert api.get(URL).json()['items'][0]['id'] == folder.name
    response = api.get(URL+RUN)
    assert response.status_code == 200, response.text
    data = response.json()
    assert data['reference_source'] == 'snapshot'
    assert data['reference'][1] == pytest.approx([1.025, 0])
    assert data['reference_speeds_mps'] == [.5, 2, 1.2]
    assert data['anchor_s'] == 1.025 and data['contact_s'] == 1.15
    a, b = data['estimates']
    assert b['x'] == pytest.approx(.075)
    assert b['lateral_error_m'] == pytest.approx(.12)
    assert b['yaw_error_rad'] == pytest.approx(.2)
    assert b['received_delay_ms'] == 20
    assert a['segment'] != b['segment']
    s = data['samples'][1]
    assert (s['steering_rad'], s['requested_steering_rad']) == (.1, .2)
    assert s['clearance_m'] == pytest.approx(.905)
    assert data['samples'][-1]['clearance_m'] == 0
    assert (s['image_to_control_ms'], s['image_to_apply_ms'], s['command_age_ms']) == (12, 18, 25)
    assert s['control_image_s'] == 1.025 and s['tracking_lateral_error_m'] == .5
    assert data['vslam'] == [dict(t=1.075, track_ms=4, callback_ms=7, vo_state=1)]


def test_runtime_switch_preserves_selected_and_controller_lines_at_contact(archive):
    api, folder, _ = archive
    doc = json.loads((folder/'hdmap.json').read_bytes())
    doc['lines']['centerline'] = dict(points=[[9, 20], [9, 21], [10, 20]])
    save(folder/'hdmap.json', doc)
    meta = json.loads((folder/'closed_loop.reference.json').read_bytes())
    meta['hdmap_sha256'] = hashlib.sha256((folder/'hdmap.json').read_bytes()).hexdigest()
    save(folder/'closed_loop.reference.json', meta)
    log(folder/'closed_loop.lines.jsonl', [dict(stamp_ns=0, line='raceline'),
                                           dict(stamp_ns=1_100_000_000, line='centerline'),
                                           dict(stamp_ns=1_105_000_000, line='centerline')])
    data = api.get(URL+RUN).json()
    assert data['run']['line'] == 'raceline'
    assert data['line_changes'] == [dict(t=1.1, line='centerline')]
    assert data['additional_references']['centerline']['points'][0] == pytest.approx([.025, 1.])
    # The choice may change before the next planner output/actuator command.
    assert data['samples'][-1]['control_line'] == 'raceline'
    assert data['reference_speeds_mps'] == [.5, 2, 1.2]


def test_legacy_reads_mesh_and_does_not_present_current_map_as_archive(archive):
    api, folder, base = archive
    (base/'maps/map1/hdmap').mkdir(parents=True)
    (folder/'hdmap.json').rename(base/'maps/map1/hdmap/map.json')
    for name in ('closed_loop.localization.jsonl', 'closed_loop.vslam.jsonl', 'closed_loop.reference.json',
                 'closed_loop.host.json', 'scene/scene.json'):
        (folder/name).unlink()
    trace = folder/'scene/rig_motion.jsonl'
    samples = [json.loads(row) for row in trace.read_text().splitlines()]
    for row in samples:
        del row['command_applied_ns']
    log(trace, samples)
    with trace.open('a') as stream:
        stream.write('{"incomplete":')
    with (folder/'scene/floor.obj').open('w') as stream:
        for i in range(4):
            j = (i+1) % 4
            for x, y in (LEFT[i], RIGHT[i], RIGHT[j], LEFT[j]):
                stream.write(f'v {x} {y} 0\n')
    response = api.get(URL+RUN)
    assert response.status_code == 200, response.text
    data = response.json()
    assert data['run']['map_id'] == 'map1' and data['run']['line'] == 'raceline'
    assert len(data['estimates']) == 2
    assert data['reference_source'] == 'current_map' and data['reference']
    assert data['estimates'][1]['received_delay_ms'] is None
    assert data['samples'][1]['clearance_m'] == pytest.approx(.905)
    assert data['samples'][1]['image_to_apply_ms'] is None
    assert data['samples'][1]['image_to_control_ms'] == 12
    assert data['vslam'] == []
    assert any('末尾行' in msg for msg in data['warnings'])


def test_no_tracking_and_no_trace_have_no_fabricated_contact(archive):
    api, _, base = archive
    failed = base/'runs/gazebo/odaiba/boot'
    save(failed/'closed_loop.host.json', dict(status='error', reason='localize failed'))
    save(failed.parent/'map/run.json', dict(mode='map'))
    assert len(api.get(URL).json()['items']) == 2
    data = api.get(URL+'/odaiba/boot').json()
    assert data['run']['reason'] == 'localize failed'
    assert data['contact_s'] is None and data['anchor_s'] is None
    assert data['samples'] == data['estimates'] == data['reference'] == []


def test_hash_verification_and_confined_paths(archive):
    api, folder, base = archive
    (folder/'hdmap.json').write_text('{}')
    data = api.get(URL+RUN).json()
    assert data['reference_source'] == 'missing' and data['estimates']
    (folder.parent/'alias').symlink_to(folder, target_is_directory=True)
    assert api.get(URL+'/odaiba/alias').status_code == 400
    assert [row['id'] for row in api.get(URL).json()['items']] == ['contact']
    (folder/'closed_loop.localization.jsonl').unlink()
    (folder/'closed_loop.localization.jsonl').symlink_to(base.parent/'elsewhere')
    assert api.get(URL+RUN).status_code == 400


def test_verified_current_map_and_noncontact_finish(archive):
    api, folder, base = archive
    path = base/'maps/map1/hdmap/map.json'
    path.parent.mkdir(parents=True)
    (folder/'hdmap.json').rename(path)
    save(folder/'closed_loop.result.json', dict(status='completed', finish_sim_s=1.15,
                                               laps_completed=1, laps_requested=1, lap_times_s=[.15]))
    data = api.get(URL+RUN).json()
    assert data['contact_s'] is None
    assert data['run']['lap_times_s'] == [.15]
    assert data['reference_source'] == 'verified_map'
    assert data['reference_speeds_mps'] == [.5, 2, 1.2]
    doc = json.loads(path.read_bytes())
    doc['lines']['raceline']['points'][1][0] += .3
    doc['lines']['raceline']['speeds'][1] = 3
    save(path, doc)
    current = api.get(URL+RUN).json()
    assert current['reference_source'] == 'current_map'
    assert current['reference_speeds_mps'] == [.5, 3, 1.2]


@pytest.mark.parametrize('speeds,profile,expected', [
    (None, 'missing', []), (None, 'aligned', [.5, 2, 1.2]), (None, 'misaligned', []),
    ([1, 2], 'aligned', []), ([.5, None, 1.2], 'aligned', []), ([.5, -1, 1.2], 'aligned', []),
])
def test_legacy_or_invalid_speed_profile_keeps_the_reference_geometry(archive, speeds, profile, expected):
    api, folder, _ = archive
    doc = json.loads((folder/'hdmap.json').read_bytes())
    line = doc['lines']['raceline']
    if profile != 'missing':
        stations = [0, 1, 1 + math.sqrt(2)]
        line['profile'] = [[s, x, y, 0, 0, v, 0] for s, (x, y), v in zip(stations, line['points'], line['speeds'])]
        if profile == 'misaligned':
            line['profile'][1][1] += 1
    if speeds is None:
        del line['speeds']
    else:
        line['speeds'] = speeds
    save(folder/'hdmap.json', doc)
    meta = json.loads((folder/'closed_loop.reference.json').read_bytes())
    meta['hdmap_sha256'] = hashlib.sha256((folder/'hdmap.json').read_bytes()).hexdigest()
    save(folder/'closed_loop.reference.json', meta)
    data = api.get(URL+RUN).json()
    assert data['reference_source'] == 'snapshot' and len(data['reference']) == 3
    assert data['reference_speeds_mps'] == expected


def test_no_extrapolation_wrap_yaw_and_protect_contact_samples():
    truth = GroundTruth()
    truth.append(1_000_000_000, 0, 0, math.radians(179))
    truth.append(1_100_000_000, 1, 0, math.radians(-179))
    truth.append(2_000_000_000, 2, 0, 0)
    assert truth.at(1_050_000_000)[2] == pytest.approx(math.pi)
    compare = PoseComparison(truth, None)
    for ns in (500_000_000, 1_500_000_000, 2_100_000_000):
        compare.observe(pose(ns, 0, 0, 0))
    assert compare.anchor is None
    series = DisplaySeries(contact=9000)
    for t in range(20000):
        series.append(dict(t=t))
    shown = [s['t'] for s in series.result()]
    assert 0 == shown[0] and shown[-1] == 19999 and len(shown) < 8300
    assert set(range(8990, 9011)).issubset(shown)


def test_parallel_ekf_and_issued_steering_at_image_time(archive):
    api, folder, _ = archive
    identity = json.loads((folder/'closed_loop.reference.json').read_text())
    save(folder/'closed_loop.reference.json', dict(identity, map_sha256='a'*64))
    t, d = 1_025_000_000, 50_000_000
    loc = [pose(t, 10, 20, math.pi/2), pose(t+d, 9.8, 20.05, math.pi/2)]
    log(folder/'closed_loop.localization.jsonl',
        [dict(r, header_stamp_ns=r['image_stamp_ns'], z_m=0.) for r in loc])

    def odom(ns, frame, x):
        return dict(stamp_ns=ns, frame=frame, xyz=[x,0.,0.], xyzw=[0.,0.,0.,1.])
    log(folder/'closed_loop.ekf.jsonl', [odom(t-d//2+i*d, 'wheel', i*.05) for i in range(3)])
    log(folder/'closed_loop.vo.jsonl', [odom(t+i*d, 'odom', i*.05) for i in range(2)])
    log(folder/'closed_loop.commands.jsonl', [dict(stamp_ns=t+d, steering_rad=.31)])
    data = api.get(URL+RUN).json()
    a, b = data['ekf']
    assert [p['x'] for p in data['ekf_path']] == pytest.approx([.05, .1])
    assert a['steering_rad'] is None and b['steering_rad'] == .31
    assert a['correction_left_m'] is None and b['correction_left_m'] == pytest.approx(.2)
    assert b['x'] == pytest.approx(.075) and b['y'] == pytest.approx(0.)
    assert data['estimates'][1]['y'] == pytest.approx(.2)

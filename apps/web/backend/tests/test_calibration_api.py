"""実際の API・worker・成果物配信を通し、外部の ROS コマンドだけを置き換える。"""
from hashlib import sha256
import os

import pytest
from fastapi.testclient import TestClient

from atd_api.main import create_app
from atd_api.store import atomic_json


@pytest.mark.parametrize('new_pose', [True, False], ids=['generate', 'reuse-legacy'])
def test_calibration_job_publishes_diagnostics_after_reopening(tmp_path, monkeypatch, new_pose):
    record, results = tmp_path / 'record', tmp_path / 'results'
    bag = record / 'rosbags/day1/short'
    bag.mkdir(parents=True)
    metadata = b'test metadata'
    (bag / 'metadata.yaml').write_bytes(metadata)
    if not new_pose:
        source = dict(kind='nvidia_offline', bag='day1/short', metadata_sha256=sha256(metadata).hexdigest(),
                      inputs=dict(base_frame='base_link'))
        folder = results / 'maps/test-map'
        atomic_json(folder / 'mapping.json', dict(source=source))
        poses = folder / 'nvidia/poses/odom_poses.tum'
        poses.parent.mkdir(parents=True)
        poses.write_text('1 0 0 0 0 0 0 1\n')

    # Use the regular worker subprocess, without sourcing a host ROS installation.
    commands = tmp_path / 'bin'
    commands.mkdir()
    shell = commands / 'bash'
    shell.write_text('#!/bin/sh\n[ "$3" = atd-ros-worker ] || exit 99\nshift 3\nexec "$@"\n')
    shell.chmod(0o755)
    ros = commands / 'ros2'
    ros.write_text('''#!/usr/bin/env python3
from hashlib import sha256
from pathlib import Path
import json
import sys

args = sys.argv[1:]
if args[:3] == ['run', 'isaac_mapping_ros', 'create_map_offline.py']:
    root = Path(next(flag.split('=', 1)[1] for flag in args if flag.startswith('--base_output_folder='))) / 'run'
    (root / 'cuvslam_map').mkdir(parents=True)
    (root / 'cuvslam_map/test.mdb').write_bytes(b'map')
    (root / 'poses').mkdir()
    (root / 'poses/odom_poses.tum').write_text('1 0 0 0 0 0 0 1\\n')
    sys.exit(0)
if args[:2] != ['run', 'atd_vehicle_calibration']:
    sys.exit(99)
if args[2] == 'check_bag' or args == ['run', 'atd_vehicle_calibration', 'analyze', '--help']:
    sys.exit(0)
def arg(name):
    return args[args.index(name) + 1]
output, bag = Path(arg('--output')), Path(arg('--bag'))
output.mkdir()
source = dict(bag=str(bag), poses=arg('--poses'), pose_frame=arg('--pose-frame'),
              metadata_sha256=sha256((bag / 'metadata.yaml').read_bytes()).hexdigest(),
              poses_sha256=sha256(Path(arg('--poses')).read_bytes()).hexdigest())
summary = dict(schema_version=1, source=source, analysis={}, eligible_samples=0, timeseries_samples=8)
(output / 'summary.json').write_text(json.dumps(summary))
(output / 'timeseries.csv').write_text('stamp,valid\\n1,false\\n')
(output / 'response.png').write_bytes(b'test png body')
sys.exit(2)
''')
    ros.chmod(0o755)
    monkeypatch.setenv('PATH', str(commands) + os.pathsep + os.environ['PATH'])
    with TestClient(create_app(record, results)) as api:
        options = api.get('/api/calibration/options', params={'bag': 'day1/short'})
        assert options.status_code == 200
        assert [p['map_id'] for p in options.json()['poses']] == ([] if new_pose else ['test-map'])
        request = {'bag': 'day1/short'}
        if not new_pose:
            assert api.post('/api/calibration/runs', json=request).status_code == 400
            request['pose_frame'] = 'camera_link'
        started = api.post('/api/calibration/runs', json=request)
        assert started.status_code == 200, started.text
        job_id = started.json()['id']
        api.app.state.jobs.children[job_id].wait(timeout=20)

    with TestClient(create_app(record, results)) as reopened:
        state = reopened.get('/api/jobs/' + job_id).json()
        assert state['status'] == 'succeeded', reopened.get(f'/api/jobs/{job_id}/log').text
        assert state['result']['outcome'] == 'no_eligible_samples'
        assert state['result']['pose_reused'] == (not new_pose)
        report = reopened.get('/api/calibration/options').json()['runs'][0]
        assert report['id'] == state['result']['run_id'] and report['bag'] == 'day1/short'
        root = '/api/calibration/runs/' + report['id'] + '/'
        assert reopened.get(root + 'response.png').headers['content-type'] == 'image/png'
        table = reopened.get(root + 'timeseries.csv')
        assert table.text.startswith('stamp,valid') and 'attachment' in table.headers['content-disposition']
        summary = reopened.get(root + 'summary.json').json()
        assert summary['eligible_samples'] == 0
        assert summary['source']['pose_frame'] == ('base_link' if new_pose else 'camera_link')
        assert reopened.get(root + 'metadata.yaml').status_code == 400

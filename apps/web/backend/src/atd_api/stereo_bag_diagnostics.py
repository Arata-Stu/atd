"""Notebook に保存済みの rosbag のステレオ欠落診断。"""
import json
from pathlib import Path
from tempfile import TemporaryDirectory

from fastapi import APIRouter, Request
from pydantic import Field

from .manual_drive_replay import Selection, recording


router = APIRouter(prefix='/stereo-bag-diagnostics')


class Diagnosis(Selection):
    fps: int = Field(ge=1, le=120)


@router.post('')
def start(request: Request, choice: Diagnosis):
    store, jobs = request.app.state.store, request.app.state.jobs
    source = recording(store, choice.bag)
    return jobs.start('stereo_bag_diagnostics', dict(bag=source.bag, fps=choice.fps, resource=source.resource),
                      ['record:' + source.bag], 'ステレオ画像の欠落診断 · ' + source.bag)


def execute(store, task, payload):
    source = recording(store, payload['bag'])
    if source.resource != payload['resource']:
        raise ValueError('選択後に bag が変更されました')
    with TemporaryDirectory(prefix='atd-stereo-bag-') as working:
        output = Path(working) / 'diagnosis.json'
        task.run(['ros2', 'run', 'atd_web_bridge', 'diagnose_stereo_bag', '--bag', str(source.directory),
                  '--fps', str(payload['fps']), '--output', str(output)])
        result = json.loads(output.read_text())
    if recording(store, source.bag).resource != source.resource:
        raise ValueError('診断中に bag が変更されました')
    task.check()
    return dict(bag=source.bag, **result)

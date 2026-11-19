"""Notebook の loopback API とビルド済み Web UI。"""
from __future__ import annotations

import math
import os
from pathlib import Path
import shutil
import time

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field

from atd_processing.geometry import settings as generation_settings
from .jobs import Jobs
from .offline import DEFAULT_INPUTS, build_payload, vgl_payload
from .store import Store, atomic_json, confined, read_json
from . import transfers
from . import capture as capture_service
from . import calibration


class Strict(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)


class CreateMap(Strict):
    id: str
    title: str = ''


class Generate(Strict):
    revision: int = Field(ge=1)
    kind: str
    settings: dict = Field(default_factory=dict)


class Connection(Strict):
    host: str
    user: str
    port: int = 22
    record_root: str
    map_root: str
    bridge_port: int = 8790


class Relative(Strict):
    relative: str = ''


class Push(Strict):
    id: str


class VGL(Strict):
    model_dir: str


class Capture(Strict):
    revision: int = Field(ge=1)
    bag: str | None = None
    model_dir: str | None = None


class CalibrationRun(Strict):
    bag: str
    pose_frame: str | None = Field(default=None, min_length=1)
    map_id: str | None = None
    wheelbase_m: float | None = Field(default=None, gt=0, allow_inf_nan=False)
    reference_run: str | None = None


class AdoptCalibration(Strict):
    revision: int = Field(ge=1)
    run_id: str
    proposal_sha256: str = Field(pattern=r'^[0-9a-f]{64}$')
    fields: list[str] = Field(min_length=1, max_length=4)


class Build(Strict):
    id: str
    bag: str
    left_image: str = DEFAULT_INPUTS['left_image']
    right_image: str = DEFAULT_INPUTS['right_image']
    left_info: str = DEFAULT_INPUTS['left_info']
    right_info: str = DEFAULT_INPUTS['right_info']
    base_frame: str = DEFAULT_INPUTS['base_frame']
    left_frame: str = DEFAULT_INPUTS['left_frame']
    right_frame: str = DEFAULT_INPUTS['right_frame']


def create_app(record_root=None, result_root=None, static_dir=None):
    app = FastAPI(title='ATD Studio', version='0.1.0')
    # record_root は既存 mount の基底で、bag の保存単位はその rosbags/ 配下。
    store = Store(Path(record_root or os.environ.get('ATD_WEB_RECORD_ROOT', '/data/record')) / 'rosbags',
                  result_root or os.environ.get('ATD_WEB_RESULT_ROOT', '/data/results'))
    app.state.store = store
    app.state.jobs = jobs = Jobs(store)

    @app.exception_handler(ValueError)
    async def invalid(_request, error):
        return JSONResponse(status_code=400, content={'detail': str(error)})

    @app.exception_handler(OSError)
    async def io_error(_request, error):
        return JSONResponse(status_code=503, content={'detail': 'ファイル・外部処理を確認してください: ' + str(error)})

    @app.middleware('http')
    async def local_request(request: Request, call_next):
        host = request.headers.get('host', '').split(':')[0]
        if host not in ('127.0.0.1', 'localhost', 'testserver'):
            return JSONResponse(status_code=403, content={'detail': 'loopback 専用です'})
        origin = request.headers.get('origin')
        if origin and origin not in ('http://' + request.headers.get('host', ''), 'https://' + request.headers.get('host', '')):
            return JSONResponse(status_code=403, content={'detail': '異なる Origin の要求を拒否しました'})
        if request.method in ('POST', 'PUT', 'DELETE', 'PATCH'):
            if request.headers.get('content-type', '').split(';')[0] != 'application/json' and request.headers.get('content-length', '0') != '0':
                return JSONResponse(status_code=415, content={'detail': 'application/json を指定してください'})
            try:
                if int(request.headers.get('content-length', '0')) > 32 * 1024 * 1024:
                    return JSONResponse(status_code=413, content={'detail': '要求が大きすぎます'})
            except ValueError:
                return JSONResponse(status_code=400, content={'detail': 'Content-Length が不正です'})
        response = await call_next(request)
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Cache-Control'] = 'no-store'
        response.headers['Referrer-Policy'] = 'same-origin'
        return response

    @app.get('/api/health')
    def health():
        return dict(status='ok', service='atd-web', timestamp=time.time())

    @app.get('/api/config')
    def config():
        return dict(record_root=str(store.record_root), result_root=str(store.result_root), map_root=str(store.maps_root),
                    capabilities=dict(ros=bool(shutil.which('ros2')), ssh=bool(shutil.which('ssh')), scp=bool(shutil.which('scp'))),
                    mapping_defaults=DEFAULT_INPUTS)

    @app.get('/api/connection')
    def get_connection():
        return store.connection()

    @app.put('/api/connection')
    def set_connection(body: Connection):
        return store.save_connection(body.model_dump())

    @app.get('/api/records')
    def records():
        return dict(items=store.records())

    @app.get('/api/calibration/options')
    def calibration_options(bag: str | None = None):
        return calibration.options(store, bag)

    @app.post('/api/calibration/runs')
    def start_calibration(body: CalibrationRun):
        payload = calibration.payload(store, body.model_dump(exclude_none=True))
        return jobs.start('calibration', payload, calibration.resources(payload), '車両校正: ' + payload['bag'])

    @app.get('/api/calibration/runs/{run_id}/candidates')
    def calibration_candidates(run_id: str):
        return calibration.candidates(store, run_id)

    @app.get('/api/calibration/runs/{run_id}/feedforward.json')
    def calibration_feedforward(run_id: str, proposal_sha256: str):
        return JSONResponse(content=calibration.feedforward_document(store, run_id, proposal_sha256),
                            headers={'Content-Disposition': f'attachment; filename="throttle-ff-{run_id}.json"'})

    @app.get('/api/calibration/runs/{run_id}/{filename}')
    def calibration_artifact(run_id: str, filename: str):
        path = calibration.artifact(store, run_id, filename)
        if filename == 'response.png':
            return FileResponse(path, media_type='image/png')
        media_type = 'application/json' if filename == 'summary.json' else 'text/csv'
        return FileResponse(path, media_type=media_type, filename=filename)

    @app.get('/api/maps')
    def maps():
        return dict(items=store.list_maps())

    @app.post('/api/maps')
    def create_map(body: CreateMap):
        return store.create_map(body.id, body.title)

    @app.post('/api/maps/build')
    def build(body: Build):
        payload = build_payload(store, body.model_dump())
        return jobs.start('map_build', payload, ['map:' + body.id, 'ros-domain:93', 'offline-mapping'], '公式 cuVSLAM 地図作成: ' + body.id)

    @app.get('/api/maps/{key}')
    def load_map(key: str):
        return store.load_map(key)

    @app.get('/api/maps/{key}/live-reference')
    def live_reference(key: str):
        return store.live_reference(key)

    @app.put('/api/maps/{key}')
    def save_map(key: str, body: dict):
        with jobs.registry():
            jobs.require_available(['map:' + key])
            return store.save_map(key, body)

    @app.post('/api/maps/{key}/raceline-calibration')
    def adopt_calibration(key: str, body: AdoptCalibration):
        proposal = calibration.reviewed_candidates(store, body.run_id, body.proposal_sha256)
        adoption = calibration.selected_limits(proposal, body.fields)
        with jobs.registry():
            jobs.require_available(['map:' + key])
            return store.apply_raceline_calibration(key, body.revision, adoption)

    @app.delete('/api/maps/{key}/raceline-calibration')
    def remove_calibration(key: str, revision: int):
        with jobs.registry():
            jobs.require_available(['map:' + key])
            return store.apply_raceline_calibration(key, revision, None)

    @app.post('/api/maps/{key}/generate')
    def generate(key: str, body: Generate):
        document = store.load_map(key)
        if body.kind not in ('centerline', 'raceline', 'custom'):
            raise ValueError('未知のライン種別です')
        if document['revision'] != body.revision:
            raise ValueError('地図を保存してから生成してください')
        if not capture_service.ready_document(document):
            raise ValueError('保存地図から点群を取得してからラインを生成してください')
        generation_settings(body.settings)
        return jobs.start('generate', dict(id=key, **body.model_dump()), ['map:' + key], body.kind + ' 生成: ' + key)

    @app.post('/api/maps/{key}/export')
    def export(key: str):
        from .exports import export_map
        with jobs.registry():
            jobs.require_available(['map:' + key])
            return export_map(store, key)

    @app.post('/api/maps/{key}/vgl')
    def vgl(key: str, body: VGL):
        payload = vgl_payload(store, key, body.model_dump())
        return jobs.start('vgl', payload, ['map:' + key, 'ros-domain:93', 'offline-mapping'], '公式 VGL 地図準備: ' + key)

    @app.get('/api/maps/{key}/capture')
    def capture_options(key: str, bag: str | None = None, model_dir: str | None = None):
        return capture_service.preflight(store, key, bag, model_dir)

    @app.post('/api/maps/{key}/capture')
    def capture_map(key: str, body: Capture):
        payload = capture_service.payload(store, key, body.model_dump(exclude_none=True))
        return jobs.start('capture', payload, ['map:' + key, 'ros-domain:94', 'offline-mapping', 'record:' + payload['bag']],
                          '保存地図から点群取得: ' + key)

    @app.get('/api/maps/{key}/cloud')
    def cloud(key: str):
        folder = store.folder(key)
        document = store.load_map(key)
        source = document.get('source', {})
        if not capture_service.ready_document(document):
            return dict(points=[], frame='map', source_count=0, truncated=False, status='pending',
                        trajectory=[], message='保存地図は作成済みです。VGL の準備後に点群を取得してください')
        relative = source.get('capture_relative')
        cloud_relative = relative + '/cloud.json' if relative else 'hdmap/cloud.json'
        path = confined(folder, cloud_relative, exists=False)
        if not path.is_file():
            return dict(points=[], frame='map', source_count=0, truncated=False, status='unavailable', message='表示用点群は未取得です')
        data = read_json(path)
        all_points = data['points']
        stride = max(1, math.ceil(len(all_points) / 250000))
        source_count = data.get('source_count', len(all_points))
        return dict(points=all_points[::stride], frame='map', source_count=source_count,
                    truncated=source_count > len(all_points[::stride]), status='ready',
                    trajectory=data.get('trajectory', []), provenance=data.get('provenance', {}))

    @app.post('/api/maps/{key}/cloud')
    def import_cloud(key: str, body: dict):
        # map frame の明示インポートのみ。odom や単位の推測で変換しない。
        if body.get('frame') != 'map' or not isinstance(body.get('points'), list) or len(body['points']) > 1000000:
            raise ValueError('100万点以内の map 座標点群を指定してください')
        for point in body['points']:
            if not isinstance(point, list) or len(point) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in point):
                raise ValueError('点群は有限の [x,y,z] メートル座標で指定してください')
        with jobs.registry(), store.lock('map-' + key):
            jobs.require_available(['map:' + key])
            document = store.load_map(key)
            if document.get('source', {}).get('kind') == 'nvidia_offline':
                raise ValueError('NVIDIA 保存地図の点群は「保存地図から点群取得」で準備してください')
            if body.get('revision') != document['revision']:
                raise ValueError('地図 revision が変わりました')
            atomic_json(store.folder(key) / 'hdmap/cloud.json', dict(frame='map', points=body['points'], provenance='explicit_import'))
            document['source']['pointcloud_status'] = 'ready'
            document.update(revision=document['revision'] + 1, updated=time.time())
            atomic_json(store.folder(key) / 'hdmap/map.json', document)
        return document

    @app.post('/api/transfer/browse')
    def browse(body: Relative):
        p = transfers.connection(store.connection())
        return transfers.request(p, 'browse', p['record_root'], body.relative)

    @app.post('/api/transfer/pull')
    def pull(body: Relative):
        p = transfers.connection(store.connection())
        relative = transfers.relative(body.relative)
        if not relative or relative == '.':
            raise ValueError('bag ディレクトリを選択してください')
        if confined(store.record_root, relative, False).exists():
            raise ValueError('受信先が既に存在します')
        return jobs.start('pull', dict(connection=p, relative=relative), ['transfer', 'record:' + relative], 'SCP bag 受信: ' + relative)

    @app.post('/api/transfer/push')
    def push(body: Push):
        p = transfers.connection(store.connection())
        store.load_map(body.id)
        return jobs.start('push', dict(connection=p, id=body.id), ['transfer', 'map:' + body.id], 'SCP 地図送信: ' + body.id)

    @app.get('/api/jobs')
    def job_list():
        return dict(items=jobs.list())

    @app.get('/api/jobs/{key}')
    def job_get(key: str):
        return jobs.get(key)

    @app.get('/api/jobs/{key}/log')
    def job_log(key: str, offset: int = 0):
        return jobs.log(key, offset)

    @app.post('/api/jobs/{key}/cancel')
    def job_cancel(key: str):
        return jobs.cancel(key)

    from .localization_validation import router as validation_router
    from .localization_results import router as validation_results_router
    app.include_router(validation_router, prefix='/api')
    app.include_router(validation_results_router, prefix='/api')
    from .planning_replay import router as planning_replay_router
    app.include_router(planning_replay_router, prefix='/api')
    from .speed_replay import router as speed_replay_router
    app.include_router(speed_replay_router, prefix='/api')
    from .speed_tables import router as speed_tables_router
    app.include_router(speed_tables_router, prefix='/api')
    from .manual_drive_replay import router as manual_drive_replay_router
    app.include_router(manual_drive_replay_router, prefix='/api')
    from .live_input import router
    app.include_router(router, prefix='/api')
    static = Path(static_dir or os.environ.get('ATD_WEB_STATIC_DIR', Path(__file__).resolve().parents[3] / 'frontend/dist'))
    if (static / 'index.html').is_file():
        if (static / 'assets').is_dir():
            app.mount('/assets', StaticFiles(directory=static / 'assets'), name='assets')

        @app.get('/favicon.svg')
        def favicon():
            if not (static / 'favicon.svg').is_file():
                raise HTTPException(status_code=404, detail='favicon がありません')
            return FileResponse(static / 'favicon.svg', media_type='image/svg+xml')

        @app.get('/')
        def home():
            return FileResponse(static / 'index.html')
    else:
        @app.get('/')
        def build_required():
            return JSONResponse(status_code=503, content={'detail': 'frontend が未buildです。scripts/tools/webui.sh を使って準備してください'})
    return app


# import だけで /data を作らない。factory=True で uvicorn から起動する。

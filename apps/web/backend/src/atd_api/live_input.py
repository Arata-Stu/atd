"""Live observations and input/role configuration; no driving commands."""
from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request

from .jetson_bridge import BridgeUnavailable, request as bridge_request

router = APIRouter()


def call(request: Request, path: str, method: str = 'GET', body: dict | None = None):
    try:
        return bridge_request(request.app.state.store.connection(), path, method, body)
    except BridgeUnavailable as exc:
        if method == 'GET' and path == '/esc/status':
            return {'connected': False, 'enabled': False, 'trial': None, 'message': str(exc)}
        if method == 'GET' and path == '/recording/status':
            return {'connected': False, 'status': None, 'message': str(exc),
                    'availability': {'bridge': 'disconnected', 'status': 'missing'}}
        if method == 'GET' and path.endswith('/status'):
            status = {'connected': False, 'device_connected': False, 'ready': False,
                      'pose': None, 'velocity': None, 'raw': None, 'normalized': None,
                      'status': None, 'configuring': None,
                      'board': None, 'input': None, 'rc_valid': False,
                      'message': str(exc), 'availability': {'bridge': 'disconnected'}}
            if path == '/input/status':
                status['drive_monitor'] = dict(joy=None, board=None, joy_reason=str(exc), board_reason=str(exc))
            if path == '/live/status':
                status['auto_drive'] = None
            return status
        raise HTTPException(status_code=503, detail=str(exc)) from exc


@router.get('/live/status')
def live_status(request: Request):
    return call(request, '/live/status')


@router.get('/live/parameters')
def parameters(request: Request, node: str = ''):
    return call(request, '/live/parameters', 'POST', {'operation': 'get', 'node': node})


@router.post('/live/parameters')
def set_parameters(request: Request, body: dict):
    if not isinstance(body.get('node'), str) or not isinstance(body.get('values'), dict) or not body['values']:
        raise HTTPException(status_code=422, detail='node と values の指定が必要です')
    return call(request, '/live/parameters', 'POST', {**body, 'operation': 'set'})


@router.get('/input/status')
def input_status(request: Request):
    return call(request, '/input/status')


@router.get('/input/profile')
def profile(request: Request):
    return call(request, '/input/profile')


@router.put('/input/profile')
def save_profile(request: Request, body: dict):
    return call(request, '/input/profile', 'PUT', body)


@router.post('/input/reload')
def reload_profile(request: Request):
    return call(request, '/input/reload', 'POST')


@router.post('/input/configure')
def configure(request: Request, body: dict):
    if type(body.get('enabled')) is not bool:
        raise HTTPException(status_code=422, detail='enabled は true / false で指定してください')
    return call(request, '/input/configure', 'POST', body)


@router.get('/teleop/status')
def teleop_status(request: Request):
    return call(request, '/teleop/status')


@router.get('/teleop/profile')
def teleop_profile(request: Request):
    return call(request, '/teleop/profile')


@router.put('/teleop/profile')
def save_teleop_profile(request: Request, body: dict):
    return call(request, '/teleop/profile', 'PUT', body)


@router.post('/teleop/reload')
def reload_teleop_profile(request: Request, body: dict):
    return call(request, '/teleop/reload', 'POST', body)


@router.post('/teleop/configure')
def configure_teleop(request: Request, body: dict):
    if type(body.get('enabled')) is not bool:
        raise HTTPException(status_code=422, detail='enabled は true / false で指定してください')
    return call(request, '/teleop/configure', 'POST', body)


@router.get('/propo/status')
def propo_status(request: Request):
    return call(request, '/propo/status')


@router.get('/propo/profile')
def propo_profile(request: Request):
    return call(request, '/propo/profile')


@router.put('/propo/profile')
def save_propo_profile(request: Request, body: dict):
    return call(request, '/propo/profile', 'PUT', body)


@router.post('/propo/reload')
def reload_propo_profile(request: Request, body: dict):
    return call(request, '/propo/reload', 'POST', body)


@router.post('/propo/configure')
def configure_propo(request: Request, body: dict):
    if type(body.get('enabled')) is not bool:
        raise HTTPException(status_code=422, detail='enabled は true / false で指定してください')
    return call(request, '/propo/configure', 'POST', body)


@router.get('/vehicle/status')
def vehicle_status(request: Request):
    return call(request, '/vehicle/status')


@router.get('/vehicle/profile')
def vehicle_profile(request: Request):
    return call(request, '/vehicle/profile')


@router.put('/vehicle/profile')
def save_vehicle_profile(request: Request, body: dict):
    return call(request, '/vehicle/profile', 'PUT', body)


@router.get('/recording/status')
def recording_status(request: Request):
    return call(request, '/recording/status')


@router.get('/esc/status')
def esc_status(request: Request):
    return call(request, '/esc/status')


@router.post('/esc/prepare')
def esc_prepare(request: Request, body: dict):
    return call(request, '/esc/prepare', 'POST', body)


@router.post('/esc/cancel')
def esc_cancel(request: Request, body: dict):
    return call(request, '/esc/cancel', 'POST', body)

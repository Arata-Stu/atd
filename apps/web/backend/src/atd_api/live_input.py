"""Live observations and physical joystick configuration; no driving commands."""
from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request

from .jetson_bridge import BridgeUnavailable, request as bridge_request

router = APIRouter()


def call(request: Request, path: str, method: str = 'GET', body: dict | None = None):
    try:
        return bridge_request(request.app.state.store.connection(), path, method, body)
    except BridgeUnavailable as exc:
        if method == 'GET' and path.endswith('/status'):
            return {'connected': False, 'device_connected': False, 'ready': False,
                    'pose': None, 'velocity': None, 'raw': None, 'normalized': None,
                    'message': str(exc), 'availability': {'bridge': 'disconnected'}}
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

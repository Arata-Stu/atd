"""SSH-only adapter to the Jetson-local HTTP/ROS bridge; never joins DDS."""
from __future__ import annotations

import json
import re
import shlex
import subprocess
import uuid


class BridgeUnavailable(RuntimeError):
    pass


# The remote host needs only the Python standard library, not this repository.
REMOTE = r'''
import json, sys, urllib.request, urllib.error
request = json.load(sys.stdin)
port = request['port']
if type(port) is not int or not 1024 <= port <= 65535:
    raise ValueError('Invalid bridge port')
path = request['path']
allowed = {
    '/live/status': ('GET',), '/live/parameters': ('POST',),
    '/input/status': ('GET',), '/input/profile': ('GET', 'PUT'),
    '/input/reload': ('POST',), '/input/configure': ('POST',),
    '/teleop/status': ('GET',), '/teleop/profile': ('GET', 'PUT'),
    '/teleop/reload': ('POST',), '/teleop/configure': ('POST',),
    '/propo/status': ('GET',), '/propo/profile': ('GET', 'PUT'),
    '/propo/reload': ('POST',), '/propo/configure': ('POST',),
}
if path not in allowed or request['method'] not in allowed[path]:
    raise ValueError('Unsupported bridge operation')
data = json.dumps(request.get('body', {})).encode() if request['method'] != 'GET' else None
req = urllib.request.Request('http://127.0.0.1:%d%s' % (port, path), data=data,
    method=request['method'], headers={'Content-Type':'application/json', 'X-ATD-Request':request['nonce']})
try:
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(req, timeout=7) as response:
        result = response.read(2097153)
except urllib.error.HTTPError as error:
    result = error.read(2097153)
if len(result) > 2097152:
    raise ValueError('Bridge response too large')
sys.stdout.write(result.decode())
'''


def request(connection: dict, path: str, method: str = 'GET', body: dict | None = None) -> dict:
    host, user = connection.get('host', ''), connection.get('user', '')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.-]{0,252}', str(host)):
        raise BridgeUnavailable('Jetson の SSH 接続先を設定してください')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]{0,63}', str(user)):
        raise BridgeUnavailable('Jetson の SSH ユーザーを設定してください')
    port, bridge_port = connection.get('port', 22), connection.get('bridge_port', 8790)
    if type(port) is not int or not 1 <= port <= 65535:
        raise BridgeUnavailable('SSH port が不正です')
    if type(bridge_port) is not int or not 1024 <= bridge_port <= 65535:
        raise BridgeUnavailable('Bridge port が不正です')
    nonce = uuid.uuid4().hex
    command = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
               '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=5',
               '-o', 'ServerAliveCountMax=1', '-p', str(port), f'{user}@{host}',
               'python3 -c ' + shlex.quote(REMOTE)]
    try:
        result = subprocess.run(command, input=json.dumps(dict(port=bridge_port, path=path,
            method=method, body=body or {}, nonce=nonce)), capture_output=True, text=True, timeout=14)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise BridgeUnavailable('Jetson bridge に接続できません。操作は自動再送しません') from exc
    if result.returncode:
        raise BridgeUnavailable('SSH / Jetson bridge 接続失敗: ' + result.stderr[-600:])
    try:
        value = json.loads(result.stdout)
        identity = value['identity']
        if identity.get('schema') != 'atd.web_bridge.v1' or not identity.get('instance_id') or value.get('request_id') != nonce:
            raise ValueError('identity mismatch')
    except (ValueError, KeyError, TypeError) as exc:
        raise BridgeUnavailable('Jetson bridge の応答・識別情報が一致しません') from exc
    if value.get('error'):
        raise BridgeUnavailable(value['error'])
    if not isinstance(value.get('data'), dict):
        raise BridgeUnavailable('Jetson bridge の応答データが不正です')
    return {**value['data'], 'identity': identity, 'connected': True}

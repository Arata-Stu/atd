"""鍵認証 SSH と SCP。一時配置、内容 hash 検査後に公開する。"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import time
import subprocess
import tempfile
import uuid
import unicodedata

from .store import confined, identifier

OPTIONS = ['-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=8',
           '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=3']


def connection(raw):
    p = dict(raw)
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]{0,63}', str(p.get('user', ''))):
        raise ValueError('SSH ユーザー名を設定してください')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.-]{0,252}', str(p.get('host', ''))):
        raise ValueError('SSH ホスト名または IPv4 を設定してください')
    if type(p.get('port')) is not int or not 1 <= p['port'] <= 65535:
        raise ValueError('SSH port が不正です')
    for key in ('record_root', 'map_root'):
        p[key] = data_path(p.get(key), absolute=True).rstrip('/')
    return p


def data_path(value, *, absolute=False):
    # OpenSSH 9 の SCP (SFTP 既定) へ単一 argv として渡す。shell 用の引用は加えない。
    # Unicode と空白を保持し、展開・制御・相対脱出につながる文字だけを拒否する。
    if not isinstance(value, str):
        raise ValueError('データパスは文字列で指定してください')
    forbidden = set('\\\'"`$;&|<>(){}[]*?!~')
    if any(unicodedata.category(char).startswith('C') or char in forbidden for char in value):
        raise ValueError('データパスに制御文字・shell 展開文字・wildcard は使えません')
    if '..' in value.split('/') or (not absolute and value.startswith('-')):
        raise ValueError('データパスに .. や先頭の option は使えません')
    if absolute:
        if not value.startswith('/') or not value.strip('/'):
            raise ValueError('Jetson の保存先は root 以外の絶対パスで指定してください')
    elif value.startswith('/'):
        raise ValueError('転送元は保存先からの相対パスで指定してください')
    return value


def relative(value):
    return data_path(value)


def inventory(root, check=lambda: None):
    root = Path(root)
    if root.is_symlink() or not root.is_dir():
        raise ValueError('通常のディレクトリを選択してください')
    files = {}
    for current, dirs, names in os.walk(root, followlinks=False):
        for name in dirs + names:
            path = Path(current) / name
            if path.is_symlink():
                raise ValueError('転送対象に symlink は使えません: ' + str(path))
            if path.is_file():
                digest = hashlib.sha256()
                with path.open('rb') as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                        check()
                        digest.update(chunk)
                files[path.relative_to(root).as_posix()] = dict(size=path.stat().st_size, sha256=digest.hexdigest())
            elif not path.is_dir():
                raise ValueError('特殊ファイルは転送できません')
    return files


# クライアントから任意の Python / shell を渡す窓口は提供しない。
REMOTE = r'''
import hashlib,json,os,pathlib,re,shutil,sys

def path(root,relative=''):
    root=pathlib.Path(root)
    value=pathlib.Path(relative)
    if not root.is_absolute() or value.is_absolute() or '..' in value.parts: raise ValueError('invalid path')
    target=root/value
    current=target
    while current != root:
        if current.is_symlink(): raise ValueError('symlink refused')
        current=current.parent
    if root.is_symlink(): raise ValueError('symlink root refused')
    return target

def inventory(root):
    files={}
    if root.is_symlink() or not root.is_dir(): raise ValueError('directory missing')
    for current,dirs,names in os.walk(root,followlinks=False):
        for name in dirs+names:
            p=pathlib.Path(current)/name
            if p.is_symlink(): raise ValueError('symlink refused')
            if p.is_file():
                h=hashlib.sha256()
                with p.open('rb') as f:
                    for data in iter(lambda:f.read(1024*1024),b''): h.update(data)
                files[p.relative_to(root).as_posix()]={'size':p.stat().st_size,'sha256':h.hexdigest()}
            elif not p.is_dir(): raise ValueError('special file refused')
    return files

try:
    b=json.load(sys.stdin); target=path(b['root'],b.get('relative','')); action=b['action']
    if action=='browse':
        items=[]
        for p in sorted(target.iterdir()):
            if p.name.startswith('.') or p.is_symlink() or not p.is_dir(): continue
            items.append({'name':p.name,'relative':p.relative_to(pathlib.Path(b['root'])).as_posix(),'is_dir':True,'is_bag':(p/'metadata.yaml').is_file(),'bytes':None})
        result={'items':items}
    elif action=='inventory': result={'files':inventory(target)}
    elif action=='reserve':
        if not target.name.startswith('.atd-upload-'): raise ValueError('invalid stage')
        target.mkdir(parents=False,exist_ok=False); result={'ok':True}
    elif action=='publish':
        if not target.name.startswith('.atd-upload-'): raise ValueError('invalid stage')
        source=path(target,'bundle'); dest=path(b['root'],b['destination'])
        if dest.exists(): raise ValueError('destination exists')
        if inventory(source)!=b['files']: raise ValueError('content hash mismatch')
        os.rename(source,dest); target.rmdir(); result={'map':b['destination'],'verified':True}
    elif action=='cleanup':
        if not target.name.startswith('.atd-upload-'): raise ValueError('invalid stage')
        if target.exists(): shutil.rmtree(target)
        result={'ok':True}
    else: raise ValueError('unknown action')
    print(json.dumps(result))
except Exception as e:
    print(json.dumps({'error':str(e)})); sys.exit(1)
'''


def request(raw, action, root, relative_path='', task=None, **extra):
    p = connection(raw)
    body = dict(action=action, root=root, relative=relative(relative_path), **extra)
    argv = ['ssh', *OPTIONS, '-p', str(p['port']), f"{p['user']}@{p['host']}", 'python3 -c ' + shlex.quote(REMOTE)]
    if task is None:
        try:
            result = subprocess.run(argv, input=json.dumps(body), text=True, capture_output=True, timeout=120)
        except subprocess.TimeoutExpired as error:
            raise ValueError('SSH 応答がタイムアウトしました。変更要求の結果は未確認です。状態を確認してください') from error
    else:
        process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
        first, deadline = True, time.monotonic() + 120
        try:
            while True:
                task.check()
                try:
                    output, errors = process.communicate(json.dumps(body) if first else None, timeout=0.2)
                    break
                except subprocess.TimeoutExpired:
                    first = False
                    if time.monotonic() >= deadline:
                        raise ValueError('SSH 応答がタイムアウトしました')
            result = subprocess.CompletedProcess(argv, process.returncode, output, errors)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.communicate(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.communicate()
    try:
        response = json.loads(result.stdout)
    except ValueError as error:
        raise ValueError('SSH 応答を確認してください: ' + result.stderr[-1000:]) from error
    if result.returncode or response.get('error'):
        raise ValueError(response.get('error') or result.stderr[-1000:])
    return response


def pull(store, task, raw, bag):
    p, bag = connection(raw), relative(bag)
    if not bag or bag == '.':
        raise ValueError('bag ディレクトリを選択してください')
    target = confined(store.record_root, bag, exists=False)
    if target.exists():
        raise ValueError('同名の bag があります')
    before = request(p, 'inventory', p['record_root'], bag, task=task)['files']
    if 'metadata.yaml' not in before:
        raise ValueError('metadata.yaml がある録画停止済み bag を選択してください')
    store.record_root.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.atd-pull-', dir=store.record_root))
    try:
        task.run(['scp', *OPTIONS, '-P', str(p['port']), '-r', f"{p['user']}@{p['host']}:{p['record_root']}/{bag}", str(stage / 'bag')])
        task.check()
        if inventory(stage / 'bag', task.check) != before or request(p, 'inventory', p['record_root'], bag, task=task)['files'] != before:
            raise ValueError('bag の内容が変化したか、転送後の hash が一致しません')
        target.parent.mkdir(parents=True, exist_ok=True)
        target = confined(store.record_root, bag, exists=False)
        if target.exists():
            raise ValueError('受信先が既に存在します')
        task.check()
        os.rename(stage / 'bag', target)
        return dict(record=bag, verified=True)
    finally:
        shutil.rmtree(stage)


def push(store, task, raw, key):
    p, key = connection(raw), identifier(key)
    from .exports import export_map
    export_map(store, key)
    source = confined(store.maps_root, key)
    stage = '.atd-upload-' + uuid.uuid4().hex
    # 保存中の編集を取り込まないようローカル snapshot を固定する。
    with tempfile.TemporaryDirectory(prefix='.atd-map-push-', dir=store.state_root) as temporary:
        bundle = Path(temporary) / 'bundle'
        inventory(source, task.check)
        shutil.copytree(source, bundle)
        expected = inventory(bundle, task.check)
        try:
            # mkdir 後に応答が失われても、自分の UUID 一時領域を掃除する。
            request(p, 'reserve', p['map_root'], stage)
            task.run(['scp', *OPTIONS, '-P', str(p['port']), '-r', str(bundle), f"{p['user']}@{p['host']}:{p['map_root']}/{stage}/"])
            task.check()
            return request(p, 'publish', p['map_root'], stage, destination=key, files=expected)
        finally:
            try:
                request(p, 'cleanup', p['map_root'], stage)
            except (ValueError, OSError, subprocess.TimeoutExpired) as error:
                task.log('転送の一時領域の掃除は未確認です: ' + str(error))

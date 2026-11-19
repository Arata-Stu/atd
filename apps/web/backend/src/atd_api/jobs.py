"""固定 action を独立 worker で実行。永続状態と lease で再接続を扱う。"""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

from .store import atomic_json, read_json, identifier

ACTIVE = {'starting', 'running', 'queued'}
ACTIONS = {'generate', 'map_build', 'vgl', 'capture', 'pull', 'push', 'calibration', 'localization_validation',
           'planning_replay', 'speed_replay', 'manual_drive_replay', 'manual_drive_vo'}


ROS_SETUP = ('set -e; source /opt/ros/lyrical/setup.bash; '
             'if [ -n "${ISAAC_ROS_WS:-}" ] && [ -f "$ISAAC_ROS_WS/install/setup.bash" ]; then '
             'source "$ISAAC_ROS_WS/install/setup.bash"; fi; ')


class Cancelled(Exception):
    pass


def identity(pid):
    try:
        stat = Path(f'/proc/{int(pid)}/stat').read_text().rsplit(')', 1)[1].split()
        return stat[19] if stat[0] != 'Z' else None
    except (OSError, ValueError, IndexError):
        try:
            result = subprocess.run(['ps', '-p', str(int(pid)), '-o', 'lstart='], capture_output=True, text=True, timeout=2)
            return result.stdout.strip() or None
        except (OSError, ValueError, subprocess.TimeoutExpired):
            return None


def leased(path):
    with path.open('a') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return False
        except BlockingIOError:
            return True


class Jobs:
    def __init__(self, store):
        self.store = store
        self.root = store.state_root / 'jobs'
        self.root.mkdir(parents=True, exist_ok=True)
        self.children = {}

    @contextmanager
    def registry(self):
        with (self.root / 'registry.lock').open('a') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            yield

    def get(self, key):
        identifier(key)
        path = self.root / (key + '.json')
        if not path.is_file():
            raise ValueError('ジョブが見つかりません')
        state = read_json(path)
        if state['status'] in ACTIVE and time.time() - state['started'] > 8:
            # 完了を書いて lease を解放した worker の final state を古い読取値で上書きしない。
            # worker と同じ lease を保持して再読込・reconcile を行う。
            with (self.root / (key + '.lease')).open('a') as stream:
                try:
                    fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    pass
                else:
                    state = read_json(path)
                    if state['status'] in ACTIVE and time.time() - state['started'] > 8:
                        pid, mark = state.get('process_pid'), state.get('process_identity')
                        if pid and identity(pid) == mark and mark:
                            state.update(status='orphaned', message='worker 喪失。実行プロセスが残っています。結果は未確認です')
                        else:
                            state.update(status='interrupted', message='実行プロセスがありません。自動再実行は行いません', finished=time.time())
                        atomic_json(path, state)
        state['cancel_requested'] = (self.root / (key + '.cancel')).exists()
        return state

    def list(self):
        for key, child in list(self.children.items()):
            if child.poll() is not None:
                self.children.pop(key)
        states = [self.get(p.stem) for p in self.root.glob('*.json') if not p.name.endswith('.request.json')]
        return sorted(states, key=lambda item: item['started'], reverse=True)[:200]

    def require_available(self, resources):
        for state in self.list():
            if state['status'] in ACTIVE | {'orphaned'} and set(resources) & set(state['resources']):
                raise ValueError('対象はジョブで使用中です: ' + state['title'])

    def start(self, action, payload, resources, title):
        if action not in ACTIONS:
            raise ValueError('許可されていない処理です')
        with self.registry():
            self.require_available(resources)
            key = uuid.uuid4().hex
            state = dict(id=key, action=action, title=title, status='starting', message='worker を起動しています',
                         started=time.time(), finished=None, resources=resources, worker_pid=None,
                         process_pid=None, result=None, cancel_requested=False)
            request = dict(action=action, payload=payload, record_root=str(self.store.record_root),
                           result_root=str(self.store.result_root), job_id=key)
            atomic_json(self.root / (key + '.request.json'), request)
            atomic_json(self.root / (key + '.json'), state)
            try:
                with (self.root / (key + '.log')).open('a') as stream:
                    self.children[key] = subprocess.Popen([sys.executable, '-m', 'atd_api.worker', str(self.root / (key + '.request.json'))],
                        stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True, close_fds=True)
            except OSError as error:
                state.update(status='failed', message=str(error), finished=time.time())
                atomic_json(self.root / (key + '.json'), state)
                raise
        return state

    def cancel(self, key):
        state = self.get(key)
        if state['status'] == 'orphaned':
            raise ValueError('worker を失った実行プロセスは状態を確認して停止してください')
        if state['status'] in ACTIVE:
            atomic_json(self.root / (key + '.cancel'), {'requested': time.time()})
            state['cancel_requested'] = True
        return state

    def log(self, key, offset=0):
        self.get(key)
        path = self.root / (key + '.log')
        if not path.exists():
            return dict(text='', offset=0)
        with path.open('rb') as stream:
            stream.seek(max(0, min(offset, path.stat().st_size)))
            data = stream.read(256 * 1024)
            return dict(text=data.decode(errors='replace'), offset=stream.tell())


class Task:
    def __init__(self, jobs, key):
        self.jobs, self.key = jobs, key
        self.path = jobs.root / (key + '.json')
        self.state = read_json(self.path)
        self.state.update(worker_pid=os.getpid(), worker_identity=identity(os.getpid()), status='running', message='実行中')
        self.persist()

    def persist(self):
        self.state['heartbeat'] = time.time()
        atomic_json(self.path, self.state)

    def check(self):
        if (self.jobs.root / (self.key + '.cancel')).exists():
            raise Cancelled('中止しました。未完成の成果物は公開しません')

    def log(self, text):
        print(text, flush=True)
        self.state['message'] = text
        self.persist()

    def run(self, argv, env=None, ros=False, allowed_exit_codes=(0,)):
        self.check()
        self.log('実行: ' + ' '.join(map(str, argv)))
        if ros or argv[0] == 'ros2':
            env = dict(os.environ if env is None else env)
            web_venv = env.pop('VIRTUAL_ENV', None)
            if web_venv:
                web_bin = Path(web_venv) / 'bin'
                env['PATH'] = os.pathsep.join(entry for entry in os.get_exec_path(env) if Path(entry) != web_bin)
                env.pop('PYTHONPATH', None)
            argv = ['bash', '-c', ROS_SETUP + 'exec "$@"', 'atd-ros-worker', *argv]
        process = subprocess.Popen(list(map(str, argv)), stdin=subprocess.DEVNULL, start_new_session=True, env=env)
        self.state.update(process_pid=process.pid, process_identity=identity(process.pid))
        self.persist()
        try:
            while process.poll() is None:
                self.check()
                time.sleep(0.15)
            if process.returncode not in allowed_exit_codes:
                raise ValueError(f'外部処理が終了コード {process.returncode} で失敗しました。ログを確認してください')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            self.state.update(process_pid=None, process_identity=None)
            self.persist()
        self.check()
        return process.returncode

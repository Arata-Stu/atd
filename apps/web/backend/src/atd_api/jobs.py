"""固定 action を独立 worker で実行。永続状態と lease で再接続を扱う。"""
from __future__ import annotations

import codecs
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

from .store import atomic_json, confined, read_json, identifier

ACTIVE = {'starting', 'running', 'queued'}
FINISHED = {'succeeded', 'completed', 'failed', 'cancelled', 'canceled', 'interrupted'}
LOG_PAGE_BYTES = 256 * 1024
ACTIONS = {'generate', 'map_build', 'map_duplicate', 'vgl', 'capture', 'pull', 'push', 'calibration', 'localization_validation',
           'planning_replay', 'speed_replay', 'manual_drive_replay', 'manual_drive_vo', 'simulation_benchmark'}


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
        self.root = confined(store.state_root, 'jobs', exists=False)
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

    def _states(self):
        for key, child in list(self.children.items()):
            if child.poll() is not None:
                self.children.pop(key)
        return [self.get(p.stem) for p in self.root.glob('*.json')
                if not p.name.endswith('.request.json') and not p.is_symlink()]

    def list(self):
        with self.registry():
            return sorted(self._states(), key=lambda item: item['started'], reverse=True)[:200]

    def history(self):
        with self.registry():
            states = self._states()
            items = [dict(id=s['id'], title=s.get('title') or s['id'], status=s['status'], started=s['started'])
                     for s in states if s['status'] in FINISHED]
        return dict(items=sorted(items, key=lambda item: item['started'], reverse=True),
                    protected=len(states) - len(items))

    def delete_history(self, keys):
        # 削除対象は画面で確認したスナップショットの ID に限定する。
        confined(self.store.result_root, self.root.relative_to(self.store.result_root))
        with self.registry():
            targets = []
            for key in dict.fromkeys(keys):
                identifier(key)
                path = confined(self.root, key + '.json', exists=False)
                if not path.exists():
                    continue  # 別の画面ですでに削除された履歴
                state = read_json(path)
                if state.get('id') != key or state.get('status') not in FINISHED:
                    raise ValueError('終了を確認できたジョブだけ削除できます: ' + key)
                lease = confined(self.root, key + '.lease', exists=False)
                if lease.exists() and leased(lease):
                    raise ValueError('worker が終了処理中です: ' + key)
                targets.append(key)
            for key in targets:
                # 結果ディレクトリとジョブ履歴は独立。状態ファイルを最後に消す。
                for suffix in ('.log', '.cancel', '.lease', '.request.json', '.json'):
                    (self.root / (key + suffix)).unlink(missing_ok=True)
        return dict(deleted=len(targets))

    def require_available(self, resources):
        for state in self._states():
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

    def log(self, key, offset=0, *, tail=False, before=None):
        state = self.get(key)
        path = self.root / (key + '.log')
        try:
            stream = path.open('rb')
        except FileNotFoundError:
            return dict(text='', start=0, offset=0) if tail else dict(text='', offset=0)
        with stream:
            size = os.fstat(stream.fileno()).st_size
            if not tail:
                stream.seek(max(0, min(offset, size)))
                data = stream.read(LOG_PAGE_BYTES)
                return dict(text=data.decode(errors='replace'), offset=stream.tell())

            end = size if before is None else max(0, min(before, size))
            if before is None and offset == size:
                return dict(text='', start=end, offset=end)
            start = max(0, end - LOG_PAGE_BYTES)
            stream.seek(start)
            data = stream.read(end - start)
            # A UTF-8 code point crossing the start belongs to the preceding page.
            skip = 0
            if start:
                while skip < min(3, len(data)) and 0x80 <= data[skip] < 0xc0:
                    skip += 1
            decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
            final = end == size and state['status'] not in ACTIVE
            text = decoder.decode(data[skip:], final=final)
            partial, _ = decoder.getstate()
            return dict(text=text, start=start + skip, offset=stream.tell() - len(partial))


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

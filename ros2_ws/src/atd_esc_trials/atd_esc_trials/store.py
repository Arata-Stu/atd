"""Jetson-local immutable trial plans, exclusive execution and observation log."""
import contextlib
import fcntl
import json
import os
from pathlib import Path
import re
import tempfile
import time
import uuid

from .model import validate_steps


class TrialStore:
    def __init__(self, root=None):
        self.root = Path(root or '/data/results/runs/esc_trials')
        self.lease = None
        self.claimed_id = None

    @contextlib.contextmanager
    def _lock(self):
        self.root.mkdir(parents=True, exist_ok=True)
        with (self.root / '.store.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield

    def _save(self, path, value):
        with tempfile.NamedTemporaryFile('w', dir=self.root, delete=False) as out:
            name = out.name
            try:
                json.dump(value, out, ensure_ascii=False, allow_nan=False)
                out.flush()
                os.replace(name, path)
            finally:
                Path(name).unlink(missing_ok=True)

    def _record(self, trial):
        trial['updated_at'] = time.time()
        history = trial.setdefault('state_history', [])
        observed = ('status', 'step_index', 'esc_state', 'throttle', 'pwm_us', 'requested_pwm_us', 'pwm_observed')
        if not history or any(history[-1].get(k) != trial.get(k) for k in observed):
            history.append({k: trial.get(k) for k in (*observed, 'updated_at', 'message')})
            del history[:-1024]
        self._save(self.root / (trial['id'] + '.json'), trial)
        with (self.root / (trial['id'] + '.jsonl')).open('a') as log:
            event = {k: v for k, v in trial.items() if k not in ('steps', 'created_at', 'state_history')}
            log.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')

    def _lease(self):
        file = (self.root / '.runner.lock').open('a')
        try:
            fcntl.flock(file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            file.close()
            return None
        return file

    def _current(self):
        pointer = self.root / 'latest.json'
        if not pointer.exists():
            return None
        key = json.loads(pointer.read_text())
        if not isinstance(key, str) or not re.fullmatch('[0-9a-f]{32}', key):
            raise ValueError('試験 ID が不正です')
        trial = json.loads((self.root / (key + '.json')).read_text())
        if trial['status'] in ('running', 'cancel_requested') and self.claimed_id != key:
            lease = self._lease()
            if lease:
                with lease:
                    trial.update(status='failed', esc_state='unknown', throttle=None, pwm_us=None,
                                 requested_pwm_us=None, pwm_observed=None,
                                 message='Jetson 実行プロセスが終了しました。現在の出力は不明です')
                    self._record(trial)
        return trial

    def status(self):
        with self._lock():
            trial = self._current()
            if trial and trial['status'] in ('running', 'cancel_requested'):
                # The bridge may still answer while the terminal process is suspended.
                reported = trial.get('feedback_at')
                age = time.time() - reported if reported is not None else None
                trial['feedback_stale'] = age is None or not 0 <= age <= .75
            return {'trial': trial}

    def prepare(self, steps):
        normalized = validate_steps(steps)
        with self._lock():
            current = self._current()
            if current and current['status'] in ('queued', 'running', 'cancel_requested'):
                raise ValueError('予約中または実行中の試験があります')
            trial = dict(id=uuid.uuid4().hex, created_at=time.time(), status='queued',
                         steps=normalized, step_index=None, esc_state='unknown', throttle=None,
                         pwm_us=None, message='Jetson 端末での開始確認を待っています')
            self._record(trial)
            self._save(self.root / 'latest.json', trial['id'])
            return {'trial': trial}

    def _require(self, key):
        current = self._current()
        if not current or current['id'] != key:
            raise ValueError('対象の試験が変わりました。画面を更新してください')
        return current

    def claim(self, key):
        with self._lock():
            trial = self._require(key)
            if trial['status'] != 'queued' or self.lease is not None:
                raise ValueError('この試験は開始できません')
            lease = self._lease()
            if lease is None:
                raise ValueError('別の実行プロセスが起動中です')
            try:
                trial.update(status='running', message='Jetson 端末で開始を確認しました')
                self._record(trial)
            except BaseException:
                lease.close()
                raise
            self.lease, self.claimed_id = lease, key
            return {'trial': trial}

    def cancel(self, key):
        with self._lock():
            trial = self._require(key)
            if trial['status'] == 'queued':
                trial.update(status='cancelled', message='開始前に予約を取り消しました')
            elif trial['status'] == 'running':
                trial.update(status='cancel_requested', message='Jetson に中止を要求しました')
            else:
                return {'trial': trial}
            self._record(trial)
            return {'trial': trial}

    def _owned(self, key, fields):
        if self.claimed_id != key or self.lease is None:
            raise ValueError('この端末は試験の実行者ではありません')
        if {'id', 'status', 'steps', 'created_at', 'updated_at', 'state_history'} & fields.keys():
            raise ValueError('試験の識別情報・手順は変更できません')
        return self._require(key)

    def update(self, key, **fields):
        with self._lock():
            trial = self._owned(key, fields)
            trial.update(fields)
            if 'feedback_sequence' in fields:
                trial['feedback_at'] = time.time()
            self._record(trial)
            return {'trial': trial}

    def finish(self, key, status, **fields):
        if status not in ('succeeded', 'failed', 'cancelled'):
            raise ValueError('不正な試験終了状態です')
        with self._lock():
            trial = self._owned(key, fields)
            if status == 'succeeded' and trial['status'] == 'cancel_requested':
                status = 'cancelled'
                fields['message'] = '試験終了時に中止を受け付けました'
            try:
                trial.update(status=status, **fields)
                self._record(trial)
            finally:
                self.lease.close()
                self.lease = self.claimed_id = None
            return {'trial': trial}

import io
import json
from pathlib import Path
import runpy
import sys
from types import ModuleType
from unittest.mock import Mock

import pytest


@pytest.mark.parametrize('other_bridge', [False, True])
def test_foreground_runner_requires_own_bridge_and_runs_each_id_once(monkeypatch, other_bridge):
    calls = []
    bridge = Mock(pid=123)
    runner = Mock(returncode=0)
    runner.poll.return_value = 0
    observations = 0

    class Queue:
        def status(self):
            nonlocal observations
            observations += 1
            if observations == 3:
                bridge.poll.return_value = 1
            return {'trial': {'status': 'queued', 'id': 'immutable'}}

    def spawn(args, **kwargs):
        calls.append((args, kwargs))
        if args[0].endswith('/web_bridge'):
            bridge.poll.return_value = None
            return bridge
        return runner

    class Opener:
        def open(self, request, timeout):
            data = {'enabled': True}
            pid = 456 if other_bridge else bridge.pid
            reply = {'request_id': request.headers['X-atd-request'], 'identity': {'pid': pid}, 'data': data}
            return io.StringIO(json.dumps(reply))

    for name, key, value in [('ament_index_python.packages', 'get_package_prefix', lambda _: '/install'),
                             ('atd_esc_trials.store', 'TrialStore', Queue)]:
        module = ModuleType(name)
        setattr(module, key, value)
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / 'scripts/esc_trial_session'
    entry = runpy.run_path(str(path))
    monkeypatch.setattr(sys, 'argv', [str(path), 'port:=/dev/board', 'stop_source:=visual'])
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: True)
    monkeypatch.setattr(sys.stdout, 'isatty', lambda: True)
    monkeypatch.setattr(entry['subprocess'], 'Popen', spawn)
    monkeypatch.setattr(entry['urllib'].request, 'build_opener', lambda *_: Opener())
    monkeypatch.setattr(entry['time'], 'sleep', lambda _: None)
    with pytest.raises(RuntimeError, match='別の Web bridge' if other_bridge else 'bridge が終了'):
        entry['run']()
    assert calls[0][0][-1] == 'esc_trials_enabled:=true'
    if other_bridge:
        assert len(calls) == 1
    else:
        assert len(calls) == 2
        assert calls[1] == (['/install/lib/atd_esc_trials/run_trial', '--port', '/dev/board',
                             '--stop-source', 'visual'], {})

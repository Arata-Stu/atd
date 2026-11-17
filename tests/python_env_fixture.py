"""用途別 uv 環境の同期先・Python 設定・実行境界を一時ディレクトリで確認する。"""

import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / 'docker/scripts/atd-python.sh'
FAKE_UV = r'''
import json
import os
from pathlib import Path
import sys
import venv

args = sys.argv[1:]
with open(os.environ['ATD_TEST_LOG'], 'a') as stream:
    stream.write(json.dumps({'args': args, 'env': {key: os.environ.get(key) for key in (
        'UV_PROJECT_ENVIRONMENT', 'UV_CACHE_DIR', 'VIRTUAL_ENV', 'PYTHONPATH', 'PYTHONHOME')}}) + '\n')
if args[0] == 'venv':
    output = Path(args[-1])
    venv.EnvBuilder(with_pip=False, system_site_packages='--system-site-packages' in args).create(output)
    if os.environ.get('ATD_TEST_FAKE_VENV_PYTHON'):
        python = output / 'bin/python'
        python.unlink()
        python.write_text('#!' + sys.executable + '\n' + """
import json
import os
import sys
if sys.argv[1:2] == ['-I']:
    os.execv(sys.executable, [sys.executable, *sys.argv[1:]])
with open(os.environ['ATD_TEST_BUILD_LOG'], 'w') as stream:
    json.dump({'args': sys.argv[1:], 'cwd': os.getcwd(), 'venv': os.environ.get('VIRTUAL_ENV'),
               'env': {key: os.environ.get(key) for key in ('RMW_IMPLEMENTATION', 'ROS_DISCOVERY_SERVER', 'ATD_TEST_OVERLAY', 'PYTHONPATH', 'PATH')}}, stream)
sys.exit(int(os.environ.get('ATD_TEST_BUILD_EXIT', '0')))
""")
        python.chmod(0o755)
elif args[0] == 'sync':
    sys.exit(int(os.environ.get('ATD_TEST_SYNC_EXIT', '0')))
else:
    sys.exit('unexpected uv operation')
'''


class PythonEnvironmentFixture:
    """環境操作と build のテストで共有する、テストケースを持たない fixture。"""
    script = SCRIPT

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='atd-python-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.workspace = self.root / 'ws'
        self.workspace.mkdir()
        self.projects = self.root / 'projects'
        for profile in ('ros', 'train', 'vgl-export', 'web', 'raceline'):
            project = self.projects / profile
            project.mkdir(parents=True)
            (project / 'pyproject.toml').write_text(
                f'[project]\nname = "atd-{profile}"\nversion = "0.0.0"\nrequires-python = ">=3.9"\ndependencies = []\n')
            (project / 'uv.lock').write_text('version = 1\n')
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        uv = self.bin / 'uv'
        uv.write_text(f'#!{sys.executable}\n' + FAKE_UV)
        uv.chmod(0o755)
        (self.bin / 'uname').write_text('#!/bin/sh\nif [ "$1" = -s ]; then printf "%s\\n" "${ATD_TEST_SYSTEM:-Linux}"; else printf "%s\\n" "${ATD_TEST_ARCH:-x86_64}"; fi\n')
        (self.bin / 'uname').chmod(0o755)
        (self.bin / 'id').write_text('#!/bin/sh\nprintf "%s\\n" "${ATD_TEST_UID:-1000}"\n')
        (self.bin / 'id').chmod(0o755)
        self.log = self.root / 'uv-calls.jsonl'
        self.build_log = self.root / 'build.json'
        self.ros_lib = self.root / 'ros-python'
        self.ros_lib.mkdir()
        (self.ros_lib / 'atd_fake_ros_module.py').write_text('available = True\n')
        self.setup = self.root / 'ros-setup.bash'
        self.setup.write_text(
            'export PYTHONPATH=' + shlex.quote(str(self.ros_lib)) + '\n'
            'export RMW_IMPLEMENTATION=old-ros-value\n'
            'export ROS_DISCOVERY_SERVER=old-ros-server\n')
        self.env = {
            'PATH': os.pathsep.join((str(self.bin), str(Path(sys.executable).parent), os.defpath)),
            'HOME': str(self.root),
            'ISAAC_ROS_WS': str(self.workspace),
            'ATD_PYTHON_PROJECTS_DIR': str(self.projects),
            'ATD_SYSTEM_PYTHON': sys.executable,
            'ATD_ROS_SETUP': str(self.setup),
            'ATD_TEST_LOG': str(self.log),
            'ATD_TEST_BUILD_LOG': str(self.build_log),
            'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        }

    def invoke(self, *args, env=None):
        return subprocess.run(['/bin/bash', str(self.script), *args],
                              env={**self.env, **(env or {})}, text=True,
                              capture_output=True, timeout=15)

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def venv(self, profile):
        return self.workspace / '.atd-python/x86_64' / profile


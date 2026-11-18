"""serial の実 GID 追加を、偽 account DB と device fixture で検証する。"""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "docker/scripts/configure-serial-access.sh"


FAKE_ACCOUNT_TOOL = r'''
import json
import os
from pathlib import Path
import sys

state_path = Path(os.environ['ATD_TEST_STATE'])
state = json.loads(state_path.read_text())
tool, args = Path(sys.argv[0]).name, sys.argv[1:]
if tool == 'id':
    if args == ['-u']:
        print(os.environ.get('ATD_TEST_UID', '0'))
    elif args == ['-G', 'admin']:
        print(' '.join(str(value) for value in state['memberships']))
    else:
        sys.exit(15)
elif tool == 'getent':
    if args == ['passwd', 'admin']:
        print('admin:x:1000:1000::/home/admin:/bin/bash')
    elif len(args) == 2 and args[0] == 'group':
        name = state['groups'].get(args[1])
        if name is None:
            sys.exit(2)
        print(name + ':x:' + args[1] + ':')
    else:
        sys.exit(16)
elif tool == 'groupadd':
    assert len(args) == 3 and args[0] == '--gid', args
    assert args[1] not in state['groups'], args
    state['groups'][args[1]] = args[2]
    state['created'].append(args[1])
elif tool == 'usermod':
    assert len(args) == 4 and args[:2] == ['--append', '--groups'] and args[3] == 'admin', args
    gid, = [int(gid) for gid, name in state['groups'].items() if name == args[2]]
    state['memberships'].append(gid)
    state['added'].append(gid)
elif tool == 'stat':
    assert args[:3] == ['-c', '%g', '--'], args
    if os.environ.get('ATD_TEST_STAT_GONE') == '1':
        sys.exit(1)
    print(state['devices'][Path(args[3]).name])
else:
    sys.exit('unexpected tool: ' + tool)
state_path.write_text(json.dumps(state))
'''


class SerialAccessTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="atd-serial-access-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.devices = self.root / "dev"
        self.devices.mkdir()
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.state_path = self.root / "accounts.json"
        self.state_path.write_text(json.dumps({
            "groups": {"1000": "admin", "20": "dialout"},
            "memberships": [1000], "created": [], "added": [], "devices": {},
        }))
        for name in ("id", "getent", "groupadd", "usermod", "stat"):
            path = self.bin / name
            path.write_text(f"#!{sys.executable}\n" + FAKE_ACCOUNT_TOOL)
            path.chmod(0o755)
        # いかなる chmod/chown/chgrp も実 device に到達させない。
        for name in ("chmod", "chown", "chgrp"):
            path = self.bin / name
            path.write_text('#!/bin/sh\nprintf "forbidden permission mutation\\n" >&2\nexit 90\n')
            path.chmod(0o755)
        self.env = {
            "PATH": os.pathsep.join((str(self.bin), os.defpath)),
            "ATD_SERIAL_ACCESS": "1", "ATD_SERIAL_DEVICE_DIR": str(self.devices),
            "ATD_TEST_STATE": str(self.state_path),
        }

    def state(self):
        return json.loads(self.state_path.read_text())

    def device(self, name, gid):
        # /dev/null の symlink で character-device 判定を保ち、GID だけ fixture にする。
        (self.devices / name).symlink_to("/dev/null")
        state = self.state()
        state["devices"][name] = gid
        self.state_path.write_text(json.dumps(state))

    def invoke(self, **env):
        return subprocess.run(["/bin/bash", str(HELPER)], env={**self.env, **env},
                              capture_output=True, text=True, timeout=10)

    def test_absent_device_reserves_host_group_for_hotplug_and_is_idempotent(self):
        result = self.invoke(ATD_HOST_SERIAL_GIDS="73")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.state()["created"], ["73"])
        self.assertEqual(self.state()["added"], [73])
        # 起動後に現れた同じ host GID の ttyACM は、すでに admin の group に含まれる。
        self.device("ttyACM8", 73)
        self.assertEqual(self.invoke(ATD_HOST_SERIAL_GIDS="73").returncode, 0)
        self.assertEqual(self.state()["added"], [73])

    def test_visible_device_actual_gid_and_existing_group_are_used(self):
        self.device("ttyACM7", 81)
        self.device("ttyACM9", 20)
        (self.devices / "ttyACM1").write_text("regular file is ignored")
        self.device("ttyUSB0", 99)
        before = (self.devices / "ttyACM7").stat().st_mode
        result = self.invoke(ATD_HOST_SERIAL_GIDS="20,20")
        self.assertEqual(result.returncode, 0, result.stderr)
        state = self.state()
        self.assertEqual(state["created"], ["81"])
        self.assertEqual(set(state["added"]), {20, 81})
        self.assertEqual(len(state["added"]), 2)
        self.assertEqual((self.devices / "ttyACM7").stat().st_mode, before)

    def test_pc_is_noop_and_empty_jetson_is_allowed(self):
        self.device("ttyACM4", 81)
        result = self.invoke(ATD_SERIAL_ACCESS="0", ATD_TEST_UID="1000")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.state()["added"], [])
        (self.devices / "ttyACM4").unlink()
        self.assertEqual(self.invoke().returncode, 0)
        self.assertEqual(self.state()["added"], [])

    def test_nonroot_or_invalid_host_group_never_mutates_accounts(self):
        for env in ({"ATD_TEST_UID": "1000"}, {"ATD_HOST_SERIAL_GIDS": "0"},
                    {"ATD_HOST_SERIAL_GIDS": "20,sudo"}, {"ATD_HOST_SERIAL_GIDS": "20;id"}):
            with self.subTest(env=env):
                result = self.invoke(**env)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.state()["added"], [])

    def test_root_group_is_not_granted(self):
        self.device("ttyACM0", 0)
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("GID 0", result.stderr)
        self.assertEqual(self.state()["added"], [])

    def test_hot_unplug_during_stat_does_not_fail_startup(self):
        self.device("ttyACM5", 81)
        result = self.invoke(ATD_TEST_STAT_GONE="1", ATD_HOST_SERIAL_GIDS="20")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.state()["added"], [20])


if __name__ == "__main__":
    unittest.main()

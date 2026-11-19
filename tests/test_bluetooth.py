"""Jetson ホストの接続手順を模擬 BlueZ で検証する。"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

PROJECT = Path(__file__).resolve().parents[1]
DUALSENSE = '4C:B9:9B:E0:EF:24'

BLUEZ = r'''#!/bin/bash
set -eu
while [[ "${1:-}" = --* ]]; do shift 2; done
action="$1"
shift
printf '%s\n' "$action ${1:-}" >> "$BT_TEST_STATE/calls"
cd "$BT_TEST_STATE"
case "$action" in
    show)
        printf 'Controller 00:22:44:66:88:AA\n'
        if [[ -f powered ]]; then printf '\tPowered: yes\n'; else printf '\tPowered: no\n'; fi ;;
    power)
        [[ "$1" = on ]] && touch powered ;;
    scan)
        [[ "$1" = on && -f powered ]]
        if [[ -f nearby ]]; then
            touch known
            printf '[NEW] Device %s Wireless Controller\n' "$BT_TEST_MAC"
        fi ;;
    info|pair|trust|connect)
        [[ "$1" = "$BT_TEST_MAC" ]]
        if [[ ! -f known ]]; then printf 'Device %s not available\n' "$1"; exit 1; fi
        case "$action" in
            info)
                printf 'Device %s\n' "$1"
                for field in Paired Trusted Connected; do
                    if [[ -f "$field" ]]; then value=yes; else value=no; fi
                    printf '\t%s: %s\n' "$field" "$value"
                done ;;
            pair) touch Paired ;;
            trust) touch Trusted ;;
            connect)
                if [[ -f reject ]]; then printf 'Failed to connect\n'; exit 1; fi
                if [[ -f race ]]; then
                    touch connecting
                    printf 'Failed to connect: org.bluez.Error.InProgress\n'
                    exit 1
                fi
                if [[ ! -f disconnected ]]; then touch Connected; fi ;;
        esac ;;
    *) exit 2 ;;
esac
'''


class BluetoothTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        self.script = root / 'scripts/bluetooth.sh'
        (self.script.parent / 'lib').mkdir(parents=True)
        shutil.copy2(PROJECT / 'scripts/bluetooth.sh', self.script)
        (self.script.parent / 'lib/detect_host.sh').write_text(
            'detect_atd_host() { echo "$BT_TEST_HOST"; }\n')
        self.state = root / 'state'
        self.state.mkdir()
        commands = root / 'bin'
        commands.mkdir()
        for name, content in {
            'bluetoothctl': BLUEZ,
            'sleep': '#!/bin/sh\nif [ -f "$BT_TEST_STATE/connecting" ]; then touch "$BT_TEST_STATE/Connected"; fi\n',
            'systemd-detect-virt': '#!/bin/sh\n[ "$BT_TEST_CONTAINER" = yes ]\n',
            'timeout': '#!/bin/sh\nwhile [ "$#" -gt 0 ] && [ "$1" != bluetoothctl ]; do shift; done\nexec "$@"\n',
        }.items():
            path = commands / name
            path.write_text(content)
            path.chmod(0o755)
        self.env = {**os.environ, 'PATH': str(commands) + os.pathsep + os.defpath,
                    'BT_TEST_HOST': 'jetson', 'BT_TEST_CONTAINER': 'no',
                    'BT_TEST_STATE': str(self.state), 'BT_TEST_MAC': DUALSENSE}

    def set_state(self, *markers):
        for name in markers:
            (self.state / name).touch()

    def run_entry(self, *args, **env):
        return subprocess.run([str(self.script), *args], text=True, input='\n',
                              env={**self.env, **env}, capture_output=True, timeout=15)

    def operations(self):
        log = self.state / 'calls'
        return [line.split()[0] for line in log.read_text().splitlines()] if log.exists() else []

    def test_first_connection_discovers_pairs_trusts_and_confirms(self):
        self.set_state('nearby')
        result = self.run_entry()
        self.assertEqual(result.returncode, 0, result.stderr)
        actions = self.operations()
        self.assertEqual([op for op in actions if op not in ('show', 'info')],
                         ['power', 'scan', 'pair', 'trust', 'connect'])
        self.assertEqual(actions[-1], 'info')

    def test_paired_controller_connects_without_discovery_or_repair(self):
        self.set_state('powered', 'known', 'Paired', 'Trusted')
        result = self.run_entry('connect')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([op for op in self.operations() if op not in ('show', 'info')], ['connect'])

    def test_connected_controller_and_status_do_not_change_state(self):
        self.set_state('powered', 'known', 'Paired', 'Trusted', 'Connected')
        for args in ((), ('status',)):
            with self.subTest(args=args):
                result = self.run_entry(*args)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('info', self.operations())
                self.assertLessEqual(set(self.operations()), {'show', 'info'})

    def test_unknown_controller_stops_after_scan(self):
        self.set_state('powered')
        self.assertNotEqual(self.run_entry().returncode, 0)
        self.assertIn('scan', self.operations())
        self.assertLessEqual(set(self.operations()), {'show', 'info', 'scan'})

    def test_connection_command_and_actual_link_failures_are_reported(self):
        self.set_state('powered', 'known', 'Paired', 'Trusted')
        for marker in ('reject', 'disconnected'):
            with self.subTest(marker=marker):
                self.set_state(marker)
                self.assertNotEqual(self.run_entry().returncode, 0)
                self.assertIn('connect', self.operations())
                (self.state / marker).unlink()

    def test_auto_reconnection_can_complete_after_in_progress_error(self):
        self.set_state('powered', 'known', 'Paired', 'Trusted', 'race')
        result = self.run_entry()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Connected: yes', result.stdout)
        actions = self.operations()
        self.assertEqual(actions.count('connect'), 1)
        self.assertGreaterEqual(actions[actions.index('connect') + 1:].count('info'), 2)

    def test_status_only_queries_the_requested_address(self):
        other = '11:22:33:44:55:66'
        self.set_state('known', 'Paired')
        result = self.run_entry('status', '--mac', other, BT_TEST_MAC=other)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Connected: no', result.stdout)
        self.assertIn('info ' + other, (self.state / 'calls').read_text())
        self.assertLessEqual(set(self.operations()), {'show', 'info'})

    def test_invalid_mac_container_and_notebook_are_rejected_before_bluez(self):
        for args, env in [(('--mac', 'bad'), {}), ((), {'BT_TEST_CONTAINER': 'yes'}),
                          ((), {'BT_TEST_HOST': 'pc'})]:
            with self.subTest(args=args, env=env):
                self.assertNotEqual(self.run_entry(*args, **env).returncode, 0)
                self.assertEqual(self.operations(), [])


if __name__ == '__main__':
    unittest.main()

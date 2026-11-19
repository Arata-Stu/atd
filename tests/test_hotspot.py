"""Jetson の無線を切り替えず、保存 AP の選択・更新と再接続を検証する。"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

PROJECT = Path(__file__).resolve().parents[1]
IFACE = 'wlP1p1s0'
AP = '8a1b3cee-fa5b-4a77-a404-730de7f329d7'
OTHER = 'ca760b0f-a2bd-4ef3-b9cd-ab0bf81dd65a'


def profile(ident, name='Track Wi-Fi', iface=IFACE, mode='ap', band='a', channel='44'):
    return {'connection.uuid': ident, 'connection.id': name, 'connection.type': '802-11-wireless',
            'connection.interface-name': iface, '802-11-wireless.mode': mode,
            '802-11-wireless.band': band, '802-11-wireless.channel': channel,
            '802-11-wireless.ssid': name, '802-11-wireless-security.key-mgmt': 'wpa-psk',
            '802-11-wireless-security.psk': 'fixture-only', 'ipv4.method': 'shared'}


def fake_nmcli(args):
    root = Path(os.environ['HOTSPOT_TEST_DIR'])
    state_file = root / 'state.json'
    state = json.loads(state_file.read_text())
    profiles = state['profiles']
    with (root / 'calls').open('a') as log:
        print(json.dumps(args), file=log)
    fields = ''
    while args[0].startswith('-'):
        opt = args.pop(0)
        if opt in ('-g', '-f', '--fields', '--get-values'):
            fields = args.pop(0)
        elif opt in ('--wait', '-w', '--escape', '-e', '--colors'):
            args.pop(0)
        elif opt not in ('-t', '--terse', '--show-secrets', '-s'):
            raise AssertionError(opt)

    group, action, *params = args
    if group == 'device' and action == 'show':
        (iface,) = params
        if iface != state['iface']:
            return 10
        props = {'GENERAL.TYPE': 'wifi', 'WIFI-PROPERTIES.AP': 'yes', 'GENERAL.DEVICE': iface,
                 'GENERAL.CON-UUID': state['active'] or '--', 'GENERAL.STATE': '100 (connected)',
                 'GENERAL.CONNECTION': profiles[state['active']]['connection.id'] if state['active'] else '--',
                 'IP4.ADDRESS': '10.42.0.1/24'}
        for key in fields.split(','):
            print(props[key])
        return 0
    if group == 'device' and action == 'wifi':
        assert params == ['list', 'ifname', state['iface'], '--rescan', 'no']
        if state['active']:
            print(f"yes:{state.get('live_channel', '')}")
        return 0
    if group != 'connection':
        raise AssertionError(args)
    if action == 'show':
        if not params or params == ['--active']:
            for ident, value in profiles.items():
                if params and ident != state['active']:
                    continue
                data = {'UUID': ident, 'NAME': value['connection.id'], 'TYPE': 'wifi',
                        'DEVICE': state['iface'] if state['active'] == ident else ''}
                print(':'.join(data[key] for key in fields.split(',')))
            return 0
        lookup, ident = params
        if lookup != 'uuid' or ident not in profiles:
            return 10
        for key in fields.split(','):
            print(profiles[ident].get(key, ''))
        return 0
    if action == 'add':
        props = dict(zip(params[::2], params[1::2], strict=True))
        ident = props['connection.uuid']
        if ident in profiles:
            return 4
        profiles[ident] = profile(ident, props['con-name'], props['ifname'])
        aliases = {'ssid': '802-11-wireless.ssid', 'mode': '802-11-wireless.mode',
                   'wifi.mode': '802-11-wireless.mode', 'wifi.band': '802-11-wireless.band',
                   'wifi.channel': '802-11-wireless.channel', 'wifi-sec.key-mgmt': '802-11-wireless-security.key-mgmt'}
        profiles[ident].update({aliases.get(k, k): v for k, v in props.items()})
        profiles[ident].pop('802-11-wireless-security.psk')
    else:
        assert params[0] == 'uuid' and params[1] in profiles, args
        ident = params[1]
        if action == 'up':
            assert params[2:4] == ['ifname', state['iface']], args
            if os.environ.get('FAIL_ACTIVATION'):
                return 4
            if 'passwd-file' in params:
                psk = sys.stdin.read().strip().partition(':')[2]
                expected = os.environ['EXPECTED_TEST_PSK']
                expected = hashlib.pbkdf2_hmac('sha1', expected.encode(), b'ATD-Hotspot', 4096, 32).hex()
                assert psk == expected
                profiles[ident]['802-11-wireless-security.psk'] = psk
            if not profiles[ident].get('802-11-wireless-security.psk'):
                return 4
            state['active'] = ident
            state['live_channel'] = profiles[ident]['802-11-wireless.channel']
        elif action == 'down':
            assert state['active'] == ident
            state['active'] = ''
        elif action == 'modify':
            for k, v in zip(params[2::2], params[3::2], strict=True):
                profiles[ident][k.replace('wifi.', '802-11-wireless.')] = v
        else:
            raise AssertionError(args)
    state_file.write_text(json.dumps(state))
    return 0


class HotspotTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        (self.root / 'scripts/lib').mkdir(parents=True)
        self.command = self.root / 'scripts/hotspot.sh'
        shutil.copy2(PROJECT / 'scripts/hotspot.sh', self.command)
        (self.root / 'scripts/lib/jetson_host.sh').write_text(
            'atd_require_jetson_host() { return 0; }\natd_run_as_root() { "$@"; }\n')
        fake = self.root / 'nmcli'
        fake.write_text(f'#!{sys.executable}\nimport runpy, sys\n'
                        f"sys.exit(runpy.run_path({str(Path(__file__).resolve())!r})['fake_nmcli'](sys.argv[1:]))\n")
        fake.chmod(0o755)
        self.env = {**os.environ, 'HOTSPOT_TEST_DIR': str(self.root),
                    'PATH': str(self.root) + os.pathsep + os.defpath}
        self.env.pop('ATD_HOTSPOT_PASSWORD', None)
        self.seed()

    def seed(self, profiles=(), active='', iface=IFACE):
        data = {'iface': iface, 'active': active, 'profiles': {p['connection.uuid']: p for p in profiles}}
        (self.root / 'state.json').write_text(json.dumps(data))

    def state(self):
        return json.loads((self.root / 'state.json').read_text())

    def run_cli(self, *args, ok=True, **env):
        result = subprocess.run([str(self.command), *args], input='', text=True, capture_output=True,
                                env={**self.env, **env}, timeout=25)
        if ok:
            self.assertEqual(result.returncode, 0, result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0)
        return result

    def test_status_and_off_leave_client_in_place(self):
        self.seed([profile(AP), profile(OTHER, mode='infrastructure')], active=OTHER)
        before = self.state()
        self.run_cli()
        self.run_cli('off')
        self.assertEqual(self.state(), before)

    def test_reuses_profile_with_its_saved_band_and_channel(self):
        self.seed([profile(AP, band='bg', channel='11')])
        self.run_cli('on')
        state = self.state()
        self.assertEqual(state['active'], AP)
        self.assertEqual(state['profiles'][AP]['802-11-wireless.channel'], '11')
        self.run_cli('off')
        self.assertEqual(self.state()['active'], '')

    def test_switch_band_and_channel_on_running_ap(self):
        self.seed([profile(AP)], active=AP)
        self.run_cli('on', '--band', '2.4')
        self.assertEqual(self.state()['profiles'][AP]['802-11-wireless.band'], 'bg')
        self.assertEqual(self.state()['profiles'][AP]['802-11-wireless.channel'], '6')
        self.run_cli('on', '--band', '5', '--channel', '44')
        self.assertEqual(self.state()['profiles'][AP]['802-11-wireless.channel'], '44')
        self.assertEqual(self.state()['active'], AP)

    def test_band_change_failure_keeps_the_previous_saved_settings(self):
        self.seed([profile(AP)], active=AP)
        self.run_cli('on', '--band', '2.4', ok=False, FAIL_ACTIVATION='1')
        self.assertEqual(self.state()['profiles'][AP]['802-11-wireless.band'], 'a')
        self.assertEqual(self.state()['profiles'][AP]['802-11-wireless.channel'], '44')

    def test_off_selects_the_running_ap_even_without_an_interface_binding(self):
        self.seed([profile(AP, iface=''), profile(OTHER, name='Other AP', iface='wlan1')],
                  active=AP, iface='wlan1')
        self.assertIn(AP, self.run_cli('status', '--ifname', 'wlan1').stdout)
        self.run_cli('off', '--ifname', 'wlan1')
        self.assertEqual(self.state()['active'], '')
        self.assertEqual(len(self.state()['profiles']), 2)

    def test_ambiguous_or_incompatible_selection_does_not_mutate_profiles(self):
        self.seed([profile(AP), profile(OTHER, name='Other AP')])
        before = self.state()
        self.run_cli('on', ok=False)
        self.assertEqual(self.state(), before)
        self.run_cli('on', '--profile', 'Other AP')
        self.assertEqual(self.state()['active'], OTHER)
        self.seed([profile(OTHER, mode='infrastructure')], active=OTHER)
        self.run_cli('on', '--profile', OTHER, ok=False)
        self.assertEqual(self.state()['active'], OTHER)

    def test_creates_secured_ap_once_and_reconnects_without_password_input(self):
        password = ' AP-fixture\\42 '
        first = self.run_cli('on', ATD_HOTSPOT_PASSWORD=password, EXPECTED_TEST_PSK=password)
        state = self.state()
        (settings,) = state['profiles'].values()
        self.assertEqual(settings['802-11-wireless.ssid'], 'ATD-Hotspot')
        self.assertEqual(settings['802-11-wireless.band'], 'a')
        self.assertEqual(settings['802-11-wireless.channel'], '36')
        self.assertEqual(settings['ipv4.addresses'], '10.42.0.1/24')
        self.assertEqual(settings['802-11-wireless-security.key-mgmt'], 'wpa-psk')
        output = (self.root / 'calls').read_text() + first.stdout + first.stderr
        self.assertNotIn(password, output)
        self.assertNotIn(settings['802-11-wireless-security.psk'], output)
        self.run_cli('off')
        self.run_cli('on')
        self.assertEqual(self.state(), state)

    def test_interrupted_first_activation_reuses_the_incomplete_profile(self):
        password = 'retry-test-password'
        env = {'ATD_HOTSPOT_PASSWORD': password, 'EXPECTED_TEST_PSK': password}
        self.run_cli('on', ok=False, FAIL_ACTIVATION='1', **env)
        (uuid,) = self.state()['profiles']
        self.assertEqual(self.state()['active'], '')
        self.run_cli('on', **env)
        self.assertEqual(self.state()['active'], uuid)
        self.assertEqual(list(self.state()['profiles']), [uuid])

    def test_unsupported_band_or_incompatible_channel(self):
        self.seed([profile(AP)])
        before = self.state()
        for args in (('on', '--band', '7'), ('on', '--band', '5', '--channel', '6'),
                     ('off', '--band', '5')):
            with self.subTest(args=args):
                self.run_cli(*args, ok=False)
                self.assertEqual(self.state(), before)


if __name__ == '__main__':
    unittest.main()

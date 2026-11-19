"""模擬 nvpmodel / jetson_clocks で電力モードと各設定値の検証を確認する。"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CPU = 'cpu0:  Governor=schedutil MinFreq=1728000 MaxFreq=1728000 CurrentFreq=1728000'
GPU = 'GPU MinFreq=1020000000 MaxFreq=1020000000 CurrentFreq=1020000000'
EMC = 'EMC MinFreq=3199000000 MaxFreq=3199000000 CurrentFreq=3199000000'
FAN = 'FAN Dynamic Speed Control=disabled hwmon2_pwm1=255'


class JetsonPerformanceTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        self.script = root / 'jetson_max_performance.sh'
        shutil.copy2(ROOT / 'scripts' / self.script.name, self.script)
        (root / 'lib').mkdir()
        (root / 'lib/jetson_host.sh').write_text(
            'atd_require_jetson_host() { return "${TEST_HOST_RC:-0}"; }\n'
            'atd_run_as_root() { "$@"; }\n')
        self.calls = root / 'calls'
        self.calls.touch()
        self.env = {**os.environ, 'PATH': str(root) + os.pathsep + os.defpath,
                    'TEST_CALLS': str(self.calls), 'TEST_MODE': 'MAXN_SUPER'}
        commands = {
            'nvpmodel': 'test "$1" = -q || exit 2\nprintf "NV Power Mode: %s\\n0\\n" "$TEST_MODE"\n',
            'jetson_clocks': 'case "$1" in --show) printf "%s\\n" "$TEST_REPORT" ;; '
                             '--fan) exit "${TEST_APPLY_RC:-0}" ;; *) exit 2 ;; esac\n',
        }
        for name, body in commands.items():
            script = root / name
            script.write_text('#!/bin/sh\nprintf "%s %s\\n" "${0##*/}" "$*" >> "$TEST_CALLS"\n' + body)
            script.chmod(0o755)

    def run_script(self, *args, report=(CPU, GPU, EMC, FAN), **env):
        self.calls.write_text('')
        return subprocess.run([str(self.script), *args], capture_output=True, text=True, timeout=5,
                              env={**self.env, 'TEST_REPORT': '\n'.join(report), **env})

    def test_apply_and_read_only_check(self):
        for mode in ('MAXN', 'MAXN_SUPER'):
            for args, expected in [((), ['nvpmodel -q', 'jetson_clocks --fan', 'jetson_clocks --show']),
                                   (('--check',), ['nvpmodel -q', 'jetson_clocks --show'])]:
                with self.subTest(mode=mode, args=args):
                    result = self.run_script(*args, TEST_MODE=mode)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(self.calls.read_text().splitlines(), expected)

    def test_requires_each_clock_domain_and_fan(self):
        for absent in (CPU, GPU, EMC, FAN):
            with self.subTest(absent=absent):
                report = [line for line in (CPU, GPU, EMC, FAN) if line != absent]
                if absent != CPU:
                    report.extend([CPU.replace('cpu0', f'cpu{i}') for i in (1, 2, 3)])
                self.assertNotEqual(self.run_script('--check', report=report).returncode, 0)

    def test_rejects_unfixed_malformed_and_multiple_fan_readings(self):
        for extra in (CPU.replace('MinFreq=1728000', 'MinFreq=100'), 'GPU MinFreq=0 MaxFreq=0',
                      'EMC MinFreq=invalid MaxFreq=123', FAN.replace('disabled', 'nvfancontrol'),
                      FAN.replace('255', '2550'), FAN.replace('pwm1=255', 'pwm2=128')):
            with self.subTest(extra=extra):
                self.assertNotEqual(self.run_script('--check', report=(CPU, GPU, EMC, FAN, extra)).returncode, 0)

    def test_alternative_gpu_domains_and_legacy_emc_override(self):
        gpu_lines = [GPU.replace('GPU', name) for name in ('gpu-gpc-0', 'gpu-nvd-0', 'gpu-pwr-0')]
        override = EMC.replace('MinFreq=3199000000', 'MinFreq=204000000') + ' FreqOverride=1'
        self.assertEqual(self.run_script('--check', report=(CPU, *gpu_lines, override, FAN)).returncode, 0)
        for bad in (override.replace('Override=1', 'Override=0'), override.replace('CurrentFreq=3199000000', 'CurrentFreq=10')):
            with self.subTest(bad=bad):
                self.assertNotEqual(self.run_script('--check', report=(CPU, *gpu_lines, bad, FAN)).returncode, 0)

    def test_mode_or_host_refusal_and_apply_failure(self):
        for mode in ('25W', '', 'MAXN-invalid'):
            with self.subTest(mode=mode):
                self.assertNotEqual(self.run_script(TEST_MODE=mode).returncode, 0)
                self.assertEqual(self.calls.read_text().splitlines(), ['nvpmodel -q'])
        self.assertNotEqual(self.run_script(TEST_HOST_RC='1').returncode, 0)
        self.assertEqual(self.calls.read_text(), '')
        self.assertNotEqual(self.run_script(TEST_APPLY_RC='1').returncode, 0)
        self.assertEqual(self.calls.read_text().splitlines(), ['nvpmodel -q', 'jetson_clocks --fan'])

    def test_help_and_invalid_arguments(self):
        self.assertEqual(self.run_script('--help').returncode, 0)
        for args in (('',), ('--bogus',), ('--check', '--check')):
            self.assertEqual(self.run_script(*args).returncode, 2)
        self.assertEqual(self.calls.read_text(), '')


if __name__ == '__main__':
    unittest.main()

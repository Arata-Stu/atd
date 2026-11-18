"""Docker build cache の確認と削除の範囲を偽 Docker CLI で検査する。"""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/tools/clean-docker-cache.sh"
FAKE_DOCKER = r'''
import json
import os
import sys

args = sys.argv[1:]
with open(os.environ["ATD_TEST_DOCKER_CALLS"], "a") as stream:
    stream.write(json.dumps(args) + "\n")
if args[:2] == ["context", "show"]:
    print("default")
elif args[:2] == ["context", "inspect"]:
    if os.environ.get("ATD_TEST_FAIL") == "inspect":
        sys.exit(6)
    print(os.environ.get("ATD_TEST_ENDPOINT", "unix:///var/run/docker.sock"))
elif args[:2] == ["buildx", "du"]:
    if os.environ.get("ATD_TEST_FAIL") == "du":
        sys.exit(7)
    print("Reclaimable: 8GB")
elif args[:2] == ["buildx", "prune"]:
    if sys.stdin.readline().strip() == "y":
        print("Total: 8GB")
else:
    sys.exit("Unexpected Docker command: " + repr(args))
'''


class CleanDockerCacheTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        docker = root / "docker"
        docker.write_text(f"#!{sys.executable}\n" + FAKE_DOCKER)
        docker.chmod(0o755)
        self.log = root / "calls.jsonl"
        self.env = {"PATH": os.pathsep.join((str(root), os.defpath)), "HOME": str(root),
                    "ATD_TEST_DOCKER_CALLS": str(self.log)}

    def invoke(self, *args, answer="", env=None):
        return subprocess.run([str(SCRIPT), *args], input=answer, text=True, capture_output=True,
                              timeout=10, env={**self.env, **(env or {})})

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def test_dry_run_shows_usage_without_pruning(self):
        result = self.invoke("--dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("8GB", result.stdout)
        buildx_calls = [args for args in self.calls() if args[0] == "buildx"]
        self.assertEqual(buildx_calls, [["buildx", "du", "--builder", "default"]])

    def test_prunes_only_default_build_cache_and_uses_docker_confirmation(self):
        for answer in ("n", "y"):
            with self.subTest(answer=answer):
                self.log.unlink(missing_ok=True)
                result = self.invoke(answer=answer + "\n")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual("Total: 8GB" in result.stdout, answer == "y")
                prune, = [args for args in self.calls() if "prune" in args]
                self.assertEqual(prune[:2], ["buildx", "prune"])
                self.assertCountEqual(prune[2:], ["--builder", "default", "--all"])

    def test_failed_usage_check_stops_before_pruning(self):
        for stage in ("inspect", "du"):
            with self.subTest(stage=stage):
                self.log.unlink(missing_ok=True)
                result = self.invoke(answer="y\n", env={"ATD_TEST_FAIL": stage})
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(any("prune" in args for args in self.calls()))

    def test_remote_docker_is_not_pruned(self):
        result = self.invoke(answer="y\n", env={"ATD_TEST_ENDPOINT": "ssh://user@jetson"})
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(args[0] == "buildx" for args in self.calls()))

    def test_help_and_invalid_option_never_contact_docker(self):
        self.assertEqual(self.invoke("--help").returncode, 0)
        self.assertNotEqual(self.invoke("--unknown").returncode, 0)
        self.assertEqual(self.calls(), [])

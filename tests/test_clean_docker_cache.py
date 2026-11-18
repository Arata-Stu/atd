"""Docker の使用量表示・整理範囲・全削除時の確認を偽 CLI で検査する。"""

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
elif args == ["system", "df"]:
    if os.environ.get("ATD_TEST_FAIL") == "df":
        sys.exit(8)
    print("Images 6 1 92GB")
elif os.environ.get("ATD_TEST_RESET_STATE"):
    state_file = os.environ["ATD_TEST_RESET_STATE"]
    with open(state_file) as stream:
        state = json.load(stream)
    command = args[:2]
    if args[0] == "info":
        state["info_calls"] += 1
        daemon = "changed" if os.environ.get("ATD_TEST_SWITCH") and state["info_calls"] > 1 else "notebook"
        print(daemon, os.environ.get("ATD_TEST_SWARM", "inactive"))
    elif command == ["system", "df"]:
        assert args[2:] == ["--format", "{{.Type}}\t{{.TotalCount}}\t{{.Size}}"], args
        for kind, items in (("Images", state["images"]), ("Containers", state["running"] + state["stopped"]),
                            ("Local Volumes", state["volumes"]), ("Build Cache", state["cache"])):
            print(kind, len(items), "1GB" if items else "0B", sep="\t")
    elif command == ["container", "ls"]:
        ids = list(state["running"])
        if "--all" in args or "-a" in args:
            ids.extend(state["stopped"])
        for container in ids:
            print(container)
    elif command == ["image", "ls"]:
        for image in state["images"]:
            print(image)
    elif command == ["volume", "ls"]:
        for volume in state["volumes"]:
            print(volume)
    elif command == ["network", "ls"]:
        assert "type=custom" in args, args
        for network in state["networks"]:
            print(network)
    elif command == ["container", "stop"]:
        if os.environ.get("ATD_TEST_FAIL") == "stop":
            sys.exit(12)
        for container in args[2:]:
            state["running"].remove(container)
            state["stopped"].append(container)
    elif command == ["system", "prune"]:
        assert set(args[2:]) == {"--all", "--force"}, args
        assert not state["running"], "system prune 前にコンテナを停止していない"
        for resource in ("stopped", "images", "networks", "cache"):
            state[resource].clear()
    elif command == ["volume", "prune"]:
        assert set(args[2:]) == {"--all", "--force"}, args
        assert not state["stopped"], "volume prune 前に停止済みコンテナを削除していない"
        if os.environ.get("ATD_TEST_FAIL") == "volume":
            sys.exit(13)
        state["volumes"].clear()
        leftover = os.environ.get("ATD_TEST_LEFTOVER")
        if leftover:
            state[leftover].append("leftover")
    else:
        sys.exit("Unexpected reset command: " + repr(args))
    with open(state_file, "w") as stream:
        json.dump(state, stream)
elif args[:2] == ["image", "ls"]:
    if os.environ.get("ATD_TEST_FAIL") == "images":
        sys.exit(9)
    print(os.environ.get("ATD_TEST_IMAGES", ""), end="")
elif args[:2] == ["container", "ls"]:
    pass
elif args[:2] == ["image", "inspect"]:
    print(os.environ["ATD_TEST_IMAGE_TAGS"])
elif args[:2] == ["image", "rm"]:
    print(f"Untagged: {args[-1]}")
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
        (root / "python3").symlink_to(sys.executable)
        self.log = root / "calls.jsonl"
        self.env = {"PATH": os.pathsep.join((str(root), os.defpath)), "HOME": str(root),
                    "ATD_TEST_DOCKER_CALLS": str(self.log)}

    def invoke(self, *args, answer="", env=None):
        return subprocess.run([str(SCRIPT), *args], input=answer, text=True, capture_output=True,
                              timeout=10, env={**self.env, **(env or {})})

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def reset_fixture(self):
        state = {"running": ["a" * 64], "stopped": ["b" * 64],
                 "images": ["ubuntu:24.04", "cached_isaac_run_dev_image_local:latest"],
                 "volumes": ["database", "1" * 64], "networks": ["other-project_default"],
                 "cache": ["buildx-default"], "info_calls": 0}
        self.engine = self.log.parent / "engine.json"
        self.engine.write_text(json.dumps(state))
        self.env["ATD_TEST_RESET_STATE"] = str(self.engine)
        return state

    def reset_mutations(self):
        return [args for args in self.calls() if any(word in args for word in ("stop", "rm", "prune"))]

    def test_reset_preview_and_unconfirmed_runs_preserve_every_resource(self):
        for flags, answer in ((("--reset-all", "--dry-run"), "RESET\n"),
                              (("--dry-run", "--reset-all"), "RESET\n"),
                              (("--reset-all",), "y\n"), (("--reset-all",), "")):
            with self.subTest(flags=flags, answer=answer):
                self.log.unlink(missing_ok=True)
                original = self.reset_fixture()
                result = self.invoke(*flags, answer=answer)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("database", result.stdout)
                self.assertIn("ubuntu:24.04", result.stdout)
                self.assertEqual(self.reset_mutations(), [])
                final = json.loads(self.engine.read_text())
                for key in ("running", "stopped", "images", "volumes", "networks", "cache"):
                    self.assertEqual(final[key], original[key])

    def test_confirmed_reset_clears_all_projects_and_volume_types(self):
        original = self.reset_fixture()
        host_bag = self.log.parent / "record" / "session.mcap"
        host_bag.parent.mkdir()
        host_bag.write_bytes(b"bag")
        result = self.invoke("--reset-all", answer="RESET\n", env={"ATD_RECORD_DIR": str(host_bag.parent)})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(host_bag.read_bytes(), b"bag")
        self.assertIn(["container", "stop", original["running"][0]], self.calls())
        final = json.loads(self.engine.read_text())
        for key in ("running", "stopped", "images", "volumes", "networks", "cache"):
            self.assertEqual(final[key], [])
        self.assertEqual(sum(args[:2] == ["system", "df"] for args in self.calls()), 2)

    def test_reset_stops_before_pruning_on_stop_failure_or_daemon_change(self):
        for overrides in ({"ATD_TEST_FAIL": "stop"}, {"ATD_TEST_SWITCH": "1"},
                          {"ATD_TEST_SWARM": "active"}, {"ATD_TEST_ENDPOINT": "ssh://user@jetson"}):
            with self.subTest(overrides=overrides):
                self.log.unlink(missing_ok=True)
                self.reset_fixture()
                result = self.invoke("--reset-all", answer="RESET\n", env=overrides)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(any("prune" in args for args in self.calls()))
                if "ATD_TEST_FAIL" not in overrides:
                    self.assertEqual(self.reset_mutations(), [])

    def test_reset_reports_partial_failure_and_can_finish_on_retry(self):
        original = self.reset_fixture()
        failed = self.invoke("--reset-all", answer="RESET\n", env={"ATD_TEST_FAIL": "volume"})
        self.assertNotEqual(failed.returncode, 0)
        self.assertEqual(json.loads(self.engine.read_text())["volumes"], original["volumes"])
        retry = self.invoke("--reset-all", answer="RESET\n")
        self.assertEqual(retry.returncode, 0, retry.stderr)
        self.assertEqual(json.loads(self.engine.read_text())["volumes"], [])

    def test_reset_reports_resources_left_after_pruning(self):
        for resource in ("running", "networks", "cache"):
            with self.subTest(resource=resource):
                self.reset_fixture()
                result = self.invoke("--reset-all", answer="RESET\n", env={"ATD_TEST_LEFTOVER": resource})
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("リソースが残っています", result.stderr)
                self.assertEqual(json.loads(self.engine.read_text())[resource], ["leftover"])

    def test_dry_run_shows_engine_images_and_build_cache_without_removing_either(self):
        result = self.invoke("--dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("92GB", result.stdout)
        self.assertIn("8GB", result.stdout)
        calls = self.calls()
        self.assertIn(["system", "df"], calls)
        self.assertTrue(any(args[:2] == ["image", "ls"] for args in calls))
        self.assertFalse(any("rm" in args or "prune" in args for args in calls))
        buildx_calls = [args for args in calls if args[0] == "buildx"]
        self.assertEqual(buildx_calls, [["buildx", "du", "--builder", "default"]])

    def test_old_images_with_empty_inventory_leaves_build_cache_alone(self):
        result = self.invoke("--old-images")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("92GB", result.stdout)
        calls = self.calls()
        self.assertIn(["system", "df"], calls)
        self.assertTrue(any(args[:2] == ["image", "ls"] for args in calls))
        self.assertFalse(any(args[0] == "buildx" or "rm" in args or "prune" in args for args in calls))

    def test_old_images_passes_selection_to_removal_without_pruning(self):
        repository = "nvcr.io/nvidia/isaac/ros:isaac_ros-realsense455-atd_"
        old_tag, kept_tag = [repository + char * 32 + "-amd64" for char in ("a", "b")]
        old_id, kept_id = ["sha256:" + char * 64 for char in ("c", "d")]
        rows = [(old_id, old_tag), (kept_id, kept_tag), (kept_id, "cached_isaac_run_dev_image_local:latest")]
        images = "".join(f"{image_id}\t{tag}\t26.4GB\n" for image_id, tag in rows)
        result = self.invoke("--old-images", answer="c" * 12 + "\ny\n", env={
            "ATD_TEST_IMAGES": images, "ATD_TEST_IMAGE_TAGS": json.dumps([old_tag]),
        })
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.calls()
        self.assertEqual([args for args in calls if args[:2] == ["image", "rm"]],
                         [["image", "rm", "--no-prune", old_tag]])
        self.assertFalse(any(args[0] == "buildx" or "prune" in args for args in calls))

    def test_prunes_only_default_build_cache_and_uses_docker_confirmation(self):
        for answer in ("n", "y"):
            with self.subTest(answer=answer):
                self.log.unlink(missing_ok=True)
                result = self.invoke(answer=answer + "\n")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("--old-images", result.stdout)
                self.assertEqual("Total: 8GB" in result.stdout, answer == "y")
                prune, = [args for args in self.calls() if "prune" in args]
                self.assertEqual(prune[:2], ["buildx", "prune"])
                self.assertCountEqual(prune[2:], ["--builder", "default", "--all"])

    def test_failed_usage_check_stops_before_pruning(self):
        for stage in ("inspect", "df", "du"):
            with self.subTest(stage=stage):
                self.log.unlink(missing_ok=True)
                result = self.invoke(answer="y\n", env={"ATD_TEST_FAIL": stage})
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(any("prune" in args for args in self.calls()))

    def test_failed_image_listing_stops_old_image_cleanup(self):
        result = self.invoke("--old-images", answer="all\n", env={"ATD_TEST_FAIL": "images"})
        self.assertNotEqual(result.returncode, 0)
        calls = self.calls()
        self.assertTrue(any(args[:2] == ["image", "ls"] for args in calls))
        self.assertFalse(any("rm" in args or "prune" in args for args in calls))

    def test_remote_docker_is_not_pruned(self):
        result = self.invoke(answer="y\n", env={"ATD_TEST_ENDPOINT": "ssh://user@jetson"})
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(args[0] == "buildx" for args in self.calls()))

    def test_help_and_invalid_option_never_contact_docker(self):
        self.assertEqual(self.invoke("--help").returncode, 0)
        self.assertNotEqual(self.invoke("--unknown").returncode, 0)
        self.assertNotEqual(self.invoke("--dry-run", "--old-images").returncode, 0)
        self.assertNotEqual(self.invoke("--reset-all", "--old-images").returncode, 0)
        self.assertEqual(self.calls(), [])

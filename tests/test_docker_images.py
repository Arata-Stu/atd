"""タグの共有、使用中・CLI 用の ID、選択後の状態変化を偽の Docker 応答で確認する。"""

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.lib import docker_images as cleanup


def tags(first, second, platform="amd64"):
    return {
        f"{cleanup.REGISTRY}/isaac_ros-realsense455-atd_{first * 32}-{platform}:latest",
        f"{cleanup.REGISTRY}:isaac_ros-realsense455-atd_{second * 32}-{platform}",
    }


class DockerImageTests(unittest.TestCase):
    def setUp(self):
        self.old = "a" * 64
        self.cached = "b" * 64
        self.used = "c" * 64
        self.custom = "d" * 64
        self.tags = {
            self.old: tags("0", "1"),
            self.cached: tags("2", "3") | {cleanup.CACHED},
            self.used: tags("4", "5"),
            self.custom: tags("6", "7") | {"local/saved:latest"},
            "e" * 64: {f"{cleanup.REGISTRY}/isaac_ros-realsense455_{'8' * 32}-amd64:latest"},
            "f" * 64: {"ubuntu:24.04"},
        }
        self.in_use = {self.used}
        patcher = patch.object(cleanup, "docker", side_effect=self.docker)
        patcher.start()
        self.addCleanup(patcher.stop)
        patcher = patch.object(cleanup.subprocess, "run")
        self.remove = patcher.start()
        self.addCleanup(patcher.stop)

    def docker(self, *args):
        if args[:2] == ("image", "ls"):
            return "\n".join(f"sha256:{image_id}\t{tag}\t71.2GB"
                             for image_id, refs in self.tags.items() for tag in refs)
        if args[:4] == ("container", "ls", "--all", "--quiet"):
            self.assertEqual(args[4], "--filter")
            image_id = args[5].removeprefix("ancestor=sha256:")
            return "container-id\n" if image_id in self.in_use else ""
        if args[:3] == ("image", "inspect", "--format"):
            if args[-1] == cleanup.CACHED:
                return "sha256:" + next(key for key, refs in self.tags.items() if cleanup.CACHED in refs)
            return json.dumps(sorted(self.tags[args[-1].removeprefix("sha256:")]))
        self.fail(f"unexpected Docker command {args}")

    def run_helper(self, answer=None):
        args = ["docker_images.py"] + (["--remove"] if answer is not None else [])
        output = io.StringIO()
        with patch.object(sys, "argv", args), redirect_stdout(output), patch("builtins.input") as prompt:
            if callable(answer):
                prompt.side_effect = answer
            else:
                prompt.return_value = answer
            cleanup.main()
            if answer is None:
                prompt.assert_not_called()
        return output.getvalue()

    def test_preview_and_filtered_selection(self):
        result = self.run_helper()
        self.assertIn("4 ID", result)
        self.assertEqual(result.count("[削除候補]"), 1)
        for ref in self.tags[self.old]:
            self.assertIn(ref, result)
        self.assertNotIn("ubuntu:24.04", result)
        self.remove.assert_not_called()
        self.run_helper(self.old[:12] + " " + self.old)
        self.remove.assert_called_once_with(["docker", "image", "rm", "--no-prune", *sorted(self.tags[self.old])],
                                            check=True)

    def test_protected_unknown_or_ambiguous_input_never_removes_images(self):
        self.tags["a" * 63 + "0"] = tags("8", "9")
        for answer in (self.old[:12], self.cached, self.custom, self.used, self.old + " bogus"):
            with self.subTest(answer=answer), self.assertRaises(ValueError):
                self.run_helper(answer)
        self.run_helper("")
        self.remove.assert_not_called()

    def test_all_requires_cached_atd_image_for_each_platform(self):
        self.run_helper("all")
        self.assertEqual(self.remove.call_count, 1)
        self.remove.reset_mock()
        self.tags["4" * 64] = tags("8", "9", "arm64-jetpack")
        with self.assertRaises(ValueError):
            self.run_helper("all")
        self.remove.assert_not_called()

    def test_retag_or_container_appearing_during_confirmation_aborts(self):
        def retag(_):
            self.tags[self.old].remove(next(iter(self.tags[self.old])))
            return self.old

        with self.assertRaises(ValueError):
            self.run_helper(retag)

        def start_using(_):
            self.in_use.add(self.old)
            return self.old

        with self.assertRaises(ValueError):
            self.run_helper(start_using)
        self.remove.assert_not_called()

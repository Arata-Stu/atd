import copy
from pathlib import Path
import tempfile
import unittest

from atd_joy_tools import profile


class ProfileTests(unittest.TestCase):
    def test_atomic_save_preserves_previous_on_invalid_input(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "profile.yaml"
            original = profile.default_profile()
            profile.save(path, original)
            invalid = copy.deepcopy(original)
            invalid["axes"]["l2"]["positive"] = -1
            with self.assertRaises(ValueError):
                profile.save(path, invalid)
            self.assertEqual(profile.load(path), original)

    def test_button_capture_ignores_already_held_button_and_noise(self):
        baseline = dict(generation=1, connected=True, raw_axes=[0.0], raw_buttons=[1, 0],
                        axes=[dict(code=0)], buttons=[dict(code=0), dict(code=1)])
        current = copy.deepcopy(baseline)
        current["raw_axes"][0] = 0.03
        self.assertIsNone(profile.detect(baseline, current, "buttons"))
        current["raw_buttons"][1] = 1
        self.assertEqual(profile.detect(baseline, current, "buttons")["code"], 1)

    def test_axis_dpad_capture_and_reconnect_abort(self):
        baseline = dict(generation=1, connected=True, raw_axes=[0.0], raw_buttons=[],
                        axes=[dict(code=0)], buttons=[])
        current = copy.deepcopy(baseline)
        current["raw_axes"][0] = -1.0
        self.assertEqual(profile.detect(baseline, current, "buttons"),
                         dict(source="axis", code=0, direction=-1, threshold=0.5))
        current["generation"] = 2
        with self.assertRaises(ValueError):
            profile.detect(baseline, current, "axes")


if __name__ == "__main__":
    unittest.main()

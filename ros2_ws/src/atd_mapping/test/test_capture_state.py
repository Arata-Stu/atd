import unittest

from atd_mapping.capture_state import LocalizationGate, validate_final_samples


class GateTests(unittest.TestCase):
    def final_gate(self):
        gate = LocalizationGate()
        gate.update(100, {'localized_in_exist_map': 'Yes', 'vo_status': 'OK'}, 1)
        gate.update(1000, {'localized_in_exist_map': 'Yes', 'vo_status': 'OK'}, 2)
        return gate

    def test_current_epoch_cloud_can_precede_tail_with_explicit_warning_metadata(self):
        result = validate_final_samples(self.final_gate(), 1000, 10, 200, 1000, 1000)
        self.assertFalse(result['cloud_within_tail_tolerance'])
        self.assertGreater(result['cloud_tail_age_s'], 0)

    def test_old_epoch_cloud_and_old_path_or_tf_are_refused(self):
        for cloud, path, tf in ((99,1000,1000), (200,800,1000), (200,1000,800), (200,1000,None)):
            with self.subTest(cloud=cloud, path=path, tf=tf), self.assertRaises(ValueError):
                validate_final_samples(self.final_gate(), 1000, 10, cloud, path, tf)

    def test_loss_before_tail_never_yields_successful_snapshot(self):
        gate = self.final_gate()
        gate.update(1010, {'localized_in_exist_map': 'Yes', 'vo_status': 'Lost'}, 3)
        with self.assertRaises(ValueError):
            validate_final_samples(gate, 1010, 10, 1000, 1010, 1010)

    def test_requires_localized_and_tracking(self):
        gate = LocalizationGate()
        self.assertFalse(gate.accepts(100))
        gate.update(100, {"localized_in_exist_map": "No", "vo_status": "OK"}, 1)
        with self.assertRaises(ValueError):
            gate.require_final(100, 0)
        gate.update(110, {"localized_in_exist_map": "Yes", "vo_status": "OK"}, 2)
        self.assertFalse(gate.accepts(109))
        self.assertTrue(gate.accepts(110))
        gate.require_final(110, 0)

    def test_old_diagnostics_do_not_reset_current_epoch(self):
        gate = LocalizationGate()
        gate.update(200, {"localized_in_exist_map": "Yes", "vo_status": "OK"}, 1)
        gate.update(100, {"localized_in_exist_map": "No"}, 2)
        self.assertTrue(gate.localized)
        self.assertEqual(gate.since, 200)

    def test_loss_requires_new_epoch_and_tail(self):
        gate = LocalizationGate()
        gate.update(100, {"localized_in_exist_map": "Yes", "vo_status": "OK"}, 1)
        gate.update(110, {"localized_in_exist_map": "Yes", "vo_status": "Lost"}, 2)
        self.assertFalse(gate.accepts(120))
        gate.update(130, {"localized_in_exist_map": "Yes", "vo_status": "OK"}, 3)
        self.assertEqual(gate.since, 130)
        with self.assertRaises(ValueError):
            gate.require_final(200, 10)
        gate.require_final(140, 10)

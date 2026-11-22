import unittest
from unittest.mock import Mock, patch

from atd_mapping.capture_snapshot import drain_capture
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

    def test_missing_diagnostics_or_hint_can_be_identified_from_failure(self):
        with self.assertRaises(ValueError) as caught:
            validate_final_samples(LocalizationGate(), 1000, 10, None, None, None,
                                   hint_required=True)
        message = str(caught.exception)
        self.assertIn('/diagnostics', message)
        self.assertIn('VGL hint=未受信', message)
        self.assertIn('入力末尾=1000', message)

    def test_never_localized_with_hint_distinct_from_loss_and_late_diagnostics(self):
        gate = LocalizationGate()
        gate.update(10**9, {'localized_in_exist_map': 'No', 'vo_status': 'OK'}, 1)
        with self.assertRaises(ValueError) as caught:
            validate_final_samples(gate, 10**9, 0, None, None, None,
                                   hint_required=True, hint_stamp=1234)
        self.assertIn('一度も', str(caught.exception))
        self.assertIn('localized_in_exist_map=No', str(caught.exception))
        self.assertIn('VGL hint=1234', str(caught.exception))
        gate.update(2*10**9, {'localized_in_exist_map': 'Yes', 'vo_status': 'OK'}, 2)
        gate.update(3*10**9, {'localized_in_exist_map': 'Yes', 'vo_status': 'Lost'}, 3)
        gate.update(10**9, {'localized_in_exist_map': 'No'}, 4)
        with self.assertRaises(ValueError) as caught:
            gate.require_final(5*10**9, 500_000_000)
        for value in ('最後の成功 2000000000', 'vo_status=Lost', '診断stamp=3000000000',
                      '2.000 秒遅れ', '末尾許容=0.500 秒'):
            self.assertIn(value, str(caught.exception))

    def test_drain_allows_late_path_or_tf_after_diagnostic_reached_tail(self):
        node = Mock(last_update=0.)
        node.check_final_inputs.side_effect = [ValueError('map→odom 未受信'), {}]
        options = dict(drain_timeout_s=3., settle_s=.5)
        spin = Mock()
        with patch('atd_mapping.capture_snapshot.time.monotonic', side_effect=[1., 1.6, 2.2]):
            drain_capture(node, Mock(**{'poll.return_value': None}), spin, 1000, 10, options)
        self.assertEqual(spin.call_count, 2)

    def test_drain_reports_unmet_condition_at_timeout(self):
        node = Mock(last_update=0.)
        node.check_final_inputs.side_effect = ValueError('map→odom 未受信')
        with patch('atd_mapping.capture_snapshot.time.monotonic', side_effect=[1., 2.]):
            with self.assertRaisesRegex(ValueError, 'map→odom 未受信'):
                drain_capture(node, Mock(**{'poll.return_value': None}), Mock(), 1000, 10,
                              dict(drain_timeout_s=1., settle_s=.5))

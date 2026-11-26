import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from atd_localization.replay_player import TypedBagInput


class TypedBagInputTests(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.client = self.node.create_client.return_value
        self.spin = Mock()
        modules = patch.dict(sys.modules, {
            'rclpy': NS(spin_once=self.spin),
            'rclpy.qos': NS(DurabilityPolicy=NS(TRANSIENT_LOCAL=1), QoSProfile=Mock()),
            'std_msgs.msg': NS(Bool=object), 'std_srvs.srv': NS(Trigger=NS(Request=object)),
        })
        modules.start()
        self.addCleanup(modules.stop)
        self.input = TypedBagInput(self.node)
        self.addCleanup(self.input.close)

    def test_resuming_requires_local_readers_but_eof_counts_survive_reader_disconnect(self):
        response = self.client.call_async.return_value.result
        response.side_effect = [
            NS(success=False, message='{"left_intra_readers":0}'),
            NS(success=True, message='{"left_intra_readers":2}'),
            NS(success=False, message='{"left_published":91,"left_intra_readers":0}'),
        ]
        self.assertEqual(self.input.wait_for_connections(None, 10)['left_intra_readers'], 2)
        self.spin.assert_called_once()
        self.assertEqual(self.input.counts(None, 10)['left_published'], 91)

    def test_reports_a_failed_eof(self):
        notify = self.node.create_subscription.call_args.args[2]
        self.assertFalse(self.input.finished)
        notify(NS(data=True))
        self.assertTrue(self.input.finished)
        notify(NS(data=False))
        with self.assertRaisesRegex(RuntimeError, 'bag 再生に失敗'):
            _ = self.input.finished

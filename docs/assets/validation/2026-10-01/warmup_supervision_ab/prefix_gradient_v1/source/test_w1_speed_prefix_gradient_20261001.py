"""HH_261001 - Prove the hard-clamp gradient corner case without changing the driving model."""

import unittest
from types import SimpleNamespace
import torch
from portable_e2e.model import ModelConfig, PerspectiveTrajectoryModel, PHYSICAL_STOPMIX_MODEL_ID, FEATURE_NAMES
from scripts.e2e.diagnose_w1_speed_prefix_gradient_20261001 import decoder_derivatives


class PrefixGradientTests(unittest.TestCase):
    def setUp(self):
        cfg = ModelConfig(model_id=PHYSICAL_STOPMIX_MODEL_ID, candidate_count=12)
        self.model = SimpleNamespace(config=cfg)
        self.model._decode_physical_v1 = lambda raw, ego, route, mask: PerspectiveTrajectoryModel._decode_physical_v1(self.model, raw, ego, route, mask)
        self.tensors = dict(ego_history=torch.zeros(1, 10, 13), route_xy=torch.zeros(1, 128, 2), route_mask=torch.ones(1, 128, dtype=torch.bool))
        self.tensors['route_xy'][0, :, 0] = torch.arange(128, dtype=torch.float)

    def test_negative_prefix_blocks_direct_regression_gradient(self):
        raw = torch.ones(1, 6, 64, 2)
        raw[:, :, :, 1] = 0
        raw[:, :, :20, 0] = -1
        _, speed, rows = decoder_derivatives(self.model, raw, self.tensors)
        self.assertGreater(float(speed[0, 0, -1]), 0)
        for row in rows:
            self.assertTrue(row['first20_speed_all_exact_zero'])
            self.assertTrue(row['first20_acceleration_logits_all_negative'])
            self.assertEqual(row['max_abs_near_speed_sum_derivative_first20_accel_logits'], 0)
            self.assertEqual(row['max_abs_full_speed_sum_derivative_first20_accel_logits'], 0)
            self.assertEqual(row['max_abs_full_xy_sum_derivative_first20_accel_logits'], 0)

    def test_positive_prefix_has_gradient(self):
        raw = torch.full((1, 6, 64, 2), .1)
        raw[:, :, :, 1] = 0
        _, _, rows = decoder_derivatives(self.model, raw, self.tensors)
        for row in rows:
            self.assertFalse(row['first20_speed_all_exact_zero'])
            self.assertGreater(row['max_abs_near_speed_sum_derivative_first20_accel_logits'], 0)

    def test_stopped_prefix_depends_on_observed_speed(self):
        raw = torch.full((1, 6, 64, 2), -.1)
        self.tensors['ego_history'][:, -1, FEATURE_NAMES.index('velocity_x_mps')] = 5
        _, _, rows = decoder_derivatives(self.model, raw, self.tensors)
        self.assertTrue(all(not row['first20_speed_all_exact_zero'] for row in rows))


if __name__ == '__main__':
    unittest.main()

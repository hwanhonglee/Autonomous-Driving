"""HH_261001 - Verify explicit surrogate semantics and unchanged DRIVE/STOP forward outputs."""

import unittest
import torch
from portable_e2e.model import ModelConfig, PerspectiveTrajectoryModel, PHYSICAL_STOPMIX_MODEL_ID, PHYSICAL_MAXIMUM_SPEED_MPS
from portable_e2e.directional_speed_gradient_research import (
    DirectionalLowerSpeedGradientModel, GradientTrace, directional_lower_speed_clamp, trace_gradients,
)


class DirectionalGradientTests(unittest.TestCase):
    def test_exact_forward(self):
        values = torch.tensor([-9., -.1, 0., .1, PHYSICAL_MAXIMUM_SPEED_MPS, 20.])
        self.assertTrue(torch.equal(directional_lower_speed_clamp(values), values.clamp(0, PHYSICAL_MAXIMUM_SPEED_MPS)))

    def test_inward_only_lower_and_native_upper(self):
        z = torch.tensor([-.1, -.1, 0., .1, PHYSICAL_MAXIMUM_SPEED_MPS, 20., 20.], requires_grad=True)
        incoming = torch.tensor([-2., 2., 2., -2., 2., -2., 2.])
        directional_lower_speed_clamp(z).backward(incoming)
        self.assertTrue(torch.equal(z.grad, torch.tensor([-2., 0., 2., -2., 2., 0., 0.])))

    def test_no_invented_movement_for_correct_stop_target(self):
        z = torch.tensor([-.1], requires_grad=True)
        directional_lower_speed_clamp(z).square().sum().backward()
        self.assertEqual(float(z.grad), 0.)

    def test_synthetic_positive_target_can_escape_lower_saturation(self):
        z = torch.tensor([-.1], requires_grad=True)
        loss = (directional_lower_speed_clamp(z) - .5).square().sum()
        loss.backward()
        with torch.no_grad():
            updated = z - .2 * z.grad
        self.assertGreater(float(directional_lower_speed_clamp(updated)), 0.)
        native = z.detach().clone().requires_grad_(True)
        (native.clamp(0, PHYSICAL_MAXIMUM_SPEED_MPS) - .5).square().sum().backward()
        self.assertEqual(float(native.grad), 0.)

    def test_trace_is_detached_and_scope_restored(self):
        trace = GradientTrace()
        with trace_gradients(trace):
            z = torch.tensor([-.1, -.2, -.3, .1], requires_grad=True)
            directional_lower_speed_clamp(z).backward(torch.tensor([-1., 1., 0., 1.]))
        self.assertEqual(trace.receipt()['lower_saturated'], 3)
        self.assertEqual(trace.receipt()['lower_inward'], 1)
        self.assertEqual(trace.receipt()['lower_outward'], 1)
        self.assertEqual(trace.receipt()['lower_zero'], 1)
        directional_lower_speed_clamp(torch.tensor([-.1], requires_grad=True)).sum().backward()
        self.assertEqual(trace.calls, 1)

    def test_full_model_same_parameters_forward_and_stop(self):
        torch.manual_seed(20261001)
        cfg = ModelConfig(model_id=PHYSICAL_STOPMIX_MODEL_ID, candidate_count=12, maximum_step_m=1.)
        native = PerspectiveTrajectoryModel(cfg).eval()
        candidate = DirectionalLowerSpeedGradientModel(cfg).eval()
        candidate.load_state_dict(native.state_dict(), strict=True)
        images = torch.zeros(1, 6, 3, cfg.image_height, cfg.image_width)
        calibration = torch.zeros(1, 6, cfg.calibration_features)
        ego = torch.zeros(1, 10, cfg.ego_features)
        ego_mask = torch.ones(1, 10, dtype=torch.bool)
        route = torch.zeros(1, cfg.route_points, 2)
        route[0, :, 0] = torch.arange(cfg.route_points, dtype=torch.float)
        mask = torch.ones(1, cfg.route_points, dtype=torch.bool)
        with torch.no_grad():
            a = native(images, calibration, ego, ego_mask, route, mask)
            b = candidate(images, calibration, ego, ego_mask, route, mask)
        self.assertEqual(set(native.state_dict()), set(candidate.state_dict()))
        self.assertTrue(all(torch.equal(x, y) for x, y in zip(a, b)))
        self.assertTrue(torch.equal(a[0][:, 6:], b[0][:, 6:]))
        self.assertTrue(torch.equal(a[1][:, 6:], b[1][:, 6:]))


if __name__ == '__main__':
    unittest.main()

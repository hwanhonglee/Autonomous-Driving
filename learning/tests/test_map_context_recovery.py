"""HH_261001 - Check baseline parity and trainable context without using DEV data."""

import unittest
import torch
from portable_e2e.map_perception import MapPerception, semantic_loss
from portable_e2e.map_context_recovery import MapContextRecovery


class ContextRecoveryTest(unittest.TestCase):
    def setUp(self):
        torch.set_num_threads(2)
        torch.manual_seed(20261001)
        self.base = MapPerception()
        self.model = MapContextRecovery()
        self.model.load_baseline(self.base.state_dict())

    def test_exact_initial_parity_even_and_odd_sizes(self):
        for height, width in ((32, 64), (35, 67)):
            image = torch.rand(2, 3, height, width)
            with torch.no_grad():
                reference, actual = self.base(image), self.model(image)
            self.assertEqual(actual.shape, (2, 7, height, width))
            self.assertTrue(torch.equal(reference, actual))

    def test_rejects_missing_baseline_weight(self):
        state = self.base.state_dict()
        state.pop(next(iter(state)))
        with self.assertRaises(ValueError):
            self.model.load_baseline(state)

    def test_rejects_unexpected_weight(self):
        state = self.base.state_dict()
        state["unexpected"] = torch.zeros(1)
        with self.assertRaises(ValueError):
            self.model.load_baseline(state)

    def test_context_learns_after_zero_initialized_projection(self):
        optimizer = torch.optim.AdamW(self.model.parameters(), lr=3e-5)
        image = torch.rand(2, 3, 32, 64)
        target = torch.randint(0, 7, (2, 32, 64))
        for _ in range(2):
            optimizer.zero_grad()
            loss = semantic_loss(self.model(image), target, torch.ones(7))
            self.assertTrue(torch.isfinite(loss))
            loss.backward()
            optimizer.step()
        self.assertGreater(self.model.context[0].weight.grad.abs().sum().item(), 0)
        self.assertGreater(self.model.context[-1].weight.abs().sum().item(), 0)


if __name__ == "__main__":
    unittest.main()

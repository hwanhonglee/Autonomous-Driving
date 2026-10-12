"""Focused fail-closed tests for the Town05 right-turn CPU DEV diagnostic."""
from __future__ import annotations

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from scripts.e2e import evaluate_map_town05_right_dev as right


class Town05RightEvalTest(unittest.TestCase):
    def test_registered_frames_and_denominator(self) -> None:
        self.assertEqual(right.SELECTED, ("frames/0000", "frames/0105", "frames/0203"))
        self.assertEqual(right.MODEL_NAMES, ("ai_v2", "candidate_c_rejected"))

    def test_modified_plan_fails_before_inference(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            plan = directory / "PLAN.md"
            plan.write_text("changed", encoding="utf-8")
            with patch.object(right, "OUTPUT", directory), patch.object(right, "PLAN", plan):
                with self.assertRaisesRegex(ValueError, "registered PLAN changed"):
                    right.fixed_source_checks()

    def test_exclusive_output_never_overwrites(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            with patch.object(right, "OUTPUT", directory):
                destination = directory / "summary.json"
                right.write_new(destination, b"first")
                with self.assertRaisesRegex(ValueError, "output already exists"):
                    right.write_new(destination, b"second")
                self.assertEqual(destination.read_bytes(), b"first")

    def test_road_lane_gate_excludes_sign_and_never_promotes(self) -> None:
        a = {"classes": {"road": {"iou": .80}, "lane_marking": {"iou": .50},
                         "traffic_sign": {"iou": .99}}}
        c = {"classes": {"road": {"iou": .79}, "lane_marking": {"iou": .48},
                         "traffic_sign": {"iou": 0.0}}}
        gate = right.road_lane_gate(a, c)
        self.assertAlmostEqual(gate["road_iou_delta_c_minus_a_pp"], -1.0)
        self.assertAlmostEqual(gate["lane_iou_delta_c_minus_a_pp"], -2.0)
        self.assertTrue(gate["town05_directional_nonregression"])
        self.assertFalse(gate["sign_used_for_gate"])
        self.assertFalse(gate["automatic_promotion"])
        self.assertFalse(gate["deployment_approved"])

    def test_full_fov_panel_keeps_every_native_pixel(self) -> None:
        rgb = np.zeros((360, 640, 3), dtype=np.uint8)
        rgb[..., 0] = 47
        target = np.ones((360, 640), dtype=np.uint8)
        a = np.full((360, 640), 2, dtype=np.uint8)
        c = np.full((360, 640), 3, dtype=np.uint8)
        image = np.asarray(right.panel(right.SELECTED[0], rgb, target, a, c))
        self.assertEqual(image.shape, (816, 1280, 3))
        np.testing.assert_array_equal(image[69:429, :640], rgb)
        np.testing.assert_array_equal(image[69:429, 640:], right.prior.COLORS[target])
        np.testing.assert_array_equal(image[454:814, :640], right.prior.COLORS[a])
        np.testing.assert_array_equal(image[454:814, 640:], right.prior.COLORS[c])
        with self.assertRaisesRegex(ValueError, "native full FOV"):
            right.panel(right.SELECTED[0], rgb[:359], target, a, c)


if __name__ == "__main__":
    unittest.main()

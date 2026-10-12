"""HH_261001 - Verify ego centering and prior acknowledged rather than future control selection."""

import math
import numpy as np
from scripts.e2e import publish_town07_w1_pi_gallery_20261001 as gallery


def test_measured_origin_is_center_for_every_yaw():
    for yaw in (0., math.pi/2, -1.):
        state = dict(x=7., y=-2., yaw=yaw)
        np.testing.assert_array_equal(gallery.centered([(7., -2.)], state), [[0., 0.]])


def test_rotated_forward_and_left_are_not_swapped():
    state = dict(x=0., y=0., yaw=math.pi/2)
    np.testing.assert_allclose(gallery.centered([(0., 3.), (-2., 0.)], state), [[3., 0.], [0., 2.]], atol=1e-15)


def test_current_or_future_ack_cannot_be_shown_as_observed_effect():
    command = dict(command_frame=10, inference_frame=10,
        application=dict(learned_control_applied=True), selected=dict(xy_base_m=[[1., 0.]]))
    origin = dict(x=4., y=2., yaw=math.pi/2, frame=10)
    assert gallery.acknowledged_plan([command], {10: origin}, origin) == (None, [])
    row, plan = gallery.acknowledged_plan([command], {10: origin}, dict(frame=11))
    assert row is command
    np.testing.assert_allclose(plan, [[4., 3.]], atol=1e-15)


def test_unacknowledged_control_is_excluded():
    command = dict(command_frame=9, application=dict(learned_control_applied=False))
    assert gallery.acknowledged_plan([command], {}, dict(frame=11)) == (None, [])


def test_resampling_supports_already_installed_pillow():
    resampling = getattr(gallery.Image, "Resampling", gallery.Image)
    assert gallery.Image.new("RGB", (2, 2)).resize((1, 1), resampling.LANCZOS).size == (1, 1)

"""HH_261008 - Prospective TRAIN-only crop-box choice; this scalar helper grants no payload read or training.

Protection is fixed before observation: global_exposure_ordinal % 2 == 0.
Only a genuine positive full-frame sign target whose ORIGINAL proposed crop's
NEAREST label result has zero sign support receives the genuine full-FOV box.
Every other proposal is returned unchanged, including genuine GT-negative and
unprotected background crops. No new labels, flips, random draws or sample IDs.
Scalar counts alone do not authenticate source pixels; the separately reviewed
CPU replay must prove these witnesses from admitted original TRAIN target bytes.
"""

STEPS, BATCH, PIXELS = 20000, 8, 360*640
FULL_FOV = (0, 0, 360, 640)
LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture", "traffic_sign", "pole")
SIGN_INDEX = 5


def require(condition, message):
    if not condition: raise ValueError(message)


def exposure_index(batch_ordinal, batch_position):
    require(type(batch_ordinal) is int and 0 <= batch_ordinal < STEPS
        and type(batch_position) is int and 0 <= batch_position < BATCH,
        "exact zero-based declared20k/B8 chronology required")
    return batch_ordinal*BATCH+batch_position


def checked_box(proposed):
    require(type(proposed) in (list, tuple) and len(proposed) == 4
        and all(type(value) is int for value in proposed), "exact four integer crop coordinates required")
    top, left, height, width = proposed
    require(top >= 0 and left >= 0 and height > 0 and width > 0
        and top+height <= 360 and left+width <= 640, "original calibrated source crop bounds required")
    return proposed


def select_box(global_exposure_ordinal, proposed, full_GT_sign_pixels, original_NEAREST_sign_pixels):
    """HH_261008 - Choose only the box; original shared RGB/GT apply_crop math is not replaced."""
    require(type(global_exposure_ordinal) is int and 0 <= global_exposure_ordinal < STEPS*BATCH,
        "exact global original exposure ordinal required")
    checked_box(proposed)
    require(type(full_GT_sign_pixels) is type(original_NEAREST_sign_pixels) is int
        and 0 <= full_GT_sign_pixels <= PIXELS and 0 <= original_NEAREST_sign_pixels <= PIXELS,
        "typed finite native sign-support witnesses required")
    require(full_GT_sign_pixels != 0 or original_NEAREST_sign_pixels == 0,
        "NEAREST cannot invent label5 in a genuine GT-negative frame")
    if tuple(proposed) == FULL_FOV:
        require(original_NEAREST_sign_pixels == full_GT_sign_pixels, "genuine full-FOV target copy support differs")
    protected = global_exposure_ordinal % 2 == 0
    rescued = protected and full_GT_sign_pixels > 0 and original_NEAREST_sign_pixels == 0
    selected = list(FULL_FOV) if rescued else proposed
    return dict(selected_box=selected, protected=protected, full_GT_negative=full_GT_sign_pixels == 0,
        full_GT_positive=full_GT_sign_pixels > 0, rescued=rescued,
        original_proposal_unchanged=not rescued, same_selected_box_required_for_future_RGB_and_GT=True)


def contract():
    return dict(provenance="HH_261008 - SOURCE-only prospective data preparation; no model/training approval.",
        steps=STEPS, batch=BATCH, exposures=STEPS*BATCH, sign_index=SIGN_INDEX, labels=list(LABELS),
        protection="global_exposure_ordinal % 2 == 0", full_FOV_box=list(FULL_FOV),
        rescue_rule="protected AND genuine fullGT label5>0 AND original proposed NEAREST label5==0",
        all_original_indices_random_proposals_FALSE_bits_preserved=True,
        negative_and_unprotected_proposals_returned_identically=True, crop_reroll=False,
        original_RGB_BILINEAR_GT_NEAREST_math_unchanged=True, labels_or_thresholds_changed=False,
        DEV_HOLDOUT_TEST_or_denied_RAW_eligible=False, model_inferences=0, training_steps=0,
        actual_payload_read_authorized=False, model_capability_or_causality_claimed=False)

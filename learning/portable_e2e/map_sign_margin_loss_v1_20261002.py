"""HH_261002 - Prepare a source/positive-image balanced sign loss, not an object head.

This module changes no model, label, crop, sampler or inference threshold.  The
auxiliary is an uncapped NLL with a fixed coefficient, not a capability approval.
Importing this prepared module does not import Torch or authorize GPU execution.
"""
from __future__ import annotations

SCHEMA = "portable_e2e.map_source_positive_image_sign_margin_loss.20261002.v1"
SIGN_CLASS, CLASSES, HARD_NEGATIVES, COEFFICIENT = 5, 7, 64, .10
SOURCE_GROUPS = ("Town01_original497", "Town02_original232",
                 "Town01_p02_FALSE240", "Town01_p04_FALSE240")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def prepared_contract():
    # HH_261002 - A source-array group or positive image is not a physical sign instance.
    return dict(schema=SCHEMA, classes=CLASSES, sign_class=SIGN_CLASS,
        source_groups=list(SOURCE_GROUPS), coefficient=COEFFICIENT,
        margin="sign_logit-logsumexp(the_existing_six_nonsign_logits)",
        positive_reduction="mean_actual_GT_sign_pixels_then_mean_positive_images_within_source",
        negative_reduction="mean_min(64,GT_nonsign_count)_hardest_softplus(m)_per_image_then_all_source_images",
        fractional_threshold_ties="detached_threshold; above=1; all_ties=(k-above_count)/tie_count; divide_by_k",
        source_reduction="equal_mean_of_present_exact_source_groups",
        group_terms=dict(positive=.5, negative=.5),
        empty_positive_group="differentiable_zero; all_negative_images_remain",
        empty_nonsign_image="differentiable_zero; effective_k=0",
        control="one_direct_original_weighted_semantic_CE_call",
        candidate="same_original_weighted_semantic_CE_plus_0.10_auxiliary",
        coefficient_is_loss_cap=False, physical_object_identity_claimed=False,
        new_head=False, pseudo_labels=False, sign_dependent_crop_reroll=False,
        inference_argmax_or_threshold_changed=False, training_authorized=False,
        model_promoted=False, vehicle_control_approved=False)


def _tensor_inputs(logits, target, source_groups):
    import torch
    require(isinstance(logits, torch.Tensor) and logits.layout == torch.strided
        and logits.dtype == torch.float32 and logits.ndim == 4
        and logits.shape[0] > 0 and logits.shape[1] == CLASSES
        and min(logits.shape[2:]) > 0, "finite float32 [N,7,H,W] logits required")
    require(isinstance(target, torch.Tensor) and target.layout == torch.strided
        and target.dtype == torch.int64 and target.device == logits.device
        and target.shape == (logits.shape[0], *logits.shape[2:]),
        "same-device int64 full target shape required")
    require(bool(torch.isfinite(logits).all()) and bool(((target >= 0) & (target < CLASSES)).all()),
        "all logits/true seven-class target pixels must be valid")
    require(type(source_groups) in (tuple, list) and len(source_groups) == logits.shape[0]
        and all(type(group) is str and group in SOURCE_GROUPS for group in source_groups),
        "one exact authenticated source-array group per image required")


def deterministic_hard_negative_mean(values):
    """HH_261002 - Tie gradients are shared over all ties, never arbitrary top-k IDs."""
    import torch
    require(isinstance(values, torch.Tensor) and values.layout == torch.strided
        and values.dtype == torch.float32 and values.ndim == 1
        and bool(torch.isfinite(values).all()) and bool((values >= 0).all()),
        "one finite nonnegative float32 GT-nonsign NLL vector required")
    count, k = values.numel(), min(HARD_NEGATIVES, values.numel())
    if k == 0:
        return values.sum() * 0., dict(valid_nonsign_pixels=0, effective_k=0,
            above_threshold_pixels=0, threshold_tie_pixels=0, fractional_tie_weight=0.)
    # HH_261002 - Sorting only detached values avoids random/instance-dependent tie selection.
    threshold = torch.sort(values.detach(), descending=True).values[k-1]
    above, tied = values.detach() > threshold, values.detach() == threshold
    above_count, tie_count = int(above.sum()), int(tied.sum())
    require(0 <= above_count < k and tie_count >= k-above_count, "hard-negative threshold census differs")
    # HH_261002 - This is exactly the fractional-weight rule, with a mean over every threshold tie.
    result = (values[above].sum() + values[tied].mean() * (k-above_count)) / k
    require(bool(torch.isfinite(result)), "nonfinite hard-negative reduction")
    return result, dict(valid_nonsign_pixels=count, effective_k=k,
        above_threshold_pixels=above_count, threshold_tie_pixels=tie_count,
        fractional_tie_weight=(k-above_count)/tie_count)


def sign_margin_auxiliary(logits, target, source_groups):
    """HH_261002 - Balance true positive images and sources without replacing CE."""
    import torch
    from torch.nn import functional as F
    _tensor_inputs(logits, target, source_groups)
    nonsign = [i for i in range(CLASSES) if i != SIGN_CLASS]
    margin = logits[:, SIGN_CLASS] - torch.logsumexp(logits[:, nonsign], dim=1)
    require(bool(torch.isfinite(margin).all()), "nonfinite one-versus-rest sign margin")
    positive_nll, negative_nll = F.softplus(-margin), F.softplus(margin)
    require(bool(torch.isfinite(positive_nll).all()) and bool(torch.isfinite(negative_nll).all()),
        "nonfinite sign NLL")
    zero = logits.reshape(-1)[0] * 0.
    positives, negatives, image_counts = [], [], []
    for i in range(logits.shape[0]):
        positive = target[i] == SIGN_CLASS
        sign_count = int(positive.sum())
        positives.append(positive_nll[i][positive].mean() if sign_count else None)
        negative, census = deterministic_hard_negative_mean(negative_nll[i][~positive])
        negatives.append(negative)
        image_counts.append(dict(source_group=source_groups[i], actual_GT_sign_pixels=sign_count,
            positive_term_present=sign_count > 0, **census))
    terms, group_counts = [], {}
    for group in SOURCE_GROUPS:
        ids = [i for i, name in enumerate(source_groups) if name == group]
        if not ids:
            continue
        positive_ids = [i for i in ids if positives[i] is not None]
        positive_mean = torch.stack([positives[i] for i in positive_ids]).mean() if positive_ids else zero
        negative_mean = torch.stack([negatives[i] for i in ids]).mean()
        terms.append(.5 * positive_mean + .5 * negative_mean)
        group_counts[group] = dict(images=len(ids), positive_images=len(positive_ids),
            actual_GT_sign_pixels=sum(image_counts[i]["actual_GT_sign_pixels"] for i in ids),
            negative_images=len(ids), actual_GT_nonsign_pixels=sum(image_counts[i]["valid_nonsign_pixels"] for i in ids),
            effective_hard_negative_mass=sum(image_counts[i]["effective_k"] for i in ids),
            positive_mean=float(positive_mean.detach()), negative_mean=float(negative_mean.detach()))
    auxiliary = torch.stack(terms).mean()
    require(bool(torch.isfinite(auxiliary)), "nonfinite source-balanced auxiliary")
    return auxiliary, dict(schema=SCHEMA, auxiliary=float(auxiliary.detach()),
        present_sources=len(terms), groups=group_counts, images=image_counts,
        coefficient_is_cap=False, physical_object_identity_claimed=False)


def candidate_loss(logits, target, weights, source_groups, semantic_loss):
    """HH_261002 - Invoke the original weighted CE once, then add the fixed auxiliary."""
    import torch
    _tensor_inputs(logits, target, source_groups)
    require(isinstance(weights, torch.Tensor) and weights.dtype == torch.float32
        and weights.shape == (CLASSES,) and weights.device == logits.device
        and bool(torch.isfinite(weights).all()) and bool((weights > 0).all())
        and callable(semantic_loss), "original positive seven-class weights/provider required")
    ce = semantic_loss(logits, target, weights)
    require(isinstance(ce, torch.Tensor) and ce.ndim == 0 and bool(torch.isfinite(ce)), "finite original CE required")
    auxiliary, stats = sign_margin_auxiliary(logits, target, source_groups)
    loss = ce + COEFFICIENT * auxiliary
    require(bool(torch.isfinite(loss)), "nonfinite candidate loss")
    return loss, dict(**stats, original_CE=float(ce.detach()), total_loss=float(loss.detach()), coefficient=COEFFICIENT)
